import math
from ambiance import Atmosphere
import jatayu_fc_uav as uav
import mission_settings_fc as ms
import Phases.cruise_climb as cc

# ------------------------------------------------------------------------------
# 1. INITIAL CONDITIONS FROM CRUISE-CLIMB
# ------------------------------------------------------------------------------
t_step = 1.0  # seconds
rho = float(Atmosphere(ms.cruise_altitude).density[0]) if hasattr(Atmosphere(ms.cruise_altitude).density, '__len__') else float(Atmosphere(ms.cruise_altitude).density)

# Option A: Direct flight straight to target station (no standoff cutoff)
target_cruise_distance = ms.cruise_distance - (cc.s_hor / 1000.0)  # km

m_initial = cc.m
m = m_initial
SoC = cc.SoC
H2_mass = cc.H2_mass

dist_covered = 0.0  # km
time_cruise = 0.0   # seconds

eta_prop = 0.76  # Propeller efficiency at cruise

# Minimum-drag cruise coefficients (maximum L/D cruise point)
cl_cruise = math.sqrt(uav.Cd_0 / uav.K)
cd_cruise = uav.Cd_0 + uav.K * cl_cruise**2

t_baseline = cc.t_baseline + cc.t  # Continuous cumulative time elapsed before cruise out

# 28V Avionics auxiliary draw on the high-voltage bus
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck  # kW

Phase_Success = True

# ------------------------------------------------------------------------------
# 2. TIME-MARCHING CRUISE-OUT SIMULATION
# ------------------------------------------------------------------------------
while dist_covered < target_cruise_distance:
    # Level cruise velocity at current mass
    vel_cruise = math.sqrt((2.0 * m * 9.81) / (rho * uav.S) * math.sqrt(uav.K / uav.Cd_0))  # m/s
    
    # Aerodynamics & Propeller Shaft Power Required
    D_cruise = 0.5 * rho * (vel_cruise**2) * uav.S * cd_cruise  # Newtons
    P_prop_req = (D_cruise * vel_cruise) / 1000.0                # kW
    P_shaft_prop = P_prop_req / eta_prop                         # kW

    # Electrical motor input power required from inverter
    P_EM_elec = P_shaft_prop / (uav.eta_inverter * uav.eta_motor)  # kW

    # Total high-voltage bus load (Traction + Avionics)
    P_bus_load = P_EM_elec + P_avionics_bus  # kW

    # Check fuel cell capacity at 3000 m cruise altitude
    P_FC_gross_avail, P_FC_net_avail, P_BoP = uav.get_fc_available_power(ms.cruise_altitude)

    if P_bus_load > P_FC_net_avail:
        print('''
        Error: Required cruise bus power exceeds available net Fuel Cell capacity.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    # Battery floats on standby (0 kW assist, 0 kW charge)
    P_bat_discharge = 0.0
    DoH = 0.0

    # 2. Net-to-Gross transmission efficiency at current altitude
    # Accounts for BoP parasitics and Boost DC-DC converter losses
    eta_net_bus = (1.0 - (P_BoP / P_FC_gross_avail)) * uav.eta_DCDC_boost
    P_FC_gross_demand = P_bus_load / eta_net_bus  # kW

    # Solve Fuel Cell gross output demanded for current net bus load
    P_FC_gross_demand = P_bus_load / eta_net_bus  # kW

    # Hydrogen reactant mass consumption
    m_H2_consumed = ((P_FC_gross_demand * 1000.0) / (uav.eta_FC_rated * uav.LHV_H2)) * t_step  # kg
    H2_mass -= m_H2_consumed
    m -= m_H2_consumed

    # Distance and time integration
    dist_covered += (vel_cruise * t_step) / 1000.0  # km
    time_cruise += t_step                          # seconds


# --- TELEMETRY LOGGING (Cruise Out) ---
    fuel_remaining = H2_mass - ms.mass_reserve_H2
    cumulative_time = t_baseline + time_cruise
    P_BoP_demand = P_FC_gross_demand * (P_BoP / P_FC_gross_avail)
    ms.log_telemetry(
        cumulative_time,      # 1. t
        'Cruise_Out',         # 2. phase
        ms.cruise_altitude,   # 3. alt
        vel_cruise,           # 4. v
        P_prop_req,           # 5. p_req
        P_FC_gross_demand,    # 6. p_fc_gross
        P_bus_load,           # 7. p_fc_net
        P_BoP_demand,         # 8. p_bop
        P_shaft_prop,         # 9. p_em
        P_bat_discharge,      # 10. p_bat
        H2_mass,              # 11. h2_mass
        SoC                   # 12. soc
    )
    if H2_mass < ms.mass_reserve_H2:
        print('''
        Error: Insufficient H2 fuel remaining for mission (Reserve breached).
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if SoC < ms.SoC_min:
        print('''
        Error: Battery SoC dropped below minimum limit.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 3. RESULTS REPORTING
# ------------------------------------------------------------------------------
m_H2_burned = m_initial - m
status_str = "Successful" if Phase_Success else "Failed"

print(f'''
    ------- Cruise Out {status_str} -------
    Target distance to station: {target_cruise_distance:.2f} km
    Distance covered: {dist_covered:.2f} km
    Transit Time: {time_cruise/60:.2f} min ({time_cruise/3600:.2f} hours)
    Cruise Velocity: {vel_cruise:.2f} m/s
    Propeller Shaft Power: {P_shaft_prop:.2f} kW
    FC Net Bus Load: {P_bus_load:.2f} kW
    FC Gross Output: {P_FC_gross_demand:.2f} kW
    Total UAV Mass: {m:.2f} kg
    H2 reactant burned: {m_H2_burned*1000:.2f} g ({m_H2_burned:.2f} kg)
    Remaining usable H2 mass: {H2_mass - ms.mass_reserve_H2:.2f} kg
    Remaining battery SoC: {SoC*100:.2f} %
''')