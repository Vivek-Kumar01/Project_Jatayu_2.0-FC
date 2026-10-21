import math
from ambiance import Atmosphere
import jatayu_fc_uav as uav
import mission_settings_fc as ms
import Phases.loiter as lo

# ------------------------------------------------------------------------------
# 1. INITIAL CONDITIONS & SETTINGS FROM LOITER
# ------------------------------------------------------------------------------
t_step = 1.0  # seconds
rho = float(Atmosphere(ms.cruise_altitude).density[0]) if hasattr(Atmosphere(ms.cruise_altitude).density, '__len__') else float(Atmosphere(ms.cruise_altitude).density)

descent_dist = 50.0  # km (distance allocated from base for descent phase)

# Total cruise-in transit required before initiating descent
target_cruise_in_dist = ms.cruise_distance - descent_dist  # km (150 km - 50 km = 100 km)

m_initial = lo.m
m = m_initial
SoC = lo.SoC
H2_mass = lo.H2_mass

dist_covered = 0.0  # km
time_cruise = 0.0   # seconds

eta_prop = 0.76  # Propeller efficiency at cruise

# Minimum-drag cruise coefficients (maximum L/D cruise point)
cl_cruise = math.sqrt(uav.Cd_0 / uav.K)
cd_cruise = uav.Cd_0 + uav.K * cl_cruise**2

Phase_Success = True
t_baseline = lo.t_baseline + lo.endurance

# 28V Avionics auxiliary draw on the high-voltage bus
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck  # kW

# ------------------------------------------------------------------------------
# 2. TIME-MARCHING CRUISE-IN SIMULATION (PURE FC)
# ------------------------------------------------------------------------------
while dist_covered < target_cruise_in_dist:
    # Level cruise velocity at current mass
    vel_cruise = math.sqrt((2.0 * m * 9.81) / (rho * uav.S) * math.sqrt(uav.K / uav.Cd_0))  # m/s

    # Aerodynamics & Propeller Shaft Power Required
    D_cruise = 0.5 * rho * (vel_cruise**2) * uav.S * cd_cruise  # N
    P_prop_req = (D_cruise * vel_cruise) / 1000.0                # kW (aerodynamic power)
    P_shaft_prop = P_prop_req / eta_prop                         # kW (mechanical shaft power)

    # Electrical motor input power required from inverter
    P_EM_elec = P_shaft_prop / (uav.eta_inverter * uav.eta_motor)  # kW

    # Total high-voltage bus load (Traction + Avionics)
    P_bus_load = P_EM_elec + P_avionics_bus  # kW

    # Check fuel cell capacity at 3000 m cruise altitude
    P_FC_gross_avail, P_FC_net_avail, P_BoP = uav.get_fc_available_power(ms.cruise_altitude)

    if P_bus_load > P_FC_net_avail:
        print('''
        Error: Required cruise-in bus power exceeds available net Fuel Cell capacity.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    # Battery floats on standby (0 kW assist, 0 kW charge)
    P_bat_discharge = 0.0

    # Net-to-Gross transmission efficiency (accounts for BoP parasitics + Boost DC-DC)
    eta_net_bus = (1.0 - (P_BoP / P_FC_gross_avail)) * uav.eta_DCDC_boost
    P_FC_gross_demand = P_bus_load / eta_net_bus                  # kW
    P_BoP_demand = P_FC_gross_demand * (P_BoP / P_FC_gross_avail) # kW

    # Hydrogen reactant mass consumption (LHV = 120 MJ/kg)
    m_H2_consumed = ((P_FC_gross_demand * 1000.0) / (uav.eta_FC_rated * uav.LHV_H2)) * t_step  # kg
    H2_mass -= m_H2_consumed
    m -= m_H2_consumed

    # Distance and time integration
    dist_covered += (vel_cruise * t_step) / 1000.0  # km
    time_cruise += t_step                          # seconds

    # Telemetry logging (12 arguments)
    cumulative_time = t_baseline + time_cruise
    ms.log_telemetry(
        cumulative_time, 'Cruise_In', ms.cruise_altitude, vel_cruise,
        P_prop_req, P_FC_gross_demand, P_bus_load, P_BoP_demand,
        P_shaft_prop, P_bat_discharge, H2_mass, SoC
    )

    if H2_mass < ms.mass_reserve_H2:
        print('''
        Error: Insufficient H2 fuel remaining during Cruise In (Reserve breached).
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
    ------- Cruise In {status_str} -------
    Target distance to descent gate: {target_cruise_in_dist:.2f} km
    Distance covered: {dist_covered:.2f} km
    Transit Time: {time_cruise/60:.2f} min ({time_cruise/3600:.2f} hours)
    Cruise Velocity: {vel_cruise:.2f} m/s
    Propeller Shaft Power: {P_shaft_prop:.2f} kW
    FC Net Bus Load: {P_bus_load:.2f} kW
    FC Gross Load: {P_FC_gross_demand:.2f} kW
    Total UAV Mass: {m:.2f} kg
    H2 reactant burned: {m_H2_burned*1000:.2f} g ({m_H2_burned:.2f} kg)
    Remaining usable H2 mass: {H2_mass - ms.mass_reserve_H2:.2f} kg
    Remaining battery SoC: {SoC*100:.2f} %
    Total Mission Time Elapsed So Far: {(t_baseline + time_cruise)/3600:.2f} hours
''')