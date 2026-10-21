import math
from ambiance import Atmosphere
import jatayu_fc_uav as uav
import mission_settings_fc as ms
import Phases.descent as ds

# ------------------------------------------------------------------------------
# 1. INITIAL CONDITIONS & AERODYNAMIC DIRTY CONFIGURATION
# ------------------------------------------------------------------------------
m_initial = ds.m
m = ds.m
SoC = ds.SoC
H2_mass = ds.H2_mass

t_step = 0.1  # seconds
h_target = ms.obstacle_height  # 15 m obstacle clearance altitude

V = ds.V
h = ds.h

# Dirty Configuration Drag Increments (Landing flaps + extended landing gear)
delta_Cd0_flaps = uav.Cd_0
k_uc = 3.16e-5
delta_CD_0_gear = (m * 9.81 / uav.S) * k_uc * (uav.MTOW ** -0.215)
Cd0_land = uav.Cd_0 + delta_CD_0_gear + delta_Cd0_flaps

# Landing stall speed and target approach speed
safety_margin = 1.30  # 30% margin above stall
rho_init = float(Atmosphere(h).density[0]) if hasattr(Atmosphere(h).density, '__len__') else float(Atmosphere(h).density)
V_stall_land = math.sqrt((2.0 * m * 9.81) / (rho_init * uav.Cl_max_Land * uav.S))
V_target = V_stall_land * safety_margin

# Hotel avionics bus load
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck  # kW

# 6% Gross Idle Setpoint for Deceleration Subphase
percent_idle_power = 6.0
P_FC_gross_idle = (percent_idle_power / 100.0) * uav.P_FC_gross_rated  # 3.60 kW

t = 0.0
s_hor = 0.0
Phase_Success = True
t_baseline = ds.t_baseline + ds.t
rho0 = 1.2250  # kg/m^3

# ==============================================================================
# SUBPHASE 1: HORIZONTAL DECELERATION TO V_TARGET (FLIGHT IDLE)
# ==============================================================================
print("--> Initiating Phase 7A: Approach Level Deceleration to 1.3 V_stall...")

while V > V_target:
    atmos = Atmosphere(h)
    rho = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)
    sigma = rho / rho0

    # Aerodynamic drag in dirty landing configuration (level flight, theta = 0)
    Cl = (m * 9.81) / (0.5 * rho * (V**2) * uav.S)
    Cd = Cd0_land + uav.K * (Cl**2)
    D = 0.5 * rho * (V**2) * uav.S * Cd  # Newtons
    P_req = (D * V) / 1000.0             # kW (aerodynamic power)

    # Net deceleration (zero net thrust at flight idle)
    a_net = -D / m
    V += a_net * t_step
    s_hor += V * t_step
    t += t_step

    # Dynamic BoP parasitics and net available bus power at idle throttle
    f_comp_alt = uav.f_parasitic_compressor * (1.0 / sigma) ** 0.60
    f_BoP_alt = f_comp_alt + uav.f_parasitic_coolant
    P_BoP_idle = P_FC_gross_idle * f_BoP_alt
    P_FC_net_idle = (P_FC_gross_idle - P_BoP_idle) * uav.eta_DCDC_boost

    # Leftover power spins propeller at idle torque
    P_EM_elec = max(0.0, P_FC_net_idle - P_avionics_bus)
    P_shaft_prop = P_EM_elec * (uav.eta_inverter * uav.eta_motor)
    P_bat_discharge = 0.0

    # H2 reactant consumption at idle
    m_H2_consumed = ((P_FC_gross_idle * 1000.0) / (uav.eta_FC_rated * uav.LHV_H2)) * t_step
    H2_mass -= m_H2_consumed
    m -= m_H2_consumed

    # Telemetry logging (12 arguments)
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Approach_Decel', h, V,
        P_req, P_FC_gross_idle, P_FC_net_idle, P_BoP_idle,
        P_shaft_prop, P_bat_discharge, H2_mass, SoC
    )

    if H2_mass < ms.mass_reserve_H2:
        print("Error: Emergency H2 fuel reserve breached during deceleration!")
        Phase_Success = False
        break

# ==============================================================================
# SUBPHASE 2: 3° GLIDESLOPE POWERED DESCENT TO OBSTACLE CLEARANCE HEIGHT
# ==============================================================================
print(f"--> Initiating Phase 7B: 3-deg Glideslope Descent to {h_target:.1f} m...")

theta = math.radians(3.0)  # 3-degree ILS standard glideslope
ROS = V * math.sin(theta)  # m/s rate of descent
eta_prop = 0.60            # Propeller efficiency during low-speed high-thrust approach

percent_power_final = 0.0

while h > h_target and Phase_Success:
    atmos = Atmosphere(h)
    rho = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)
    sigma = rho / rho0

    # Steady-state glideslope aerodynamics at V = V_target
    Cl = (m * 9.81) / (0.5 * rho * (V**2) * uav.S)
    Cd = Cd0_land + uav.K * (Cl**2)
    D = 0.5 * rho * (V**2) * uav.S * Cd  # Newtons

    # Thrust required balancing drag minus along-path gravity component
    T_req = D - (m * 9.81 * math.sin(theta))  # Newtons
    P_avail = max(0.0, T_req * V) / 1000.0   # kW (aerodynamic propulsive power)
    P_shaft_prop = P_avail / eta_prop        # kW (mechanical shaft power)

    # Electrical motor demand from inverter
    P_EM_elec = P_shaft_prop / (uav.eta_inverter * uav.eta_motor)  # kW
    P_bus_load = P_EM_elec + P_avionics_bus                        # kW

    # Fuel Cell available net capacity at current approach altitude
    P_FC_gross_avail, P_FC_net_avail, P_BoP = uav.get_fc_available_power(h)

    percent_power = P_bus_load / P_FC_net_avail
    percent_power_final = percent_power

    if percent_power > 1.0:
        print(f'''
        MISSION FAILED: Approach bus power exceeds available FC net capacity!
        Load Demand: {P_bus_load:.2f} kW, Net Avail: {P_FC_net_avail:.2f} kW
        ''')
        Phase_Success = False
        break

    # Battery floats on standby (0 kW assist)
    P_bat_discharge = 0.0

    # Fuel cell gross demand & BoP parasitics
    eta_net_bus = (1.0 - (P_BoP / P_FC_gross_avail)) * uav.eta_DCDC_boost
    P_FC_gross_demand = P_bus_load / eta_net_bus
    P_BoP_demand = P_FC_gross_demand * (P_BoP / P_FC_gross_avail)

    # Reactant consumption
    m_H2_consumed = ((P_FC_gross_demand * 1000.0) / (uav.eta_FC_rated * uav.LHV_H2)) * t_step
    H2_mass -= m_H2_consumed
    m -= m_H2_consumed

    # Kinematics integration
    h -= ROS * t_step
    s_hor += V * math.cos(theta) * t_step
    t += t_step

    # Telemetry logging (12 arguments)
    P_req = (D * V) / 1000.0
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Approach_Glide', h, V,
        P_req, P_FC_gross_demand, P_bus_load, P_BoP_demand,
        P_shaft_prop, P_bat_discharge, H2_mass, SoC
    )

    if H2_mass < ms.mass_reserve_H2:
        print("Error: Emergency H2 fuel reserve breached during glideslope!")
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 3. RESULTS REPORTING
# ------------------------------------------------------------------------------
m_H2_burned = m_initial - m
status_str = "Successful" if Phase_Success else "Failed"

print(f'''
    ------- Approach {status_str} -------
    Horizontal Distance covered: {s_hor/1000.0:.2f} km
    Approach Time: {t/60.0:.2f} min
    Final Altitude: {h:.2f} m
    Final Mass: {m:.2f} kg
    H2 reactant burned: {m_H2_burned*1000.0:.2f} g ({m_H2_burned:.2f} kg)
    Remaining usable H2 mass: {H2_mass - ms.mass_reserve_H2:.2f} kg
    Remaining battery SoC: {SoC*100.0:.2f} %
    Final Approach Velocity: {V:.2f} m/s
    Percent FC Capacity Utilized: {percent_power_final*100.0:.2f} %
    Total Mission Time Elapsed So Far: {(t_baseline + t)/3600.0:.2f} hours
''')
