import math
from ambiance import Atmosphere
import jatayu_fc_uav as uav
import mission_settings_fc as ms
import Phases.cruise_in as ci

# ------------------------------------------------------------------------------
# 1. INITIAL CONDITIONS FROM CRUISE-IN
# ------------------------------------------------------------------------------
m_initial = ci.m
m = m_initial
SoC = ci.SoC
H2_mass = ci.H2_mass

t_step = 0.1  # seconds
h_target = ms.altitude_takeoff + 300.0  # 300 m AGL descent termination gate

# Commanded gross idle setpoint (6% of gross rated capacity)
percent_idle_power = 6.0  # %
P_FC_gross_idle = (percent_idle_power / 100.0) * uav.P_FC_gross_rated  # 3.60 kW commanded at altitude

# 28V Avionics auxiliary draw on the high-voltage bus
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck  # kW

# Flight idle produces zero net thrust (unpowered clean glide on wing L/D)
P_avail = 0.0
eta_prop = 0.0
P_bat_discharge = 0.0

V = ci.vel_cruise
t = 0.0
s_hor = 0.0
h = ms.cruise_altitude

Phase_Success = True
t_baseline = ci.t_baseline + ci.time_cruise
rho0 = 1.2250  # kg/m^3

# ------------------------------------------------------------------------------
# 2. TIME-MARCHING DESCENT SIMULATION
# ------------------------------------------------------------------------------
while h > h_target:
    atmos = Atmosphere(h)
    rho = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)
    sigma = rho / rho0

    # Explicit altitude-scaled BoP parasitics at current commanded gross idle
    f_comp_alt = uav.f_parasitic_compressor * (1.0 / sigma) ** 0.60
    f_BoP_alt = f_comp_alt + uav.f_parasitic_coolant

    P_BoP_idle = P_FC_gross_idle * f_BoP_alt
    P_FC_net_idle = (P_FC_gross_idle - P_BoP_idle) * uav.eta_DCDC_boost

    #Balance of bus power: Avionics gets served, leftover spins prop at idle
    P_EM_elec = max(0.0, P_FC_net_idle - P_avionics_bus)
    P_shaft_prop = P_EM_elec * (uav.eta_inverter * uav.eta_motor)

    # Aerodynamics for unpowered steady glide
    Cl = (m * 9.81) / (0.5 * rho * (V**2) * uav.S)
    Cd = uav.Cd_0 + uav.K * (Cl**2)
    D = 0.5 * rho * (V**2) * uav.S * Cd
    P_req = D * V  # Watts

    # Rate of Sink (ROS)
    ROS = P_req / (m * 9.81)  # m/s
    theta = math.asin(ROS / V)

    # State integration
    h -= ROS * t_step
    s_hor += V * math.cos(theta) * t_step
    t += t_step

    # Hydrogen reactant mass consumption during idle (kg)
    m_H2_consumed = ((P_FC_gross_idle * 1000.0) / (uav.eta_FC_rated * uav.LHV_H2)) * t_step
    H2_mass -= m_H2_consumed
    m -= m_H2_consumed

    # Telemetry logging (12 arguments)
    fuel_remaining = H2_mass - ms.mass_reserve_H2
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Descent', h, V,
        P_req / 1000.0, P_FC_gross_idle, P_FC_net_idle, P_BoP_idle,
        P_shaft_prop, P_bat_discharge, H2_mass, SoC
    )

    if H2_mass < ms.mass_reserve_H2:
        print('''
        Error: Emergency H2 reserve breached during descent!
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
    ------- Descent {status_str} -------
    Horizontal Distance covered: {s_hor/1000.0:.2f} km
    Descent Time: {t/60.0:.2f} min
    Final Altitude: {h:.2f} m
    Final Mass: {m:.2f} kg
    H2 reactant burned: {m_H2_burned*1000.0:.2f} g ({m_H2_burned:.2f} kg)
    Remaining usable H2 mass: {H2_mass - ms.mass_reserve_H2:.2f} kg
    Remaining battery SoC: {SoC*100.0:.2f} %
    Final Velocity: {V:.2f} m/s
    Total Mission Time Elapsed So Far: {(t_baseline + t)/3600.0:.2f} hours
''')