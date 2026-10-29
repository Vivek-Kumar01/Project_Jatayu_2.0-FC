import math
from ambiance import Atmosphere
import numpy as np
import jatayu_fc_uav as uav
import mission_settings_fc as ms
import Phases.takeoff as to

# ------------------------------------------------------------------------------
# 1. INITIAL CONDITIONS FROM TAKEOFF ROTATION
# ------------------------------------------------------------------------------
m_initial = to.m
m = to.m
SoC = to.SoC
H2_mass = to.H2_mass
h = to.h
V = to.V
s_roll = to.s_roll

t = 0.0
t_step = 0.1  # seconds

# ------------------------------------------------------------------------------
# 2. ELECTRICAL BUS & POWER SPLIT CONFIGURATION
# ------------------------------------------------------------------------------
# 28V Avionics auxiliary draw on the 700V bus
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck  # kW

# Inverter & Motor continuous/surge capacity
P_EM_elec_max = ms.power_inverter  # 105 kW electrical inverter limit

# Propeller thrust parameters
eta_prop = 0.68
theta = 6.5  # degrees, climb angle
Cd_0_p1 = uav.Cd_0 + ms.delta_CD_0 + ms.delta_CD_0_flap

Phase_Success = True

# ------------------------------------------------------------------------------
# 3. AERODYNAMICS & TIME-MARCHING PULL-UP SIMULATION
# ------------------------------------------------------------------------------
while h < ms.obstacle_height:
    # Update atmospheric density with altitude
    atmos_curr = Atmosphere(h)
    rho_curr = float(atmos_curr.density[0]) if hasattr(atmos_curr.density, '__len__') else float(atmos_curr.density)

    # Dynamic fuel cell available generation and BoP at altitude h
    P_FC_gross_avail, P_FC_net_avail, P_BoP = uav.get_fc_available_power(h)
    P_FC_net = P_FC_net_avail
    P_FC_gross = P_FC_gross_avail

    # Dynamic load fraction and empirical efficiency
    xi_fc = float(np.clip(P_FC_gross / P_FC_gross_avail, 0.05, 1.0)) if P_FC_gross_avail > 0.0 else 1.0
    eta_fc = uav.get_fc_efficiency(P_FC_gross, P_FC_gross_avail)

    # Power split on the 700V DC bus
    P_bus_demand = P_EM_elec_max + P_avionics_bus
    P_bat_discharge = P_bus_demand - P_FC_net

    if P_bat_discharge > ms.power_battery_discharge_surge:
        P_bat_discharge = ms.power_battery_discharge_surge
        P_bus_demand = P_FC_net + P_bat_discharge
        P_EM_elec = P_bus_demand - P_avionics_bus
    else:
        P_EM_elec = P_EM_elec_max

    # Shaft and propulsive power
    P_shaft_prop = P_EM_elec * uav.eta_inverter * uav.eta_motor
    P_EM_mech = P_shaft_prop
    P_available = P_shaft_prop * eta_prop
    DoH = P_bat_discharge / P_bus_demand

    # Flight mechanics
    L = m * 9.81 * math.cos(math.radians(theta))  # N
    Cl = L / (0.5 * rho_curr * V**2 * uav.S)
    Cd = Cd_0_p1 + uav.K * Cl**2
    D = 0.5 * rho_curr * V**2 * uav.S * Cd  # N
    T = P_available * 1000.0 / V  # N
    F_net = T - D - m * 9.81 * math.sin(math.radians(theta))  # N
    a_net = F_net / m  # m/s^2

    V = V + a_net * t_step  # m/s
    h = h + V * math.sin(math.radians(theta)) * t_step  # m
    t = t + t_step  # seconds
    s_roll = s_roll + V * math.cos(math.radians(theta)) * t_step  # m

    # Dynamic hydrogen mass consumption (LHV = 120 MJ/kg)
    m_H2_consumed = ((P_FC_gross * 1000.0) / (eta_fc * uav.LHV_H2)) * t_step  # kg
    H2_mass = H2_mass - m_H2_consumed
    m = m - m_H2_consumed

    # Buffer battery energy depletion
    elec_energy_used = ((P_bat_discharge / uav.eta_discharge) * t_step) / 3600.0  # kWh
    SoC = SoC - (elec_energy_used / ms.battery_capacity)

    # --- TELEMETRY LOGGING (Pull-Up) ---
    P_req = (D * V) / 1000.0  # kW
    cumulative_time = (to.t + to.N) + t
    ms.log_telemetry(
        cumulative_time, 'Pull_Up', h, V, P_req, P_FC_gross, P_FC_net, P_BoP,
        P_EM_mech, P_bat_discharge, H2_mass, SoC, xi_fc, eta_fc
    )

    if h < ms.obstacle_height and s_roll > ms.max_permissible_stot:
        print('''
        Error: Total distance over obstacle exceeded maximum permissible limit.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if SoC < ms.SoC_min:
        print('''
        Error: Battery state of charge dropped below minimum limit.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if H2_mass < ms.mass_reserve_H2:
        print('''
        Error: Insufficient H2 fuel remaining for mission.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 4. RESULTS REPORTING
# ------------------------------------------------------------------------------
status_str = "Successful" if Phase_Success else "Failed"
print(f'''
    ------- Pull Up {status_str} -------
    P_FC_gross: {P_FC_gross:.2f} kW | Load Ratio (xi): {xi_fc:.2f} | Stack Eff: {eta_fc*100:.2f} %
    P_FC_net: {P_FC_net:.2f} kW, P_Battery_surge: {P_bat_discharge:.2f} kW, P_shaft: {P_shaft_prop:.2f} kW
    Degree of Hybridization (Battery share): {DoH*100:.1f} %
    Total phase 1 climb time: {t:.2f} seconds
    Total horizontal distance to clear 15m obstacle: {s_roll:.2f} meters
    Final speed: {V:.2f} m/s
    Final altitude: {h:.2f} meters
    H2 Burned: {(m_initial - m)*1000:.2f} g
    Remaining usable H2 mass: {H2_mass - ms.mass_reserve_H2:.2f} kg
    Remaining battery SoC: {SoC*100:.2f} %
''')