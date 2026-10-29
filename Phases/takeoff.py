import math
from ambiance import Atmosphere
import numpy as np
import jatayu_fc_uav as uav
import mission_settings_fc as ms

# ------------------------------------------------------------------------------
# 1. ATMOSPHERE & INITIAL STATE
# ------------------------------------------------------------------------------
h = ms.altitude_takeoff
atmos = Atmosphere(h)
rho = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)
t_step = 0.1  # seconds

# ------------------------------------------------------------------------------
# 2. ELECTRICAL BUS & POWER SPLIT SIZING
# ------------------------------------------------------------------------------
# 28V Avionics auxiliary draw placed on the 700V bus via isolated buck converter
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck  # kW

# Fuel Cell generation at field elevation (altitude-derated)
P_FC_gross_avail, P_FC_net_avail, P_BoP = uav.get_fc_available_power(h)

# Full FC continuous net power committed to the bus during takeoff roll
P_FC_net = P_FC_net_avail  # Usable electrical kW delivered to 700V bus
P_FC_gross = P_FC_gross_avail

# Dynamic load fraction and empirical thermodynamic efficiency
xi_fc = float(np.clip(P_FC_gross / P_FC_gross_avail, 0.05, 1.0)) if P_FC_gross_avail > 0 else 1.0
eta_fc = uav.get_fc_efficiency(P_FC_gross, P_FC_gross_avail)

# High-voltage bus power allocation:
P_EM_elec_max = ms.power_inverter  # kW electrical capacity of inverter
P_shaft_max = P_EM_elec_max * uav.eta_inverter * uav.eta_motor  # ~96.7 kW mechanical

# Power split on the 700V high-voltage DC bus:
P_bus_demand = P_EM_elec_max + P_avionics_bus
P_bat_discharge = P_bus_demand - P_FC_net  # kW pulled from battery

# Guard against battery burst discharge rating
if P_bat_discharge > ms.power_battery_discharge_surge:
    P_bat_discharge = ms.power_battery_discharge_surge
    P_bus_demand = P_FC_net + P_bat_discharge
    P_EM_elec = P_bus_demand - P_avionics_bus
else:
    P_EM_elec = P_EM_elec_max

# Mechanical shaft power delivered to constant-speed propeller:
P_shaft_prop = P_EM_elec * uav.eta_inverter * uav.eta_motor  # kW
P_EM_mech = P_shaft_prop

# Degree of Hybridization (Battery share of total bus power demand)
DoH = P_bat_discharge / P_bus_demand

# ------------------------------------------------------------------------------
# 3. THRUST COMPUTATION
# ------------------------------------------------------------------------------
Merit_Factor = 0.7
Static_thrust = (Merit_Factor ** (2 / 3)) * (
    2 * rho * (math.pi / 4 * uav.D_prop**2) * (P_shaft_prop * 1000.0) ** 2
) ** (1 / 3)

N = 1  # seconds after rotation on ground roll
V_stall = math.sqrt((2 * uav.MTOW * 9.81) / (rho * uav.S * uav.Cl_max_ground_TO))  # m/s
V_LO = 1.15 * V_stall  # m/s

prop_eff_liftoff = 0.65
Thrust_liftoff = prop_eff_liftoff * P_shaft_prop * 1000.0 / V_LO  # N

def thrust_available(V, Static_thrust=Static_thrust, Thrust_liftoff=Thrust_liftoff, V_LO=V_LO):
    return Static_thrust + (Thrust_liftoff - Static_thrust) * (V / V_LO) ** 2

# ------------------------------------------------------------------------------
# 4. TIME-MARCHING GROUND ROLL SIMULATION
# ------------------------------------------------------------------------------
t = 0.0          # seconds
s_roll = 0.0     # m
V = 0.0          # m/s
m = uav.MTOW     # kg
SoC = ms.SoC_max # Initial buffer battery SoC
H2_mass = ms.mass_H2_total  # kg initial reactant

while V < V_LO:
    T = thrust_available(V)
    D = 0.5 * rho * V**2 * uav.S * ms.CD_takeoff
    W = m * 9.81
    L = 0.5 * rho * V**2 * uav.S * uav.Cl_ground
    s_roll = s_roll + V * t_step
    F_net = T - D - (W - L) * ms.rolling_friction_coefficient
    a_net = F_net / m

    # Dynamic Hydrogen mass consumption (LHV = 120 MJ/kg)
    m_H2_consumed = ((P_FC_gross * 1000.0) / (eta_fc * uav.LHV_H2)) * t_step  # kg
    H2_mass = H2_mass - m_H2_consumed
    m = m - m_H2_consumed  # Aircraft mass update

    # Buffer battery energy depletion
    elec_energy_used = ((P_bat_discharge / uav.eta_discharge) * t_step) / 3600.0  # kWh
    SoC = SoC - (elec_energy_used / ms.battery_capacity)

    V = V + a_net * t_step
    t = t + t_step

    # --- TELEMETRY LOGGING (Step) ---
    P_req = (D * V) / 1000.0  # kW
    ms.log_telemetry(
        t, 'Takeoff', h, V, P_req, P_FC_gross, P_FC_net, P_BoP,
        P_EM_mech, P_bat_discharge, H2_mass, SoC, xi_fc, eta_fc
    )

    if s_roll > ms.max_permissible_sroll:
        print("Takeoff failed: maximum permissible ground roll exceeded.")
        break

    if SoC < ms.SoC_min:
        print("Takeoff failed: battery SoC dropped below minimum.")
        break

    if H2_mass < ms.mass_reserve_H2:
        print("Takeoff failed: insufficient H2 fuel remaining.")
        break

# ------------------------------------------------------------------------------
# 5. ROTATION SEGMENT
# ------------------------------------------------------------------------------
s_roll = s_roll + N * V_LO

# Hydrogen and battery consumption during rotation interval (N seconds)
m_H2_consumed_rot = ((P_FC_gross * 1000.0) / (eta_fc * uav.LHV_H2)) * N
H2_mass = H2_mass - m_H2_consumed_rot
m = m - m_H2_consumed_rot

elec_energy_used_rot = ((P_bat_discharge / uav.eta_discharge) * N) / 3600.0
SoC = SoC - (elec_energy_used_rot / ms.battery_capacity)

# --- TELEMETRY LOGGING (Post-Rotation) ---
P_req_rot = (0.5 * rho * V_LO**2 * uav.S * ms.CD_takeoff * V_LO) / 1000.0  # kW
ms.log_telemetry(
    t + N, 'Takeoff_Rotation', h, V_LO, P_req_rot, P_FC_gross, P_FC_net, P_BoP,
    P_EM_mech, P_bat_discharge, H2_mass, SoC, xi_fc, eta_fc
)

Phase_Success = s_roll < ms.max_permissible_sroll

if not Phase_Success:
    print("Takeoff failed: maximum permissible ground roll exceeded.")
else:
    print(f'''
    ------- Takeoff Successful -------
    P_FC_gross: {P_FC_gross:.2f} kW | Load Ratio (xi): {xi_fc:.2f} | Stack Eff: {eta_fc*100:.2f} %
    P_FC_net: {P_FC_net:.2f} kW, P_Battery_surge: {P_bat_discharge:.2f} kW, P_shaft: {P_shaft_prop:.2f} kW
    Degree of Hybridization (Battery share): {DoH*100:.1f} %
    Total ground roll distance: {s_roll:.2f} m
    Lift off velocity: {V_LO:.2f} m/s
    Time taken to reach lift off speed: {t + N:.2f} seconds
    H2 reactant consumed during takeoff: {(ms.mass_H2_total - H2_mass)*1000:.2f} g
    Usable H2 left (excluding reserve): {H2_mass - ms.mass_reserve_H2:.2f} kg
    Battery SoC after takeoff: {SoC*100:.2f} %\n
''')