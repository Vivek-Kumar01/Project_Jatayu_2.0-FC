import math
from ambiance import Atmosphere
import numpy as np
import jatayu_fc_uav as uav
import mission_settings_fc as ms
import Phases.climb as cl

# ------------------------------------------------------------------------------
# 1. INITIAL CONDITIONS FROM CLIMB PHASE 1
# ------------------------------------------------------------------------------
m_initial = cl.m
m = cl.m
SoC = cl.SoC
H2_mass = cl.H2_mass

V = cl.V
h = cl.h
s_hor = cl.s_hor

t = 0.0
t_step = 0.5  # seconds
t_baseline = cl.t_baseline + cl.t  # Continuous cumulative mission time

h_target = ms.cruise_altitude  # 3000 m target

guess_mass_H2_consumed = 0.8  # kg
rho_cruise = float(Atmosphere(ms.cruise_altitude).density[0]) if hasattr(Atmosphere(ms.cruise_altitude).density, '__len__') else float(Atmosphere(ms.cruise_altitude).density)
vel_cruise = math.sqrt(
    (2.0 * (m - guess_mass_H2_consumed) * 9.81) / (rho_cruise * uav.S) * math.sqrt(uav.K / uav.Cd_0)
)

eta_prop = 0.74
dV_dh = (vel_cruise - cl.V) / (ms.cruise_altitude - cl.h_target)

# ------------------------------------------------------------------------------
# 2. CONTINUOUS BATTERY DISCHARGE LIMIT & AVIONICS
# ------------------------------------------------------------------------------
# Throttle fraction of continuous battery rating (0.0 to 1.0)
throttle_bat_cruise_climb = 0.4

# Continuous discharge assist capability
P_bat_max = throttle_bat_cruise_climb * ms.power_battery_discharge_cont  # kW

# 28V Avionics auxiliary draw on the 700V bus
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck  # kW

Phase_Success = True

# ------------------------------------------------------------------------------
# 3. TIME-MARCHING CRUISE-CLIMB SIMULATION
# ------------------------------------------------------------------------------
while h < h_target:
    atmos = Atmosphere(h)
    rho = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)

    # 1. Dynamic altitude-derated fuel cell capacity
    P_FC_gross_avail, P_FC_net_avail, P_BoP = uav.get_fc_available_power(h)
    P_FC_gross = P_FC_gross_avail
    P_FC_net = P_FC_net_avail

    # Dynamic load fraction and empirical efficiency
    xi_fc = float(np.clip(P_FC_gross / P_FC_gross_avail, 0.05, 1.0)) if P_FC_gross_avail > 0.0 else 1.0
    eta_fc = uav.get_fc_efficiency(P_FC_gross, P_FC_gross_avail)

    # 2. Maximum combined electrical power delivered to motor inverter
    P_bus_available = P_FC_net + P_bat_max - P_avionics_bus
    P_EM_elec = min(P_bus_available, ms.power_inverter)

    # Actual continuous battery discharge load
    P_bat_discharge = (P_EM_elec + P_avionics_bus) - P_FC_net

    # Degree of Hybridization (Battery share of traction bus draw)
    DoH = P_bat_discharge / (P_EM_elec + P_avionics_bus)

    # 3. Shaft and thrust power
    P_prop_shaft = P_EM_elec * uav.eta_inverter * uav.eta_motor  # kW
    P_avail = P_prop_shaft * eta_prop                            # kW

    # 4. Aerodynamics & Rate of Climb (Energy-Height formulation)
    Cl = (m * 9.81) / (0.5 * rho * V**2 * uav.S)
    Cd = uav.Cd_0 + uav.K * Cl**2
    D = 0.5 * rho * V**2 * uav.S * Cd
    P_req = D * V  # Watts

    denominator = m * (9.81 + V * dV_dh)
    R_C = (P_avail * 1000.0 - P_req) / denominator  # m/s

    if R_C <= 0.0:
        print("\nError: Rate of climb collapsed to <= 0 m/s before reaching cruise altitude.")
        Phase_Success = False
        break

    theta = math.asin(R_C / V)

    # State update
    V += R_C * dV_dh * t_step
    h += R_C * t_step
    s_hor += V * math.cos(theta) * t_step
    t += t_step

    # 5. Dynamic Hydrogen mass consumption (LHV = 120 MJ/kg)
    m_H2_consumed = ((P_FC_gross * 1000.0) / (eta_fc * uav.LHV_H2)) * t_step  # kg
    H2_mass -= m_H2_consumed
    m -= m_H2_consumed

    # 6. Battery buffer depletion
    elec_energy_used = ((P_bat_discharge / uav.eta_discharge) * t_step) / 3600.0  # kWh
    SoC -= (elec_energy_used / ms.battery_capacity)

    # --- TELEMETRY LOGGING (Cruise Climb) ---
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Cruise_Climb', h, V, P_req / 1000.0,
        P_FC_gross, P_FC_net, P_BoP, P_prop_shaft, P_bat_discharge,
        H2_mass, SoC, xi_fc, eta_fc
    )

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
    ------- Cruise Climb (to {h_target:.0f}m) {status_str} -------
    EOP P_FC_gross: {P_FC_gross:.2f} kW | Load Ratio (xi): {xi_fc:.2f} | Stack Eff: {eta_fc*100:.2f} %
    EOP P_FC_net: {P_FC_net:.2f} kW, P_Battery_cont: {P_bat_discharge:.2f} kW
    EOP Shaft Power: {P_prop_shaft:.2f} kW, Thrust Power: {P_avail:.2f} kW
    DoH (Battery fraction): {DoH*100:.1f} %
    Rate of Climb (Final): {R_C:.2f} m/s
    Total phase climb time: {t/60:.2f} mins
    Final speed: {V:.2f} m/s (Target cruise: {vel_cruise:.2f} m/s)
    Horizontal distance travelled: {s_hor/1000:.2f} km
    H2 reactant burned: {(m_initial - m)*1000:.2f} g
    Remaining usable H2 mass: {H2_mass - ms.mass_reserve_H2:.2f} kg
    Remaining battery SoC: {SoC*100:.2f} %
''')