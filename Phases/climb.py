import math
from ambiance import Atmosphere
import jatayu_fc_uav as uav
import mission_settings_fc as ms
import Phases.pullup as pu

# ------------------------------------------------------------------------------
# 1. INITIAL CONDITIONS FROM PULL-UP
# ------------------------------------------------------------------------------
m_initial = pu.m
m = pu.m
SoC = pu.SoC
H2_mass = pu.H2_mass
h = pu.h
V = pu.V
s_hor = 0.0

t = 0.0
t_step = 0.5  # seconds
t_baseline = pu.to.t + pu.to.N + pu.t  # Continuous elapsed time

# Mid-altitude climb segment target (1500 m)
h_target = ms.altitude_takeoff + (ms.cruise_altitude - ms.altitude_takeoff) / 2.0  # 1500.0 m

# Propeller efficiency during steady climb
eta_prop = 0.74

# ------------------------------------------------------------------------------
# 2. CONTINUOUS BATTERY DISCHARGE LIMIT & AVIONICS
# ------------------------------------------------------------------------------
# Disengage 10C takeoff surge; switch strictly to continuous safe discharge (4C = 32.0 kW)
throttle_setting = 0.4
P_bat_max = throttle_setting * ms.power_battery_discharge_cont  # 32.0 kW

# 28V Avionics auxiliary draw on the 700V bus
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck  # kW

Phase_Success = True

# ------------------------------------------------------------------------------
# 3. TIME-MARCHING CLIMB SIMULATION
# ------------------------------------------------------------------------------
while h < h_target:
    atmos = Atmosphere(h)
    rho = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)

    # 1. Dynamic altitude-derated fuel cell capacity
    P_FC_gross_avail, P_FC_net_avail, P_BoP = uav.get_fc_available_power(h)

    # 2. Maximum combined electrical power delivered to motor inverter
    # Both FC and continuous battery assist feed the 700V bus
    P_bus_available = P_FC_net_avail + P_bat_max - P_avionics_bus
    P_EM_elec = min(P_bus_available, ms.power_inverter)

    # Actual continuous battery discharge load
    P_bat_discharge = (P_EM_elec + P_avionics_bus) - P_FC_net_avail

    # Degree of Hybridization (Battery share of traction bus draw)
    DoH = P_bat_discharge / (P_EM_elec + P_avionics_bus)

    # 3. Shaft & thrust power available at propeller
    P_prop_shaft = P_EM_elec * uav.eta_inverter * uav.eta_motor  # kW
    P_avail = P_prop_shaft * eta_prop                            # kW

    # 4. Aerodynamics & Rate of Climb
    Cl = (m * 9.81) / (0.5 * rho * V**2 * uav.S)
    Cd = uav.Cd_0 + uav.K * Cl**2
    D = 0.5 * rho * V**2 * uav.S * Cd
    P_req = D * V  # Watts

    R_C = (P_avail * 1000.0 - P_req) / (m * 9.81)  # m/s

    # Guard against negative climb rate if thrust power falls below required
    if R_C <= 0.0:
        print("\nError: Rate of climb collapsed to <= 0 m/s. Ceiling reached.")
        Phase_Success = False
        break

    theta = math.degrees(math.asin(R_C / V))

    # State integration
    h += R_C * t_step
    t += t_step
    s_hor += V * math.cos(math.radians(theta)) * t_step

    # 5. Hydrogen mass consumption (LHV = 120 MJ/kg, eta_FC = 0.55)
    m_H2_consumed = ((P_FC_gross_avail * 1000.0) / (uav.eta_FC_rated * uav.LHV_H2)) * t_step  # kg
    H2_mass -= m_H2_consumed
    m -= m_H2_consumed

    # 6. Battery buffer depletion
    elec_energy_used = ((P_bat_discharge / uav.eta_discharge) * t_step) / 3600.0  # kWh
    SoC -= (elec_energy_used / ms.battery_capacity)

    # --- TELEMETRY LOGGING (Climb) ---
    fuel_remaining = H2_mass - ms.mass_reserve_H2
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Climb', h, V, P_req / 1000.0,
        P_FC_gross_avail, P_FC_net_avail, P_BoP, P_prop_shaft, P_bat_discharge,
        H2_mass, SoC
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
    ------- Climb (to {h_target:.0f}m) {status_str} -------
    EOP P_FC_net: {P_FC_net_avail:.2f} kW, P_Battery_cont: {P_bat_discharge:.2f} kW
    EOP Shaft Power: {P_prop_shaft:.2f} kW, Thrust Power: {P_avail:.2f} kW
    DoH (Battery fraction): {DoH*100:.1f} %
    Rate of Climb (Final): {R_C:.2f} m/s
    Total climb time: {t/60:.2f} mins
    Horizontal Distance Travelled: {s_hor/1000:.2f} km
    Final Altitude: {h:.2f} m
    Airspeed: {V:.2f} m/s
    H2 reactant burned: {(m_initial - m)*1000:.2f} g
    Remaining usable H2 mass: {H2_mass - ms.mass_reserve_H2:.2f} kg
    Remaining battery SoC: {SoC*100:.2f} %
''')