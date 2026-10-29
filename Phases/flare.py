import math
from ambiance import Atmosphere
import numpy as np
import jatayu_fc_uav as uav
import mission_settings_fc as ms
import Phases.approach as ap

# ------------------------------------------------------------------------------
# 1. INITIAL CONDITIONS & FLARE GEOMETRY INITIALIZATION
# ------------------------------------------------------------------------------
m_initial = ap.m
m = ap.m
SoC = ap.SoC
H2_mass = ap.H2_mass

t_step = 0.1  # seconds
rho0 = 1.2250  # ISA Sea Level density (kg/m^3)

# Hotel avionics bus load
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck  # kW

theta = ap.theta
n_flare = 1.05  # Load factor during flare rotation (5% incremental g)
V_flare = 1.23 * ap.V_stall_land  # Average speed between approach (1.30 Vs) and touchdown (1.15 Vs)
R_f = (V_flare ** 2) / ((n_flare - 1.0) * 9.81)  # Flare radius of curvature (m)

h_flare = R_f * (1.0 - math.cos(theta))  # Height AGL where flare rotation begins (m)

h = ap.h
V = ap.V
Cd0_land = ap.Cd0_land

ROS = V * math.sin(theta)  # m/s glideslope sink rate
eta_prop = 0.60            # Propeller efficiency during low-speed high-thrust approach

s_hor_app = 0.0
t_app = 0.0
Phase_Success = True
t_baseline = ap.t_baseline + ap.t

xi_fc_app = 0.0
eta_fc_app = 0.50

# ==============================================================================
# PART 1: POWERED FINAL APPROACH GLIDESLOPE (Down to Flare Initiation Height)
# ==============================================================================
print(f"--> Initiating Phase 8A: Final Approach Glideslope to flare height ({h_flare:.2f} m AGL)...")

while h > (ms.altitude_takeoff + h_flare) and Phase_Success:
    atmos = Atmosphere(h)
    rho = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)
    sigma = rho / rho0

    # Aerodynamics on glideslope
    Cl = (m * 9.81) / (0.5 * rho * (V ** 2) * uav.S)
    Cd = Cd0_land + uav.K * (Cl ** 2)
    D = 0.5 * rho * (V ** 2) * uav.S * Cd  # Newtons

    # Thrust required balancing drag minus along-path gravity component
    T_req = D - (m * 9.81 * math.sin(theta))
    P_avail = max(0.0, T_req * V) / 1000.0   # kW (aerodynamic power)
    P_shaft_prop = P_avail / eta_prop        # kW (mechanical shaft power)

    # Electrical motor demand from inverter
    P_EM_elec = P_shaft_prop / (uav.eta_inverter * uav.eta_motor)  # kW
    P_bus_load = P_EM_elec + P_avionics_bus                        # kW

    # Fuel Cell available net capacity at current approach altitude
    P_FC_gross_avail, P_FC_net_avail, P_BoP = uav.get_fc_available_power(h)

    percent_power = P_bus_load / P_FC_net_avail
    if percent_power > 1.0:
        print(f'''
        MISSION FAILED: Final approach power exceeds available FC net capacity!
        Percent Power: {percent_power * 100:.2f} %
        ''')
        Phase_Success = False
        break

    # Battery floats on standby (0 kW assist)
    P_bat_discharge = 0.0

    # Fuel cell gross demand & BoP parasitics
    eta_net_bus = (1.0 - (P_BoP / P_FC_gross_avail)) * uav.eta_DCDC_boost
    P_FC_gross_demand = P_bus_load / eta_net_bus
    P_BoP_demand = P_FC_gross_demand * (P_BoP / P_FC_gross_avail)

    # Dynamic throttle fraction and empirical stack efficiency
    xi_fc_app = float(np.clip(P_FC_gross_demand / P_FC_gross_avail, 0.05, 1.0)) if P_FC_gross_avail > 0.0 else 1.0
    eta_fc_app = uav.get_fc_efficiency(P_FC_gross_demand, P_FC_gross_avail)

    # Dynamic hydrogen reactant mass consumption
    m_H2_consumed = ((P_FC_gross_demand * 1000.0) / (eta_fc_app * uav.LHV_H2)) * t_step
    H2_mass -= m_H2_consumed
    m -= m_H2_consumed

    # Kinematics integration
    h -= ROS * t_step
    s_hor_app += V * math.cos(theta) * t_step
    t_app += t_step

    # Telemetry logging (14 arguments)
    P_req = (D * V) / 1000.0
    cumulative_time = t_baseline + t_app
    ms.log_telemetry(
        cumulative_time, 'Final_Approach', h, V,
        P_req, P_FC_gross_demand, P_bus_load, P_BoP_demand,
        P_shaft_prop, P_bat_discharge, H2_mass, SoC,
        xi_fc_app, eta_fc_app
    )

    if H2_mass < ms.mass_reserve_H2:
        print("Error: Emergency H2 fuel reserve breached during final approach!")
        Phase_Success = False
        break

# ==============================================================================
# PART 2: FLARE ROTATION TO TOUCHDOWN (FLIGHT IDLE)
# ==============================================================================
print("--> Initiating Phase 8B: Circular Flare Arc to Touchdown (Flight Idle)...")

# Commanded gross idle setpoint (6% of gross rated capacity)
percent_idle_power = 6.0
P_FC_gross_idle = (percent_idle_power / 100.0) * uav.P_FC_gross_rated  # 3.60 kW

# Kinematic flare arc parameters
s_hor_flare = R_f * math.sin(theta)  # Ground distance traversed during flare (m)
dist_flare = R_f * theta             # Arc length traversed (m)
t_flare = dist_flare / V_flare       # Flare duration (s)

# Sea-level / runway atmospheric conditions for flare evaluation
h = ms.altitude_takeoff
atmos_flare = Atmosphere(h)
rho_flare = float(atmos_flare.density[0]) if hasattr(atmos_flare.density, '__len__') else float(atmos_flare.density)
sigma_flare = rho_flare / rho0

# Dynamic fuel cell available capacity at runway elevation
P_FC_gross_avail_flare, _, _ = uav.get_fc_available_power(h)

# Dynamic throttle fraction and empirical stack efficiency during flare idle
xi_fc_flare = float(np.clip(P_FC_gross_idle / P_FC_gross_avail_flare, 0.05, 1.0)) if P_FC_gross_avail_flare > 0.0 else 0.05
eta_fc_flare = uav.get_fc_efficiency(P_FC_gross_idle, P_FC_gross_avail_flare)

# BoP parasitics and net available bus power at idle
f_comp_alt = uav.f_parasitic_compressor * (1.0 / sigma_flare) ** 0.60
f_BoP_alt = f_comp_alt + uav.f_parasitic_coolant
P_BoP_idle = P_FC_gross_idle * f_BoP_alt
P_FC_net_idle = (P_FC_gross_idle - P_BoP_idle) * uav.eta_DCDC_boost

# Avionics priority on bus; residual power turns prop at zero-thrust idle
P_EM_elec_idle = max(0.0, P_FC_net_idle - P_avionics_bus)
P_shaft_prop_idle = P_EM_elec_idle * (uav.eta_inverter * uav.eta_motor)
P_bat_discharge = 0.0

# Dynamic reactant consumption across flare arc duration
m_H2_consumed_flare = ((P_FC_gross_idle * 1000.0) / (eta_fc_flare * uav.LHV_H2)) * t_flare
H2_mass -= m_H2_consumed_flare
m -= m_H2_consumed_flare

# Touchdown velocity (1.15 Vs)
V = 1.15 * ap.V_stall_land

# --- TELEMETRY LOGGING (Touchdown Instant at Runway Datum) ---
Cl_flare = (m * 9.81) / (0.5 * rho_flare * (V ** 2) * uav.S)
Cd_flare = Cd0_land + uav.K * (Cl_flare ** 2)
D_flare = 0.5 * rho_flare * (V ** 2) * uav.S * Cd_flare
P_req_flare = (D_flare * V) / 1000.0  # kW

cumulative_time = t_baseline + t_app + t_flare
ms.log_telemetry(
    cumulative_time, 'Flare_Touchdown', h, V,
    P_req_flare, P_FC_gross_idle, P_FC_net_idle, P_BoP_idle,
    P_shaft_prop_idle, P_bat_discharge, H2_mass, SoC,
    xi_fc_flare, eta_fc_flare
)

if H2_mass < ms.mass_reserve_H2:
    print("Error: Emergency H2 fuel reserve breached in flare!")
    Phase_Success = False

# ------------------------------------------------------------------------------
# 3. RESULTS REPORTING
# ------------------------------------------------------------------------------
m_H2_burned_total = m_initial - m
status_str = "Successful" if Phase_Success else "Failed"

print(f'''
    ------- Final Approach & Flare {status_str} -------
    Approach Distance: {s_hor_app:.2f} m
    Approach Time: {t_app:.2f} sec
    Flare Height from runway: {h_flare:.2f} m
    Flare Distance: {s_hor_flare:.2f} m
    Flare Time: {t_flare:.2f} sec
    Touchdown Airspeed: {V:.2f} m/s
    Final Aircraft Mass: {m:.2f} kg
    Flare Idle Load Ratio (xi): {xi_fc_flare:.2f} | Flare Stack Eff: {eta_fc_flare * 100.0:.2f} %
    H2 reactant burned (Final Approach + Flare): {m_H2_burned_total * 1000.0:.2f} g
    Remaining Usable H2 (excluding safety reserve): {H2_mass - ms.mass_reserve_H2:.2f} kg
    Untouched Safety Reserve: {ms.mass_reserve_H2:.2f} kg
    Final Buffer Battery SoC: {SoC * 100.0:.2f} %
    Total Mission Duration: {cumulative_time / 3600.0:.2f} hours ({(cumulative_time / 60.0):.1f} min)
''')