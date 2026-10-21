import math
from ambiance import Atmosphere
import jatayu_fc_uav as uav
import mission_settings_fc as ms
import Phases.flare as fl

# ------------------------------------------------------------------------------
# 1. INITIAL CONDITIONS & GROUND ROLL SETUP
# ------------------------------------------------------------------------------
m_initial = fl.m
m = fl.m
SoC = fl.SoC
H2_mass = fl.H2_mass
V = fl.V
h = fl.h

atmos = Atmosphere(h)
rho = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)
rho0 = 1.2250  # kg/m^3
sigma = rho / rho0

t_step = 0.1  # seconds

# Commanded gross idle setpoint (6% of gross rated capacity)
percent_idle_power = 6.0
P_FC_gross_idle = (percent_idle_power / 100.0) * uav.P_FC_gross_rated  # 3.60 kW

# Sea-level / runway BoP parasitics and usable net bus power
f_comp_alt = uav.f_parasitic_compressor * (1.0 / sigma) ** 0.60
f_BoP_alt = f_comp_alt + uav.f_parasitic_coolant
P_BoP_idle = P_FC_gross_idle * f_BoP_alt
P_FC_net_idle = (P_FC_gross_idle - P_BoP_idle) * uav.eta_DCDC_boost

# Hotel avionics bus draw
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck

# Residual idle power overcoming motor/prop rotational drag
P_EM_elec_idle = max(0.0, P_FC_net_idle - P_avionics_bus)
P_shaft_prop_idle = P_EM_elec_idle * (uav.eta_inverter * uav.eta_motor)
P_bat_discharge = 0.0

Cd_0_land = fl.Cd0_land
delta_Cl_flaps = 0.40  # or uav.delta_Cl_flaps
Cl = uav.Cl_ground + delta_Cl_flaps
Cd = Cd_0_land + ms.K_grd * (Cl ** 2)
mu_r = 0.40  # Braking coefficient of rolling friction

# Free-roll delay after touchdown before wheel brakes engage
N_freeroll = 1.0  # seconds
s_land_roll = N_freeroll * V

# H2 consumed during free roll
m_H2_freeroll = ((P_FC_gross_idle * 1000.0) / (uav.eta_FC_rated * uav.LHV_H2)) * N_freeroll
H2_mass -= m_H2_freeroll
m -= m_H2_freeroll

t = N_freeroll
Phase_Success = True
t_baseline = fl.t_baseline + fl.t_app + fl.t_flare

# Maximum Landing Weight verification
if m > uav.max_landing_mass:
    print(
        f"Structural Limit Exceeded: Landing weight {m:.2f} kg exceeds Maximum Landing Weight ({uav.max_landing_mass} kg)"
    )
    Phase_Success = False

# ==============================================================================
# 2. TIME-MARCHING GROUND BRAKING ROLLOUT
# ==============================================================================
while V > 0.0 and Phase_Success:
    D = 0.5 * rho * (V ** 2) * uav.S * Cd
    L = 0.5 * rho * (V ** 2) * uav.S * Cl
    net_F = -(D + mu_r * (m * 9.81 - L))  # Total retarding force (N)
    a_net = net_F / m

    # Check for complete stop within the time step
    if V + a_net * t_step <= 0.0:
        dt_stop = -V / a_net  # Exact remaining seconds to full stop
        s_land_roll += 0.5 * (V ** 2) / abs(a_net)
        t += dt_stop

        m_H2_stop = ((P_FC_gross_idle * 1000.0) / (uav.eta_FC_rated * uav.LHV_H2)) * dt_stop
        H2_mass -= m_H2_stop
        m -= m_H2_stop
        V = 0.0

        # --- TELEMETRY LOGGING (Full Stop Datum) ---
        fuel_remaining = H2_mass - ms.mass_reserve_H2
        cumulative_time = t_baseline + t
        ms.log_telemetry(
            cumulative_time, 'Mission_Complete', h, 0.0,
            0.0, P_FC_gross_idle, P_FC_net_idle, P_BoP_idle,
            0.0, 0.0, H2_mass, SoC
        )
        break

    s_land_roll += V * t_step
    V += a_net * t_step
    t += t_step

    # Regular step reactant consumption
    m_H2_consumed = ((P_FC_gross_idle * 1000.0) / (uav.eta_FC_rated * uav.LHV_H2)) * t_step
    H2_mass -= m_H2_consumed
    m -= m_H2_consumed

    # Telemetry logging (12 arguments)
    P_req = (D * V) / 1000.0  # kW
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Landing_Rollout', h, V,
        P_req, P_FC_gross_idle, P_FC_net_idle, P_BoP_idle,
        P_shaft_prop_idle, P_bat_discharge, H2_mass, SoC
    )

    if H2_mass < ms.mass_reserve_H2:
        print('''
        Error: Emergency H2 reserve breached during landing rollout!
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if s_land_roll > ms.max_permissible_sroll:
        print("Touchdown failed: maximum permissible ground roll exceeded.")
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 3. RESULTS REPORTING
# ------------------------------------------------------------------------------
m_H2_burned_total = m_initial - m
status_str = "Successful" if Phase_Success else "Failed"

print(f'''
    ------- Touchdown {status_str} -------
    FC Gross Idle Output: {P_FC_gross_idle:.2f} kW
    Total Ground Roll Distance: {s_land_roll:.2f} m (Limit: {ms.max_permissible_sroll:.1f} m)
    Time Taken to Stop: {t:.2f} sec
    Touchdown Airspeed: {fl.V:.2f} m/s -> Final: {V:.2f} m/s
    H2 Burned During Rollout: {m_H2_burned_total * 1000.0:.2f} g
    Untouched Safety Reserve: {ms.mass_reserve_H2:.2f} kg
    Final Usable H2 Remaining: {H2_mass - ms.mass_reserve_H2:.2f} kg
    Final Buffer Battery SoC: {SoC * 100.0:.2f} %
    Final Total Aircraft Mass: {m:.2f} kg
    Total Elapsed Mission Time: {(t_baseline + t) / 3600.0:.2f} hours ({((t_baseline + t) / 60.0):.1f} min)
''')