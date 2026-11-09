import math
from ambiance import Atmosphere
import numpy as np
import jatayu_fc_uav as uav
import mission_settings_fc as ms
import Phases.cruise_out as co

# ------------------------------------------------------------------------------
# 1. LOITER ENVIRONMENTAL & AERODYNAMIC CONSTANTS
# ------------------------------------------------------------------------------
t_step = 1.0  # seconds
rho = float(Atmosphere(ms.loiter_altitude).density[0]) if hasattr(Atmosphere(ms.loiter_altitude).density, '__len__') else float(Atmosphere(ms.loiter_altitude).density)

safe_margin = 1.25  # Ratio of loiter to stall velocity
# Standoff orbit bank angle (continuous 20 deg circle)
phi_orbit = math.radians(20.0)
n_load = 1.0 / math.cos(phi_orbit)  # n ~ 1.0642

# Minimum-power loiter speed condition: Cl = sqrt(3 * Cd0 / K)
# Clamped safely below clean stall margin (Cl_max_clean = 1.40)
cl_opt_endurance = math.sqrt(3.0 * uav.Cd_0 / uav.K)
cl_loiter = min(cl_opt_endurance, uav.Cl_max_clean / (safe_margin ** 2))

# Effective drag polar during turn
cd_loiter = uav.Cd_0 + uav.K * ((cl_loiter * n_load) ** 2)
n_prop = 0.66  # Propeller efficiency at loiter advance ratio

# State initializations from Cruise-Out
m_initial = co.m  # kg
m = m_initial
SoC = co.SoC
H2_mass = co.H2_mass
endurance = 0.0  # Total elapsed loiter time (s)

# Keep sufficient reactant to execute cruise-in back to base
fraction_keep = 1.25  # Fuel burn margin factor for return transit
min_H2_cruisein = ms.mass_reserve_H2 + (fraction_keep * co.m_H2_burned)

Phase_Success = True
t_baseline = co.t_baseline + co.time_cruise

# Avionics auxiliary draw on the high-voltage bus
P_avionics_bus = ms.power_avionics / uav.eta_DCDC_buck  # kW

# ------------------------------------------------------------------------------
# 2. CONTINUOUS ON-STATION LOITER SIMULATION (PURE FC)
# ------------------------------------------------------------------------------
print("--> Initiating Phase 5: Sustained On-Station Loiter on Fuel Cell...")

while H2_mass > min_H2_cruisein:
    # Loiter speed governed by Cl margin
    vel_loiter = math.sqrt((2.0 * m * 9.81) / (rho * uav.S * cl_loiter))  # m/s
    
    # Aerodynamic drag and propeller shaft power required
    D_loiter = 0.5 * rho * (vel_loiter ** 2) * uav.S * cd_loiter          # N
    P_prop_req = (D_loiter * vel_loiter) / 1000.0                         # kW (aerodynamic power)
    P_shaft_prop = P_prop_req / n_prop                                    # kW (mechanical shaft power)

    # Motor traction electrical demand
    P_EM_elec = P_shaft_prop / (uav.eta_inverter * uav.eta_motor)         # kW

    # Total high-voltage bus load (Propulsion + Avionics)
    P_bus_load = P_EM_elec + P_avionics_bus                               # kW

    # Check fuel cell capacity at loiter altitude
    P_FC_gross_avail, P_FC_net_avail, P_BoP = uav.get_fc_available_power(ms.loiter_altitude)

    if P_bus_load > P_FC_net_avail:
        print('''
        Error: Required loiter bus power exceeds available net Fuel Cell capacity.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    # Battery floats on standby (dead weight, 0 kW assist, 0 kW charge)
    P_bat_discharge = 0.0

    # Net-to-Gross transmission efficiency (accounts for BoP parasitics + Boost DC-DC)
    eta_net_bus = (1.0 - (P_BoP / P_FC_gross_avail)) * uav.eta_DCDC_boost
    P_FC_gross_demand = P_bus_load / eta_net_bus                          # kW
    P_BoP_demand = P_FC_gross_demand * (P_BoP / P_FC_gross_avail)         # kW

    # Dynamic throttle fraction and empirical stack efficiency
    xi_fc = float(np.clip(P_FC_gross_demand / P_FC_gross_avail, 0.05, 1.0)) if P_FC_gross_avail > 0.0 else 1.0
    eta_fc = uav.get_fc_efficiency(P_FC_gross_demand, P_FC_gross_avail)

    # Dynamic hydrogen reactant mass consumption (LHV = 120 MJ/kg)
    m_H2_consumed = ((P_FC_gross_demand * 1000.0) / (eta_fc * uav.LHV_H2)) * t_step  # kg
    H2_mass -= m_H2_consumed
    m -= m_H2_consumed

    endurance += t_step

    # Guardrail check against emergency reserve breach
    if H2_mass <= ms.mass_reserve_H2:
        print("Emergency H2 fuel reserve breached during loiter! RTB immediately.")
        Phase_Success = False
        break

    # Telemetry logging (14 arguments)
    cumulative_time = t_baseline + endurance
    ms.log_telemetry(
        cumulative_time, 'Loiter', ms.loiter_altitude, vel_loiter,
        P_prop_req, P_FC_gross_demand, P_bus_load, P_BoP_demand,
        P_shaft_prop, P_bat_discharge, H2_mass, SoC,
        xi_fc, eta_fc
    )

# ------------------------------------------------------------------------------
# 3. RESULTS REPORTING
# ------------------------------------------------------------------------------
m_H2_burned_loiter = m_initial - m
status_str = "SUCCESS" if Phase_Success else "FAILED"

print(f'''
------------------ Complete Loiter Phase Summary ------------------
Status: {status_str}
Total On-Station Loiter Endurance: {endurance / 3600.0:.2f} hours ({(endurance / 60.0):.1f} mins)
Loiter Airspeed: {vel_loiter:.2f} m/s
Propeller Shaft Power: {P_shaft_prop:.2f} kW
FC Gross Load: {P_FC_gross_demand:.2f} kW | Load Ratio (xi): {xi_fc:.2f} | Stack Eff: {eta_fc * 100.0:.2f} %
FC Net Bus Load: {P_bus_load:.2f} kW
Final Aircraft Mass: {m:.2f} kg
H2 reactant burned in loiter: {m_H2_burned_loiter:.2f} kg ({m_H2_burned_loiter * 1000.0:.1f} g)
Buffer Battery SoC: {SoC * 100.0:.2f} % (Floated)
Usable H2 Remaining for Cruise-In: {H2_mass - ms.mass_reserve_H2:.2f} kg
Total Mission Time Elapsed So Far: {(t_baseline + endurance) / 3600.0:.2f} hours
''')