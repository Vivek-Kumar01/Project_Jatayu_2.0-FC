# ==============================================================================
# PROJECT JATAYU 2.0 - FC (FUEL CELL POWERTRAIN CONFIGURATION)
# ==============================================================================
import math
import numpy as np
from ambiance import Atmosphere

# ------------------------------------------------------------------------------
# 1. FIXED MASS PROPERTIES (Airframe & Payload)
# ------------------------------------------------------------------------------
MTOW = 1000.0          # kg (Maximum Takeoff Weight)
M_structure = 275.0     # kg (Airframe structure)
M_propeller = 14.5      # kg (Constant-speed propeller assembly)
M_payload = 200.0        # kg (Mission sensor / surveillance payload)
M_avionics = 45.0       # kg (Flight control computers, telemetry, wiring)

# Mass budget reserved exclusively for the complete Fuel Cell Electric Powertrain
# MTOW - (275.0 + 14.5 + 200.0 + 45.0) = 465.5 kg
M_power_pack = MTOW - (M_structure + M_propeller + M_payload + M_avionics)  # 465.5 kg

# ------------------------------------------------------------------------------
# 2. AERODYNAMIC & GEOMETRIC CONSTANTS
# ------------------------------------------------------------------------------
b = 18.0                # m (Wing span)
h = 1.6                 # m (Ground clearance height)
S = 22.0                # m^2 (Wing reference area)
lamb = 0.45             # Wing taper ratio
k1 = 0.009              # Parasitic drag vs CL^2 proportionality
AR = b**2 / S           # Aspect Ratio (~14.73)
delta = 0.02            # Induced drag factor
e = 1.0 / (1.0 + delta) # Oswald span efficiency factor
k3 = 1.0 / (math.pi * AR * e)
K = k1 + k3             # Total induced drag polar coefficient
Cd_0_clean = 0.022            # Zero-lift parasitic drag coefficient


Cl_ground = 0.35        # Ground roll lift coefficient
Cl_max_ground_TO = 1.25
Cl_max_TO = 1.70        # Takeoff flap CL_max
Cl_max_Land = 1.85      # Landing flap CL_max
Cl_max_clean = 1.40     # Clean wing CL_max

D_prop = 1.75           # m (Propeller diameter)
N_prop = 2400.0         # RPM (Constant speed propeller)

# ------------------------------------------------------------------------------
# 3. TRACTION ELECTRIC DRIVE SIZING METRICS
# ------------------------------------------------------------------------------
P_EM_takeoff_rated = 105.0  # kW (Full takeoff shaft power requirement)
specific_power_motor = 4.0   # kW/kg (Continuous PMSM motor)
specific_power_inverter = 12.0 # kW/kg (SiC high-frequency motor controller)

# ------------------------------------------------------------------------------
# 4. PEM FUEL CELL & BALANCE OF PLANT (BoP) SIZING METRICS
# ------------------------------------------------------------------------------
P_FC_gross_rated = 60.0     # kW (Gross electrochemical stack rating - sweep variable)


# Belly radiator cooling drag penalty
P_FC_gross_ref = 35.0          # kW (Reference stack size for aerodynamic calibration)
delta_Cd_0_cooling_ref = 0.0020 # Cooling duct, core pressure drop, and diverter penalty at 35 kW

# Dynamically scaled parasitic drag (scales core area with stack heat rejection)
delta_Cd_0_cooling = delta_Cd_0_cooling_ref * (P_FC_gross_rated / P_FC_gross_ref)
Cd_0 = Cd_0_clean + delta_Cd_0_cooling  # ~0.0240 total parasitic drag

# BoP Parasitic Loads (Fraction of Gross FC Output)
f_parasitic_compressor = 0.100  # 10.0% gross power to cathode compressor
f_parasitic_coolant = 0.015     # 1.5% to DI coolant pump, valves, and ECU
f_BoP_total = f_parasitic_compressor + f_parasitic_coolant  # 11.5% total BoP overhead

# Electrical converter efficiency
eta_DCDC_boost = 0.94           # High-voltage SiC boost converter efficiency

# Specific power densities
sp_power_FC_stack = 2.5     # kW/kg (Aviation metallic bipolar plate PEMFC bare stack)
k_BoP_mass_factor = 0.52    # kg/kW_gross (Air compressor, radiator, DI pump, ejector, humidifier)
sp_power_DCDC_boost = 8.0   # kW/kg (High-voltage unidirectional SiC boost converter)
sp_power_DCDC_buck = 0.5  # kW/kg (Low-voltage SiC buck converter)

# Electrochemical Constants
LHV_H2 = 120.0e6            # J/kg (Lower Heating Value of Hydrogen: 120 MJ/kg)
LHV_H2_kWh = LHV_H2 / 3.6e6 # 33.333 kWh/kg

# Stack Efficiency Characteristics (vs Gross Output)
eta_FC_peak = 0.60          # Peak stack electrical efficiency (part-load loiter)
eta_FC_rated = 0.49         # Stack electrical efficiency at 100% rated power

# ------------------------------------------------------------------------------
# 5. ENERGY STORAGE & TANK GRAVIMETRICS
# ------------------------------------------------------------------------------
# High-Power Buffer Battery (Peak burst power for takeoff/go-around)
specific_energy_battery_cell = 180.0  # Wh/kg (High-rate power cells)
k_bat_pack = 1.20                    # 20% overhead for module casing, cold plates, BMS, HV contactors

C_discharge_continuous_safe = 4.0    # 1/h (Continuous cruise/climb boost)
C_discharge_surge_safe = 10.0        # 1/h (Takeoff burst limit, <= 60 s)
C_charge_continuous_safe = 2.0       # 1/h (Recharging from excess FC cruise power)

# 700-bar Type IV Hydrogen Storage (Composite shell, liner, boss, and primary regulator)
# eta_tank_gravimetric = m_H2 / (m_tank_sys + m_H2)
eta_tank_gravimetric = 0.055         # 5.5% gravimetric efficiency
k_tank_sys = (1.0 - eta_tank_gravimetric) / eta_tank_gravimetric  # ~17.18 kg of tank structure per kg of H2


# ------------------------------------------------------------------------------
# 6. ELECTRICAL EFFICIENCY CONSTANTS
# ------------------------------------------------------------------------------
eta_motor = 0.95            # PMSM motor mechanical efficiency
eta_inverter = 0.97         # Traction inverter electrical efficiency
eta_DCDC_buck = 0.92        # Avionics buck converter efficiency
eta_discharge = 0.94        # Battery discharge internal efficiency
eta_charging = 0.95         # Battery charging internal efficiency

# ------------------------------------------------------------------------------
# 7. OPERATIONAL LIMITS
# ------------------------------------------------------------------------------
max_landing_mass_percent = 100.0  # % of MTOW
max_landing_mass = (max_landing_mass_percent / 100.0) * MTOW



def get_fc_available_power(altitude_m, P_gross_rated=P_FC_gross_rated):
  """Computes altitude-derated fuel cell gross capacity, BoP parasitics,

  and available net bus power using the 1976 Standard Atmosphere (ambience).
  """
  # Evaluate ISA conditions
  atmos = Atmosphere(altitude_m)
  rho_alt = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)
  rho0 = 1.2250  # ISA Sea Level density (kg/m^3)

  sigma = rho_alt / rho0

  # 1. Gross electrochemical stack capability derating
  P_gross_avail = P_gross_rated * (sigma**0.80)

  # 2. Altitude-scaled compressor parasitic load fraction
  f_comp_alt = f_parasitic_compressor * (1.0 / sigma) ** 0.60
  f_BoP_alt = f_comp_alt + f_parasitic_coolant

  # 3. Available Net Power to 700V Bus after BoP and Boost DCDC
  P_BoP = P_gross_avail * f_BoP_alt
  P_net_avail = (P_gross_avail - P_BoP) * eta_DCDC_boost

  return P_gross_avail, P_net_avail, P_BoP


def get_fc_efficiency(P_gross, P_gross_avail):
    """
    Computes load-dependent PEMFC stack efficiency (LHV basis).
    Normalized against local altitude-derated capacity P_gross_avail(h).
    Captures:
      - Idle / activation losses at low throttle (xi < 0.15)
      - Peak thermodynamic sweet spot (~60% efficiency) around 25-35% part load
      - Ohmic and concentration overpotentials (~49% efficiency) at max throttle (xi = 1.0)
    """
    if P_gross_avail <= 0.0 or P_gross <= 0.0:
        return 0.50
    
    # Normalized throttle fraction relative to local available capacity
    xi = float(np.clip(P_gross / P_gross_avail, 0.05, 1.0))
    
    # Empirical 2nd-order polarization efficiency curve
    eta_fc = -0.22 * (xi ** 2) + 0.13 * xi + 0.58
    return float(eta_fc)


# Derive baseline Sea-Level reference values directly from the dynamic function
(
    P_FC_gross_SL,
    P_FC_net_SL,
    P_FC_BoP_SL,
) = get_fc_available_power(0.0)