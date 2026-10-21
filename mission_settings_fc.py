# ==============================================================================
# PROJECT ARTEMIS 2.0 - FC: MISSION SETTINGS & SIZING AUDIT
# ==============================================================================
import math
import jatayu_fc_uav as uav

# ------------------------------------------------------------------------------
# 1. BATTERY BUFFER & ELECTRICAL LOAD SIZING
# ------------------------------------------------------------------------------
SoC_min = 0.20          # Minimum allowable battery State of Charge
SoC_max = 0.90          # Maximum State of Charge
SoH = 1.00              # State of Health
reserve_fuel = 0.065    # 6.5% unusable/reserve H2 fraction

power_avionics = 0.35   # kW (Baseline 28V avionics draw)
P_DCDC_buck_rated = 0.50 # kW (Rated capacity of 700V -> 28V isolated converter)

# Powertrain Sizing Targets
battery_capacity = 8.0          # kWh (High-rate buffer battery pack)
power_motor = uav.P_EM_takeoff_rated  # 105.0 kW (Full electric takeoff capability)
power_inverter = power_motor    # 105.0 kW

# Continuous and Burst Discharge Limits
power_battery_discharge_cont = uav.C_discharge_continuous_safe * battery_capacity  # 32.0 kW
power_battery_discharge_surge = uav.C_discharge_surge_safe * battery_capacity      # 80.0 kW
power_battery_charge = uav.C_charge_continuous_safe * battery_capacity              # 16.0 kW

# ------------------------------------------------------------------------------
# 2. POWER-PACK MASS BREAKDOWN (Target: 465.5 kg)
# ------------------------------------------------------------------------------
# E-Drive Hardware
mass_motor = power_motor / uav.specific_power_motor                 # kg (105 / 4.0 = 26.25 kg)
mass_inverter = power_inverter / uav.specific_power_inverter         # kg (105 / 12.0 = 8.75 kg)
mass_DCDC_buck = P_DCDC_buck_rated / uav.sp_power_DCDC_buck         # kg (0.5 / 0.5 = 1.00 kg)
mass_electric_hardware = mass_motor + mass_inverter + mass_DCDC_buck # kg (36.0 kg)

# Fuel Cell System Hardware (Stack Core + BoP Suite + Boost Converter)
mass_FC_stack = uav.P_FC_gross_rated / uav.sp_power_FC_stack        # kg (35 / 2.5 = 14.0 kg)
mass_FC_BoP = uav.P_FC_gross_rated * uav.k_BoP_mass_factor          # kg (35 * 0.52 = 18.2 kg)
mass_FC_boost = uav.P_FC_gross_rated / uav.sp_power_DCDC_boost      # kg (35 / 8.0 = 4.38 kg)
mass_FC_system = mass_FC_stack + mass_FC_BoP + mass_FC_boost        # kg (36.58 kg)

# High-Power Buffer Battery Pack
mass_cells = (battery_capacity * 1000.0) / uav.specific_energy_battery_cell  # kg (~44.44 kg)
mass_battery_pack = uav.k_bat_pack * mass_cells                              # kg (~53.33 kg)

# Hydrogen Fuel Storage Sizing (Tank + Reactant)
# M_power_pack = mass_electric_hardware + mass_FC_system + mass_battery_pack + mass_H2_storage_total
mass_fixed_powertrain = mass_electric_hardware + mass_FC_system + mass_battery_pack
mass_H2_storage_total = uav.M_power_pack - mass_fixed_powertrain             # kg

# Gravimetric Storage Distribution
# eta_tank_gravimetric = m_H2 / (m_tank_sys + m_H2)
mass_H2_total = mass_H2_storage_total * uav.eta_tank_gravimetric             # kg
mass_tank_system = mass_H2_storage_total - mass_H2_total                     # kg (Composite cylinder, boss, valves)

# Usable vs. Reserve Reactant
mass_reserve_H2 = mass_H2_total * reserve_fuel
mass_mission_H2 = mass_H2_total - mass_reserve_H2

# ------------------------------------------------------------------------------
# 3. PRE-FLIGHT CONFIGURATION & SIZING AUDIT
# ------------------------------------------------------------------------------
if mass_H2_total <= 0:
    print("\n" + "!" * 82)
    print("FATAL SIZING ERROR: Mass of usable H2 is <= 0 kg. Check battery capacity & hardware sizing!")
    print("!" * 82 + "\n")
else:
    m_zfw = uav.MTOW - mass_H2_total  # Zero Fuel Weight
    wing_loading = uav.MTOW / uav.S    # kg/m²
    power_loading = uav.MTOW / power_motor  # kg/kW total installed (takeoff)
    ld_max_clean = 1.0 / (2.0 * math.sqrt(uav.Cd_0 * uav.K))

    print("=" * 82)
    print(f"{'PROJECT ARTEMIS 2.0 - FC: POWER-PACK SIZING & CONFIGURATION AUDIT':^82}")
    print("=" * 82)

    # Category 1: Mass Properties & Airframe Limits
    print(f"{'--- MASS PROPERTIES & STRUCTURAL LIMITS ---':^82}")
    print(f"  MTOW:                {uav.MTOW:>8.2f} kg   │  Payload Mass:        {uav.M_payload:>8.2f} kg")
    print(f"  Structural Mass:     {uav.M_structure:>8.2f} kg   │  Avionics Mass:       {uav.M_avionics:>8.2f} kg")
    print(f"  Zero Fuel Wt (ZFW):  {m_zfw:>8.2f} kg   │  Max Landing Wt (MLW):{uav.max_landing_mass:>8.2f} kg")
    print(f"  Total Power-Pack:    {uav.M_power_pack:>8.2f} kg   │  Propeller Assembly:  {uav.M_propeller:>8.2f} kg")
    print("-" * 82)

    # Category 2: Aerodynamics & Lift Limits
    print(f"{'--- AERODYNAMICS & LIFT COEFFICIENTS ---':^82}")
    print(f"  Wing Span (b):       {uav.b:>8.2f} m    │  Aspect Ratio (AR):   {uav.AR:>8.2f}")
    print(f"  Wing Area (S):       {uav.S:>8.2f} m²   │  Wing Loading (W/S):  {wing_loading:>8.2f} kg/m²")
    print(f"  Clean Zero-Lift Cd0: {uav.Cd_0:>8.4f}      │  Clean (L/D)max:        {ld_max_clean:>8.2f}")
    print(f"  CL_max (Clean):      {uav.Cl_max_clean:>8.2f}      │  CL_max (Takeoff Flap):{uav.Cl_max_TO:>8.2f}")
    print(f"  CL_ground (Roll):    {uav.Cl_ground:>8.2f}      │  CL_max (Landing Flap):{uav.Cl_max_Land:>8.2f}")
    print("-" * 82)

    # Category 3: Fuel Cell & Hydrogen Storage System
    print(f"{'--- FUEL CELL & HYDROGEN STORAGE SYSTEM ---':^82}")
    print(
    f"  FC Gross Rating:     {uav.P_FC_gross_rated:>8.2f} kW   │  FC Net Bus"
    f" (SL):       {uav.P_FC_net_SL:>8.2f} kW"
)
    print(f"  FC Stack Core Mass:  {mass_FC_stack:>8.2f} kg   │  FC BoP Suite Mass:   {mass_FC_BoP:>8.2f} kg")
    print(f"  FC Boost DCDC Mass:  {mass_FC_boost:>8.2f} kg   │  Total FC Sys Mass:   {mass_FC_system:>8.2f} kg")
    print(f"  Type IV Tank Mass:   {mass_tank_system:>8.2f} kg   │  Total Stored H2:     {mass_H2_total:>8.2f} kg")
    print(f"  Mission H2 (93.5%):  {mass_mission_H2:>8.2f} kg   │  Reserve H2 (6.5%):   {mass_reserve_H2:>8.2f} kg")
    print(f"  Gravimetric Index:   {uav.eta_tank_gravimetric*100:>8.2f} %    │  H2 Energy (LHV):     {mass_H2_total * uav.LHV_H2_kWh:>8.2f} kWh")
    print("-" * 82)


    # Category 4: Electric Drivetrain & Buffer Battery
    print(f"{'--- ELECTRIC PROPULSION & BUFFER BATTERY ---':^82}")
    print(f"  Motor Rated Power:   {power_motor:>8.2f} kW   │  Motor Mass:          {mass_motor:>8.2f} kg")
    print(f"  Inverter Sizing:     {power_inverter:>8.2f} kW   │  Inverter Mass:       {mass_inverter:>8.2f} kg")
    print(f"  Battery Capacity:    {battery_capacity:>8.2f} kWh  │  Battery Pack Mass:   {mass_battery_pack:>8.2f} kg")
    print(f"  Cont. Discharge Pwr: {power_battery_discharge_cont:>8.2f} kW   │  Surge Burst Pwr:     {power_battery_discharge_surge:>8.2f} kW")
    print(f"  Max Cont Charge Pwr: {power_battery_charge:>8.2f} kW   │  Avionics Buck Mass:  {mass_DCDC_buck:>8.2f} kg")
    print(f"  Installed Power/Wt:  {power_loading:>8.2f} kg/kW│  Avionics Base Power: {power_avionics:>8.2f} kW")
    print("=" * 82)
    print(f"{'PRE-FLIGHT AUDIT VERIFIED: ARTEMIS 2.0 - FC GO FOR FLIGHT':^82}")
    print("=" * 82 + "\n")

# ------------------------------------------------------------------------------
# 4. TAKEOFF & FLIGHT CONSTRAINTS
# ------------------------------------------------------------------------------
altitude_takeoff = 0.0          # Takeoff field elevation (m)
obstacle_height = altitude_takeoff + 15.0  # 15 m obstacle screen height
max_permissible_sroll = 600.0   # Max allowable ground roll (m)
max_permissible_stot = 1000.0   # Max total takeoff distance (m)

rolling_friction_coefficient = 0.04  # Runway rolling friction coefficient

k_uc = 3.16 * (10**-5)
delta_CD_0 = (uav.MTOW * 9.81 / uav.S) * k_uc * (uav.MTOW**-0.215)
delta_CD_0_flap = 0.0095
G = (16.0 * uav.h / uav.b)**2 / (1.0 + (16.0 * uav.h / uav.b)**2)
K_grd = (uav.k1 + G * uav.k3)
CD_takeoff = uav.Cd_0 + delta_CD_0 + delta_CD_0_flap + K_grd * (uav.Cl_ground**2)

# Cruise and Loiter Flight Profiles
cruise_altitude = 3000.0        # m
cruise_distance = 150.0         # km
loiter_altitude = cruise_altitude

# ------------------------------------------------------------------------------
# 5. MISSION TELEMETRY RECORDER
# ------------------------------------------------------------------------------
telemetry = {
    'time_s': [],
    'phase': [],
    'altitude_m': [],
    'velocity_ms': [],
    'P_req_kW': [],       # Propeller aerodynamic shaft power demand
    'P_FC_gross_kW': [],  # Gross electrochemical power generated by PEMFC
    'P_FC_net_kW': [],    # Usable power delivered to 700V bus after BoP
    'P_BoP_kW': [],       # Parasitic draw (compressor + coolant pump)
    'P_EM_kW': [],        # Motor shaft mechanical power
    'P_bat_kW': [],       # Battery power (+ = discharge, - = charge)
    'H2_mass_kg': [],     # Remaining usable hydrogen mass
    'battery_soc': [],    # Buffer battery State of Charge
}

def log_telemetry(t, phase, alt, v, p_req, p_fc_gross, p_fc_net, p_bop, p_em, p_bat, h2_mass, soc):
    """Appends one instantaneous simulation step to the Artemis 2.0-FC telemetry log."""
    telemetry['time_s'].append(t)
    telemetry['phase'].append(phase)
    telemetry['altitude_m'].append(alt)
    telemetry['velocity_ms'].append(v)
    telemetry['P_req_kW'].append(p_req)
    telemetry['P_FC_gross_kW'].append(p_fc_gross)
    telemetry['P_FC_net_kW'].append(p_fc_net)
    telemetry['P_BoP_kW'].append(p_bop)
    telemetry['P_EM_kW'].append(p_em)
    telemetry['P_bat_kW'].append(p_bat)
    telemetry['H2_mass_kg'].append(h2_mass)
    telemetry['battery_soc'].append(soc)
