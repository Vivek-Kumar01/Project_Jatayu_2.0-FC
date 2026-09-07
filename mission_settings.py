import math
import artemis_uav as uav

# UAV constraints and parameters
SoC_min = 0.2  # Minimum state of charge for battery pack
SoC_max = 0.9  # Maximum state of charge for battery pack
SoH = 1.0 # State of health for battery pack (1.0 = new, 0.8 = end of life)
reserve_fuel = 0.065 # Reserve fuel fraction for safety

battery_capacity = 22 #kWh
power_motor = 45 #kW
power_inverter = power_motor #kW
power_battery_discharge = uav.C_discharge_continuous_safe * battery_capacity #kW
power_battery_charge = uav.C_charge_continuous_safe * battery_capacity #kW

mass_battery = battery_capacity*1000 / uav.specific_energy_battery #kg
mass_motor = power_motor / uav.specific_power_motor #kg
mass_inverter = power_inverter / uav.specific_power_inverter #kg

mass_fuel = uav.M_power_pack - (mass_battery + mass_motor + mass_inverter) #kg

if mass_fuel > 0:
    print(f'''
    ================================================================
                            MISSION ARTEMIS
    ================================================================

    UAV Charged and Fueled.
    ''')
else:
    print("Error: Mass of fuel is negative. Please check the battery capacity and power requirements.")

# Take off constraints
altitude_takeoff = 0.0  # Takeoff altitude (m)
obstacle_height = 15.0  # 50 ft screen height (m)
max_permissible_sroll = 600.0  # Max allowable ground roll (m)
max_permissible_stot = 1000.0  # Max total distance over 15 m obstacle (m)

rolling_friction_coefficient = 0.04  # Rolling friction coefficient for takeoff ground roll

k_uc = 3.16 * (10**-5)
delta_CD_0 = (uav.MTOW / uav.S)*k_uc*(uav.MTOW**-0.215)
G = (16 * uav.h / uav.b)**2 / (1 + (16 * uav.h / uav.b)**2)
CD_takeoff = uav.Cd_0 + delta_CD_0 + (uav.k1 + G * uav.k3) * (uav.Cl_ground**2)

# Climb


# Cruise Out and In
cruise_altitude = 3000 #m
cruise_distance = 150 #km


# Loiter
loiter_altitude = cruise_altitude #m

# Descent



# Landing

