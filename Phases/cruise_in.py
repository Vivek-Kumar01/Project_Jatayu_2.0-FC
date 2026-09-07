import math
from ambiance import Atmosphere
import artemis_uav as uav
import mission_settings as ms
import Phases.loiter as lo


# Cruise In simulation setting
t_step = 1.0  # seconds
rho = Atmosphere(ms.cruise_altitude).density[0]  # kg/m^3

P_ICE_full = uav.P_ICE_rated * ((1.132 * rho / Atmosphere(0).density[0]) - 0.132)  # kW
P_electric_full = (min(ms.power_battery_discharge, ms.power_inverter/uav.eta_inverter))*uav.eta_inverter*uav.eta_motor  # kW

# percent_ICE_power = 0.5 # Percentage of ICE power
# percent_electric_power = 0.5 # Percentage of electric power

# P_ICE = percent_ICE_power * P_ICE_full  # kW
# P_electric = percent_electric_power * P_electric_full  # kW

# DoH = P_electric / (P_ICE + P_electric)  # Hybridization factor (0 = pure ICE, 1 = pure electric)
# P_shaft_prop = (P_ICE + P_electric) * uav.eta_gearbox_prop  # kW

n_prop = 0.76  # Propeller efficiency

dist_covered = 0.0  # km
m_inital = lo.m # kg
m = m_inital  # kg
SoC = lo.SoC
time_cruise = 0.0  # seconds

percent_electric_power = 0.1 # Percentage of electric power

cl_cruise = math.sqrt(uav.Cd_0 / uav.K)
cd_cruise = uav.Cd_0 + uav.K * cl_cruise**2

while dist_covered < ms.cruise_distance:
    vel_cruise = (2 * m * 9.81 / (rho * uav.S) * (uav.K / uav.Cd_0)**0.5)**0.5  # m/s
    D_cruise = 0.5 * rho * vel_cruise**2 * uav.S * cd_cruise
    P_prop_req = D_cruise * vel_cruise / 1000  # kW
    P_shaft_prop = P_prop_req / n_prop  # kW

    if P_shaft_prop > (P_ICE_full + P_electric_full)*uav.eta_gearbox_prop:
        print('''
        Error: Required propeller shaft power exceeds maximum available power.
        MISSION FAILED.
        ''')
        break

    if SoC > ms.SoC_min:
        P_electric = percent_electric_power * P_electric_full  # kW
        P_ICE = P_shaft_prop / uav.eta_gearbox_prop - P_electric  # kW

        if P_ICE > P_ICE_full:
            print('''
            Error: Required ICE power exceeds maximum available power.
            MISSION FAILED.
            ''')
            break

        if P_ICE < 0:
            print('''
            Bad Input: Required ICE power is negative. Too much electric power is being used for loitering.
            MISSION FAILED.
            ''')
            break

        m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
        elec_energy_used = ((P_electric/(uav.eta_inverter*uav.eta_motor*uav.eta_discharge)) * t_step) / 3600  # kWh

        m = m - m_fuel
        SoC = SoC - (elec_energy_used / ms.battery_capacity)
    else:
        P_ICE = P_shaft_prop / uav.eta_gearbox_prop  # kW
        P_electric = 0.0  # kW

        if P_ICE > P_ICE_full:
            print('''
            Error: Required ICE power exceeds maximum available power and battery SoC is below minimum.
            MISSION FAILED.
            ''')
            break

        m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
        m = m - m_fuel


    if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        break

    dist_covered = dist_covered + (vel_cruise * t_step) / 1000  # km
    time_cruise = time_cruise + t_step  # seconds

m_fuel_burned = m_inital - m

if dist_covered >= ms.cruise_distance:
    print(f'''
    -------Cruise In Successful-------
    Distance covered: {dist_covered:.2f} km, 
    Time: {time_cruise/60:.2f} min, 
    Final Mass: {m:.2f} kg, 
    Fuel Burned: {m_fuel_burned:.2f} kg,
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg, 
    Final SoC: {SoC:.2f}
    Final Velocity: {vel_cruise:.2f} m/s,
''')

else:

    print(f'''
    -------Cruise In Failed-------
    Distance covered: {dist_covered:.2f} km, 
    Time: {time_cruise/60:.2f} min, 
    Final Mass: {m:.2f} kg, 
    Fuel Burned: {m_fuel_burned:.2f} kg, 
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg,
    Final SoC: {SoC:.2f}
    Final Velocity: {vel_cruise:.2f} m/s,
''')


