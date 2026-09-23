import math
from ambiance import Atmosphere
import artemis_uav as uav
import mission_settings as ms
import Phases.cruise_out as co


# Loiter simulation setting
t_step = 1.0  # seconds
rho = Atmosphere(ms.loiter_altitude).density[0]  # kg/m^3

P_ICE_full = uav.P_ICE_rated * ((1.132 * rho / Atmosphere(0).density[0]) - 0.132)  # kW

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW
P_ICE_full = P_ICE_full - P_ICE_avionics  # kW

P_electric_full = (min(ms.power_battery_discharge, ms.power_inverter/uav.eta_inverter))*uav.eta_inverter*uav.eta_motor  # kW

n_prop = 0.72  # Propeller efficiency

m_inital = co.m # kg
m = m_inital  # kg
SoC = co.SoC
endurance = 0.0  # seconds

fraction_keep = 0.9 # Percentage of fuel burned in cruise out to be keep for cruising in
min_mass_cruisein = uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel) +  fraction_keep * co.m_fuel_burned  # kg

percent_electric_power = 0.05 # Percentage of electric power


Loiter_Phase_Status = True

safe_margin = 1.25 # Ratio of loiter to stall velocity (as cal by Cl_max_clean)
cl_loiter = uav.Cl_max_clean / (safe_margin)**2
cd_loiter = uav.Cd_0 + uav.K * cl_loiter**2


while m > min_mass_cruisein:
    vel_loiter = (2 * m * 9.81 / (rho * uav.S * cl_loiter))**0.5  # m/s
    D_loiter = 0.5 * rho * vel_loiter**2 * uav.S * cd_loiter
    P_prop_req = D_loiter * vel_loiter / 1000  # kW
    P_shaft_prop = P_prop_req / n_prop  # kW

    if P_shaft_prop > P_ICE_full*uav.eta_gearbox_ICE  + P_electric_full:
        Loiter_Phase_Status = False
        print('''
        Error: Required propeller shaft power exceeds maximum available power.
        MISSION FAILED.
        ''')
        break

    if SoC > ms.SoC_min:
        P_electric = percent_electric_power * P_electric_full  # kW
        P_ICE = (P_shaft_prop - P_electric) / uav.eta_gearbox_ICE  # kW

        if P_ICE > P_ICE_full:
            Loiter_Phase_Status = False
            print('''
            Error: Required ICE power exceeds maximum available power.
            MISSION FAILED.
            ''')
            break

        if P_ICE < 0:
            Loiter_Phase_Status = False
            print('''
            Bad Input: Required ICE power is negative. Too much electric power is being used for loitering.
            MISSION FAILED.
            ''')
            break

        m_fuel = (P_ICE * (uav.BSFC_ICE_low / 3600)) * t_step / 1000  # kg
        elec_energy_used = ((P_electric/(uav.eta_inverter*uav.eta_motor*uav.eta_discharge)) * t_step) / 3600  # kWh

        m = m - m_fuel - ((P_ICE_avionics * uav.BSFC_ICE_low / 3600) * t_step / 1000)
        SoC = SoC - (elec_energy_used / ms.battery_capacity)
    else:
        P_ICE = P_shaft_prop / uav.eta_gearbox_ICE  # kW
        P_electric = 0.0  # kW

        if P_ICE > P_ICE_full:
            Loiter_Phase_Status = False
            print('''
            Error: Required ICE power exceeds maximum available power and battery SoC is below minimum.
            MISSION FAILED.
            ''')
            break

        m_fuel = (P_ICE * (uav.BSFC_ICE_low / 3600)) * t_step / 1000  # kg
        m = m - m_fuel - ((P_ICE_avionics * uav.BSFC_ICE_low / 3600) * t_step / 1000)

    if m <= (uav.MTOW - ms.mass_fuel * (1.0 - ms.reserve_fuel)):
        print("Emergency fuel reserve breached during loiter! RTB immediately.")
        break

    endurance = endurance + t_step  # seconds

if Loiter_Phase_Status:
    print(f'''
    -------Loiter Successful-------
    Endurance: {endurance/3600:.2f} hours
    Final Mass: {m:.2f} kg
    Final SoC: {SoC:.2f}
    Final Velocity: {vel_loiter:.2f} m/s
    Fuel Burned: {m_inital - m:.2f} kg
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    ''')

else:
    print(f'''
    -------Loiter Failed-------
    Endurance: {endurance/3600:.2f} hours
    Final Mass: {m:.2f} kg
    Final SoC: {SoC:.2f}
    Final Velocity: {vel_loiter:.2f} m/s
    Fuel Burned: {m_inital - m:.2f} kg
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    ''')


