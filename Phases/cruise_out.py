import math
from ambiance import Atmosphere
import artemis_uav as uav
import mission_settings as ms


# Cruise Out simulation setting
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
m_inital = 997 # kg
m = m_inital  # kg
SoC = 0.5
time_cruise = 0.0  # seconds

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

    if P_shaft_prop <= P_ICE_full * uav.eta_gearbox_prop:
        DoH = 0.0  # Pure ICE operation

        if SoC < ms.SoC_max:
            P_ICE = P_shaft_prop / uav.eta_gearbox_prop + min(ms.power_battery_charge/(uav.eta_inverter*uav.eta_gearbox_gen), ms.power_inverter*uav.eta_gearbox_gen)  # kW

            if P_ICE < P_ICE_full:
                P_charging = min(ms.power_battery_charge/(uav.eta_inverter*uav.eta_gearbox_gen), ms.power_inverter*uav.eta_gearbox_gen) * uav.eta_gearbox_gen * uav.eta_inverter  # kW
            else:
                P_ICE = P_ICE_full  # kW
                P_extra = P_ICE - P_shaft_prop / uav.eta_gearbox_prop  # kW
                P_charging = P_extra*uav.eta_gearbox_gen*uav.eta_inverter  # kW

            energy_charged = (P_charging * uav.eta_charging * t_step) / 3600  # kWh
            SoC = SoC + (energy_charged / ms.battery_capacity)

            m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
            m = m - m_fuel

        else:
            P_ICE = P_shaft_prop / uav.eta_gearbox_prop  # kW
            m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
            m = m - m_fuel

    elif P_shaft_prop > P_ICE_full * uav.eta_gearbox_prop:
        P_ICE = P_ICE_full  # kW
        P_electric = (P_shaft_prop / uav.eta_gearbox_prop - P_ICE) / uav.eta_gearbox_prop  # kW
        DoH = P_electric / (P_ICE + P_electric)  # Hybridization factor (0 = pure ICE, 1 = pure electric)

        if P_electric/(uav.eta_inverter * uav.eta_motor) > ms.power_battery_discharge:
            print('''
            Error: Required electric power exceeds battery discharge limit.
            MISSION FAILED.
            ''')
            break

        if P_electric/(uav.eta_inverter * uav.eta_motor) > ms.power_inverter:
            print('''
            Error: Required electric power exceeds inverter limit.
            MISSION FAILED.
            ''')
            break

        elec_energy_used = ((P_electric/(uav.eta_inverter*uav.eta_motor*uav.eta_discharge)) * t_step) / 3600  # kWh
        SoC = SoC - (elec_energy_used / ms.battery_capacity)

        m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
        m = m - m_fuel

    if SoC < ms.SoC_min:
        print('''
        Error: Battery SoC dropped below minimum.
        MISSION FAILED.
        ''')
        break

    if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        break

    dist_covered = dist_covered + (vel_cruise * t_step) / 1000  # km
    time_cruise = time_cruise + t_step  # seconds


print("\n")

m_fuel_burned = m_inital - m

if dist_covered >= ms.cruise_distance:
    print(f'''
    -------Cruise Out Successful-------
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
    -------Cruise Out Failed-------
    Distance covered: {dist_covered:.2f} km, 
    Time: {time_cruise/60:.2f} min, 
    Final Mass: {m:.2f} kg, 
    Fuel Burned: {m_fuel_burned:.2f} kg,
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg,
    Final SoC: {SoC:.2f}
    Final Velocity: {vel_cruise:.2f} m/s,
''')


