import math
from ambiance import Atmosphere
import artemis_uav as uav
import mission_settings as ms


# Take-off simulation setting
rho = Atmosphere(ms.altitude_takeoff).density[0]  # kg/m^3
t_step = 0.1  # seconds

P_ICE_full = uav.P_ICE_rated * ((1.132 * rho / Atmosphere(0).density[0]) - 0.132)  # kW
P_electric_full = (min(ms.power_battery_discharge, ms.power_inverter/uav.eta_inverter))*uav.eta_inverter*uav.eta_motor  # kW

percent_ICE_power = 1 # Percentage of ICE power
percent_electric_power = 1 # Percentage of electric power

P_ICE = percent_ICE_power * P_ICE_full  # kW
P_electric = percent_electric_power * P_electric_full  # kW

DoH = P_electric / (P_ICE + P_electric)  # Hybridization factor (0 = pure ICE, 1 = pure electric)
P_shaft_prop = (P_ICE + P_electric) * uav.eta_gearbox_prop  # kW

Merit_Factor = 0.7 # Ideal power req to real power given
Static_thrust = (Merit_Factor**(2/3)) * (2 * rho * (math.pi /4 * uav.D_prop**2) * (P_shaft_prop * 1000)**2) ** (1/3)

N = 1 # seconds after rotation on ground roll
V_stall = math.sqrt((2 * uav.MTOW * 9.81) / (rho * uav.S * uav.Cl_max_ground_TO))  # m/s
V_LO = 1.1 * V_stall  # m/s

prop_eff_liftoff = 0.65
Thrust_liftoff = prop_eff_liftoff * P_shaft_prop * 1000 / V_LO  # N

def thrust_available(V, Static_thrust = Static_thrust, Thrust_liftoff = Thrust_liftoff, V_LO = V_LO):
    return Static_thrust + (Thrust_liftoff - Static_thrust) * (V / V_LO)**2


t = 0.0  # seconds
s_roll = 0.0  # m
V = 0.0  # m/s
m = uav.MTOW  # kg
SoC = ms.SoC_max  # Initial state of charge for battery pack

while V < V_LO:
    T = thrust_available(V)
    D = 0.5 * rho * V**2 * uav.S * ms.CD_takeoff
    W = m * 9.81
    L = 0.5 * rho * V**2 * uav.S * uav.Cl_ground
    s_roll = s_roll + V * t_step
    F_net = T - D - (W - L) * ms.rolling_friction_coefficient
    a_net = F_net / m
    m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
    elec_energy_used = ((P_electric/(uav.eta_inverter*uav.eta_motor*uav.eta_discharge)) * t_step) / 3600  # kWh

    V = V + a_net * t_step
    t = t + t_step
    SoC = SoC - (elec_energy_used / ms.battery_capacity)
    m = m - m_fuel


    if s_roll > ms.max_permissible_sroll:
        print("Takeoff failed: maximum permissible ground roll exceeded.")
        break

    if SoC < ms.SoC_min:
        print("Takeoff failed: battery SoC dropped below minimum.")
        break

    if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
        print("Takeoff failed: insufficient fuel remaining.")
        break

s_roll = s_roll + N*V_LO

m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * N / 1000  # kg
m = m - m_fuel

elec_energy_used = ((P_electric/(uav.eta_inverter*uav.eta_motor)) * N) / 3600  # kWh
SoC = SoC - (elec_energy_used / ms.battery_capacity)

if s_roll > ms.max_permissible_sroll:
    print("Takeoff failed: maximum permissible ground roll exceeded.")
else:
    print(f'''
    -------Takeoff Successful-------
    P_ICE: {P_ICE:.2f} kW, P_electric: {P_electric:.2f} kW, DoH: {DoH:.2f}
    Total ground roll distance: {s_roll:.2f} m
    Time taken to reach lift off speed: {t + N:.2f} seconds
    Fuel burned during takeoff: {(uav.MTOW - m)*1000:.2f} g
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    Battery SoC after takeoff: {SoC*100:.2f} %\n
''')