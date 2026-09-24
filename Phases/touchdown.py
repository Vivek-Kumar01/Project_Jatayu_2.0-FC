from ambiance import Atmosphere
import artemis_uav as uav
import mission_settings as ms
import Phases.flare as fl

m_initial = fl.m
m = fl.m
SoC = fl.SoC
V = fl.V
h = fl.h
rho = Atmosphere(h).density[0]

t_step = 0.1 # sec

percent_idle_power = 6 # Percentage of total rated power at sea level
P_ICE_Idle = (percent_idle_power/100) * uav.P_ICE_rated # kW

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW

P_ICE_shaft = P_ICE_Idle - P_ICE_avionics # kW
P_prop_shaft = P_ICE_shaft * uav.eta_gearbox_ICE

eta_prop = 0 # Assuming not thrust, no lift coeff on blades of prop, only drag
P_avail = P_prop_shaft * eta_prop

if P_prop_shaft < 0:
    print('''
WARNING: Idling power percentage is too low, unable to power avionics. Increase the percentage.
''')

Cd_0_land = fl.Cd0_land
Cl = uav.Cl_ground
Cd = Cd_0_land + ms.K_grd * Cl**2
mu_r = 0.4

N_freeroll = 1 # sec, free rolling assuming constant velocity = 1.15 V_stall_land = V
s_land_roll = N_freeroll * V

m_fuel_freeroll = (
    (P_ICE_Idle * (uav.BSFC_ICE_low / 3600.0)) * N_freeroll / 1000.0
)  # kg
m -= m_fuel_freeroll

t = N_freeroll
Phase_Success = True


if m > uav.max_landing_mass:
    print(
        f"Structural Limit Exceeded: Landing weight {m:.2f} kg exceeds Maximum Landing Weight ({uav.max_landing_mass} kg)"
    )
    Phase_Success = False


while V > 0:
    D = 0.5 * rho * V**2 * uav.S * Cd
    L = 0.5 * rho * V**2 * uav.S * Cl
    net_F = -(D + mu_r * (m * 9.81 - L)) # -ve decelerating
    a_net = net_F/m


    if V + a_net * t_step <= 0.0:
        dt_stop = -V / a_net  # Exact fractional time to stop (V / |a|)
        s_land_roll += 0.5 * (V**2) / abs(a_net)  # Exact remaining distance
        t += dt_stop
        m -= (
            (P_ICE_Idle * (uav.BSFC_ICE_low / 3600.0)) * dt_stop / 1000.0
        )  # Exact fuel
        V = 0.0
        break

    s_land_roll += V * t_step
    V += a_net * t_step
    t += t_step
    
    m_fuel = (P_ICE_Idle * (uav.BSFC_ICE_low / 3600)) * t_step / 1000  # kg
    m -= m_fuel

    if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if s_land_roll > ms.max_permissible_sroll:
        print("Touchdown failed: maximum permissible ground roll exceeded.")
        Phase_Success = False
        break

        

if Phase_Success:
    print(f'''
    -------Touchdown Successful-------
    P_ICE: {P_ICE_Idle:.2f} kW
    Total ground roll distance: {s_land_roll:.2f} m
    Final velocity: {V:.2f} m/s
    Time taken to stop: {t:.2f} seconds
    Fuel burned during landing: {(m_initial - m)*1000:.2f} g
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    Final Battery SoC: {SoC*100:.2f} %
''')

if not Phase_Success:
    print(f'''
    -------Touchdown Failed-------
    P_ICE: {P_ICE_Idle:.2f} kW
    Total ground roll distance: {s_land_roll:.2f} m
    Final velocity: {V:.2f} m/s
    Time taken to stop: {t:.2f} seconds
    Fuel burned during landing: {(m_initial - m)*1000:.2f} g
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    Final Battery SoC: {SoC*100:.2f} %
''')
    