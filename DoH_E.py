import importlib
import math
import os
from contextlib import redirect_stdout
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Suppress sizing audit on initial import
with open(os.devnull, 'w', encoding='utf-8') as fnull:
    with redirect_stdout(fnull):
        import jatayu_fc_uav as uav
        import mission_settings_fc as ms

# Chronological phase sequence
PHASE_MODULE_NAMES = [
    'Phases.takeoff',
    'Phases.pullup',
    'Phases.climb',
    'Phases.cruise_climb',
    'Phases.cruise_out',
    'Phases.loiter',
    'Phases.cruise_in',
    'Phases.descent',
    'Phases.approach',
    'Phases.flare',
    'Phases.touchdown',
]

# Baseline fixed anchors
P_MOTOR_TOTAL = float(ms.power_motor)
P_FC_GROSS_BASE = float(uav.P_FC_gross_rated)

# ------------------------------------------------------------------------------
# 1. DEFINE BATTERY CAPACITY SWEEP RANGE (kWh)
# ------------------------------------------------------------------------------
E_BATT_SWEEP = np.linspace(4.0, 14.0, 13)
sweep_records = []

print('=' * 96)
print(f"{'ENERGY DEGREE OF HYBRIDIZATION SWEEP (DoH_E vs. MISSION METRICS)':^96}")
print('=' * 96)

for idx, e_batt in enumerate(E_BATT_SWEEP):
    # --------------------------------------------------------------------------
    # A. MUTATE BATTERY & MASS SPLIT
    # --------------------------------------------------------------------------
    ms.battery_capacity = float(e_batt)

    ms.power_battery_discharge_cont = (
        uav.C_discharge_continuous_safe * ms.battery_capacity
    )
    ms.power_battery_discharge_surge = (
        uav.C_discharge_surge_safe * ms.battery_capacity
    )
    ms.power_battery_charge = uav.C_charge_continuous_safe * ms.battery_capacity

    ms.mass_cells = (
        ms.battery_capacity * 1000.0
    ) / uav.specific_energy_battery_cell
    ms.mass_battery_pack = uav.k_bat_pack * ms.mass_cells

    ms.mass_fixed_powertrain = (
        ms.mass_electric_hardware + ms.mass_FC_system + ms.mass_battery_pack
    )
    ms.mass_H2_storage_total = uav.M_power_pack - ms.mass_fixed_powertrain

    if ms.mass_H2_storage_total <= 0.0:
        print(
            f'[{idx+1:02d}/{len(E_BATT_SWEEP):02d}] E_batt = {e_batt:4.1f} kWh'
            ' --> SKIPPED (Storage mass <= 0 kg)'
        )
        continue

    ms.mass_H2_total = ms.mass_H2_storage_total * uav.eta_tank_gravimetric
    ms.mass_tank_system = ms.mass_H2_storage_total - ms.mass_H2_total
    ms.mass_reserve_H2 = ms.mass_H2_total * ms.reserve_fuel
    ms.mass_mission_H2 = ms.mass_H2_total - ms.mass_reserve_H2
    ms.m_zfw = uav.MTOW - ms.mass_H2_total

    E_H2_kWh = ms.mass_H2_total * uav.LHV_H2_kWh
    doh_e = ms.battery_capacity / (ms.battery_capacity + E_H2_kWh)

    # --------------------------------------------------------------------------
    # B. RESET TELEMETRY BUFFER
    # --------------------------------------------------------------------------
    ms.telemetry = {
        'time_s': [],
        'phase': [],
        'altitude_m': [],
        'velocity_ms': [],
        'P_req_kW': [],
        'P_FC_gross_kW': [],
        'P_FC_net_kW': [],
        'P_BoP_kW': [],
        'P_EM_kW': [],
        'P_bat_kW': [],
        'H2_mass_kg': [],
        'battery_soc': [],
    }

    # --------------------------------------------------------------------------
    # C. EXECUTE PHASES SILENTLY & VALIDATE
    # --------------------------------------------------------------------------
    sim_failed = False
    failed_phase_name = None

    with open(os.devnull, 'w', encoding='utf-8') as fnull:
        with redirect_stdout(fnull):
            for mod_name in PHASE_MODULE_NAMES:
                try:
                    mod = importlib.import_module(mod_name)
                    importlib.reload(mod)

                    phase_ok = getattr(mod, 'Phase_Success', True)
                    if not phase_ok:
                        sim_failed = True
                        failed_phase_name = mod_name
                        break
                except Exception as e:
                    sim_failed = True
                    failed_phase_name = f'{mod_name} (Crash: {e})'
                    break

    if sim_failed:
        print(
            f'[{idx+1:02d}/{len(E_BATT_SWEEP):02d}] Batt: {e_batt:4.1f} kWh |'
            f' H2: {ms.mass_H2_total:5.2f} kg --> DISCARDED (Failed in'
            f' {failed_phase_name})'
        )
        continue

    # --------------------------------------------------------------------------
    # D. EXTRACT METRICS
    # --------------------------------------------------------------------------
    df_sim = pd.DataFrame(ms.telemetry)
    total_flight_time_hr = df_sim['time_s'].iloc[-1] / 3600.0

    df_loiter = df_sim[df_sim['phase'] == 'Loiter']
    t_loiter_hr = (
        (df_loiter['time_s'].iloc[-1] - df_loiter['time_s'].iloc[0]) / 3600.0
        if not df_loiter.empty
        else 0.0
    )

    soc_post_climb_percent = (
        df_loiter['battery_soc'].iloc[0] * 100.0
        if not df_loiter.empty
        else df_sim['battery_soc'].iloc[-1] * 100.0
    )
    soc_min_mission_percent = df_sim['battery_soc'].min() * 100.0
    h2_burned_kg = df_sim['H2_mass_kg'].iloc[0] - df_sim['H2_mass_kg'].iloc[-1]

    sweep_records.append({
        'E_batt_kWh': e_batt,
        'Mass_Batt_kg': ms.mass_battery_pack,
        'Mass_H2_Total_kg': ms.mass_H2_total,
        'Mass_H2_Mission_kg': ms.mass_mission_H2,
        'DoH_E_percent': doh_e * 100.0,
        'Total_Endurance_hr': total_flight_time_hr,
        'Sustained_Loiter_hr': t_loiter_hr,
        'Post_Climb_SoC_percent': soc_post_climb_percent,
        'Min_Mission_SoC_percent': soc_min_mission_percent,
        'H2_Burned_kg': h2_burned_kg,
    })

    print(
        f'[{idx+1:02d}/{len(E_BATT_SWEEP):02d}] Batt: {e_batt:4.1f} kWh | H2:'
        f' {ms.mass_H2_total:5.2f} kg | DoH_E: {doh_e*100:4.2f}% | Climb End'
        f' SoC: {soc_post_climb_percent:4.1f}% | Loiter: {t_loiter_hr:5.2f} hr'
        f' | Total: {total_flight_time_hr:5.2f} hr'
    )

# ------------------------------------------------------------------------------
# 2. DUAL-AXIS TRADE-OFF PLOT (DoH_E)
# ------------------------------------------------------------------------------
df_results = pd.DataFrame(sweep_records)
csv_filename = 'fc_hardware_sweep_dohe_results.csv'
df_results.to_csv(csv_filename, index=False)
print(f'\n--> Sweep data exported to {csv_filename}')

fig, ax1 = plt.subplots(figsize=(10.5, 6.2), dpi=300)

color_loiter = '#1F78B4'    # Steel Blue for Loiter
color_endurance = '#1B9E77' # Teal for Total Endurance
color_soc = '#6A3D9A'       # Deep Purple for Battery SoC

# 1. Left Axis - Sustained Loiter
line1 = ax1.plot(
    df_results['E_batt_kWh'],
    df_results['Sustained_Loiter_hr'],
    marker='o',
    color=color_loiter,
    lw=2.2,
    label='Sustained On-Station Loiter [hr]',
)

# 2. Left Axis - Total Endurance
line2 = ax1.plot(
    df_results['E_batt_kWh'],
    df_results['Total_Endurance_hr'],
    marker='D',
    color=color_endurance,
    lw=2.0,
    linestyle='-.',
    label='Total Mission Endurance [hr]',
)

ax1.set_xlabel('Installed Buffer Battery Capacity [kWh]', fontweight='bold', fontsize=11, labelpad=6)
ax1.set_ylabel('Mission Flight Duration [Hours]', color='#222222', fontweight='bold', fontsize=11, labelpad=8)
ax1.tick_params(axis='y', labelcolor='#222222')
ax1.grid(True, linestyle=':', alpha=0.55)

# 3. Right Axis - Battery Level-off SoC
ax2 = ax1.twinx()
line3 = ax2.plot(
    df_results['E_batt_kWh'],
    df_results['Post_Climb_SoC_percent'],
    marker='s',
    color=color_soc,
    lw=2.2,
    linestyle='--',
    label='Post-Climb Buffer Battery SoC [%]',
)
ax2.set_ylabel('Buffer Battery SoC at Cruise Level-off [%]', color=color_soc, fontweight='bold', fontsize=11, labelpad=8)
ax2.tick_params(axis='y', labelcolor=color_soc)

# ------------------------------------------------------------------------------
# CONSTRAINTS & FEASIBLE REGIONS
# ------------------------------------------------------------------------------
MIN_LOITER_REQ_HR = 5.0
MIN_SOC_REQ_PERCENT = 20.0

line_t1 = ax1.axhline(
    MIN_LOITER_REQ_HR,
    color='#E31A1C',
    linestyle=':',
    lw=1.6,
    label=f'Req: Min Loiter ({MIN_LOITER_REQ_HR:.1f} hr)',
)
line_t2 = ax2.axhline(
    MIN_SOC_REQ_PERCENT,
    color='#FF7F00',
    linestyle=':',
    lw=1.6,
    label=f'Req: Min SoC Floor ({MIN_SOC_REQ_PERCENT:.0f}%)',
)

# Left Y-Axis Scaling: Accommodate both Loiter and Total Endurance curves cleanly
y1_max = math.ceil(max(df_results['Total_Endurance_hr'].max(), df_results['Sustained_Loiter_hr'].max()) + 1.2)
ax1.set_ylim(bottom=0.0, top=y1_max)
ax1.axhspan(MIN_LOITER_REQ_HR, y1_max, color='#33A02C', alpha=0.06, label=f'Compliant Loiter Region (≥{MIN_LOITER_REQ_HR:.0f} hr)')

ax2.set_ylim(bottom=0.0, top=100.0)
ax2.axhspan(MIN_SOC_REQ_PERCENT, 100.0, color='#6A3D9A', alpha=0.06, label=f'Protected Battery Regime (≥{MIN_SOC_REQ_PERCENT:.0f}%)')

# ------------------------------------------------------------------------------
# SECONDARY TOP X-AXIS: DoH_E (%)
# ------------------------------------------------------------------------------
doh_vals = df_results['DoH_E_percent'].values
ebatt_vals = df_results['E_batt_kWh'].values

def ebatt_to_dohe(x):
    return np.interp(x, ebatt_vals, doh_vals)

def dohe_to_ebatt(x):
    return np.interp(x, doh_vals, ebatt_vals)

ax_doh = ax1.secondary_xaxis('top', functions=(ebatt_to_dohe, dohe_to_ebatt))
ax_doh.set_xlabel('Energy Degree of Hybridization ($DoH_E$) [%]', fontweight='bold', fontsize=11, labelpad=10)
ax_doh.tick_params(axis='x', pad=4)

# ------------------------------------------------------------------------------
# CLEAN MULTI-ROW LEGEND
# ------------------------------------------------------------------------------
handles1, labels1 = ax1.get_legend_handles_labels()
handles2, labels2 = ax2.get_legend_handles_labels()
all_handles = handles1 + handles2
all_labels = labels1 + labels2

ax1.legend(
    all_handles,
    all_labels,
    loc='upper center',
    bbox_to_anchor=(0.5, 0.96),
    framealpha=0.92,
    fontsize=8.2,
    ncol=3,
)

# Generic professional title
plt.title('Fuel Cell UAV: Energy Hybridization Envelope ($DoH_E$)', fontweight='bold', fontsize=13, pad=38)

plt.figtext(
    0.5,
    0.015,
    f'Held Constant: Power-Pack Mass Budget = {uav.M_power_pack:.1f} kg  |  FC Gross = {P_FC_GROSS_BASE:.0f} kW  |  '
    f'Motor Peak = {P_MOTOR_TOTAL:.0f} kW  |  MTOW = {uav.MTOW:.0f} kg',
    ha='center',
    fontsize=9.5,
    style='italic',
    bbox=dict(boxstyle='square,pad=0.4', facecolor='#F0F0F0', edgecolor='#CCCCCC', lw=0.8),
)

# Extra margin right and top to completely avoid overlapping labels
fig.tight_layout(rect=[0.02, 0.05, 0.98, 0.93])

plot_filename = 'plot_fc_dohe_tradeoff_pareto.png'
plt.savefig(plot_filename, dpi=300)
plt.close(fig)
print(f'--> Updated FC trade-off Pareto curve saved as {plot_filename}')