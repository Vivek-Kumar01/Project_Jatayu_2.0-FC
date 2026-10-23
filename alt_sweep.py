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

# List of phase modules in exact chronological sequence[cite: 1]
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

# Baseline fixed constants
P_FC_GROSS_BASE = float(uav.P_FC_gross_rated)
E_BATT_BASE = float(ms.battery_capacity)

# ------------------------------------------------------------------------------
# 1. DEFINE ALTITUDE SWEEP RANGE (m MSL)
# ------------------------------------------------------------------------------
# Sweeping cruise/loiter altitude from 1,000 m to 6,000 m (11 points)
ALTITUDE_SWEEP_M = np.linspace(1000.0, 6000.0, 11)
sweep_records = []

print('=' * 96)
print(
    f"{'PROJECT JATAYU 2.0 - FC: CRUISE & LOITER ALTITUDE SWEEP':^96}"
)
print('=' * 96)

for idx, alt_m in enumerate(ALTITUDE_SWEEP_M):
    # --------------------------------------------------------------------------
    # A. MUTATE ALTITUDE PROFILE IN MISSION SETTINGS
    # --------------------------------------------------------------------------
    ms.cruise_altitude = float(alt_m)
    ms.loiter_altitude = float(alt_m)

    # Reset UAV baseline attributes
    uav.P_FC_gross_rated = P_FC_GROSS_BASE
    uav.delta_Cd_0_cooling = uav.delta_Cd_0_cooling_ref * (
        uav.P_FC_gross_rated / uav.P_FC_gross_ref
    )
    uav.Cd_0 = uav.Cd_0_clean + uav.delta_Cd_0_cooling

    # Reset mission settings baseline hardware sizing
    ms.battery_capacity = E_BATT_BASE
    ms.mass_cells = (
        ms.battery_capacity * 1000.0
    ) / uav.specific_energy_battery_cell
    ms.mass_battery_pack = uav.k_bat_pack * ms.mass_cells

    ms.mass_FC_stack = uav.P_FC_gross_rated / uav.sp_power_FC_stack
    ms.mass_FC_BoP = uav.P_FC_gross_rated * uav.k_BoP_mass_factor
    ms.mass_FC_boost = uav.P_FC_gross_rated / uav.sp_power_DCDC_boost
    ms.mass_FC_system = ms.mass_FC_stack + ms.mass_FC_BoP + ms.mass_FC_boost

    ms.mass_fixed_powertrain = (
        ms.mass_electric_hardware + ms.mass_FC_system + ms.mass_battery_pack
    )
    ms.mass_H2_storage_total = uav.M_power_pack - ms.mass_fixed_powertrain
    ms.mass_H2_total = ms.mass_H2_storage_total * uav.eta_tank_gravimetric
    ms.mass_tank_system = ms.mass_H2_storage_total - ms.mass_H2_total
    ms.mass_reserve_H2 = ms.mass_H2_total * ms.reserve_fuel
    ms.mass_mission_H2 = ms.mass_H2_total - ms.mass_reserve_H2
    ms.m_zfw = uav.MTOW - ms.mass_H2_total

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
            f'[{idx+1:02d}/{len(ALTITUDE_SWEEP_M):02d}] Alt: {alt_m:5.0f} m '
            f'--> DISCARDED (Failed in {failed_phase_name})'
        )
        continue

    # --------------------------------------------------------------------------
    # D. EXTRACT METRICS FROM TELEMETRY
    # --------------------------------------------------------------------------
    df_sim = pd.DataFrame(ms.telemetry)

    total_flight_time_hr = df_sim['time_s'].iloc[-1] / 3600.0

    df_loiter = df_sim[df_sim['phase'] == 'Loiter']
    t_loiter_hr = (
        (df_loiter['time_s'].iloc[-1] - df_loiter['time_s'].iloc[0]) / 3600.0
        if not df_loiter.empty
        else 0.0
    )

    loiter_tas_ms = df_loiter['velocity_ms'].mean() if not df_loiter.empty else 0.0
    loiter_p_fc_net = df_loiter['P_FC_net_kW'].mean() if not df_loiter.empty else 0.0
    loiter_p_bop = df_loiter['P_BoP_kW'].mean() if not df_loiter.empty else 0.0

    soc_post_climb_percent = (
        df_loiter['battery_soc'].iloc[0] * 100.0
        if not df_loiter.empty
        else df_sim['battery_soc'].iloc[-1] * 100.0
    )

    sweep_records.append({
        'Altitude_m': alt_m,
        'Altitude_ft': alt_m * 3.28084,
        'Sustained_Loiter_hr': t_loiter_hr,
        'Total_Endurance_hr': total_flight_time_hr,
        'Loiter_TAS_ms': loiter_tas_ms,
        'Loiter_P_FC_net_kW': loiter_p_fc_net,
        'Loiter_P_BoP_kW': loiter_p_bop,
        'Post_Climb_SoC_percent': soc_post_climb_percent,
    })

    print(
        f'[{idx+1:02d}/{len(ALTITUDE_SWEEP_M):02d}] Alt: {alt_m:5.0f} m | '
        f'TAS: {loiter_tas_ms:4.1f} m/s | BoP: {loiter_p_bop:4.1f} kW | '
        f'Climb End SoC: {soc_post_climb_percent:4.1f}% | Loiter: {t_loiter_hr:5.2f} hr'
    )

# ------------------------------------------------------------------------------
# 2. EXPORT & DUAL-AXIS TRADE-OFF PLOT (ALTITUDE SWEEP)
# ------------------------------------------------------------------------------
df_results = pd.DataFrame(sweep_records)
csv_filename = 'fc_hardware_sweep_alt_results.csv'
df_results.to_csv(csv_filename, index=False)
print(f'\n--> Sweep data exported to {csv_filename}')

fig, ax1 = plt.subplots(figsize=(10, 6.2), dpi=300)

# Colors matching project aesthetics[cite: 3]
color_loiter = '#1F78B4'  # Steel Blue for Loiter
color_bop = '#E31A1C'     # Crimson Red for BoP Parasitic Overhead

# 1. Plot Left Axis: Sustained Loiter Endurance
line1 = ax1.plot(
    df_results['Altitude_m'],
    df_results['Sustained_Loiter_hr'],
    marker='o',
    color=color_loiter,
    lw=2.2,
    label='Sustained On-Station Loiter [Hours]',
)
ax1.set_xlabel('Loiter & Cruise Altitude [m MSL]', fontweight='bold', fontsize=11)
ax1.set_ylabel(
    'Sustained On-Station Loiter [Hours]',
    color=color_loiter,
    fontweight='bold',
    fontsize=11,
)
ax1.tick_params(axis='y', labelcolor=color_loiter)
ax1.grid(True, linestyle=':', alpha=0.5)

# 2. Plot Right Axis: Cathode Compressor & BoP Parasitic Draw
ax2 = ax1.twinx()
line2 = ax2.plot(
    df_results['Altitude_m'],
    df_results['Loiter_P_BoP_kW'],
    marker='^',
    color=color_bop,
    lw=2.2,
    linestyle='--',
    label='BoP Parasitic Load at Loiter [kW]',
)
ax2.set_ylabel(
    'BoP Parasitic Load (Compressor + Pump) [kW]',
    color=color_bop,
    fontweight='bold',
    fontsize=11,
)
ax2.tick_params(axis='y', labelcolor=color_bop)

# ------------------------------------------------------------------------------
# CONSTRAINT THRESHOLDS & FEASIBLE REGION SHADING
# ------------------------------------------------------------------------------
MIN_LOITER_REQ_HR = 5.0  # 5.0 hr minimum persistence threshold

line_t1 = ax1.axhline(
    MIN_LOITER_REQ_HR,
    color='#33A02C',
    linestyle=':',
    lw=1.6,
    label=f'Req: Min Loiter ({MIN_LOITER_REQ_HR:.1f} hr)',
)

# Vertical marker for baseline design point (3000 m)
ax1.axvline(
    3000.0,
    color='#666666',
    linestyle='-.',
    lw=1.3,
    alpha=0.8,
    label='Design Baseline (3,000 m)',
)

y1_max = max(df_results['Sustained_Loiter_hr'].max() * 1.10, MIN_LOITER_REQ_HR + 3.0)
ax1.set_ylim(bottom=0.0, top=y1_max)
ax1.axhspan(
    MIN_LOITER_REQ_HR,
    y1_max,
    color='#33A02C',
    alpha=0.07,
    label=f'Compliant Loiter Region (≥{MIN_LOITER_REQ_HR:.0f} hr)',
)

# ------------------------------------------------------------------------------
# SECONDARY X-AXIS: Altitude in Feet
# ------------------------------------------------------------------------------
def m_to_ft(x):
    return x * 3.28084

def ft_to_m(x):
    return x / 3.28084

ax_alt_ft = ax1.secondary_xaxis('top', functions=(m_to_ft, ft_to_m))
ax_alt_ft.set_xlabel('Altitude [Feet MSL]', fontweight='bold', fontsize=11, labelpad=8)
ax_alt_ft.tick_params(axis='x')

# ------------------------------------------------------------------------------
# INVARIANT ANCHOR & COMBINED LEGEND
# ------------------------------------------------------------------------------
handles1, labels1 = ax1.get_legend_handles_labels()
handles2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(
    handles1 + handles2,
    labels1 + labels2,
    loc='lower left',
    framealpha=0.92,
    fontsize=8.5,
    ncol=2,
)

plt.title(
    'Project Jatayu 2.0 - FC: Flight Ceiling & Loiter Altitude Trade Study',
    fontweight='bold',
    fontsize=13,
    pad=38,
)

plt.figtext(
    0.5,
    0.015,
    f'Held Constant: Power-Pack Mass Budget = {uav.M_power_pack:.1f} kg  |  '
    f'FC Gross = {P_FC_GROSS_BASE:.0f} kW  |  Buffer Battery = {E_BATT_BASE:.1f} kWh  |  MTOW = {uav.MTOW:.0f} kg',
    ha='center',
    fontsize=9.5,
    style='italic',
    bbox=dict(
        boxstyle='square,pad=0.4',
        facecolor='#F0F0F0',
        edgecolor='#CCCCCC',
        lw=0.8,
    ),
)

fig.tight_layout(rect=[0.0, 0.05, 1.0, 0.95])
plot_filename = 'plot_fc_altitude_tradeoff.png'
plt.savefig(plot_filename, dpi=300)
plt.close(fig)
print(f'--> Altitude sweep trade study plot saved as {plot_filename}')