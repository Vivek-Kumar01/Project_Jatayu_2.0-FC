import importlib
import math
import os
from contextlib import redirect_stdout
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Suppress sizing audit printed on initial import
with open(os.devnull, 'w', encoding='utf-8') as fnull:
    with redirect_stdout(fnull):
        import jatayu_fc_uav as uav
        import mission_settings_fc as ms

# List of phase modules in exact chronological execution order
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
P_EM_TAKEOFF = float(uav.P_EM_takeoff_rated)  # 105.0 kW
E_BATT_FIXED = float(ms.battery_capacity)    # 8.0 kWh fixed baseline

# ------------------------------------------------------------------------------
# 1. DEFINE GROSS FUEL CELL RATING SWEEP RANGE (kW)
# ------------------------------------------------------------------------------
P_FC_SWEEP = np.linspace(40.0, 80.0, 13)
sweep_records = []

print('=' * 96)
print(f"{'POWER DEGREE OF HYBRIDIZATION SWEEP (DoH_P vs. MISSION METRICS)':^96}")
print('=' * 96)

for idx, p_fc_gross in enumerate(P_FC_SWEEP):
    # --------------------------------------------------------------------------
    # A. MUTATE FUEL CELL & AERODYNAMIC PARAMETERS
    # --------------------------------------------------------------------------
    uav.P_FC_gross_rated = float(p_fc_gross)

    # Dynamic scaling of belly radiator cooling drag
    uav.delta_Cd_0_cooling = uav.delta_Cd_0_cooling_ref * (
        uav.P_FC_gross_rated / uav.P_FC_gross_ref
    )
    uav.Cd_0 = uav.Cd_0_clean + uav.delta_Cd_0_cooling

    # Recalculate Sea-Level reference power ratings
    (
        uav.P_FC_gross_SL,
        uav.P_FC_net_SL,
        uav.P_FC_BoP_SL,
    ) = uav.get_fc_available_power(0.0, uav.P_FC_gross_rated)

    # --------------------------------------------------------------------------
    # B. MUTATE POWER-PACK MASS BREAKDOWN
    # --------------------------------------------------------------------------
    ms.mass_FC_stack = uav.P_FC_gross_rated / uav.sp_power_FC_stack
    ms.mass_FC_BoP = uav.P_FC_gross_rated * uav.k_BoP_mass_factor
    ms.mass_FC_boost = uav.P_FC_gross_rated / uav.sp_power_DCDC_boost
    ms.mass_FC_system = ms.mass_FC_stack + ms.mass_FC_BoP + ms.mass_FC_boost

    ms.battery_capacity = E_BATT_FIXED
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
            f'[{idx+1:02d}/{len(P_FC_SWEEP):02d}] P_FC = {p_fc_gross:4.1f} kW'
            ' --> SKIPPED (Storage mass <= 0 kg)'
        )
        continue

    ms.mass_H2_total = ms.mass_H2_storage_total * uav.eta_tank_gravimetric
    ms.mass_tank_system = ms.mass_H2_storage_total - ms.mass_H2_total

    ms.mass_reserve_H2 = ms.mass_H2_total * ms.reserve_fuel
    ms.mass_mission_H2 = ms.mass_H2_total - ms.mass_reserve_H2
    ms.m_zfw = uav.MTOW - ms.mass_H2_total

    # Altitude-agnostic takeoff power split
    (
        P_FC_gross_TO,
        P_FC_net_TO,
        P_FC_BoP_TO,
    ) = uav.get_fc_available_power(ms.altitude_takeoff, uav.P_FC_gross_rated)

    P_bat_surge_req = max(0.0, P_EM_TAKEOFF - P_FC_net_TO)
    doh_p = P_bat_surge_req / P_EM_TAKEOFF
    c_rate_takeoff_surge = P_bat_surge_req / ms.battery_capacity

    # --------------------------------------------------------------------------
    # C. RESET TELEMETRY BUFFER
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
    # D. EXECUTE PHASES SILENTLY & VALIDATE INTEGRATION
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
            f'[{idx+1:02d}/{len(P_FC_SWEEP):02d}] P_FC: {p_fc_gross:4.1f} kW |'
            f' H2: {ms.mass_H2_total:5.2f} kg --> DISCARDED (Failed in'
            f' {failed_phase_name})'
        )
        continue

    # --------------------------------------------------------------------------
    # E. EXTRACT MISSION METRICS
    # --------------------------------------------------------------------------
    df_sim = pd.DataFrame(ms.telemetry)
    total_flight_time_hr = df_sim['time_s'].iloc[-1] / 3600.0

    df_loiter = df_sim[df_sim['phase'] == 'Loiter']
    t_loiter_hr = (
        (df_loiter['time_s'].iloc[-1] - df_loiter['time_s'].iloc[0]) / 3600.0
        if not df_loiter.empty
        else 0.0
    )

    df_to = df_sim[df_sim['phase'] == 'Takeoff']
    s_ground_roll = (
        df_to['velocity_ms'].iloc[-1] * df_to['time_s'].iloc[-1] * 0.5
        if not df_to.empty
        else 0.0
    )

    soc_post_climb_percent = (
        df_loiter['battery_soc'].iloc[0] * 100.0
        if not df_loiter.empty
        else df_sim['battery_soc'].iloc[-1] * 100.0
    )

    sweep_records.append({
        'P_FC_gross_kW': p_fc_gross,
        'P_FC_net_SL_kW': uav.P_FC_net_SL,
        'P_bat_surge_kW': P_bat_surge_req,
        'DoH_P_percent': doh_p * 100.0,
        'C_rate_surge': c_rate_takeoff_surge,
        'Mass_FC_sys_kg': ms.mass_FC_system,
        'Mass_H2_Total_kg': ms.mass_H2_total,
        'Cd_0_total': uav.Cd_0,
        'Total_Endurance_hr': total_flight_time_hr,
        'Sustained_Loiter_hr': t_loiter_hr,
        'Post_Climb_SoC_percent': soc_post_climb_percent,
    })

    print(
        f'[{idx+1:02d}/{len(P_FC_SWEEP):02d}] P_FC: {p_fc_gross:4.1f} kW |'
        f' DoH_P: {doh_p*100:4.1f}% | Bat Surge: {P_bat_surge_req:4.1f} kW'
        f' ({c_rate_takeoff_surge:4.1f}C) | H2: {ms.mass_H2_total:5.2f} kg |'
        f' Loiter: {t_loiter_hr:5.2f} hr | Total: {total_flight_time_hr:5.2f} hr'
    )

# ------------------------------------------------------------------------------
# 2. EXPORT & DUAL-AXIS TRADE-OFF PLOT (DoH_P)
# ------------------------------------------------------------------------------
df_results = pd.DataFrame(sweep_records)
csv_filename = 'fc_hardware_sweep_dohp_results.csv'
df_results.to_csv(csv_filename, index=False)
print(f'\n--> Sweep data exported to {csv_filename}')

fig, ax1 = plt.subplots(figsize=(10.5, 6.2), dpi=300)

color_loiter = '#1F78B4'    # Steel Blue for Loiter
color_endurance = '#1B9E77' # Teal for Total Endurance
color_crate = '#D95F02'     # Warm Amber / Firebrick for C-rate

# 1. Left Axis - Sustained Loiter
line1 = ax1.plot(
    df_results['P_FC_gross_kW'],
    df_results['Sustained_Loiter_hr'],
    marker='o',
    color=color_loiter,
    lw=2.2,
    label='Sustained On-Station Loiter [hr]',
)

# 2. Left Axis - Total Endurance
line2 = ax1.plot(
    df_results['P_FC_gross_kW'],
    df_results['Total_Endurance_hr'],
    marker='D',
    color=color_endurance,
    lw=2.0,
    linestyle='-.',
    label='Total Mission Endurance [hr]',
)

ax1.set_xlabel(
    'Gross Fuel Cell Stack Rating ($P_{FC,gross}$) [kW]',
    fontweight='bold',
    fontsize=11,
    labelpad=6,
)
ax1.set_ylabel(
    'Mission Flight Duration [Hours]',
    color='#222222',
    fontweight='bold',
    fontsize=11,
    labelpad=8,
)
ax1.tick_params(axis='y', labelcolor='#222222')
ax1.grid(True, linestyle=':', alpha=0.55)

# 3. Right Axis - Peak Battery Takeoff C-Rate
ax2 = ax1.twinx()
line3 = ax2.plot(
    df_results['P_FC_gross_kW'],
    df_results['C_rate_surge'],
    marker='^',
    color=color_crate,
    lw=2.2,
    linestyle='--',
    label='Peak Battery Takeoff C-Rate [1/h]',
)
ax2.set_ylabel(
    'Takeoff Surge Discharge C-Rate [1/h]',
    color=color_crate,
    fontweight='bold',
    fontsize=11,
    labelpad=8,
)
ax2.tick_params(axis='y', labelcolor=color_crate)

# ------------------------------------------------------------------------------
# CONSTRAINTS & FEASIBLE REGIONS
# ------------------------------------------------------------------------------
MIN_LOITER_REQ_HR = 5.0      # Tactical loiter requirement (5.0 hr)
MAX_CRATE_LIMIT = 10.0       # Max safe cell burst limit (10C)

line_t1 = ax1.axhline(
    MIN_LOITER_REQ_HR,
    color='#E31A1C',
    linestyle=':',
    lw=1.6,
    label=f'Req: Min Loiter ({MIN_LOITER_REQ_HR:.1f} hr)',
)
line_t2 = ax2.axhline(
    MAX_CRATE_LIMIT,
    color='#6A3D9A',
    linestyle=':',
    lw=1.6,
    label=f'Limit: Max Cell C-Rate ({MAX_CRATE_LIMIT:.0f}C)',
)

# Left Y-Axis Scaling
y1_max = math.ceil(max(df_results['Total_Endurance_hr'].max(), df_results['Sustained_Loiter_hr'].max()) + 1.2)
ax1.set_ylim(bottom=0.0, top=y1_max)
ax1.axhspan(
    MIN_LOITER_REQ_HR,
    y1_max,
    color='#33A02C',
    alpha=0.06,
    label=f'Compliant Loiter Region (≥{MIN_LOITER_REQ_HR:.0f} hr)',
)

# Right Y-Axis Scaling: Give comfortable margin above 10C limit
y2_max = max(df_results['C_rate_surge'].max() * 1.15, MAX_CRATE_LIMIT + 2.0)
ax2.set_ylim(bottom=0.0, top=y2_max)
ax2.axhspan(
    0.0,
    MAX_CRATE_LIMIT,
    color='#1F78B4',
    alpha=0.06,
    label=f'Safe Battery Discharge Regime (≤{MAX_CRATE_LIMIT:.0f}C)',
)

# ------------------------------------------------------------------------------
# SECONDARY TOP X-AXIS: DoH_P (%)
# ------------------------------------------------------------------------------
dohp_vals = df_results['DoH_P_percent'].values
pfc_vals = df_results['P_FC_gross_kW'].values


def pfc_to_dohp(x):
    return np.interp(x, pfc_vals, dohp_vals)


def dohp_to_pfc(x):
    return np.interp(x, dohp_vals, pfc_vals)


ax_dohp = ax1.secondary_xaxis('top', functions=(pfc_to_dohp, dohp_to_pfc))
ax_dohp.set_xlabel(
    'Power Degree of Hybridization ($DoH_P$) [%]',
    fontweight='bold',
    fontsize=11,
    labelpad=10,
)
# Padding ticks away to prevent collision with right axis top tick
ax_dohp.tick_params(axis='x', pad=6)

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

plt.title(
    'Fuel Cell UAV: Power Hybridization Envelope ($DoH_P$)',
    fontweight='bold',
    fontsize=13,
    pad=38,
)

plt.figtext(
    0.5,
    0.015,
    f'Held Constant: Power-Pack Mass Budget = {uav.M_power_pack:.1f} kg  |  '
    f'Buffer Battery Capacity = {E_BATT_FIXED:.1f} kWh  |  '
    f'Motor Takeoff Power = {P_EM_TAKEOFF:.0f} kW  |  MTOW = {uav.MTOW:.0f} kg',
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

# Extra right margin to guarantee zero overlap with secondary tick labels
fig.tight_layout(rect=[0.02, 0.05, 0.96, 0.93])

plot_filename = 'plot_fc_dohp_tradeoff_pareto.png'
plt.savefig(plot_filename, dpi=300)
plt.close(fig)
print(f'--> Updated FC Power Hybridization Pareto curve saved as {plot_filename}')