import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import jatayu_fc_uav as uav
import mission_settings_fc as ms

# ------------------------------------------------------------------------------
# 1. SEQUENTIAL PHASE EXECUTION
# ------------------------------------------------------------------------------
print("\n" + "=" * 80)
print("       PROJECT JATAYU 2.0 - FC: FULL MISSION PROFILE SIMULATION")
print("=" * 80 + "\n")

import Phases.takeoff as to
import Phases.pullup as pu
import Phases.climb as cl
import Phases.cruise_climb as cc
import Phases.cruise_out as co
import Phases.loiter as lo
import Phases.cruise_in as ci
import Phases.descent as ds
import Phases.approach as ap
import Phases.flare as fl
import Phases.touchdown as td

# ------------------------------------------------------------------------------
# 2. EXPORT RAW TELEMETRY TO CSV
# ------------------------------------------------------------------------------
df = pd.DataFrame(ms.telemetry)
df['time_hr'] = df['time_s'] / 3600.0
df['time_min'] = df['time_s'] / 60.0

csv_filename = 'mission_telemetry_fc_complete.csv'
df.to_csv(csv_filename, index=False)
print(f"\n--> Exported {len(df)} simulation steps to {csv_filename}")

# ------------------------------------------------------------------------------
# 3. GENERATE SEGMENTED MISSION TELEMETRY PLOTS
# ------------------------------------------------------------------------------
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 9

segments = [
    {
        'title': 'Part 1: Takeoff Ground Roll & Obstacle Pull-Up',
        'filename': 'plot_part1_takeoff_pullup.png',
        'phases': ['Takeoff', 'Pull_Up'],
        'time_col': 'time_s',
        'time_label': 'Elapsed Time [Seconds]'
    },
    {
        'title': 'Part 2: Climb, Cruise-Climb & Cruise-Out',
        'filename': 'plot_part2_climb_transit_out.png',
        'phases': ['Climb', 'Cruise_Climb', 'Cruise_Out'],
        'time_col': 'time_min',
        'time_label': 'Elapsed Time [Minutes]'
    },
    {
        'title': 'Part 3: Sustained On-Station Loiter',
        'filename': 'plot_part3_sustained_loiter.png',
        'phases': ['Loiter'],
        'time_col': 'time_hr',
        'time_label': 'Elapsed Time [Hours]'
    },
    {
        'title': 'Part 4: Cruise-In, Descent & Initial Approach',
        'filename': 'plot_part4_cruisein_descent_approach.png',
        'phases': ['Cruise_In', 'Descent', 'Approach_Decel', 'Approach_Glide'],
        'time_col': 'time_min',
        'time_label': 'Elapsed Time [Minutes]'
    },
    {
        'title': 'Part 5: Final Approach, Flare & Braking Rollout',
        'filename': 'plot_part5_flare_touchdown.png',
        'phases': ['Final_Approach', 'Flare_Touchdown', 'Landing_Rollout', 'Mission_Complete'],
        'time_col': 'time_s',
        'time_label': 'Elapsed Time [Seconds]'
    }
]

for seg in segments:
    sub_df = df[df['phase'].isin(seg['phases'])].copy()
    if sub_df.empty:
        print(f"Skipping empty segment: {seg['title']}")
        continue

    # Zero-base elapsed time for this specific segment
    t0 = sub_df[seg['time_col']].iloc[0]
    t = sub_df[seg['time_col']] - t0
    sub_df['t_plot'] = t

    # 5-subplot figure to explicitly include Load Ratio and Stack Efficiency
    fig, axs = plt.subplots(5, 1, figsize=(10.5, 13.0), sharex=True, dpi=300)
    plt.subplots_adjust(hspace=0.26)

    # Subplot A: Flight Trajectory (Altitude & True Airspeed)
    ax1 = axs[0]
    ax1.plot(t, sub_df['altitude_m'], color='#1F78B4', lw=1.8, label='Altitude (m)')
    ax1.set_ylabel('Altitude [m MSL]', color='#1F78B4', fontweight='bold')
    ax1.grid(True, linestyle=':', alpha=0.6)

    ax1_t = ax1.twinx()
    ax1_t.plot(t, sub_df['velocity_ms'], color='#6A3D9A', lw=1.4, linestyle='--', label='Airspeed (m/s)')
    ax1_t.set_ylabel('TAS [m/s]', color='#6A3D9A', fontweight='bold')
    ax1.set_title(f"{seg['title']} — Flight Trajectory", fontweight='bold', loc='left')

    # Subplot B: Power Demand & Fuel Cell Net/Gross Output
    ax2 = axs[1]
    ax2.plot(t, sub_df['P_req_kW'], color='black', lw=1.2, linestyle=':', label='P_req (Aero Drag)')
    ax2.plot(t, sub_df['P_FC_gross_kW'], color='#FF7F00', lw=1.5, linestyle='--', label='FC Gross Generation')
    ax2.plot(t, sub_df['P_FC_net_kW'], color='#008080', lw=1.8, label='FC Net Bus Power')
    ax2.plot(t, sub_df['P_EM_kW'], color='#33A02C', lw=1.5, label='Motor Shaft Power')
    ax2.plot(t, sub_df['P_BoP_kW'], color='#E31A1C', lw=1.2, linestyle='-.', label='BoP Parasitic Draw')
    ax2.set_ylabel('Power [kW]', fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', framealpha=0.9, fontsize=7.5, ncol=3)
    ax2.set_title('Aerodynamic Demand vs. Powertrain Dispatch', fontweight='bold', loc='left')

    # Subplot C: Fuel Cell Operating Point (Load Fraction xi and Stack Efficiency eta_FC)
    ax3 = axs[2]
    ax3.plot(t, sub_df['xi_FC'] * 100.0, color='#8C564B', lw=1.6, label='Load Ratio (ξ) [%]')
    ax3.set_ylabel('Load Ratio ξ [%]', color='#8C564B', fontweight='bold')
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.set_ylim(0, 110)

    ax3_t = ax3.twinx()
    ax3_t.plot(t, sub_df['eta_FC'] * 100.0, color='#2CA02C', lw=1.8, linestyle='-', label='Stack Efficiency (LHV)')
    ax3_t.set_ylabel('Stack Eff η_FC [%]', color='#2CA02C', fontweight='bold')
    ax3_t.set_ylim(45, 65)
    ax3.set_title('PEMFC Dynamic Polarization Regime (Load vs. Efficiency)', fontweight='bold', loc='left')

    # Subplot D: Energy Storage Depletion Profile (H2 Mass & Buffer Battery SoC)
    ax4 = axs[3]
    ax4.plot(t, sub_df['battery_soc'] * 100.0, color='#1F78B4', lw=1.8, label='Battery SoC [%]')
    ax4.set_ylabel('Battery SoC [%]', color='#1F78B4', fontweight='bold')
    ax4.grid(True, linestyle=':', alpha=0.6)

    ax4_t = ax4.twinx()
    ax4_t.plot(t, sub_df['H2_mass_kg'], color='#D95F02', lw=1.8, linestyle='--', label='Remaining H2 [kg]')
    ax4_t.set_ylabel('Stored H2 [kg]', color='#D95F02', fontweight='bold')
    ax4.set_title('Energy Storage Depletion Profile', fontweight='bold', loc='left')

    # Subplot E: Battery Power Dispatch (Assist vs. Standby)
    ax5 = axs[4]
    ax5.plot(t, sub_df['P_bat_kW'], color='#6A3D9A', lw=1.4, label='Battery Net Power')
    ax5.fill_between(t, 0, sub_df['P_bat_kW'], where=(sub_df['P_bat_kW'] > 0),
                     color='#E31A1C', alpha=0.3, label='Discharge Assist Mode')
    ax5.fill_between(t, 0, sub_df['P_bat_kW'], where=(sub_df['P_bat_kW'] <= 0),
                     color='#33A02C', alpha=0.15, label='Standby / Floating (0 kW)')
    ax5.axhline(0, color='black', lw=0.8)
    ax5.set_ylabel('Battery Power [kW]', fontweight='bold')
    ax5.set_xlabel(seg['time_label'], fontweight='bold')
    ax5.grid(True, linestyle=':', alpha=0.6)
    ax5.legend(loc='upper right', framealpha=0.9, fontsize=8)
    ax5.set_title('Battery Operating Regimes (+ Discharge Assist, 0 Floating)', fontweight='bold', loc='left')

    # Expand Subplot A's upper Y limit slightly to make room for phase banners
    y_min, y_max = ax1.get_ylim()
    ax1.set_ylim(y_min, y_max * 1.18 if y_max > 0 else 100)

    # --------------------------------------------------------------------------
    # PHASE INTERVAL CENTERING, BOUNDARY LINES & STAGGERED LABELS
    # --------------------------------------------------------------------------
    sub_df['phase_block'] = (sub_df['phase'] != sub_df['phase'].shift(1)).cumsum()
    blocks = sub_df.groupby('phase_block')

    stagger_toggle = 0
    for _, block in blocks:
        phase_name = block['phase'].iloc[0]
        t_start = block['t_plot'].iloc[0]
        t_end = block['t_plot'].iloc[-1]
        t_mid = (t_start + t_end) / 2.0

        if t_start > 0:
            for ax in axs:
                ax.axvline(x=t_start, color='#666666', linestyle='--', linewidth=1.1, alpha=0.75)

        clean_phase_name = phase_name.replace('_', ' ')
        y_pos = 0.93 if (stagger_toggle % 2 == 0) else 0.82
        stagger_toggle += 1

        ax1.annotate(
            clean_phase_name,
            xy=(t_mid, y_pos),
            xycoords=('data', 'axes fraction'),
            ha='center',
            va='center',
            fontsize=7.5,
            fontweight='bold',
            color='#111111',
            bbox=dict(boxstyle='round,pad=0.25', facecolor='#EEEEEE', edgecolor='#888888', alpha=0.88, lw=0.8)
        )

    plt.tight_layout()
    plt.savefig(seg['filename'], dpi=300)
    plt.close(fig)
    print(f"--> Saved segment plot: {seg['filename']}")

# ------------------------------------------------------------------------------
# 4. GENERATE DEDICATED FULL-MISSION POLARIZATION & EFFICIENCY AUDIT PLOT
# ------------------------------------------------------------------------------
fig, (ax_p1, ax_p2) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=300)

# Panel 1: Efficiency vs Mission Time
ax_p1.plot(df['time_hr'], df['eta_FC'] * 100.0, color='#2CA02C', lw=1.8, label='PEMFC Stack Efficiency (LHV)')
ax_p1.set_xlabel('Mission Time [Hours]', fontweight='bold')
ax_p1.set_ylabel('Stack Thermodynamic Efficiency [%]', color='#2CA02C', fontweight='bold')
ax_p1.set_ylim(45, 65)
ax_p1.grid(True, linestyle=':', alpha=0.6)

ax_p1_twin = ax_p1.twinx()
ax_p1_twin.plot(df['time_hr'], df['xi_FC'] * 100.0, color='#8C564B', lw=1.2, linestyle='--', label='Load Ratio (ξ)')
ax_p1_twin.set_ylabel('Load Ratio ξ = P_gross / P_gross_avail [%]', color='#8C564B', fontweight='bold')
ax_p1_twin.set_ylim(0, 110)
ax_p1.set_title('(A) Fuel Cell Operating Regimes Across Full Mission', fontweight='bold', loc='left')

# Panel 2: In-Flight Operating Points on Empirical Polynomial Curve
xi_curve = np.linspace(0.05, 1.0, 200)
eta_curve = (-0.22 * (xi_curve ** 2) + 0.13 * xi_curve + 0.58) * 100.0

ax_p2.plot(xi_curve * 100.0, eta_curve, 'k-', lw=2.0, label='Empirical Curve: $\eta = -0.22\\xi^2 + 0.13\\xi + 0.58$')
scatter = ax_p2.scatter(
    df['xi_FC'] * 100.0, df['eta_FC'] * 100.0,
    c=df['altitude_m'], cmap='viridis', s=18, alpha=0.7, edgecolors='none', label='Simulated Steps'
)
cbar = plt.colorbar(scatter, ax=ax_p2)
cbar.set_label('Altitude [m MSL]', fontweight='bold')

ax_p2.set_xlabel('Normalized Load Ratio ξ [%]', fontweight='bold')
ax_p2.set_ylabel('Stack Efficiency [%]', fontweight='bold')
ax_p2.set_ylim(45, 65)
ax_p2.set_xlim(0, 105)
ax_p2.grid(True, linestyle=':', alpha=0.6)
ax_p2.legend(loc='lower left', fontsize=8)
ax_p2.set_title('(B) In-Flight Operating Points on Polarization Curve', fontweight='bold', loc='left')

plt.tight_layout()
plt.savefig('plot_mission_fc_efficiency.png', dpi=300)
plt.close(fig)
print("--> Saved complete polarization audit plot: plot_mission_fc_efficiency.png")

print('\nAll mission telemetry and efficiency figures successfully generated!')