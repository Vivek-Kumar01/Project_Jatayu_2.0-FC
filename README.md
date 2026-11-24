# PEM Fuel Cell Hybrid MALE UAV Mission Simulation

Time-marching 11-phase point-mass flight dynamics and powertrain simulation for a **1,000 kg tactical MALE UAV** powered by a **60.0 kW PEM Fuel Cell stack**, an **8.0 kWh high-power buffer battery**, and a **105.0 kW electric traction drive**.

## Key Specifications
* **Airframe:** 1,000.0 kg MTOW | 18.0 m wingspan ($AR = 14.73$) | $22.0\text{ m}^2$ wing area | $C_{D0} = 0.0254$ (includes belly radiator cooling drag penalty) | $(L/D)_{\max} = 17.80$
* **Powerplant:** 60.0 kW PEMFC stack + Balance-of-Plant compressor suite + boost DC-DC converter (62.70 kg total) + 105.0 kW PMSM & inverter (35.00 kg)
* **Energy Storage:** 17.24 kg compressed $\text{H}_2$ at 700 bar (16.12 kg mission + 1.12 kg reserve) in a 296.23 kg Type IV composite tank (5.50% gravimetric index) + 8.0 kWh Li-ion buffer pack (53.33 kg, 10C surge / 4C continuous)
* **Energy Management Strategy:**
  * **Takeoff & Pull-Up:** 100% altitude-derated net FC + 10C buffer battery surge ($\le 80\text{ kW}$).
  * **Climb & Cruise-Climb:** 100% net FC + 40% continuous battery assist (12.80 kW).
  * **Cruise & Loiter:** Load-following PEMFC alone; buffer battery floats at standby ($\text{DoH} = 0$).
  * **Descent, Flare & Rollout:** 6.0% gross FC flight idle (3.60 kW) for membrane hydration and avionics.

## Quick Start

### 1. Install Dependencies
Requires Python 3.8+ with `ambiance` (ICAO 1993 Standard Atmosphere) and `matplotlib`:
```bash
pip install ambiance numpy matplotlib
```

### 2. Run the Baseline Simulation
Execute your main mission runner script from the repository root:
```bash
python main.py
```
*(Replace `main.py` with your top-level entry script filename if different.)*

### 3. Plug-and-Play Custom Inputs
* **`jatayu_uav.py`**: Stack rating, cathode compressor altitude parasitics, polarization efficiency curve, cooling drag increment, and motor/converter efficiencies.
* **`mission_settings.py`**: Hydrogen tank mass and gravimetric efficiency, buffer battery capacity/C-rates, and mission altitudes/distances.

## Baseline Benchmark Results
| Metric | Simulated Value |
| :--- | :--- |
| Takeoff Ground Roll / 15 m Obstacle | 214.38 m / 347.00 m |
| Total Time-to-Climb (0 to 3,000 m) | 23.42 min |
| On-Station Loiter Endurance | **5.18 h** (Mean Stack Efficiency: 53.79%) |
| Total Mission Endurance | **7.80 h** (100% combustion-free silent flight) |
| Mission Hydrogen Consumed / Final SoC | 15.73 kg $\text{H}_2$ / 20.71% SoC |
| Touchdown Gross Mass / Landing Roll | 984.27 kg / 112.25 m |
