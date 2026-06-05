# Feedback Linearization of an Islanded Grid-Forming Inverter

Code and simulation results accompanying the tutorial paper
*Feedback Linearization and Control of a Grid-Forming Power Converter
in an Islanded Microgrid*.

This package implements a full-state feedback-linearizing controller for the
four-state islanded grid-forming inverter model in the synchronous dq frame,
and compares it against the cascaded PI baseline.


## Contents

| File | Description |
| --- | --- |
| `FL_microgrid_tutorial` | Full tutorial paper. |
| `MATLAB_simulation.m` | MATLAB simulation of both controllers under three scenarios. |
| `Python_simulation.py` | Python equivalent script.. |
| `fig_scenario1_reference` | Figure 2 in the paper: reference tracking response ($v_{dq}$, $i_{dq}$, $P$, $Q$). |
| `fig_scenario2_load` | Figure 3 in the paper: load step rejection response. |
| `fig_scenario3_param` | Figure 4 in the paper: parameter mistune response. |


## What the simulation does

The simulation runs three scenarios on the four-state plant in
equations (5)--(8) of the paper. In each scenario both controllers
(FL and cascaded PI) start from the plant's true steady state at
$v_d^* = 359$ V with a 20 MW resistive load, and a single disturbance
event occurs at $t = 5$ ms. Each scenario produces one figure showing
the four plant states $(v_d, v_q, i_d, i_q)$ together with active and
reactive load power $(P, Q)$ computed from the state through the
standard dq power relations.

| Scenario | Event at $t = 5$ ms | Paper figure |
| --- | --- | --- |
| 1, reference tracking | $d$-axis voltage reference steps 359 → 320 V | Figure 2 |
| 2, load step rejection | load resistance halves, 9.67 → 4.84 mΩ | Figure 3 |
| 3, parameter mistune | FL controller's belief of $R_f$ steps 0.76 → 1.14 mΩ | Figure 4 |

Note: Figure 1 in the paper is the model and control structure diagram,
which is not produced by the simulation scripts.


## Running

### MATLAB

Requires R2020a or later (uses `exportgraphics`). Place
`ME944_simulation.m` in your working directory and run

```matlab
ME944_simulation
```

The script prints quantitative metrics to the command window and writes
six output files: `fig_scenarioN_*.{png,pdf}` for N = 1, 2, 3.

### Python

Requires Python 3.8 or later with `numpy` and `matplotlib`. From a shell:

```bash
python ME944_simulation.py
```

Same output files, same metrics printed to stdout. The Python and MATLAB
versions produce numerically identical results.


## Plant parameters

All plant parameters and PI controller gains are taken from Guzman et al.
(2025), Table 1:

| Parameter | Value |
| --- | --- |
| Filter inductance $L_f$ | 0.079 mH |
| Filter resistance $R_f$ | 0.76 mΩ |
| Filter capacitance $C_f$ | 13.7 mF |
| Commanded frequency $\omega^*$ | $2\pi \times 60$ rad/s |
| Reference voltage $v_d^*$ | 359 V peak (= 440 V RMS line-to-line) |
| Inner PI gains $(k_{pi},\, k_{ii})$ | (0.6176, 2419.9) |
| Outer PI gains $(k_{pv},\, k_{iv})$ | (10.72, 4195) |

The resistive load $R_{\mathrm{load}} = 9.67$ mΩ is sized to absorb the
full inverter rating of 20 MW at the nominal voltage. The original work
of Guzman et al. (2025) uses an RL load through a 23/0.44 kV transformer;
the resistive simplification is the scope of the present paper and is
documented in Section 4.

The feedback-linearizing controller places its closed-loop poles at
$\omega_n = 2\pi \times 500$ rad/s with damping $\zeta = 0.707$, giving
$k_0 = 9.87 \times 10^6$ and $k_1 = 4444$ from equation (28) of the paper.


## Reproducing the paper figures

Running either script regenerates Figures 2, 3, and 4 used in Section 5
of the paper exactly. The figures and their interpretation are discussed
in Sections 5.1, 5.2, and 5.3 respectively.


## Numerical method

The plant is integrated with forward Euler at a fixed step of 1 μs. This
is small enough relative to the fastest closed-loop time constant
($1/\omega_n \approx 320$ μs for FL, longer for PI) that integration
error is well below all reported metrics. For longer simulations or more
aggressive gains, a Runge-Kutta integrator would be more appropriate.


## Initial conditions

Both controllers start from the plant's true steady state at the initial
operating point. For the PI controller, the four integrator states are
pre-loaded to the values that hold the operating point so that the
cascade begins in equilibrium rather than recovering from a cold start.
The algebraic relations used for this preloading are visible in
`run_scenario` in both scripts (lines around `xi_id`, `xi_iq`, `xi_vd`,
`xi_vq`).


## Reference

Guzman, G., Madrigal, M., and Melgoza-Vázquez, E. Grid-forming inverters
for frequency support in power grids. *Electricity*, vol. 6, no. 4,
p. 65, 2025.
