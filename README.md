# Tutorial Project: Feedback Linearization of an Islanded Grid-Forming Inverter

Code and simulation results accompanying the report *Full-State Feedback
Linearization of an Islanded Grid-Forming Inverter with Resistive Load*.

This package implements a full-state feedback-linearizing controller for the
four-state islanded grid-forming inverter model in the synchronous dq frame,
and compares it against the cascaded PI baseline of Guzman, Madrigal, and
Melgoza-Vázquez (*Electricity*, 2025).


## Contents

| File | Description |
| --- | --- |
| `ME944_Report_FL_final.md`, `.docx` | Full report with abstract, six sections, and references. |
| `ME944_simulation.m` | MATLAB simulation of both controllers under three scenarios. |
| `ME944_simulation.py` | Python equivalent of the MATLAB script. Identical logic and outputs. |
| `fig_scenario1_reference.png`, `.pdf` | Figure 1: reference tracking response (vdq, idq, P, Q). |
| `fig_scenario2_load.png`, `.pdf` | Figure 2: load step rejection response. |
| `fig_scenario3_param.png`, `.pdf` | Figure 3: parameter mistune response. |


## What the simulation does

The simulation runs three scenarios on the four-state plant in equations (4)
of the report. In each scenario both controllers (FL and cascaded PI) start
from the plant's true steady state at $v_d^* = 359$ V with a 20 MW resistive
load, and a single disturbance event occurs at $t = 5$ ms. Each scenario
produces one figure showing the four plant states $(v_d, v_q, i_d, i_q)$
together with active and reactive load power $(P, Q)$ computed from the state
through the standard dq power relations.

| Scenario | Event at t = 5 ms |
| --- | --- |
| 1, reference tracking | d-axis voltage reference steps 359 → 320 V |
| 2, load step rejection | load resistance halves, 9.67 → 4.84 mΩ |
| 3, parameter mistune | FL controller's belief of $R_f$ steps 0.76 → 1.14 mΩ |


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
| Reference voltage $v_d^*$ | 359 V peak (= 440 V RMS L-L) |
| Inner PI gains $(k_{pi}, k_{ii})$ | (0.6176, 2419.9) |
| Outer PI gains $(k_{pv}, k_{iv})$ | (10.72, 4195) |

The resistive load $R_{load} = 9.67$ mΩ is sized to absorb the full inverter
rating of 20 MW at the nominal voltage. The original work in [Guzman 2025]
uses an RL load through a 23/0.44 kV transformer; the resistive simplification
is the scope of the present report and is documented in Section 4.

The feedback-linearizing controller places its closed-loop poles at
$\omega_n = 2\pi \times 500$ rad/s with damping $\zeta = 0.707$, giving
$k_0 = 9.87 \times 10^6$ and $k_1 = 4444$ in equation (22) of the report.


## Reproducing the report figures

Running either script regenerates the three figures used in Section 5 of
the report exactly. The figures and their interpretation are discussed at
length in Sections 5.1, 5.2, and 5.3.


## Numerical method

The plant is integrated with forward Euler at a fixed step of 1 μs. This
is small enough relative to the fastest closed-loop time constant
($1/\omega_n \approx 320$ μs for FL, longer for PI) that integration error
is well below all reported metrics. For longer simulations or more aggressive
gains, a Runge-Kutta integrator would be more appropriate.


## Initial conditions

Both controllers start from the plant's true steady state at the initial
operating point. For the PI controller, the four integrator states are
pre-loaded to the values that hold the operating point so that the cascade
begins in equilibrium rather than recovering from a cold start. The
algebraic relations used for this preloading are visible in `run_scenario`
in both scripts (lines around `xi_id`, `xi_iq`, `xi_vd`, `xi_vq`).


## Reference

Guzman, G., Madrigal, M., and Melgoza-Vázquez, E. *Grid-forming inverters
for frequency support in power grids.* Electricity, vol. 6, no. 4, p. 65,
2025.
