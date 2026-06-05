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


## Running

### MATLAB

Requires R2020a or later (uses `exportgraphics`). Place
`MATLAB_simulation.m` in your working directory and run

```matlab
Python_simulation
```

### Python

Requires Python 3.8 or later with `numpy` and `matplotlib`. From a shell:

```bash
python ME944_simulation.py
```

## Reproducing the paper figures

Running either script regenerates figures and their interpretation are discussed.

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
