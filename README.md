# Feedback Linearization of an Islanded Grid-Forming Inverter

Code and simulation results accompanying the tutorial paper
*Feedback Linearization and Control of a Grid-Forming Power Converter
in an Islanded Microgrid*.

This package implements a full-state feedback-linearizing controller for the
four-state islanded grid-forming inverter model in the synchronous dq frame,
and compares it against the cascaded PI baseline.

**Paper:** [arXiv:XXXX.XXXXX](https://arxiv.org/abs/XXXX.XXXXX)
<!-- Replace XXXX.XXXXX with your arXiv ID once the submission is processed -->

**MATLAB code:** [GitHub](https://github.com/YOUR-USERNAME/YOUR-REPO/blob/main/ME944_simulation.m)
<!-- Replace YOUR-USERNAME/YOUR-REPO with your actual GitHub repository path -->

**Python code:** [GitHub](https://github.com/YOUR-USERNAME/YOUR-REPO/blob/main/ME944_simulation.py)
<!-- Replace YOUR-USERNAME/YOUR-REPO with your actual GitHub repository path -->


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

Running either script regenerates the figures.


## How to cite

If this work is useful to you, please cite the paper and the code
separately, as described below.

---

### Citing the paper

Once the arXiv preprint is live, use the following BibTeX entry
(replace `XXXX.XXXXX` with the actual arXiv ID):

```bibtex
@misc{EbunleAkupan2026_FL,
  author       = {Ebunle Akupan, Rene},
  title        = {Feedback Linearization and Control of a Grid-Forming
                  Power Converter in an Islanded Microgrid},
  year         = {2026},
  eprint       = {XXXX.XXXXX},
  archivePrefix = {arXiv},
  primaryClass = {eess.SY},
  url          = {https://arxiv.org/abs/XXXX.XXXXX}
}
```

Plain-text (IEEE style):

> R. Ebunle Akupan, "Feedback Linearization and Control of a
> Grid-Forming Power Converter in an Islanded Microgrid," arXiv preprint
> arXiv:XXXX.XXXXX, 2026. [Online]. Available:
> https://arxiv.org/abs/XXXX.XXXXX

---

### Citing the MATLAB code

The MATLAB simulation is archived on Zenodo with a permanent DOI.
Use the following BibTeX entry (replace `XXXXXXX` with the actual
Zenodo record ID after you deposit the code):

```bibtex
@software{EbunleAkupan2026_MATLAB,
  author       = {Ebunle Akupan, Rene},
  title        = {{Feedback Linearization of an Islanded Grid-Forming
                  Inverter -- MATLAB Simulation}},
  year         = {2026},
  publisher    = {Zenodo},
  doi          = {10.5281/zenodo.XXXXXXX},
  url          = {https://doi.org/10.5281/zenodo.XXXXXXX},
  version      = {1.0.0}
}
```

Plain-text (IEEE style):

> R. Ebunle Akupan, "Feedback Linearization of an Islanded Grid-Forming
> Inverter -- MATLAB Simulation," Zenodo, 2026, ver. 1.0.0.
> doi: 10.5281/zenodo.XXXXXXX.

---

### Citing the paper

Once the arXiv preprint is live, use the following BibTeX entry
(replace `XXXX.XXXXX` with the actual arXiv ID):

```bibtex
@misc{EbunleAkupan2026_FL,
  author       = {Ebunle Akupan, Rene and Thein, May-Win},
  title        = {Feedback Linearization and Control of a Grid-Forming
                  Power Converter in an Islanded Microgrid},
  year         = {2026},
  eprint       = {XXXX.XXXXX},
  archivePrefix = {arXiv},
  primaryClass = {eess.SY},
  url          = {https://arxiv.org/abs/XXXX.XXXXX}
}
```

Plain-text (IEEE style):

> R. Ebunle Akupan and M.-W. Thein, "Feedback Linearization and Control of a
> Grid-Forming Power Converter in an Islanded Microgrid," arXiv preprint
> arXiv:XXXX.XXXXX, 2026. [Online]. Available:
> https://arxiv.org/abs/XXXX.XXXXX

---

### Citing the MATLAB code

The MATLAB simulation is archived on Zenodo with a permanent DOI.
Use the following BibTeX entry (replace `XXXXXXX` with the actual
Zenodo record ID after you deposit the code):

```bibtex
@software{EbunleAkupan2026_MATLAB,
  author       = {Ebunle Akupan, Rene and Thein, May-Win},
  title        = {{Feedback Linearization of an Islanded Grid-Forming
                  Inverter -- MATLAB Simulation}},
  year         = {2026},
  publisher    = {Zenodo},
  doi          = {10.5281/zenodo.XXXXXXX},
  url          = {https://doi.org/10.5281/zenodo.XXXXXXX},
  version      = {1.0.0}
}
```

Plain-text (IEEE style):

> R. Ebunle Akupan and M.-W. Thein, "Feedback Linearization of an Islanded Grid-Forming
> Inverter -- MATLAB Simulation," Zenodo, 2026, ver. 1.0.0.
> doi: 10.5281/zenodo.XXXXXXX.

---

### Citing the Python code

The Python simulation is archived on Zenodo with a separate permanent
DOI (replace `YYYYYYY` with the actual Zenodo record ID):

```bibtex
@software{EbunleAkupan2026_Python,
  author       = {Ebunle Akupan, Rene and Thein, May-Win},
  title        = {{Feedback Linearization of an Islanded Grid-Forming
                  Inverter -- Python Simulation}},
  year         = {2026},
  publisher    = {Zenodo},
  doi          = {10.5281/zenodo.YYYYYYY},
  url          = {https://doi.org/10.5281/zenodo.YYYYYYY},
  version      = {1.0.0}
}
```

Plain-text (IEEE style):

> R. Ebunle Akupan and M.-W. Thein, "Feedback Linearization of an Islanded Grid-Forming
> Inverter -- Python Simulation," Zenodo, 2026, ver. 1.0.0.
> doi: 10.5281/zenodo.YYYYYYY.

---

### Getting a Zenodo DOI for the code

If you have not yet deposited the code on Zenodo, the recommended
workflow is:

1. Go to [zenodo.org](https://zenodo.org) and log in with your GitHub
   account.
2. Under **GitHub** in your Zenodo settings, enable the repository
   containing this code.
3. Create a versioned release on GitHub (e.g., tag `v1.0.0`). Zenodo
   will automatically archive it and issue a DOI.
4. Copy the DOI from the Zenodo record page and replace the placeholder
   `10.5281/zenodo.XXXXXXX` above.

You can archive the MATLAB and Python scripts in the same repository
and Zenodo record, or in separate records if you prefer separate DOIs
for each. Either is acceptable; separate records make it easier for
users to cite only the language they used.


## Reference

Guzman, G., Madrigal, M., and Melgoza-Vázquez, E. Grid-forming inverters
for frequency support in power grids. *Electricity*, vol. 6, no. 4,
p. 65, 2025.
