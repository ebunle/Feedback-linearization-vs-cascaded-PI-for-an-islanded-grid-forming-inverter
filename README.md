# Feedback Linearization and Control of a Grid-Forming Power Converter in an Islanded Microgrid

Code and simulation results accompanying the tutorial paper
*Feedback Linearization and Control of a Grid-Forming Power Converter
in an Islanded Microgrid*.

This package implements a full-state feedback-linearizing controller for the
four-state islanded grid-forming inverter model in the synchronous dq frame,
and compares it against the cascaded PI baseline.

**Paper:** [arXiv:XXXX.XXXXX](https://arxiv.org/abs/XXXX.XXXXX)
<!-- Replace XXXX.XXXXX with your arXiv ID once the submission is processed -->

**Codes:** [![DOI](https://zenodo.org/badge/1259954426.svg)](https://doi.org/10.5281/zenodo.20559223)

## Contents

| File | Description |
| --- | --- |
| `FL_microgrid_tutorial` | Full tutorial paper. |
| `MATLAB_simulation.m` | MATLAB simulation of both controllers under three scenarios. |
| `Python_simulation.py` | Python equivalent script. |
| `fig_scenario1_reference` | Reference tracking response ($v_{dq}$, $i_{dq}$, $P$, $Q$). |
| `fig_scenario2_load` | Load step rejection response. |
| `fig_scenario3_param` | Parameter mistune response. |


## Running

### MATLAB

Requires R2020a or later (uses `exportgraphics`). Place
`MATLAB_simulation.m` in your working directory and run

```matlab
MATLAB_simulation
```

### Python

Requires Python 3.8 or later with `numpy` and `matplotlib`. From a shell:

```bash
python Python_simulation.py
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
  author        = {Ebunle Akupan, Rene and Thein, May-Win},
  title         = {Feedback Linearization and Control of a Grid-Forming
                   Power Converter in an Islanded Microgrid},
  year          = {2026},
  eprint        = {XXXX.XXXXX},
  archivePrefix = {arXiv},
  primaryClass  = {eess.SY},
  url           = {https://arxiv.org/abs/XXXX.XXXXX}
}
```

Plain-text (IEEE style):

> R. Ebunle Akupan and M.-W. Thein, "Feedback Linearization and Control
> of a Grid-Forming Power Converter in an Islanded Microgrid," arXiv
> preprint arXiv:XXXX.XXXXX, 2026. [Online]. Available:
> https://arxiv.org/abs/XXXX.XXXXX

---

### Citing the code

```bibtex
@software{EbunleAkupan2026_MATLAB,
  author    = {Ebunle Akupan, Rene and Thein, May-Win},
  title     = {{Feedback Linearization of an Islanded Grid-Forming
                Inverter -- MATLAB/PYTHON Simulation}},
  year      = {2026},
  publisher = {Zenodo},
  doi       = {10.5281/zenodo.20559223},
  url       = {https://doi.org/10.5281/zenodo.20559223},
  version   = {1.0.0}
}
```

OR:

> R. Ebunle Akupan and M.-W. Thein, "Feedback Linearization of an
> Islanded Grid-Forming Inverter -- MATLAB/PYTHON Simulation," Zenodo, 2026,
> ver. 1.0.0. doi: 10.5281/zenodo.20559223.

---

