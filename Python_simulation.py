"""
% Full-state feedback linearization vs cascaded PI for an islanded
% grid-forming inverter with resistive load.
%
% Plant parameters and PI gains: Guzman, Madrigal, Melgoza-Vazquez (2025),
% Table 1.  A balanced resistive load is sized to draw the 20 MW rating
% at the 440 V RMS line-line operating point.
%
% Three scenarios, each on its own figure showing v_dq, i_dq, P, Q:
%   (1) Reference tracking:  v_d* steps 359 -> 320 V at t = 5 ms
%   (2) Load step rejection: R_load 9.67 -> 4.84 mohm at t = 5 ms
%   (3) Parameter mistune:   FL belief of R_f 0.76 -> 1.14 mohm at t = 5 ms

Requires: numpy, matplotlib.
"""

from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt


# ==================================================================
# Plant, controllers, simulation parameters
# ==================================================================

@dataclass
class Plant:
    """Islanded inverter plant parameters ."""
    Lf: float = 0.079e-3          # filter inductance (H)
    Cf: float = 13.7e-3           # filter capacitance (F)
    Rf: float = 0.76e-3           # filter resistance (Ohm)
    Rload: float = 9.67e-3        # 9.67 mOhm -> 20 MW at 359 V peak L-N
    wstar: float = 2 * np.pi * 60 # commanded frequency (rad/s)
    vd_ref: float = 359.0         # 440 V RMS L-L -> 359 V peak L-N
    vq_ref: float = 0.0


@dataclass
class FL:
    """Feedback-linearizing controller: pole placement on double integrator."""
    wn: float = 2 * np.pi * 500   # closed-loop natural frequency (rad/s)
    zeta: float = 0.707           # damping
    Rf: float = 0.76e-3           # controller's belief about R_f
    Rload: float = 9.67e-3        # controller's belief about R_load

    @property
    def k0(self): return self.wn ** 2

    @property
    def k1(self): return 2 * self.zeta * self.wn


@dataclass
class PI:
    """Cascaded PI controller gains (Guzman et al. 2025, Table 1)."""
    kpi: float = 0.6176
    kii: float = 2419.9
    kpv: float = 10.72
    kiv: float = 4195.0


@dataclass
class Sim:
    tend: float = 0.050           # 50 ms per scenario
    dt: float = 1e-6              # 1 us step


# ==================================================================
# Plant right-hand side
# ==================================================================

def plant_rhs(x, u, P: Plant, Rl: float):
    """Continuous-time plant dynamics: xdot = f(x, u; P, Rl).

    x = [i_d, i_q, v_d, v_q]^T
    u = [e_d, e_q]^T
    Rl = current load resistance (may differ from P.Rload during scenarios).
    """
    id_, iq, vd, vq = x
    ed, eq = u
    return np.array([
        (-P.Rf * id_ + P.wstar * P.Lf * iq + ed - vd) / P.Lf,
        (-P.Rf * iq - P.wstar * P.Lf * id_ + eq - vq) / P.Lf,
        (id_ - vd / Rl) / P.Cf + P.wstar * vq,
        (iq - vq / Rl) / P.Cf - P.wstar * vd
    ])


# ==================================================================
# Main integration loop
# ==================================================================

def run_scenario(P: Plant, fl: FL, pi: PI, sim: Sim, scenario: str):
    """Integrate the plant under both controllers for one scenario.

    Returns (t, x_FL, x_PI, Rload_t):
        t       -- time vector (N,)
        x_FL    -- state trajectory under feedback linearization (4, N)
        x_PI    -- state trajectory under cascaded PI (4, N)
        Rload_t -- time-varying load resistance (N,), used by the plotter
    """
    N = int(round(sim.tend / sim.dt))
    t = np.arange(N) * sim.dt
    dt = sim.dt

    # Default profiles
    Rload_t   = np.full(N, P.Rload)
    Rf_belief = np.full(N, fl.Rf)
    vd_ref_t  = np.full(N, P.vd_ref)
    vq_ref_t  = np.full(N, P.vq_ref)

    t_event = 5e-3
    if scenario == 'reference':
        vd_ref_t[t >= t_event] = 320.0
    elif scenario == 'load':
        Rload_t[t >= t_event] = 4.84e-3
    elif scenario == 'param':
        Rf_belief[t >= t_event] = 1.14e-3
    else:
        raise ValueError(f"Unknown scenario: {scenario}")

    # Plant true steady state at (vd_ref, 0):
    #   0 = id - vd_ref/Rload + wstar*Cf*0   -> id_ss = vd_ref/Rload
    #   0 = iq - 0/Rload      - wstar*Cf*vd_ref -> iq_ss = wstar*Cf*vd_ref
    i_d_ss = P.vd_ref / P.Rload
    i_q_ss = P.wstar * P.Cf * P.vd_ref
    x0 = np.array([i_d_ss, i_q_ss, P.vd_ref, 0.0])

    x_FL = np.zeros((4, N)); x_FL[:, 0] = x0
    x_PI = np.zeros((4, N)); x_PI[:, 0] = x0

    # Pre-load PI integrators so the controller starts at the operating point
    e_d_ss = P.vd_ref + P.Rf * i_d_ss - P.wstar * P.Lf * i_q_ss
    e_q_ss = P.Rf * i_q_ss + P.wstar * P.Lf * i_d_ss
    xi_id = (e_d_ss - P.vd_ref + P.wstar * P.Lf * i_q_ss) / pi.kii
    xi_iq = (e_q_ss - 0 - P.wstar * P.Lf * i_d_ss) / pi.kii
    xi_vd = i_d_ss / pi.kiv
    xi_vq = (i_q_ss - P.wstar * P.Cf * P.vd_ref) / pi.kiv

    for k in range(N - 1):
        Rl = Rload_t[k]

        # ---- Feedback-linearizing controller ----
        Rfc = Rf_belief[k]
        Rlc = fl.Rload
        id_, iq, vd, vq = x_FL[:, k]

        # First derivatives of the outputs from the plant (control-free)
        vd_dot = (id_ - vd / Rl) / P.Cf + P.wstar * vq
        vq_dot = (iq - vq / Rl) / P.Cf - P.wstar * vd

        # Second Lie derivatives along F using controller's model parameters
        LF2h1 = ((-Rfc * id_ + P.wstar * P.Lf * iq - vd) / (P.Lf * P.Cf)
                 - vd_dot / (Rlc * P.Cf) + P.wstar * vq_dot)
        LF2h2 = ((-Rfc * iq - P.wstar * P.Lf * id_ - vq) / (P.Lf * P.Cf)
                 - vq_dot / (Rlc * P.Cf) - P.wstar * vd_dot)

        # Virtual control: pole placement on each double integrator
        nu1 = -fl.k1 * vd_dot - fl.k0 * (vd - vd_ref_t[k])
        nu2 = -fl.k1 * vq_dot - fl.k0 * (vq - vq_ref_t[k])

        # Linearizing feedback (eq. 18 in the paper)
        u_FL = np.array([
            P.Lf * P.Cf * (nu1 - LF2h1),
            P.Lf * P.Cf * (nu2 - LF2h2),
        ])
        x_FL[:, k + 1] = x_FL[:, k] + dt * plant_rhs(x_FL[:, k], u_FL, P, Rl)

        # ---- Cascaded PI controller ----
        id_, iq, vd, vq = x_PI[:, k]

        err_vd = vd_ref_t[k] - vd
        err_vq = vq_ref_t[k] - vq
        xi_vd += dt * err_vd
        xi_vq += dt * err_vq
        id_ref = -P.wstar * P.Cf * vq + pi.kpv * err_vd + pi.kiv * xi_vd
        iq_ref = +P.wstar * P.Cf * vd + pi.kpv * err_vq + pi.kiv * xi_vq

        err_id = id_ref - id_
        err_iq = iq_ref - iq
        xi_id += dt * err_id
        xi_iq += dt * err_iq

        u_PI = np.array([
            vd - P.wstar * P.Lf * iq + pi.kpi * err_id + pi.kii * xi_id,
            vq + P.wstar * P.Lf * id_ + pi.kpi * err_iq + pi.kii * xi_iq,
        ])
        x_PI[:, k + 1] = x_PI[:, k] + dt * plant_rhs(x_PI[:, k], u_PI, P, Rl)

    return t, x_FL, x_PI, Rload_t


# ==================================================================
# Plotting
# ==================================================================

def plot_scenario(t, x_FL, x_PI, Rload_t, title, filename):
    """Produce a 3x2 grid of subplots: v_d, v_q on top, i_d, i_q in middle,
    P, Q on the bottom.  P, Q are computed from the state at the PCC with
    the resistive load closure i_g = v / R_load."""
    igd_FL = x_FL[2] / Rload_t
    igq_FL = x_FL[3] / Rload_t
    igd_PI = x_PI[2] / Rload_t
    igq_PI = x_PI[3] / Rload_t

    # Standard dq active and reactive power (eqs. 5 and 6 in the paper)
    P_FL = 1.5 * (x_FL[2] * igd_FL + x_FL[3] * igq_FL)
    Q_FL = -1.5 * (x_FL[2] * igq_FL - x_FL[3] * igd_FL)
    P_PI = 1.5 * (x_PI[2] * igd_PI + x_PI[3] * igq_PI)
    Q_PI = -1.5 * (x_PI[2] * igq_PI - x_PI[3] * igd_PI)

    fig, axs = plt.subplots(3, 2, figsize=(11, 9))
    tms = t * 1e3

    panels = [
        (0, 0, x_FL[2],       x_PI[2],       'v_d (V)',   'd-axis voltage'),
        (0, 1, x_FL[3],       x_PI[3],       'v_q (V)',   'q-axis voltage'),
        (1, 0, x_FL[0],       x_PI[0],       'i_d (A)',   'd-axis current'),
        (1, 1, x_FL[1],       x_PI[1],       'i_q (A)',   'q-axis current'),
        (2, 0, P_FL / 1e6,    P_PI / 1e6,    'P (MW)',    'active power at load'),
        (2, 1, Q_FL / 1e6,    Q_PI / 1e6,    'Q (MVAr)',  'reactive power at load'),
    ]

    for r, c, yFL, yPI, ylabel, ttl in panels:
        ax = axs[r, c]
        ax.plot(tms, yFL, lw=1.6, label='FL')
        ax.plot(tms, yPI, lw=1.6, label='PI')
        ax.axvline(5, color='k', linestyle='--', linewidth=0.8)
        ax.set_ylabel(ylabel)
        ax.set_title(ttl)
        ax.grid(alpha=0.3)
        ax.legend(loc='best', fontsize=9)
        if r == 2:
            ax.set_xlabel('time (ms)')

    fig.suptitle(title, fontsize=13, fontweight='bold')
    fig.tight_layout()
    fig.savefig(f'{filename}.png', dpi=180)
    fig.savefig(f'{filename}.pdf')
    plt.close(fig)


# ==================================================================
# Metrics helpers
# ==================================================================

def settling_time(t, y, yref, band=0.02):
    """Return the time after t[0] at which |y - yref| stays below band*|yref|.
    Returns 0 if already within band, NaN if the signal never settles."""
    err = np.abs(y - yref)
    outside = err > band * abs(yref)
    if not outside.any():
        return 0.0
    last_out = np.where(outside)[0][-1]
    if last_out == len(t) - 1:
        return np.nan
    return t[last_out + 1] - t[0]


def metrics_reference(t, x_FL, x_PI, P: Plant):
    idx = t >= 5e-3
    ts_FL = settling_time(t[idx] - 5e-3, x_FL[2, idx], 320)
    ts_PI = settling_time(t[idx] - 5e-3, x_PI[2, idx], 320)
    vq_FL = np.max(np.abs(x_FL[3, idx]))
    vq_PI = np.max(np.abs(x_PI[3, idx]))
    P_initial = 1.5 * 359.0**2 / P.Rload / 1e6
    P_final   = 1.5 * 320.0**2 / P.Rload / 1e6
    print(f"  Settling time 2%:    FL = {ts_FL*1e3:.2f} ms,  PI = {ts_PI*1e3:.2f} ms")
    print(f"  v_q coupling max:    FL = {vq_FL:.3f} V,     PI = {vq_PI:.3f} V")
    print(f"  P steady state:      before = {P_initial:.2f} MW, after = {P_final:.2f} MW")


def metrics_load(t, x_FL, x_PI, P: Plant):
    idx = t >= 5e-3
    sag_FL = 359 - np.min(x_FL[2, idx])
    sag_PI = 359 - np.min(x_PI[2, idx])
    end_FL = np.mean(x_FL[2, t >= 45e-3]) - 359
    end_PI = np.mean(x_PI[2, t >= 45e-3]) - 359
    P_before = 1.5 * 359.0**2 / P.Rload / 1e6
    P_after  = 1.5 * 359.0**2 / 4.84e-3 / 1e6
    print(f"  v_d sag max:         FL = {sag_FL:.2f} V,     PI = {sag_PI:.2f} V")
    print(f"  v_d offset at end:   FL = {end_FL:.4f} V,     PI = {end_PI:.4f} V")
    print(f"  P steady state:      before = {P_before:.2f} MW, after = {P_after:.2f} MW")


def metrics_param(t, x_FL, x_PI):
    idx = t >= 45e-3
    off_FL = np.mean(x_FL[2, idx]) - 359
    off_PI = np.mean(x_PI[2, idx]) - 359
    print(f"  v_d offset:          FL = {off_FL:.4f} V,     PI = {off_PI:.4f} V")


# ==================================================================
# Main
# ==================================================================

def main():
    P = Plant()
    fl = FL()
    pi = PI()
    sim = Sim()

    print("\n========== Scenario 1: Reference tracking ==========")
    t, x_FL, x_PI, Rl = run_scenario(P, fl, pi, sim, 'reference')
    plot_scenario(
        t, x_FL, x_PI, Rl,
        title=r'Reference tracking: $v_d^*$ steps 359 $\to$ 320 V at t = 5 ms',
        filename='fig_scenario1_reference',
    )
    metrics_reference(t, x_FL, x_PI, P)

    print("\n========== Scenario 2: Load step rejection ==========")
    t, x_FL, x_PI, Rl = run_scenario(P, fl, pi, sim, 'load')
    plot_scenario(
        t, x_FL, x_PI, Rl,
        title=r'Load step: $R_{load}$ 9.67 $\to$ 4.84 m$\Omega$ at t = 5 ms',
        filename='fig_scenario2_load',
    )
    metrics_load(t, x_FL, x_PI, P)

    print("\n========== Scenario 3: Parameter mistune (R_f) ==========")
    t, x_FL, x_PI, Rl = run_scenario(P, fl, pi, sim, 'param')
    plot_scenario(
        t, x_FL, x_PI, Rl,
        title=r'$R_f$ mistune: FL belief 0.76 $\to$ 1.14 m$\Omega$ at t = 5 ms',
        filename='fig_scenario3_param',
    )
    metrics_param(t, x_FL, x_PI)

    print("\nDone. Three figures saved as PNG and PDF.")


if __name__ == '__main__':
    main()

