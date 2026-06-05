% Full-state feedback linearization vs cascaded PI for an islanded
% grid-forming inverter with resistive load.
%
% Guzman, Madrigal, Melgoza-Vazquez (2025), Table 1
%
% Scenarios:
% (1) Reference tracking:  vd* 359 -> 320 V at t = 5 ms
% (2) Load step:           Rload 9.67 -> 4.84 mOhm at t = 5 ms
% (3) Parameter mistune:  Rf (FL belief) 0.76 -> 1.14 mOhm

clear; clc; close all;

%% Plant (Guzman et al. 2025, Table 1)
P.Lf      = 0.079e-3;
P.Cf      = 13.7e-3;
P.Rf      = 0.76e-3;
P.Rload   = 9.67e-3;          % 9.67 mohm gives 20 MW at 359 V peak L-N
P.wstar   = 2*pi*60;
P.vd_ref  = 359;              % 440 V RMS L-L = 359 V peak L-N
P.vq_ref  = 0;

%% Feedback-linearizing controller (pole placement on double integrator)
FL.wn     = 2*pi*500;
FL.zeta   = 0.707;
FL.k0     = FL.wn^2;
FL.k1     = 2*FL.zeta*FL.wn;
FL.Rf     = P.Rf;
FL.Rload  = P.Rload;

%% Cascaded PI gains (Guzman et al. 2025, Table 1)
PI.kpi    = 0.6176;
PI.kii    = 2419.9;
PI.kpv    = 10.72;
PI.kiv    = 4195;

%% Simulation parameters
sim.tend  = 0.050;
sim.dt    = 1e-6;

%% ---------------- Scenario 1: Reference Tracking ----------------
fprintf('\n========== Scenario 1: Reference tracking ==========\n');
[t1, x_FL1, x_PI1, Rl1] = run_scenario(P, FL, PI, sim, 'reference');
plot_scenario(t1, x_FL1, x_PI1, Rl1, ...
    'Reference tracking: v_d^* steps 359 \rightarrow 320 V at t = 5 ms', ...
    'fig_scenario1_reference');
metrics_reference(t1, x_FL1, x_PI1, P);

%% ---------------- Scenario 2: Load Step Rejection ----------------
fprintf('\n========== Scenario 2: Load step rejection ==========\n');
[t2, x_FL2, x_PI2, Rl2] = run_scenario(P, FL, PI, sim, 'load');
plot_scenario(t2, x_FL2, x_PI2, Rl2, ...
    'Load step: R_{load} 9.67 \rightarrow 4.84 m\Omega at t = 5 ms', ...
    'fig_scenario2_load');
metrics_load(t2, x_FL2, x_PI2, P);

%% ---------------- Scenario 3: Parameter Mistune ----------------
fprintf('\n========== Scenario 3: Parameter mistune (R_f) ==========\n');
[t3, x_FL3, x_PI3, Rl3] = run_scenario(P, FL, PI, sim, 'param');
plot_scenario(t3, x_FL3, x_PI3, Rl3, ...
    'R_f mistune: FL belief 0.76 \rightarrow 1.14 m\Omega at t = 5 ms', ...
    'fig_scenario3_param');
metrics_param(t3, x_FL3, x_PI3, P);

fprintf('\nDone. Three figures saved as PNG and PDF.\n');


%% ================================================================
%  Helper functions
%  ================================================================

function [t, x_FL, x_PI, Rload_t] = run_scenario(P, FL, PI, sim, scenario)
N  = round(sim.tend/sim.dt);
t  = (0:N-1)*sim.dt;
dt = sim.dt;

Rload_t   = P.Rload    * ones(1,N);
Rf_belief = FL.Rf      * ones(1,N);
vd_ref_t  = P.vd_ref   * ones(1,N);
vq_ref_t  = P.vq_ref   * ones(1,N);

t_event = 5e-3;
switch scenario
    case 'reference'
        vd_ref_t(t >= t_event) = 320;
    case 'load'
        Rload_t(t >= t_event) = 4.84e-3;
    case 'param'
        Rf_belief(t >= t_event) = 1.14e-3;
end

i_d_ss = P.vd_ref / P.Rload;
i_q_ss = P.wstar*P.Cf*P.vd_ref;
x0 = [i_d_ss; i_q_ss; P.vd_ref; 0];
x_FL = zeros(4,N); x_FL(:,1) = x0;
x_PI = zeros(4,N); x_PI(:,1) = x0;

e_d_ss = P.vd_ref + P.Rf*i_d_ss - P.wstar*P.Lf*i_q_ss;
e_q_ss = P.Rf*i_q_ss + P.wstar*P.Lf*i_d_ss;
xi_id  = (e_d_ss - P.vd_ref + P.wstar*P.Lf*i_q_ss)/PI.kii;
xi_iq  = (e_q_ss - 0        - P.wstar*P.Lf*i_d_ss)/PI.kii;
xi_vd  = i_d_ss/PI.kiv;
xi_vq  = (i_q_ss - P.wstar*P.Cf*P.vd_ref)/PI.kiv;

for k = 1:N-1
    Rl = Rload_t(k);
    
    % ---- FL controller ----
    Rfc = Rf_belief(k);
    Rlc = FL.Rload;
    id = x_FL(1,k); iq = x_FL(2,k); vd = x_FL(3,k); vq = x_FL(4,k);
    
    vd_dot = (id - vd/Rl)/P.Cf + P.wstar*vq;
    vq_dot = (iq - vq/Rl)/P.Cf - P.wstar*vd;
    
    LF2h1 = (-Rfc*id + P.wstar*P.Lf*iq - vd)/(P.Lf*P.Cf) ...
            - vd_dot/(Rlc*P.Cf) + P.wstar*vq_dot;
    LF2h2 = (-Rfc*iq - P.wstar*P.Lf*id - vq)/(P.Lf*P.Cf) ...
            - vq_dot/(Rlc*P.Cf) - P.wstar*vd_dot;
    
    nu1 = -FL.k1*vd_dot - FL.k0*(vd - vd_ref_t(k));
    nu2 = -FL.k1*vq_dot - FL.k0*(vq - vq_ref_t(k));
    
    ed_FL = P.Lf*P.Cf*(nu1 - LF2h1);
    eq_FL = P.Lf*P.Cf*(nu2 - LF2h2);
    
    x_FL(:,k+1) = x_FL(:,k) + dt*plant_rhs(x_FL(:,k), [ed_FL;eq_FL], P, Rl);
    
    % ---- PI controller ----
    id = x_PI(1,k); iq = x_PI(2,k); vd = x_PI(3,k); vq = x_PI(4,k);
    
    err_vd = vd_ref_t(k) - vd;
    err_vq = vq_ref_t(k) - vq;
    xi_vd  = xi_vd + dt*err_vd;
    xi_vq  = xi_vq + dt*err_vq;
    id_ref = -P.wstar*P.Cf*vq + PI.kpv*err_vd + PI.kiv*xi_vd;
    iq_ref = +P.wstar*P.Cf*vd + PI.kpv*err_vq + PI.kiv*xi_vq;
    
    err_id = id_ref - id;
    err_iq = iq_ref - iq;
    xi_id  = xi_id + dt*err_id;
    xi_iq  = xi_iq + dt*err_iq;
    ed_PI = vd - P.wstar*P.Lf*iq + PI.kpi*err_id + PI.kii*xi_id;
    eq_PI = vq + P.wstar*P.Lf*id + PI.kpi*err_iq + PI.kii*xi_iq;
    
    x_PI(:,k+1) = x_PI(:,k) + dt*plant_rhs(x_PI(:,k), [ed_PI;eq_PI], P, Rl);
end
end


function xdot = plant_rhs(x, u, P, Rl)
id = x(1); iq = x(2); vd = x(3); vq = x(4);
ed = u(1); eq = u(2);
xdot = [ (-P.Rf*id + P.wstar*P.Lf*iq + ed - vd)/P.Lf;
         (-P.Rf*iq - P.wstar*P.Lf*id + eq - vq)/P.Lf;
         (id - vd/Rl)/P.Cf + P.wstar*vq;
         (iq - vq/Rl)/P.Cf - P.wstar*vd ];
end


function plot_scenario(t, x_FL, x_PI, Rload_t, title_str, filename)
% Compute load-side P and Q at PCC.  Pure resistive load -> Q = 0.
igd_FL = x_FL(3,:)./Rload_t;  igq_FL = x_FL(4,:)./Rload_t;
igd_PI = x_PI(3,:)./Rload_t;  igq_PI = x_PI(4,:)./Rload_t;
P_FL = 1.5*(x_FL(3,:).*igd_FL + x_FL(4,:).*igq_FL);
Q_FL = -1.5*(x_FL(3,:).*igq_FL - x_FL(4,:).*igd_FL);
P_PI = 1.5*(x_PI(3,:).*igd_PI + x_PI(4,:).*igq_PI);
Q_PI = -1.5*(x_PI(3,:).*igq_PI - x_PI(4,:).*igd_PI);

figure('Position',[100 100 1100 850], 'Color','w');
tms = t*1e3;

subplot(3,2,1); plot(tms,x_FL(3,:),'LineWidth',1.6); hold on; plot(tms,x_PI(3,:), '--','LineWidth',1.6);
xline(5,'k--'); ylabel('v_d (V)'); grid on; legend('FL','PI','Location','best'); title('d-axis voltage')
subplot(3,2,2); plot(tms,x_FL(4,:),'LineWidth',1.6); hold on; plot(tms,x_PI(4,:),'--','LineWidth',1.6);
xline(5,'k--'); ylabel('v_q (V)'); grid on; legend('FL','PI','Location','best'); title('q-axis voltage')
subplot(3,2,3); plot(tms,x_FL(1,:),'LineWidth',1.6); hold on; plot(tms,x_PI(1,:),'--','LineWidth',1.6);
xline(5,'k--'); ylabel('i_d (A)'); grid on; legend('FL','PI','Location','best'); title('d-axis current')
subplot(3,2,4); plot(tms,x_FL(2,:),'LineWidth',1.6); hold on; plot(tms,x_PI(2,:),'--','LineWidth',1.6);
xline(5,'k--'); ylabel('i_q (A)'); grid on; legend('FL','PI','Location','best'); title('q-axis current')
subplot(3,2,5); plot(tms,P_FL/1e6,'LineWidth',1.6); hold on; plot(tms,P_PI/1e6,'--','LineWidth',1.6);
xline(5,'k--'); xlabel('time (ms)'); ylabel('P (MW)'); grid on; legend('FL','PI','Location','best');
title('active power at load')
subplot(3,2,6); plot(tms,Q_FL/1e6,'LineWidth',1.6); hold on; plot(tms,Q_PI/1e6,'--','LineWidth',1.6);
xline(5,'k--'); xlabel('time (ms)'); ylabel('Q (MVAr)'); grid on; legend('FL','PI','t=5 ms','Location','best');
title('reactive power at load')

sgtitle(title_str, 'FontSize', 13, 'FontWeight','bold')
exportgraphics(gcf, [filename '.png'], 'Resolution', 180);
exportgraphics(gcf, [filename '.pdf']);
end


function metrics_reference(t, x_FL, x_PI, P)
idx = t >= 5e-3;
ts_FL = settling_time(t(idx)-5e-3, x_FL(3,idx), 320, 0.02);
ts_PI = settling_time(t(idx)-5e-3, x_PI(3,idx), 320, 0.02);
vq_FL = max(abs(x_FL(4,idx)));
vq_PI = max(abs(x_PI(4,idx)));
P_initial = 1.5*359^2/P.Rload / 1e6;
P_final   = 1.5*320^2/P.Rload / 1e6;
fprintf('  Settling time (2%%):  FL = %.2f ms,  PI = %.2f ms\n', ts_FL*1e3, ts_PI*1e3);
fprintf('  v_q coupling max:    FL = %.3f V,    PI = %.3f V\n', vq_FL, vq_PI);
fprintf('  P steady state:      before = %.2f MW,  after = %.2f MW\n', P_initial, P_final);
end


function metrics_load(t, x_FL, x_PI, P)
idx = t >= 5e-3;
sag_FL = 359 - min(x_FL(3,idx));
sag_PI = 359 - min(x_PI(3,idx));
end_FL = mean(x_FL(3, t >= 45e-3)) - 359;
end_PI = mean(x_PI(3, t >= 45e-3)) - 359;
P_before = 1.5*359^2/P.Rload   / 1e6;
P_after  = 1.5*359^2/4.84e-3   / 1e6;
fprintf('  v_d sag max:         FL = %.2f V,    PI = %.2f V\n', sag_FL, sag_PI);
fprintf('  v_d offset at end:   FL = %.4f V,    PI = %.4f V\n', end_FL, end_PI);
fprintf('  P steady state:      before = %.2f MW,  after = %.2f MW\n', P_before, P_after);
end


function metrics_param(t, x_FL, x_PI, ~)
idx = t >= 45e-3;
off_FL = mean(x_FL(3,idx)) - 359;
off_PI = mean(x_PI(3,idx)) - 359;
fprintf('  v_d offset:          FL = %.4f V,    PI = %.4f V\n', off_FL, off_PI);
end


function ts = settling_time(t, y, yref, band)
err = abs(y - yref);
outside = err > band*abs(yref);
if ~any(outside)
    ts = 0; return
end
last_out = find(outside, 1, 'last');
if last_out == length(t)
    ts = NaN;
else
    ts = t(last_out+1) - t(1);
end
end
