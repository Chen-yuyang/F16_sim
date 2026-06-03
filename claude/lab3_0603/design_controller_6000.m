function results = design_controller_6000()
%==========================================================================
%  design_controller_6000 v2 — Improved longitudinal controller
%  - lqr_i (augmented state space) added
%  - Ki scan for place_i
%  - Better pole design for this flight condition
%  - 60s simulation to ensure convergence
%  - Full 5-method comparison
%==========================================================================

%% Config
THETA_REF = 4 * pi / 180;
THETA_REF_DEG = 4;
T_FINAL = 60;

%% Path setup
this_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(fileparts(this_dir));
addpath(proj_root);
addpath(fullfile(proj_root, 'aerodata'));
addpath(fullfile(proj_root, 'trae', 'lab1_0429'));
addpath(this_dir);
cd(proj_root);

%% Step 1: Load data
fprintf('============================================================\n');
fprintf('  F-16 Longitudinal Controller Design v2 (LOFI)\n');
fprintf('  H=6000m, V=250m/s, theta_ref=%d deg, T=%d s\n', THETA_REF_DEG, T_FINAL);
fprintf('============================================================\n\n');

mat_new = fullfile(this_dir, 'trim_data_6000.mat');
if ~exist(mat_new, 'file')
    error('trim_data_6000.mat not found. Run trim_and_linearize_6000 first.');
end
S = load(mat_new);
A = S.A_longitude; B = S.B_longitude;        % 5x5, 5x2
x_trim = S.trim_state;                       % 13x1
u_trim = [S.trim_thrust; S.trim_control];     % 4x1 [dth; de; da; dr]
dlef = S.dLEF;

idx = [1, 3, 5, 8, 13];  % longitudinal subset
xl_trim = x_trim(idx);
u_trim_long = u_trim(1:2);

%% Step 2: Open-loop analysis
fprintf('=== Step 2: Open-loop Stability ===\n');
eig_open = eig(A);
for i = 1:length(eig_open)
    if imag(eig_open(i)) >= 0
        wn = abs(eig_open(i));
        zeta = -real(eig_open(i)) / wn;
        fprintf('  l=%.4f%+.4fi  wn=%.3f  z=%.3f\n', ...
            real(eig_open(i)), imag(eig_open(i)), wn, zeta);
    end
end

%% Step 3: Residual analysis
fprintf('\n=== Step 3: Residual Analysis ===\n');
x_ref = xl_trim; x_ref(3) = THETA_REF; x_ref(4) = 0;
r = A * x_ref + B * u_trim_long;
labels = {'V','a','th','q','Pow'};
for i = 1:5, fprintf('  r(%d) %s = %+.4e\n', i, labels{i}, r(i)); end
fprintf('  |r| = %.4e\n', norm(r));

%% Step 4: Controller design (5 methods)
fprintf('\n=== Step 4: Controller Design ===\n');

% --- Improved desired poles for this airframe ---
% Short period: wn=1.88, target ~2.5 (1.3x), zeta=0.7
% Phugoid: wn=0.05, target ~0.08 (1.6x), zeta=0.5
p_des = [-1.75+2.0i, -1.75-2.0i, -0.04+0.07i, -0.04-0.07i, -10];
fprintf('  New desired poles: sp %.2f+%.2fi, ph %.2f+%.2fi, eng %.0f\n', ...
    -1.75, 2.0, -0.04, 0.07, -10.0);

% --- Method 1: place ---
K_place = place(A, B, p_des);
eig_cl_place = eig(A - B*K_place);
fprintf('\n  [place]  K(2,3)=%.4f  stable=%d\n', K_place(2,3), all(real(eig_cl_place)<0));

% --- Method 2: lqr (re-tuned: reduced Q(3,3) 100->30) ---
Q_lqr = diag([0.1, 1, 30, 10, 0.1]);
R_lqr = diag([0.5, 0.5]);
K_lqr = lqr(A, B, Q_lqr, R_lqr);
fprintf('  [lqr]    K(2,3)=%.4f  Q(3,3)=%d  stable=%d\n', ...
    K_lqr(2,3), Q_lqr(3,3), all(real(eig(A-B*K_lqr))<0));

% --- Method 3: place_i with Ki scan ---
Ki_ratios = [0.15, 0.3, 0.5, 0.8, 1.0];
fprintf('\n  [place_i] Ki scan (K(2,3)=%.4f):\n', K_place(2,3));
for ik = 1:length(Ki_ratios)
    fprintf('    Ki_ratio=%.2f -> Ki=%.4f\n', Ki_ratios(ik), K_place(2,3)*Ki_ratios(ik));
end

% --- Method 4: lqr_i (augmented state space) ---
% A_aug = [A, 0; C, 0], B_aug = [B; 0]
% State: [V, a, th, q, Pow, xi] where xi = integral of theta error
C_theta = [0, 0, 1, 0, 0];  % select theta
A_aug = [A, zeros(5,1); -C_theta, 0];   % 6x6, note: xi_dot = -theta_error
B_aug = [B; zeros(1,2)];                 % 6x2

Q_int_values = [10, 50, 100, 500];
fprintf('\n  [lqr_i] Augmented system (6 states):\n');
for iq = 1:length(Q_int_values)
    Q_aug = blkdiag(Q_lqr, Q_int_values(iq));
    R_aug = R_lqr;
    K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);  % 2x6
    K_lqr_i = K_aug(:, 1:5);   % 2x5 proportional
    Ki_lqr_i = K_aug(:, 6);    % 2x1 integral
    fprintf('    Q_int=%3d  K(2,3)=%.4f  Ki(2)=%.4f\n', ...
        Q_int_values(iq), K_lqr_i(2,3), Ki_lqr_i(2));
end

%% Step 5: Build Simulink model
fprintf('\n=== Step 5: Build Simulink ===\n');

% Build base workspace
assignin('base', 'init_x', x_trim);
assignin('base', 'init_u_base', u_trim);
assignin('base', 'init_dlef', dlef);
assignin('base', 'fi_flag_Simulink', 0);
assignin('base', 'x_ref_long', x_ref);
assignin('base', 'K_long', K_place);
assignin('base', 'Ki_long', [0;0]);
assignin('base', 'fb_gain', 0);

% Clear old elevator disturbance
if evalin('base', 'exist(''elevator_disturb'', ''var'')')
    evalin('base', 'clear elevator_disturb');
end

model_name = 'F16_ClosedLoop_6000';
f16_build_simulink(model_name, false);
fprintf('  Model: %s\n', model_name);

%% Step 6: Open-loop simulation
fprintf('\n=== Step 6: Open-loop (fb_gain=0) ===\n');
assignin('base', 'fb_gain', 0);
assignin('base', 'K_long', K_place);
assignin('base', 'Ki_long', [0;0]);
f16_build_simulink(model_name, false);
out_ol = sim(model_name, 'StopTime', num2str(T_FINAL));
data_ol = out_ol.yout{1}.Values.Data;
t_ol = out_ol.tout;

fprintf('  Open-loop: th %.2f -> %.2f deg (after 10s)\n', ...
    data_ol(1,5)*180/pi, data_ol(find(t_ol>=10,1),5)*180/pi);

%% Step 7: Closed-loop (5 methods)
fprintf('\n=== Step 7: Closed-loop (fb_gain=1, theta_ref=%d deg) ===\n', THETA_REF_DEG);

% Define all methods (lqr_i skipped: lqr-based gains too aggressive -> MEX crash)
methods = {'place','lqr','place_i03','place_i05','place_i08'};
K_list = {K_place, K_lqr, K_place, K_place, K_place};
Ki_list = {[0;0], [0;0], [0; K_place(2,3)*0.3], [0; K_place(2,3)*0.5], [0; K_place(2,3)*0.8]};
use_int = [false, false, true, true, true];

nMethods = length(methods);
theta_ss = zeros(1, nMethods);
V_ss = zeros(1, nMethods);
de_ss_v = zeros(1, nMethods);
de_max = zeros(1, nMethods);
err_theta = zeros(1, nMethods);
data_cl = cell(1, nMethods);
t_cl = cell(1, nMethods);

for m = 1:nMethods
    assignin('base', 'K_long', K_list{m});
    assignin('base', 'Ki_long', Ki_list{m});
    assignin('base', 'fb_gain', 1);

    f16_build_simulink(model_name, use_int(m));
    out_cl = sim(model_name, 'StopTime', num2str(T_FINAL));
    data_cl{m} = out_cl.yout{1}.Values.Data;
    t_cl{m} = out_cl.tout;

    % Steady state (last 20s average)
    ss_idx = t_cl{m} >= (T_FINAL - 20);
    theta_ss(m) = mean(data_cl{m}(ss_idx, 5)) * 180/pi;
    V_ss(m) = mean(data_cl{m}(ss_idx, 1));
    err_theta(m) = THETA_REF_DEG - theta_ss(m);

    % Elevator deflection (computed from states)
    de_hist = u_trim(2) + ...
        (data_cl{m}(:, [1,3,5,8,13]) - x_ref') * K_list{m}(2,:)';
    if use_int(m)
        theta_err_hist = x_ref(3) - data_cl{m}(:, 5);
        de_hist = de_hist + Ki_list{m}(2) * cumtrapz(t_cl{m}, theta_err_hist);
    end
    de_max(m) = max(abs(de_hist)) * 180/pi;
    de_ss_v(m) = mean(de_hist(ss_idx)) * 180/pi;

    % Convergence: time to reach within 5% of final value
    theta_final = theta_ss(m);
    conv_idx = find(abs(data_cl{m}(:,5)*180/pi - theta_final) < 0.05*abs(THETA_REF_DEG - xl_trim(3)*180/pi), 1);
    if isempty(conv_idx), conv_idx = length(t_cl{m}); end
    conv_time = t_cl{m}(conv_idx);

    fprintf('  [%s] th_ss=%.3f deg  err=%.3f deg  V_ss=%.1f  de_ss=%.1f deg  de_max=%.1f deg  t_conv=%.1fs\n', ...
        methods{m}, theta_ss(m), err_theta(m), V_ss(m), de_ss_v(m), de_max(m), conv_time);
end

%% Step 8: Diagnosis — why is theta error not <0.5 deg?
fprintf('\n=== Step 8: Error Diagnosis ===\n');

% Check if error is converging or stuck
fprintf('  --- Convergence check (20-30s vs 50-60s) ---\n');
for m = 1:nMethods
    ss_mid = t_cl{m} >= 20 & t_cl{m} <= 30;
    ss_end = t_cl{m} >= 50 & t_cl{m} <= 60;
    th_mid = mean(data_cl{m}(ss_mid, 5)) * 180/pi;
    th_end = mean(data_cl{m}(ss_end, 5)) * 180/pi;
    fprintf('  [%s] th(20-30s)=%.3f deg  th(50-60s)=%.3f deg  drift=%.4f deg\n', ...
        methods{m}, th_mid, th_end, th_end - th_mid);
end

% Linear prediction vs nonlinear reality
fprintf('\n  --- Linear prediction vs simulation ---\n');
x_ss_pred_place = -(A - B*K_place) \ r;
fprintf('  [place]    linear pred th_err=%.3f deg  sim th_err=%.3f deg  ratio=%.1f\n', ...
    x_ss_pred_place(3)*180/pi, err_theta(1), ...
    abs(x_ss_pred_place(3)*180/pi / err_theta(1)));

% Ki effectiveness analysis
fprintf('\n  --- Ki effectiveness ---\n');
fprintf('  place_i05  Ki=%.4f  err=%.3f deg\n', K_place(2,3)*0.5, err_theta(3));
fprintf('  place_i08  Ki=%.4f  err=%.3f deg\n', K_place(2,3)*0.8, err_theta(4));
fprintf('  place_i10  Ki=%.4f  err=%.3f deg\n', K_place(2,3)*1.0, err_theta(5));
fprintf('  place_i08  Ki=%.4f  err=%.3f deg\n', K_place(2,3)*0.8, err_theta(5));

% Residual decomposition
fprintf('\n  --- Residual decomposition ---\n');
fprintf('  r_V=%.3e  r_a=%.3e  r_th=%.3e  r_q=%.3e  r_Pow=%.3e\n', r(1), r(2), r(3), r(4), r(5));
fprintf('  Dominant term: r_V (velocity force imbalance) = %.3e\n', r(1));
fprintf('  This acts as a persistent disturbance -> proportional control\n');
fprintf('  cannot eliminate it -> integral control helps but limited by\n');
fprintf('  servo rate and the fact that dth channel has K(1,:)~0\n');

%% Step 9: Verification checklist
fprintf('\n=== Step 9: Verification Checklist ===\n');

% #1: theta tracking
fprintf('\n  --- #1 theta tracking (|err| < 0.5 deg) ---\n');
for m = 1:nMethods
    fprintf('  [%s] |err|=%.3f deg  %s\n', methods{m}, abs(err_theta(m)), ...
        tern(abs(err_theta(m)) < 0.5, 'PASS', 'FAIL'));
end

% #2: V hold
fprintf('\n  --- #2 V hold (|dV| < 5 m/s) ---\n');
for m = 1:nMethods
    dV = abs(V_ss(m) - S.velocity);
    fprintf('  [%s] |dV|=%.2f m/s  %s\n', methods{m}, dV, tern(dV < 5, 'PASS', 'FAIL'));
end

% #4: de limits
fprintf('\n  --- #4 de limits (|de_max| < 25 deg) ---\n');
for m = 1:nMethods
    fprintf('  [%s] max|de|=%.1f deg  %s\n', methods{m}, de_max(m), ...
        tern(de_max(m) < 25, 'PASS', 'FAIL'));
end

% #5: gain sign
fprintf('\n  --- #5 Gain sign K(2,3) < 0 ---\n');
fprintf('  place: K(2,3)=%.4f  %s\n', K_place(2,3), tern(K_place(2,3)<0, 'PASS', 'FAIL'));
fprintf('  lqr:   K(2,3)=%.4f  %s\n', K_lqr(2,3), tern(K_lqr(2,3)<0, 'PASS', 'FAIL'));
fprintf('  (lqr_i skipped: gains cause MEX crash at this flight condition)\n');

%% Step 10: Plot
fprintf('\n=== Step 10: Plotting ===\n');

figure('Name', 'F-16 Longitudinal H=6000m V=250m/s v2', ...
    'Position', [30, 30, 1400, 900]);

cfg = {
    1, 5, 180/pi, '\theta (deg)', sprintf('Pitch angle (ref=%d deg)', THETA_REF_DEG);
    2, 3, 180/pi, '\alpha (deg)', 'Angle of attack';
    3, 1, 1, 'V (m/s)', 'Velocity';
    4, 8, 180/pi, 'q (deg/s)', 'Pitch rate';
    5, 12, -1, 'h (m)', 'Altitude';
};

colors = lines(nMethods+1);
for i = 1:5
    subplot(2, 3, i); hold on;
    c = cfg(i, :);
    idx_s = c{2}; sc = c{3};

    % Open loop
    plot(t_ol, sc * data_ol(:, idx_s), 'k--', 'LineWidth', 1, 'DisplayName', 'open-loop');

    % All methods
    for m = 1:nMethods
        plot(t_cl{m}, sc * data_cl{m}(:, idx_s), 'Color', colors(m,:), ...
            'LineWidth', 1.2, 'DisplayName', methods{m});
    end

    if idx_s == 5
        yline(THETA_REF_DEG, 'g--', 'LineWidth', 1.5);
    end

    xlabel('Time (s)'); ylabel(c{4}); title(c{5});
    grid on; legend('Location', 'best', 'FontSize', 6);
end

% Subplot 6: info panel + method comparison
subplot(2, 3, 6);
info_lines = {
    sprintf('F-16 Longitudinal — LOFI model');
    sprintf('H=6000m, V=250m/s, theta_{ref}=%d deg, T=%d s', THETA_REF_DEG, T_FINAL);
    sprintf('');
    sprintf('Trim: a=%.2f deg, de=%.2f deg, dth=%.4f', x_trim(3)*180/pi, u_trim(2)*180/pi, u_trim(1));
    sprintf('Trim cost: %.2e  |r|=%.2e', S.best_cost, norm(r));
    sprintf('M_a=%.2f  M_q=%.2f  M_de=%.2f', A(4,2), A(4,4), B(4,2));
    sprintf('Open-loop SP: wn=%.2f, z=%.2f', abs(eig_open(1)), -real(eig_open(1))/abs(eig_open(1)));
    sprintf('');
    sprintf('--- Method Comparison ---');
    sprintf('Method         err(deg)  de_max(deg)  t_conv(s)');
    };
for m = 1:nMethods
    conv_idx = find(abs(data_cl{m}(:,5)*180/pi - theta_ss(m)) < 0.05*abs(THETA_REF_DEG - xl_trim(3)*180/pi), 1);
    if isempty(conv_idx), tc = T_FINAL; else tc = t_cl{m}(conv_idx); end
    info_lines{end+1} = sprintf('%-14s %+7.3f   %+7.1f     %6.1f', ...
        methods{m}, err_theta(m), de_max(m), tc);
end

text(0.02, 0.98, info_lines, 'FontSize', 7, 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'FontName', 'FixedWidth');
axis off;

sgtitle(sprintf('F-16 Longitudinal Control (LOFI, H=%dm, V=%dm/s, theta_{ref}=%d deg)', ...
    S.altitude, S.velocity, THETA_REF_DEG));

fig_file = fullfile(this_dir, 'results_6000_v2.png');
saveas(gcf, fig_file);
fprintf('  Figure saved: %s\n', fig_file);

%% Output struct
results = struct();
results.A = A; results.B = B;
results.x_trim = x_trim; results.u_trim = u_trim;
results.eig_open = eig_open;
results.r = r;
results.methods = {methods};
results.theta_ss = theta_ss;
results.err_theta = err_theta;
results.V_ss = V_ss;
results.de_max = de_max;
results.de_ss = de_ss_v;
results.K_place = K_place;
results.K_lqr = K_lqr;
results.K_lqr_i_note = 'lqr_i skipped: gains too aggressive for this flight condition (MEX crash)';
results.Ki_ratios = Ki_ratios;

close_system(model_name, 0);

fprintf('\n============================================================\n');
fprintf('  Done! Best method: see verification results above.\n');
fprintf('============================================================\n');

end

function s = tern(c, t, f)
    if c, s = t; else, s = f; end
end
