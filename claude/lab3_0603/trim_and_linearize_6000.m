function trim_and_linearize_6000()
%==========================================================================
%  trim_and_linearize_6000 — F-16 LOFI trim + linearize at H=6000m, V=250m/s
%  v2: Improved fminsearch (MaxIter=3000, multi-start guesses)
%==========================================================================

%% Path setup
this_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(fileparts(this_dir));
addpath(proj_root);
addpath(fullfile(proj_root, 'aerodata'));
addpath(fullfile(proj_root, 'trae', 'lab1_0429'));
addpath(this_dir);
cd(proj_root);

%% Flight condition
global fi_flag_Simulink phi_weight theta_weight psi_weight altitude velocity
altitude = 6000;
velocity = 250;
fi_flag_Simulink = 0;
phi_weight = 10; theta_weight = 10; psi_weight = 10;

fprintf('============================================================\n');
fprintf('  F-16 Trim + Linearize v2 (LOFI)\n');
fprintf('  H=%dm, V=%dm/s\n', altitude, velocity);
fprintf('============================================================\n\n');

%% Optimizer config (improved)
% MaxIter 1000->3000, MaxFunEvals 5000->15000
OPTIONS = optimset('TolFun', 1e-10, 'TolX', 1e-10, ...
                   'MaxFunEvals', 15000, 'MaxIter', 3000, 'Display', 'off');

%% Multi-start guesses for trim
% UX = [beta; elevator; alpha; aileron; rudder; dth]
init_guesses = [
    0, -2*pi/180,  2*pi/180, 0, 0, 0.25;
    0, -2*pi/180,  5*pi/180, 0, 0, 0.30;
    0, -5*pi/180,  8*pi/180, 0, 0, 0.35;
    0, -3*pi/180,  3*pi/180, 0, 0, 0.28;
];

%% State/control indices
idx_long_state = [1, 3, 5, 8, 13];
idx_long_ctrl = [14, 15];

%% Step 1: Trim (multi-start)
fprintf('--- Step 1/3: Trim (multi-start, MaxIter=3000) ---\n');

best_cost = inf;
for ig = 1:size(init_guesses, 1)
    UX0 = init_guesses(ig, :)';
    feval('F16_trim', [], [], [], 'lincompile');
    load_system('F16_trim');
    [UX_opt, ~, ~, OUTPUT] = fminsearch('trim_fun', UX0, OPTIONS);
    [cost_i, Xdot_i, xu_i, uu_i] = trim_fun(UX_opt);
    feval('F16_trim', [], [], [], 'term');

    fprintf('  Guess%d (a=%.0f deg, de=%.0f deg): iter=%d, cost=%.4e\n', ...
        ig, UX0(3)*180/pi, UX0(2)*180/pi, OUTPUT.iterations, cost_i);

    if cost_i < best_cost
        best_cost = cost_i;
        best_Xdot = Xdot_i;
        best_xu = xu_i;
        best_uu = uu_i;
        fprintf('    -> New best!\n');
    end
end

fprintf('  Final best: cost=%.4e (v1 was 1.34e-4)\n', best_cost);

trim_state = best_xu(1:13);
trim_thrust = best_uu(1);
trim_control = [best_uu(2); best_uu(3); best_uu(4)];
dLEF = best_uu(5);

fprintf('  V=%.1f  a=%.2f deg  th=%.2f deg  q=%.4f  Pow=%.1f%%\n', ...
    trim_state(1), trim_state(3)*180/pi, trim_state(5)*180/pi, ...
    trim_state(8), trim_state(13));
fprintf('  dth=%.4f  de=%.2f deg  da=%.2f deg  dr=%.2f deg  dLEF=%.2f deg\n', ...
    trim_thrust, trim_control(1)*180/pi, trim_control(2)*180/pi, ...
    trim_control(3)*180/pi, dLEF*180/pi);

%% Step 2: Linearize
fprintf('\n--- Step 2/3: Linearize (linmod) ---\n');

load_system('F16_openloop_linearization');
assignin('base', 'fi_flag_Simulink', fi_flag_Simulink);
assignin('base', 'init_x', trim_state);
assignin('base', 'init_u', [trim_thrust; trim_control]);
assignin('base', 'init_dlef', dLEF);

[A_lo, B_lo, C_lo, D_lo] = linmod('F16_openloop_linearization', ...
    trim_state, [trim_thrust; trim_control]);
close_system('F16_openloop_linearization', 0);

mat_full = [A_lo, B_lo; C_lo, D_lo];
A_longitude = mat_full(idx_long_state, idx_long_state);
B_longitude = mat_full(idx_long_state, idx_long_ctrl);
A_full = A_lo; B_full = B_lo;

fprintf('  linmod: A(%dx%d), B(%dx%d)\n', ...
    size(A_lo,1), size(A_lo,2), size(B_lo,1), size(B_lo,2));
fprintf('  Extracted A_long(5x5), B_long(5x2)\n');

%% Step 3: Save
fprintf('\n--- Step 3/3: Save ---\n');
out_mat = fullfile(this_dir, 'trim_data_6000.mat');
save(out_mat, ...
    'A_longitude', 'B_longitude', 'A_full', 'B_full', ...
    'trim_state', 'trim_thrust', 'trim_control', 'dLEF', ...
    'best_cost', 'best_Xdot', 'altitude', 'velocity');
fprintf('  Saved: %s\n', out_mat);

%% Print results
fprintf('\n============================================================\n');
fprintf('  Longitudinal A (5x5):\n');
for i = 1:5, fprintf('  %+.4e %+.4e %+.4e %+.4e %+.4e\n', A_longitude(i,:)); end
fprintf('\n  Longitudinal B (5x2):\n');
for i = 1:5, fprintf('  %+.4e %+.4e\n', B_longitude(i,:)); end
fprintf('\n  Open-loop poles:\n');
eig_open = eig(A_longitude);
for i = 1:length(eig_open)
    if imag(eig_open(i)) >= 0
        wn = abs(eig_open(i)); zeta = -real(eig_open(i))/wn;
        fprintf('    l=%.4f%+.4fi (wn=%.3f, z=%.3f)\n', ...
            real(eig_open(i)), imag(eig_open(i)), wn, zeta);
    end
end
fprintf('  Key derivatives: M_a=%.2f  M_q=%.2f  M_de=%.2f\n', ...
    A_longitude(4,2), A_longitude(4,4), B_longitude(4,2));
fprintf('\n');

end
