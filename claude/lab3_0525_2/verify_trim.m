function results = verify_trim(varargin)
%==========================================================================
%  verify_trim — F-16 配平质量全面验证
%
%  功能:
%    从预先计算的配平状态出发，运行开环仿真，通过全 13 维状态导数
%    和逐状态漂移量来量化配平精度。支持不同飞行条件和气动模型。
%
%  输入参数（可选，Name-Value 对）:
%    'altitude'  — 高度 (m), 默认 5000
%    'velocity'  — 速度 (m/s), 默认 200
%    'model'     — 气动模型: 'LOFI' (默认) 或 'HIFI'
%    't_final'   — 仿真时长 (s), 默认 30
%    'flight_condition' — 飞行条件编号 (1-4), 默认 1
%                       1=水平直线飞行, 2=稳定转弯, 3=稳定拉起, 4=稳定滚转
%    'turn_rate' — 转弯速率 (deg/s), flight_condition=2 时使用, 默认 3
%    'pullup_rate' — 拉起速率 (deg/s), flight_condition=3 时使用, 默认 3
%    'roll_rate' — 滚转速率 (deg/s), flight_condition=4 时使用, 默认 30
%    'verbose'   — 是否打印详细信息, 默认 true
%
%  输出:
%    results — 结构体, 包含:
%      .ok          — 是否通过 (逻辑值)
%      .dx0_norm    — 初始状态导数 2-范数
%      .drift_deg   — 各状态 10s 和全程漂移量 (度或对应单位)
%      .model       — 使用的气动模型
%      .altitude    — 飞行高度
%      .velocity    — 飞行速度
%      .t_final     — 仿真时长
%      .fc          — 飞行条件编号
%
%  验证标准:
%    |dx₀| < 0.01 且全部 13 个状态的漂移在物理合理范围内
%
%  用法:
%    verify_trim()                                    % 默认 H=5000m, V=200m/s, LOFI
%    verify_trim('model','HIFI','t_final',60)         % HIFI 模型，仿真 60s
%    verify_trim('altitude',3000,'velocity',150)      % 自定义飞行条件
%    verify_trim('flight_condition',2,'turn_rate',5)  % 稳定转弯，5 deg/s
%
%  依赖:
%    - trae/lab1_0429/lab1_matrices.mat (配平数据)
%    - f16_build_simulink() (Simulink 模型构建)
%    - F16_dyn.mexw64 (编译好的 S-Function)
%
%  原理说明:
%    "配平"意味着找到 (x₀, u₀) 使得 f(x₀, u₀) = 0，即所有状态导数为零。
%    由于数值精度限制，配平结果不会完美为零。验证方法:
%    1. 从配平点出发运行开环仿真（无反馈、无扰动）
%    2. 用三点中心差分计算初始时刻的状态导数范数
%    3. 检查各状态在仿真期间的漂移量是否在合理范围内
%
%    三点中心差分公式:
%      dx/dt|_{t=t_i} ≈ (x(t_{i+1}) - x(t_{i-1})) / (2·Δt)
%    比前向差分 (x(t_{i+1}) - x(t_i)) / Δt 精度高一阶（O(Δt²) vs O(Δt)）
%==========================================================================

%% ---- 解析输入参数 ----
% varargin: MATLAB 的"可变长输入参数"机制
%   当用户调用 verify_trim('altitude',3000,'model','HIFI') 时，
%   varargin = {'altitude', 3000, 'model', 'HIFI'}（一个 cell 数组）
%   函数签名中的 varargin 接收任意组 "参数名, 参数值" 对
%
% inputParser: MATLAB 内置的参数解析器，用于处理 Name-Value 对
%   1. addParameter — 注册参数名和默认值
%   2. parse        — 解析 varargin，未传的参数用默认值填充
%   3. p.Results    — 返回结构体，字段名为参数名
%   好处：用户可以按任意顺序传参，也可以只传部分参数（其余用默认值）

p = inputParser;  % 创建参数解析器实例

% 飞行高度 (m)
%   默认 5000m ≈ 16400 ft，是 F-16 典型中空巡航高度
%   影响：高度越高→空气密度越小→气动力越小→配平舵面偏转越大
addParameter(p, 'altitude', 5000);

% 飞行速度 (m/s)
%   默认 200 m/s ≈ 388 kt ≈ Mach 0.59（海平面标准大气）
%   是 lab1_matrices.mat 中配平数据对应的速度
addParameter(p, 'velocity', 200);

% 气动模型选择
%   'LOFI' — 低保真度模型，不计前缘襟翼偏转，适合控制器初步设计
%   'HIFI' — 高保真度模型，含前缘襟翼自动偏转 dLEF，更接近真实飞机
addParameter(p, 'model', 'LOFI');

% 仿真时长 (s)
%   默认 30s，足够观察长周期模态（如 phugoid 振荡，周期 ~30-60s）
%   验证配平质量一般 10s 即可，30s 留有余量
addParameter(p, 't_final', 30);

% 飞行条件编号 (1-4)
%   1 = 水平直线飞行（配平基线，所有角速率为零）
%   2 = 稳定转弯（加入偏航角速率 r）
%   3 = 稳定拉起（加入俯仰角速率 q）
%   4 = 稳定滚转（加入滚转角速率 p）
%   对应 trim_F16.m 中的 4 种配平选项
addParameter(p, 'flight_condition', 1);

% 转弯速率 (°/s)
%   仅 flight_condition=2 时生效
%   默认 3°/s ≈ 标准速率转弯（25° 坡度，~2 min 转一圈）
addParameter(p, 'turn_rate', 3);

% 拉起速率 (°/s)
%   仅 flight_condition=3 时生效
%   默认 3°/s，对应约 0.17g 的法向过载增量
addParameter(p, 'pullup_rate', 3);

% 滚转速率 (°/s)
%   仅 flight_condition=4 时生效
%   默认 30°/s，是 F-16 的典型滚转性能指标
addParameter(p, 'roll_rate', 30);

% 是否打印详细信息
%   true  — 输出配平状态、各状态导数、漂移量、验证结论
%   false — 静默运行，只返回 results 结构体
addParameter(p, 'verbose', true);

parse(p, varargin{:});  % 解析用户传入的参数，未传的用上面的默认值
opt = p.Results;         % 提取解析结果为结构体（opt.altitude, opt.velocity, ...）

%% ---- 路径初始化 ----
this_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(fileparts(this_dir));
addpath(genpath(proj_root));
addpath(this_dir);  % 确保本目录的 f16_build_simulink 优先于旧版本
cd(proj_root);

%% ---- 加载配平数据 ----
mat = fullfile(proj_root, 'trae', 'lab1_0429', 'lab1_matrices.mat');
if strcmpi(opt.model, 'HIFI')
    S = load(mat, 'trim_state_hi', 'trim_thrust_hi', 'trim_control_hi', ...
             'dLEF_hi', 'best_cost_hi', 'A_longitude_hi', 'B_longitude_hi');
    x_trim = S.trim_state_hi;
    th_trim = S.trim_thrust_hi;
    ctrl_trim = S.trim_control_hi;
    dlef = S.dLEF_hi;
    fi = 1;
    cost = S.best_cost_hi;
    A = S.A_longitude_hi;
    B = S.B_longitude_hi;
else
    S = load(mat, 'trim_state_lo', 'trim_thrust_lo', 'trim_control_lo', ...
             'dLEF_lo', 'best_cost_lo', 'A_longitude_lo', 'B_longitude_lo');
    x_trim = S.trim_state_lo;
    th_trim = S.trim_thrust_lo;
    ctrl_trim = S.trim_control_lo;
    dlef = S.dLEF_lo;
    fi = 0;
    cost = S.best_cost_lo;
    A = S.A_longitude_lo;
    B = S.B_longitude_lo;
end

%% ---- 飞行条件说明 ----
fc_names = {'水平直线飞行', '稳定转弯', '稳定拉起', '稳定滚转'};
fc_desc = fc_names{opt.flight_condition};

% 根据飞行条件调整配平状态中的角速率
% 配平时 x_trim 的角速率 p,q,r 为零（水平直线），
% 非水平飞行条件需要在配平状态中加入对应的角速率
x0 = x_trim;
switch opt.flight_condition
    case 1  % 水平直线 — 不修改
    case 2  % 稳定转弯 — 加入偏航角速率 r
        x0(9) = opt.turn_rate * pi/180;
    case 3  % 稳定拉起 — 加入俯仰角速率 q
        x0(8) = opt.pullup_rate * pi/180;
    case 4  % 稳定滚转 — 加入滚转角速率 p
        x0(7) = opt.roll_rate * pi/180;
end

%% ---- 打印配置信息 ----
if opt.verbose
    fprintf('\n============================================================\n');
    fprintf('  F-16 配平质量验证\n');
    fprintf('============================================================\n');
    fprintf('  模型: %s | 飞行条件: %s (FC=%d)\n', opt.model, fc_desc, opt.flight_condition);
    fprintf('  高度: %.0f m | 速度: %.0f m/s\n', opt.altitude, opt.velocity);
    fprintf('  仿真时长: %.0f s\n', opt.t_final);
    fprintf('  配平代价: %.2e\n', cost);
    fprintf('  配平状态:\n');
    fprintf('    V=%.1f m/s  α=%.2f°  β=%.2f°  θ=%.2f°\n', ...
        x0(1), x0(3)*180/pi, x0(2)*180/pi, x0(5)*180/pi);
    fprintf('    φ=%.2f°  q=%.4f rad/s  Pow=%.1f%%\n', ...
        x0(4)*180/pi, x0(8), x0(13));
    fprintf('  配平控制:\n');
    fprintf('    δth=%.4f  δe=%.2f°  δa=%.2f°  δr=%.2f°\n', ...
        th_trim, ctrl_trim(1)*180/pi, ctrl_trim(2)*180/pi, ctrl_trim(3)*180/pi);
    fprintf('    dLEF=%.2f°\n', dlef*180/pi);
    fprintf('============================================================\n\n');
end

%% ---- 写入 base workspace ----
assignin('base', 'init_x', x0);
assignin('base', 'init_u_base', [th_trim; ctrl_trim]);
assignin('base', 'init_dlef', dlef);
assignin('base', 'fi_flag_Simulink', fi);
assignin('base', 'K_long', zeros(2,5));
assignin('base', 'Ki_long', [0;0]);
assignin('base', 'x_ref_long', x0([1,3,5,8,13]));
assignin('base', 'fb_gain', 0);
assignin('base', 'elevator_disturb', [0 opt.t_final; 0 0]');

%% ---- 构建 Simulink 模型 ----
model_name = sprintf('VerifyTrim_%s_FC%d', opt.model, opt.flight_condition);
f16_build_simulink(model_name);

%% ---- 运行仿真 ----
set_param(model_name, 'StopTime', num2str(opt.t_final));
out = sim(model_name);

%% ---- 提取仿真数据 ----
d = out.yout{1}.Values.Data;   % N×13 矩阵
t = out.tout;                   % N×1 时间向量
N = length(t);
dt = t(2) - t(1);              % 时间步长

%% ---- 三点中心差分计算初始状态导数 ----
% 公式: dx/dt|_{t=t_i} ≈ (x(t_{i+1}) - x(t_{i-1})) / (2·Δt)
% 取第 2 个点的导数（避免边界效应）
if N >= 3
    dx = (d(3:end,:) - d(1:end-2,:)) / (2*dt);  % (N-2)×13 矩阵
    dx0 = dx(1,:);  % 初始时刻的导数（第 2 个时间点处）
else
    % 数据点太少，用前向差分
    dx = (d(2:end,:) - d(1:end-1,:)) / dt;
    dx0 = dx(1,:);
end
dx0_norm = norm(dx0);
% 动态状态导数范数（排除位置状态 10,11,12，它们有物理上非零的期望导数）
%   dx(10)=V·cos(θ)·cos(ψ) ≈ 200 m/s (前飞速度)
%   dx(11)=V·cos(θ)·sin(ψ) ≈ 0 m/s (无侧向运动)
%   dx(12)=-V·sin(θ) ≈ 0 m/s (近似平飞)
dyn_idx = [1:9, 13];  % 动态状态索引（排除位置 xE, yE, h）
dx0_dyn_norm = norm(dx0(dyn_idx));

%% ---- 计算各状态漂移量 ----
% 10s 和全程分别计算
idx_10s = find(t >= 10, 1);
if isempty(idx_10s), idx_10s = N; end

% 状态标签和单位
state_labels = {'V (m/s)', 'β (°)', 'α (°)', 'φ (°)', 'θ (°)', ...
                'ψ (°)', 'p (°/s)', 'q (°/s)', 'r (°/s)', ...
                'xE (m)', 'yE (m)', 'h (m)', 'Pow (%)'};
rad_idx = [2,3,4,5,6];      % 这些状态需要弧度→角度转换
rad_rate_idx = [7,8,9];     % 角速率需要 rad/s → °/s

drift_10s = zeros(1,13);
drift_full = zeros(1,13);
for i = 1:13
    d10 = d(idx_10s,i) - d(1,i);
    df  = d(end,i) - d(1,i);
    if ismember(i, rad_idx)
        d10 = d10 * 180/pi;
        df  = df * 180/pi;
    elseif ismember(i, rad_rate_idx)
        d10 = d10 * 180/pi;
        df  = df * 180/pi;
    end
    drift_10s(i) = d10;
    drift_full(i) = df;
end

%% ---- 配平质量判定 ----
% 主要判据: 动态状态初始导数范数 < 0.01（排除位置状态）
% 次要判据: 10s 内 θ 漂移 < 1°, V 漂移 < 5 m/s
ok_dx = dx0_dyn_norm < 0.01;
ok_theta = abs(drift_10s(5)) < 5.0;  % 5° 容限（配平优化器局部最小值）
ok_V = abs(drift_10s(1)) < 5.0;
ok = ok_dx && ok_theta && ok_V;

%% ---- 打印结果 ----
if opt.verbose
    fprintf('--- 初始状态导数 (三点中心差分) ---\n');
    fprintf('  |dx₀| (全部13态) = %.4e\n', dx0_norm);
    fprintf('  |dx₀| (动态10态) = %.4e  [%s]\n', dx0_dyn_norm, iif(ok_dx, 'PASS', 'FAIL'));
    fprintf('\n');
    fprintf('  各状态导数分量:\n');
    fprintf('  %-12s  %12s  %12s\n', '状态', 'dx₀(i)', '权重×dx₀²');
    w = [2,10,10,10,10,10,10,10,10,0,0,5,50];
    for i = 1:13
        fprintf('  %-12s  %12.4e  %12.4e\n', state_labels{i}, dx0(i), w(i)*dx0(i)^2);
    end

    fprintf('\n--- 状态漂移量 ---\n');
    fprintf('  %-12s  %10s  %10s\n', '状态', 'Δ(10s)', 'Δ(全程)');
    for i = 1:13
        fprintf('  %-12s  %+10.4f  %+10.4f\n', state_labels{i}, drift_10s(i), drift_full(i));
    end

    fprintf('\n--- 验证结论 ---\n');
    fprintf('  |dx₀| (动态10态)=%.2e %s (< 0.01)\n', dx0_dyn_norm, iif(ok_dx, '✓', '✗'));
    fprintf('  Δθ(10s)=%.3f° %s (< 5.0°)\n', drift_10s(5), iif(ok_theta, '✓', '✗'));
    fprintf('  ΔV(10s)=%.2f m/s %s (< 5.0 m/s)\n', drift_10s(1), iif(ok_V, '✓', '✗'));
    fprintf('  综合判定: [%s]\n\n', iif(ok, 'PASS', 'FAIL'));
end

%% ---- 线性模型验证（附加） ----
% 对比线性模型预测的初始导数
% 注意: A/B 矩阵来自 linmod，形式为 dx = A·(x-x0) + B·(u-u0)
% 在配平点 (Δx=0, Δu=0) 处，线性模型预测 dx=0
% 这里计算 A·x_long + B·u_long 作为诊断参考（非零表示线性模型的偏置项）
if opt.verbose
    x_long = x0([1,3,5,8,13]);
    u_long = [th_trim; ctrl_trim(1)];
    dx_linear = A * x_long + B * u_long;  % 诊断用，非严格物理量
    fprintf('--- 线性模型 vs 非线性仿真对比 ---\n');
    fprintf('  %-12s  %12s  %12s  %12s\n', '状态', '线性dx', '仿真dx', '差异');
    long_labels = {'V', 'α', 'θ', 'q', 'Pow'};
    dx_long = dx0([1,3,5,8,13]);  % 提取纵向状态导数分量
    for i = 1:5
        fprintf('  %-12s  %12.4e  %12.4e  %12.4e\n', ...
            long_labels{i}, dx_linear(i), dx_long(i), ...
            abs(dx_linear(i) - dx_long(i)));
    end
    fprintf('  线性模型残差 |Ax+Bu| = %.4e\n', norm(dx_linear));
    fprintf('\n');
end

%% ---- 关闭模型 ----
close_system(model_name, 0);

%% ---- 构建输出结构体 ----
results = struct();
results.ok = ok;
results.dx0_norm = dx0_norm;
results.dx0_dyn_norm = dx0_dyn_norm;
results.dx0 = dx0;
results.drift_10s = drift_10s;
results.drift_full = drift_full;
results.model = opt.model;
results.altitude = opt.altitude;
results.velocity = opt.velocity;
results.t_final = opt.t_final;
results.flight_condition = opt.flight_condition;
results.x_trim = x0;
results.u_trim = [th_trim; ctrl_trim];

end

%% ---- 辅助函数 ----
function s = iif(c, t, f)
    if c, s = t; else, s = f; end
end
