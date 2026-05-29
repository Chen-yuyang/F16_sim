function lab1_step3_simulation()
%==========================================================================
%  实验1 - 步骤3：非线性开环仿真验证
%  使用 5 deg 升降舵 Doublet 脉冲输入，对比 LOFI 与 HIFI 模型的时域响应
%  生成三张标准图（Figure 5/6/7）和一张综合对比图
%
%  【背景 - 为什么做仿真】
%  步骤1/2 基于线性化模型分析稳定性，但线性化忽略了非线性效应
%  步骤3 在 Simulink 中运行完整的非线性模型，验证线性分析是否可靠
%  比较 LOFI 和 HIFI 的非线性响应差异，评估线性化模型的预测能力
%
%  【Doublet 信号】
%  形式: 0 ↑ +5° ↓ -5° ↑ 0  (升降舵的"正-负-回零"脉冲序列)
%  时间: 0-1s:0°, 1-2s:+5°, 2-5s:-5°, 5-6s:0°, 6-30s:0°
%  作用: 激励飞机的纵向响应（同时产生部分横侧向耦合）
%  选择 5° 以产生可观测响应但又不至于过大导致非线性效应过强
%
%  飞行条件: H = 5000 m, V = 200 m/s
%
%  输出: figure5_controls.png/fig  (控制输入)
%        figure6_positions.png/fig  (位置与姿态)
%        figure7_rates.png/fig      (速度与角速率)
%        figure_summary.png/fig     (综合对比)
%        lab1_simulation_data.mat   (仿真数据)
%        lab1_trim_info.txt         (配平信息)
%==========================================================================

    %% ===================== 初始化：清理工作区 =====================
    close all;
    clc;

    fprintf('========================================\n');
    fprintf('  实验1 - 步骤3：非线性开环仿真\n');
    fprintf('  输入: 5 deg 升降舵 Doublet\n');
    fprintf('========================================\n\n');

    this_file = mfilename('fullpath');
    proj_root = fileparts(fileparts(fileparts(this_file)));
    result_dir = fileparts(this_file);
    cd(proj_root);

    %% ===================== 全局变量与飞行条件 =====================
    global altitude velocity fi_flag_Simulink phi_weight theta_weight psi_weight

    altitude = 5000;
    velocity = 200;
    phi_weight = 10;
    theta_weight = 10;
    psi_weight = 10;

    % 配平优化器配置（与步骤1相同）
    OPTIONS = optimset('TolFun', 1e-10, 'TolX', 1e-10, 'MaxFunEvals', 5000, ...
                       'MaxIter', 1000, 'Display', 'off');

    % 抑制 Simulink 的连接警告（模型中存在不连接输出端口属正常）
    warning('off', 'Simulink:Engine:OutputNotConnected');
    warning('off', 'Simulink:Engine:BlockOutputNotConnected');

    %% ===================== 升降舵 Doublet 信号 =====================
    % Doublet 信号设计为 Simulink From Workspace 块格式：
    %   时间向量 t_doublet 严格单调不减
    %   信号值向量与时间点一一对应
    %   时间点重复=信号跳变（0→1s 保持 0°, 1→2s 保持 +5°, 依次类推）
    %
    % 物理目的：
    %   正阶跃 (+5°) → 飞机低头 → 速度增加、高度下降
    %   负阶跃 (-5°) → 飞机抬头 → 速度减小、高度上升
    %   整体构成一个完整的对称激励，利于观察短周期和长周期响应
    t_doublet = [0, 1, 1, 2, 2, 5, 5, 6, 6, 30];
    doublet_amp = 5 * pi/180;    % 5 度转换为弧度
    elevator_dist = [0, 0, doublet_amp, doublet_amp, -doublet_amp, -doublet_amp, 0, 0, 0, 0];
    aileron_dist = zeros(size(t_doublet));  % 副翼无扰动
    rudder_dist  = zeros(size(t_doublet));  % 方向舵无扰动

    % 配平初始猜测（与步骤1相同）
    beta = 0; elevator = 0*pi/180; alpha = 10*pi/180;
    rudder = 0; aileron = 0; dth = 0.2;
    UX0 = [beta; elevator; alpha; aileron; rudder; dth];


    %% ===================== Part A: LOFI 配平与仿真 =====================
    % 独立配平（不使用步骤1的结果，保证仿真使用的配平点与仿真模型一致）
    fprintf('--- Part A: 配平 & 仿真 LOFI 模型 ---\n');

    fi_flag_Simulink = 0;

    % 配平（过程同步骤1）
    feval('F16_trim', [], [], [], 'lincompile');
    load_system('F16_trim');
    [UX, ~, ~, ~] = fminsearch('trim_fun', UX0, OPTIONS);
    [cost_lo, ~, xu_lo, uu_lo] = trim_fun(UX);
    feval('F16_trim', [], [], [], 'term');

    fprintf('  LOFI 配平完成, cost=%.4e\n', cost_lo);

    t_lo = []; y_lo = [];

    % 将配平结果和控制信号注入 base workspace（Simulink 模型从 base workspace 读取参数）
    assignin('base', 'fi_flag_Simulink', fi_flag_Simulink);
    assignin('base', 'init_x', xu_lo(1:13));                % 初始状态
    assignin('base', 'init_u', uu_lo(1:4));                  % 初始控制（油门+舵面）
    assignin('base', 'init_dlef', uu_lo(5));                 % 前缘襟翼初始偏角
    assignin('base', 't_disturb', t_doublet);                % 扰动时间向量
    assignin('base', 'elevator_disturb', elevator_dist);     % 升降舵扰动信号
    assignin('base', 'aileron_disturb', aileron_dist);       % 副翼扰动信号
    assignin('base', 'rudder_disturb', rudder_dist);         % 方向舵扰动信号

    % --- 配置并运行 Simulink ---
    % F16_openloop.slx: 非线性开环仿真模型
    %
    % 【模型结构说明】
    % 核心动力学封装在 S-Function 模块 "F-16 dynamics"（调用 F16_dyn.c）
    % C 代码内部用 ssSetNumContStates() 声明了 13 个连续状态（积分器），
    % 所以在模型窗口中看不到独立的 Integrator 模块。
    %
    % 模型顶层没有 Outport 模块（只有 S-Function、Demux、Scope），
    % 因此仿真后 yout 是空变量，所有数据都来自 xout。
    %
    % 顶层模块组成：
    %   F-16 dynamics (S-Function)  — 核心气动/动力学，3 入 1 出
    %   Trimmed input (SubSystem)   — 舵面输入（de, da, dr, dth）
    %   LEF (SubSystem)             — 前缘襟翼计算
    %   Demux                       — S-Function 输出 1→13 路信号
    %   Scope1~4                    — 示波器（仅用于查看波形）
    %
    % 【Simulink 配置命令说明】
    %   load_system('F16_openloop') — 将 .slx 模型加载到内存，不打开窗口
    %   set_param(模型名, 参数, 值) — 修改模型级参数
    %     'StopTime', '30'          — 仿真时长 30 秒
    %   getActiveConfigSet —— 获取模型配置集句柄（相当于 Ctrl+E 打开的对话框）
    %   set_param(配置集, 参数, 值) — 修改配置参数
    %     'SaveTime', 'on'          — 仿真结束后往工作区写入 tout（时间向量）
    %     'SaveState', 'on'         — 写入 xout（状态变量，每一列对应一个连续状态）
    %     'SaveOutput', 'on'        — 写入 yout（顶层 Outport 信号，本模型为空）
    %     'SaveFormat', 'Array'     — 以普通数组格式保存（而非 Dataset）
    %
    % 【仿真输出变量】
    %   tout — 时间向量，Simulink 自动生成的默认名称
    %   xout — 连续状态时间历程，每一列对应一个状态。
    %          本模型 13 列依次为：
    %          (1) V 空速 | (2) α 迎角 | (3) β 侧滑角
    %          (4) φ 滚转角 | (5) θ 俯仰角 | (6) ψ 偏航角
    %          (7) p 滚转角速率 | (8) q 俯仰角速率 | (9) r 偏航角速率
    %          (10) x_N 北向位置 | (11) x_E 东向位置
    %          (12) h 高度 | (13) pow 发动机功率
    %          所以 xout(:,2) 就是 α 在 30 秒内的变化曲线。
    %   yout — 顶层 Outport 模块的信号。本模型没有 Outport，yout 为空。
    %
    % 【关键概念】
    %   xout 来自模型中所有连续状态（积分器/S-Function 内部状态），
    %   不是由代码起名指定，而是 Simulink 内置的默认输出名。
    %   换一个模型，xout 的列数和含义完全取决于该模型的结构。
    %
    %   yout 来自模型顶层的 Outport 模块。如果模型没有 Outport 块，
    %   即使 SaveOutput='on'，yout 也是空的。
    load_system('F16_openloop');
    set_param('F16_openloop', 'StopTime', '30');
    cs = getActiveConfigSet('F16_openloop');
    set_param(cs, 'SaveTime', 'on');
    set_param(cs, 'SaveState', 'on');
    set_param(cs, 'SaveOutput', 'on');
    set_param(cs, 'SaveFormat', 'Array');

    sim('F16_openloop');
    t_lo = tout;        % tout = Simulink 自动生成的时间向量
    y_lo = xout;        % xout = 13 个连续状态的时间历程（见上方说明）
    close_system('F16_openloop', 0);

    fprintf('  alpha=%.4f deg, elevator=%.4f deg\n', xu_lo(3)*180/pi, uu_lo(2)*180/pi);


    %% ===================== Part B: HIFI 配平与仿真 =====================
    % 与 Part A 流程完全一致，但 fi_flag=1 → 使用 HIFI 气动数据
    % 独立配平后运行非线性仿真
    fprintf('\n--- Part B: 配平 & 仿真 HIFI 模型 ---\n');

    fi_flag_Simulink = 1;

    feval('F16_trim', [], [], [], 'lincompile');
    load_system('F16_trim');
    [UX, ~, ~, ~] = fminsearch('trim_fun', UX0, OPTIONS);
    [cost_hi, ~, xu_hi, uu_hi] = trim_fun(UX);
    feval('F16_trim', [], [], [], 'term');

    fprintf('  HIFI 配平完成, cost=%.4e\n', cost_hi);

    t_hi = []; y_hi = [];
    assignin('base', 'fi_flag_Simulink', fi_flag_Simulink);
    assignin('base', 'init_x', xu_hi(1:13));
    assignin('base', 'init_u', uu_hi(1:4));
    assignin('base', 'init_dlef', uu_hi(5));
    assignin('base', 't_disturb', t_doublet);
    assignin('base', 'elevator_disturb', elevator_dist);
    assignin('base', 'aileron_disturb', aileron_dist);
    assignin('base', 'rudder_disturb', rudder_dist);

    % 同 Part A 的 Simulink 配置（详见 Part A 中的注释说明）
    % tout = 时间向量, xout = 13 个连续状态时间历程
    load_system('F16_openloop');
    set_param('F16_openloop', 'StopTime', '30');
    cs = getActiveConfigSet('F16_openloop');
    set_param(cs, 'SaveTime', 'on');
    set_param(cs, 'SaveState', 'on');
    set_param(cs, 'SaveOutput', 'on');
    set_param(cs, 'SaveFormat', 'Array');

    sim('F16_openloop');
    t_hi = tout;
    y_hi = xout;
    close_system('F16_openloop', 0);

    fprintf('  alpha=%.4f deg, elevator=%.4f deg\n', xu_hi(3)*180/pi, uu_hi(2)*180/pi);


    %% ===================== 保存仿真数据 =====================
    % 保存时间历程和配平信息，以便后续分析或与其他结果对比
    save(fullfile(result_dir, 'lab1_simulation_data.mat'), ...
        't_lo', 'y_lo', 't_hi', 'y_hi', ...
        'xu_lo', 'uu_lo', 'xu_hi', 'uu_hi', ...
        't_doublet', 'elevator_dist');

    save_trim_info(result_dir, xu_lo, uu_lo, xu_hi, uu_hi);

    fprintf('\n--- 绘图 ---\n');

    %% ===================== 绘图参数 =====================
    r2d = 180/pi;  % rad→deg 转换因子


    %% ===================== Figure 5: 控制输入 =====================
    % 显示 LOFI 和 HIFI 的四个控制通道：
    %   - 推力 (thrust)
    %   - 升降舵 (elevator) — 包含 Doublet 扰动
    %   - 副翼 (aileron)
    %   - 方向舵 (rudder)
    %
    % LOFI 用蓝色实线，HIFI 用绿色虚线
    % 注意：升降舵图中显示的是总偏角 = 配平舵偏 + Doublet 扰动
    hfig = figure('Name', 'Figure5_Controls', 'NumberTitle', 'off', ...
                  'Position', [100, 100, 900, 700]);

    title_str = sprintf('Trimmed at Velocity = %.1f m/s, Alt. = %.1f m', velocity, altitude);
    sgtitle(title_str);

    thrust_lo = tgear(uu_lo(1));  % 将油门杆位置转换为推力百分比
    thrust_hi = tgear(uu_hi(1));

    elev_trim_lo = uu_lo(2)*r2d;
    elev_trim_hi = uu_hi(2)*r2d;
    aileron_trim_lo = uu_lo(3)*r2d;
    aileron_trim_hi = uu_hi(3)*r2d;
    rudder_trim_lo  = uu_lo(4)*r2d;
    rudder_trim_hi  = uu_hi(4)*r2d;

    % 计算升降舵总偏角（配平值 + Doublet 扰动）
    elev_total_lo = zeros(size(t_doublet));
    elev_total_hi = zeros(size(t_doublet));
    for i = 1:length(t_doublet)
        elev_total_lo(i) = elev_trim_lo + elevator_dist(i)*r2d;
        elev_total_hi(i) = elev_trim_hi + elevator_dist(i)*r2d;
    end

    subplot(2,2,1);
    plot(t_lo, thrust_lo*ones(size(t_lo)), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, thrust_hi*ones(size(t_hi)), 'g--', 'LineWidth', 1.5);
    ylabel('del Thrust');
    title(title_str);
    legend('LOFI', 'HIFI', 'Location', 'best');
    grid on; xlim([0 30]);

    subplot(2,2,2);
    plot(t_doublet, elev_total_lo, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_doublet, elev_total_hi, 'g--', 'LineWidth', 1.5);
    ylabel('del Elevator (deg)');
    grid on; xlim([0 30]);

    subplot(2,2,3);
    plot(t_lo, aileron_trim_lo*ones(size(t_lo)), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, aileron_trim_hi*ones(size(t_hi)), 'g--', 'LineWidth', 1.5);
    ylabel('del Aileron (deg)');
    xlabel('Time (sec)');
    grid on; xlim([0 30]);

    subplot(2,2,4);
    plot(t_lo, rudder_trim_lo*ones(size(t_lo)), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, rudder_trim_hi*ones(size(t_hi)), 'g--', 'LineWidth', 1.5);
    ylabel('del Rudder (deg)');
    xlabel('Time (sec)');
    grid on; xlim([0 30]);

    annotation('textbox', [0.1, 0.01, 0.8, 0.04], ...
        'String', ['Figure 5: The control input, a 5 degree elevator doublet, ', ...
                   'to the low fidelity model (solid blue line) and the high fidelity ', ...
                   '(dashed green line) F-16 model at trim altitude of ', ...
                   num2str(altitude), ' m and trim velocity of ', num2str(velocity), ' m/s.'], ...
        'FontSize', 9, 'HorizontalAlignment', 'center', ...
        'LineStyle', 'none');

    save_png_fig(hfig, fullfile(result_dir, 'figure5_controls'));


    %% ===================== Figure 6: 位置与姿态角 =====================
    % y_lo/y_hi 中的列索引对应状态向量：
    %   10: xE (东向位置), 11: yE (北向位置), 12: -h (负高度)
    %   4: φ (滚转角), 5: θ (俯仰角), 6: ψ (偏航角)
    %
    % 关注点：
    %   - North/East 位置变化：显示飞机的水平机动轨迹
    %   - Altitude 高度：Doublet 扰动导致的高度变化（长周期响应）
    %   - Phi/Theta/Psi 姿态角：对 Doublet 的响应和耦合效应
    hfig2 = figure('Name', 'Figure6_Positions', 'NumberTitle', 'off', ...
                   'Position', [150, 150, 1000, 600]);

    subplot(2,3,1);
    plot(t_lo, y_lo(:,10), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,10), 'g--', 'LineWidth', 1.5);
    ylabel('North Pos. (m)');
    title(['Trimmed at Velocity = ' num2str(velocity) ' m/s, Alt. = ' num2str(altitude) ' m']);
    legend('LOFI', 'HIFI', 'Location', 'best'); grid on;

    subplot(2,3,2);
    plot(t_lo, y_lo(:,11), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,11), 'g--', 'LineWidth', 1.5);
    ylabel('East Pos. (m)'); grid on;

    subplot(2,3,3);
    plot(t_lo, -y_lo(:,12), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, -y_hi(:,12), 'g--', 'LineWidth', 1.5);
    ylabel('Altitude (m)'); grid on;

    subplot(2,3,4);
    plot(t_lo, y_lo(:,4)*r2d, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,4)*r2d, 'g--', 'LineWidth', 1.5);
    ylabel('Phi (deg)'); xlabel('Time (sec)'); grid on;

    subplot(2,3,5);
    plot(t_lo, y_lo(:,5)*r2d, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,5)*r2d, 'g--', 'LineWidth', 1.5);
    ylabel('Theta (deg)'); xlabel('Time (sec)'); grid on;

    subplot(2,3,6);
    plot(t_lo, y_lo(:,6)*r2d, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,6)*r2d, 'g--', 'LineWidth', 1.5);
    ylabel('Psi (deg)'); xlabel('Time (sec)'); grid on;

    save_png_fig(hfig2, fullfile(result_dir, 'figure6_positions'));


    %% ===================== Figure 7: 速度与角速率 =====================
    % 显示 LOFI vs HIFI 的速度和角速率响应对比
    %   1: V (空速), 2: β (侧滑角), 3: α (迎角)
    %   7: p (滚转角速率), 8: q (俯仰角速率), 9: r (偏航角速率)
    %
    % 预测差异：
    %   - α 和 q（纵向）：HIFI 的短周期振荡更快（ω_n↑），但阻尼更小（ζ↓）
    %   - V（速度）：两套模型的差异会随时间积累（长周期不同）
    %   - β, p, r（横侧向）：由于对称输入，本应零响应，但实际存在耦合
    hfig3 = figure('Name', 'Figure7_Rates', 'NumberTitle', 'off', ...
                   'Position', [200, 200, 1000, 600]);

    subplot(2,3,1);
    plot(t_lo, y_lo(:,1), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,1), 'g--', 'LineWidth', 1.5);
    ylabel('Velocity (m/s)');
    title(['Trimmed at Velocity = ' num2str(velocity) ' m/s, Alt. = ' num2str(altitude) ' m']);
    legend('LOFI', 'HIFI', 'Location', 'best'); grid on;

    subplot(2,3,2);
    plot(t_lo, y_lo(:,3)*r2d, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,3)*r2d, 'g--', 'LineWidth', 1.5);
    ylabel('Angle of Attack (deg)'); grid on;

    subplot(2,3,3);
    plot(t_lo, y_lo(:,2)*r2d, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,2)*r2d, 'g--', 'LineWidth', 1.5);
    ylabel('Side Slip (deg)'); grid on;

    subplot(2,3,4);
    plot(t_lo, y_lo(:,7)*r2d, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,7)*r2d, 'g--', 'LineWidth', 1.5);
    ylabel('Roll Rate (deg/s)'); xlabel('Time (sec)'); grid on;

    subplot(2,3,5);
    plot(t_lo, y_lo(:,8)*r2d, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,8)*r2d, 'g--', 'LineWidth', 1.5);
    ylabel('Pitch Rate (deg/s)'); xlabel('Time (sec)'); grid on;

    subplot(2,3,6);
    plot(t_lo, y_lo(:,9)*r2d, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_hi, y_hi(:,9)*r2d, 'g--', 'LineWidth', 1.5);
    ylabel('Yaw Rate (deg/s)'); xlabel('Time (sec)'); grid on;

    save_png_fig(hfig3, fullfile(result_dir, 'figure7_rates'));


    %% ===================== Figure: 综合对比图 =====================
    % 将关键状态量排列在 3×3 网格中，便于一次性对比 LOFI vs HIFI
    % 包含：高度、速度、迎角、俯仰角、滚转角、偏航角、滚转/俯仰/偏航角速率
    hfig4 = figure('Name', 'Summary', 'NumberTitle', 'off', ...
                   'Position', [250, 250, 1000, 700]);

    subplot(3,3,1);
    plot(t_lo, -y_lo(:,12), 'b-', t_hi, -y_hi(:,12), 'g--', 'LineWidth', 1.2);
    ylabel('Alt (m)'); title('Summary: LOFI vs HIFI');
    legend('LOFI', 'HIFI', 'Location', 'best'); grid on;

    subplot(3,3,2);
    plot(t_lo, y_lo(:,1), 'b-', t_hi, y_hi(:,1), 'g--', 'LineWidth', 1.2);
    ylabel('V (m/s)'); grid on;

    subplot(3,3,3);
    plot(t_lo, y_lo(:,3)*r2d, 'b-', t_hi, y_hi(:,3)*r2d, 'g--', 'LineWidth', 1.2);
    ylabel('\alpha (deg)'); grid on;

    subplot(3,3,4);
    plot(t_lo, y_lo(:,5)*r2d, 'b-', t_hi, y_hi(:,5)*r2d, 'g--', 'LineWidth', 1.2);
    ylabel('\theta (deg)'); grid on;

    subplot(3,3,5);
    plot(t_lo, y_lo(:,4)*r2d, 'b-', t_hi, y_hi(:,4)*r2d, 'g--', 'LineWidth', 1.2);
    ylabel('\phi (deg)'); grid on;

    subplot(3,3,6);
    plot(t_lo, y_lo(:,6)*r2d, 'b-', t_hi, y_hi(:,6)*r2d, 'g--', 'LineWidth', 1.2);
    ylabel('\psi (deg)'); grid on;

    subplot(3,3,7);
    plot(t_lo, y_lo(:,8)*r2d, 'b-', t_hi, y_hi(:,8)*r2d, 'g--', 'LineWidth', 1.2);
    ylabel('q (deg/s)'); xlabel('Time (s)'); grid on;

    subplot(3,3,8);
    plot(t_lo, y_lo(:,7)*r2d, 'b-', t_hi, y_hi(:,7)*r2d, 'g--', 'LineWidth', 1.2);
    ylabel('p (deg/s)'); xlabel('Time (s)'); grid on;

    subplot(3,3,9);
    plot(t_lo, y_lo(:,9)*r2d, 'b-', t_hi, y_hi(:,9)*r2d, 'g--', 'LineWidth', 1.2);
    ylabel('r (deg/s)'); xlabel('Time (s)'); grid on;

    save_png_fig(hfig4, fullfile(result_dir, 'figure_summary'));

    fprintf('  所有图表已保存\n');
    fprintf('\n========================================\n');
    fprintf('  步骤3完成!\n');
    fprintf('========================================\n');

end


%% ===================== 子函数 =====================

function save_png_fig(h, basename)
    % 同时保存 .png 和 .fig 格式
    % .png 用于快速查看（文档/报告）
    % .fig 用于后续 MATLAB 中二次编辑
    saveas(h, [basename '.png']);
    savefig(h, [basename '.fig']);
end

function save_trim_info(out_dir, xu_lo, uu_lo, xu_hi, uu_hi)
    % 保存仿真配平信息到文本文件
    % 包含 LOFI 和 HIFI 的配平参数对比
    fid = fopen(fullfile(out_dir, 'lab1_trim_info.txt'), 'w');
    fprintf(fid, '========================================\n');
    fprintf(fid, '  F-16 配平信息 - 实验1 步骤3\n');
    fprintf(fid, '========================================\n\n');
    fprintf(fid, '飞行条件: H = 5000 m, V = 200 m/s\n\n');

    fprintf(fid, '--- LOFI ---\n');
    fprintf(fid, '  alpha    = %.4f deg\n', xu_lo(3)*180/pi);
    fprintf(fid, '  theta    = %.4f deg\n', xu_lo(5)*180/pi);
    fprintf(fid, '  elevator = %.4f deg\n', uu_lo(2)*180/pi);
    fprintf(fid, '  aileron  = %.4f deg\n', uu_lo(3)*180/pi);
    fprintf(fid, '  rudder   = %.4f deg\n', uu_lo(4)*180/pi);
    fprintf(fid, '  throttle = %.4f\n', uu_lo(1));
    fprintf(fid, '  thrust   = %.4f %%\n', tgear(uu_lo(1)));
    fprintf(fid, '  power    = %.4f %%\n', xu_lo(13));
    fprintf(fid, '  dLEF     = %.4f deg\n\n', uu_lo(5)*180/pi);

    fprintf(fid, '--- HIFI ---\n');
    fprintf(fid, '  alpha    = %.4f deg\n', xu_hi(3)*180/pi);
    fprintf(fid, '  theta    = %.4f deg\n', xu_hi(5)*180/pi);
    fprintf(fid, '  elevator = %.4f deg\n', uu_hi(2)*180/pi);
    fprintf(fid, '  aileron  = %.4f deg\n', uu_hi(3)*180/pi);
    fprintf(fid, '  rudder   = %.4f deg\n', uu_hi(4)*180/pi);
    fprintf(fid, '  throttle = %.4f\n', uu_hi(1));
    fprintf(fid, '  thrust   = %.4f %%\n', tgear(uu_hi(1)));
    fprintf(fid, '  power    = %.4f %%\n', xu_hi(13));
    fprintf(fid, '  dLEF     = %.4f deg\n\n', uu_hi(5)*180/pi);

    fprintf(fid, '输入: 5 deg 升降舵 Doublet\n');
    fclose(fid);
end
