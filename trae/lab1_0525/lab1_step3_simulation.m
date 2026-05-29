function lab1_step3_simulation()
%==========================================================================
%  实验1 - 步骤3：非线性开环仿真验证（带升降舵 Doublet 扰动注入）
%  使用 5 deg 升降舵 Doublet 脉冲输入，对比 LOFI 与 HIFI 模型的时域响应
%  生成三张标准图（Figure 5/6/7）和一张综合对比图
%
%  【与 lab1_0429 版本的区别】
%  本脚本使用 trae/lab1_0525/F16_openloop.slx，该模型在 Trimmed Input 子
%  系统中加入了 Elev_Disturb FromWorkspace 模块，通过 Sum_Disturb 将升降舵
%  Doublet 扰动真正注入到舵面指令中（旧模型的扰动模块未连接）。
%
%  【Doublet 信号】
%  形式: 0 ↑ +5° ↓ -5° ↑ 0  (升降舵的"正-负-回零"脉冲序列)
%  时间: 0-1s:0°, 1-2s:+5°, 2-5s:-5°, 5-6s:0°, 6-30s:0°
%  作用: 激励飞机的纵向响应（同时产生部分横侧向耦合）
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
    fprintf('  (含升降舵 Doublet 扰动注入, lab1_0525)\n');
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

    OPTIONS = optimset('TolFun', 1e-10, 'TolX', 1e-10, 'MaxFunEvals', 5000, ...
                       'MaxIter', 1000, 'Display', 'off');

    warning('off', 'Simulink:Engine:OutputNotConnected');
    warning('off', 'Simulink:Engine:BlockOutputNotConnected');

    %% ===================== 升降舵 Doublet 信号 =====================
    % elev_dist_data 格式: 2 列矩阵 [时间, 信号值]，符合 FromWorkspace 块要求
    t_doublet = [0, 1, 1, 2, 2, 5, 5, 6, 6, 30];
    doublet_amp = 5 * pi/180;
    elevator_dist = [0, 0, doublet_amp, doublet_amp, -doublet_amp, -doublet_amp, 0, 0, 0, 0];
    elev_dist_data = [t_doublet', elevator_dist'];

    beta = 0; elevator = 0*pi/180; alpha = 10*pi/180;
    rudder = 0; aileron = 0; dth = 0.2;
    UX0 = [beta; elevator; alpha; aileron; rudder; dth];


    %% ===================== Part A: LOFI 配平与仿真 =====================
    fprintf('--- Part A: 配平 & 仿真 LOFI 模型 ---\n');

    fi_flag_Simulink = 0;

    feval('F16_trim', [], [], [], 'lincompile');
    load_system('F16_trim');
    [UX, ~, ~, ~] = fminsearch('trim_fun', UX0, OPTIONS);
    [cost_lo, ~, xu_lo, uu_lo] = trim_fun(UX);
    feval('F16_trim', [], [], [], 'term');

    fprintf('  LOFI 配平完成, cost=%.4e\n', cost_lo);

    t_lo = []; y_lo = [];

    assignin('base', 'fi_flag_Simulink', fi_flag_Simulink);
    assignin('base', 'init_x', xu_lo(1:13));
    assignin('base', 'init_u', uu_lo(1:4));
    assignin('base', 'init_dlef', uu_lo(5));
    assignin('base', 'elev_dist_data', elev_dist_data);  % Doublet 扰动数据

    % 修改后的模型文件名（区别于原始 F16_openloop.slx）
    dist_model = 'F16_openloop_dist';

    % 加载修改后的模型 (含 Elev_Disturb + Sum_Disturb)
    model_path = fullfile(result_dir, [dist_model '.slx']);
    if bdIsLoaded(dist_model)
        close_system(dist_model, 0);
    end
    load_system(model_path);
    set_param(dist_model, 'StopTime', '30');
    cs = getActiveConfigSet(dist_model);
    set_param(cs, 'SaveTime', 'on');
    set_param(cs, 'SaveState', 'on');
    set_param(cs, 'SaveOutput', 'on');
    set_param(cs, 'SaveFormat', 'Array');

    sim(dist_model);
    t_lo = tout;
    y_lo = xout;
    close_system(dist_model, 0);

    fprintf('  alpha=%.4f deg, elevator=%.4f deg\n', xu_lo(3)*180/pi, uu_lo(2)*180/pi);


    %% ===================== Part B: HIFI 配平与仿真 =====================
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
    assignin('base', 'elev_dist_data', elev_dist_data);

    load_system(model_path);
    set_param(dist_model, 'StopTime', '30');
    cs = getActiveConfigSet(dist_model);
    set_param(cs, 'SaveTime', 'on');
    set_param(cs, 'SaveState', 'on');
    set_param(cs, 'SaveOutput', 'on');
    set_param(cs, 'SaveFormat', 'Array');

    sim(dist_model);
    t_hi = tout;
    y_hi = xout;
    close_system(dist_model, 0);

    fprintf('  alpha=%.4f deg, elevator=%.4f deg\n', xu_hi(3)*180/pi, uu_hi(2)*180/pi);


    %% ===================== 保存仿真数据 =====================
    save(fullfile(result_dir, 'lab1_simulation_data.mat'), ...
        't_lo', 'y_lo', 't_hi', 'y_hi', ...
        'xu_lo', 'uu_lo', 'xu_hi', 'uu_hi', ...
        't_doublet', 'elevator_dist');

    save_trim_info(result_dir, xu_lo, uu_lo, xu_hi, uu_hi);

    fprintf('\n--- 绘图 ---\n');

    %% ===================== 绘图参数 =====================
    r2d = 180/pi;


    %% ===================== Figure 5: 控制输入 =====================
    hfig = figure('Name', 'Figure5_Controls', 'NumberTitle', 'off', ...
                  'Position', [100, 100, 900, 700]);

    title_str = sprintf('Trimmed at Velocity = %.1f m/s, Alt. = %.1f m', velocity, altitude);
    sgtitle(title_str);

    thrust_lo = tgear(uu_lo(1));
    thrust_hi = tgear(uu_hi(1));

    elev_trim_lo = uu_lo(2)*r2d;
    elev_trim_hi = uu_hi(2)*r2d;
    aileron_trim_lo = uu_lo(3)*r2d;
    aileron_trim_hi = uu_hi(3)*r2d;
    rudder_trim_lo  = uu_lo(4)*r2d;
    rudder_trim_hi  = uu_hi(4)*r2d;

    elev_total_lo = elev_trim_lo + elevator_dist*r2d;
    elev_total_hi = elev_trim_hi + elevator_dist*r2d;

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
    saveas(h, [basename '.png']);
    savefig(h, [basename '.fig']);
end

function save_trim_info(out_dir, xu_lo, uu_lo, xu_hi, uu_hi)
    fid = fopen(fullfile(out_dir, 'lab1_trim_info.txt'), 'w');
    fprintf(fid, '========================================\n');
    fprintf(fid, '  F-16 配平信息 - 实验1 步骤3 (lab1_0525)\n');
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

    fprintf(fid, '输入: 5 deg 升降舵 Doublet (通过 Elev_Disturb 注入)\n');
    fclose(fid);
end
