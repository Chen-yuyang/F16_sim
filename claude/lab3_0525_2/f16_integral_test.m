%% f16_integral_test.m — 积分控制参数调试与对比测试
%  功能:
%    尝试不同积分参数 (积分极点位置 / LQR 积分权重),
%    测试积分控制能否消除稳态误差, 记录 Ki 值和仿真结果。
%
%  测试内容:
%    1. place_i: 积分极点取不同值 [-0.2, -0.5, -1.0, -2.0] 对比
%    2. lqr_i:   积分权重取不同值 [1, 5, 10, 50] 对比
%    3. 与纯比例控制 (place, lqr) 对比稳态误差
%    4. 绘制对比曲线
%
%  用法:
%    >> f16_integral_test          % 运行全部测试
%    >> f16_integral_test('plot')  % 仅重新绘图 (使用已有数据)
%
%  生成文件:
%    test_models/IntTest_*.slx     — 每个测试的 Simulink 模型
%    results_integral.mat          — 测试结果数据
%
%  积分控制原理:
%    增广系统 [dx/dt; dx_I/dt] = [A 0; -C_θ 0][x; x_I] + [B; 0]u
%    控制律: u = u_trim + K*(x_ref - x) + K_i*∫(θ_ref - θ)dt
%    积分极点越靠近原点 (如 -0.2), 积分越慢但越稳定;
%    积分极点越远离原点 (如 -2.0), 积分越快但 Ki 越大, 易振荡。
%
%  参考:
%    f16_controller_design.m — method='place_i'/'lqr_i' 的实现
%==========================================================================
clear; clc;

%% ==== 配置 ====
this_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(fileparts(this_dir));
addpath(this_dir);
addpath(proj_root);
addpath(fullfile(proj_root,'aerodata'));
addpath(fullfile(proj_root,'trae','lab1_0429'));
cd(proj_root);

% 测试用例定义
% place_i 测试: 不同积分极点
test_place_i = [
    -0.2;   % 很慢的积分, Ki 很小, 最稳定但收敛慢
    -0.5;   % ★ 推荐值: 平衡稳定性和收敛速度
    -1.0;   % 中等速度, Ki 开始变大
    -2.0    % 原值: 太快, Ki 过大, 可能振荡
];

% lqr_i 测试: 不同积分权重 Q(6,6)
test_lqr_i = [
    1;      % 很轻的积分惩罚, Ki 很小
    5;      % 中等
    10;     % ★ 推荐值
    50      % 原值: 惩罚偏重, Ki 偏大
];

%% ==== 运行测试 ====
fprintf('=============================================\n');
fprintf('  F-16 积分控制参数调试与对比测试\n');
fprintf('=============================================\n\n');

% --- 基准测试: 纯比例控制 (无积分) ---
fprintf('--- 基准: 纯比例控制 ---\n');
fprintf('  1. LOFI + place (无积分)... ');
r_place = f16_controller_design('LOFI', 'place');
try
    f16_build_simulink('IntTest_place');
    assignin('base','fb_gain',1);
    f16_build_simulink('IntTest_place');
    out = sim('IntTest_place','StopTime','30');
    d = out.yout{1}.Values.Data; t = out.tout;
    ss = t>=10; th_place = mean(d(ss,5))*180/pi;
    err_place = 3.0 - th_place;
    fprintf('θ_ss=%.2f° err=%.2f° [%s]\n', th_place, err_place, ...
        iif(abs(err_place)<0.5,'OK','偏差'));
    close_system('IntTest_place',0);
catch ME
    fprintf('CRASH: %s\n', ME.message);
end

fprintf('  2. LOFI + lqr (无积分)... ');
try
    r_lqr = f16_controller_design('LOFI', 'lqr');
    f16_build_simulink('IntTest_lqr');
    assignin('base','fb_gain',1);
    f16_build_simulink('IntTest_lqr');
    out = sim('IntTest_lqr','StopTime','30');
    d = out.yout{1}.Values.Data; t = out.tout;
    ss = t>=10; th_lqr = mean(d(ss,5))*180/pi;
    err_lqr = 3.0 - th_lqr;
    fprintf('θ_ss=%.2f° err=%.2f° [%s]\n', th_lqr, err_lqr, ...
        iif(abs(err_lqr)<0.5,'OK','偏差'));
    close_system('IntTest_lqr',0);
catch ME
    fprintf('CRASH: %s\n', ME.message);
end

% --- place_i 测试: 扫描积分极点 ---
fprintf('\n--- place_i: 扫描积分极点 ---\n');
results_place_i = [];
for i = 1:length(test_place_i)
    p_int = test_place_i(i);
    tag = sprintf('IntP%.1f', p_int);
    fprintf('  place_i p_int=%.1f (Ki)... ', p_int);

    % ★ 修改 f16_controller_design 的内部极点
    % 这里直接调用一个修改版的控制器设计
    % 由于我们无法轻易修改函数内部, 我们手动构造增广系统
    try
        [A,B,~,~,dlef,fi,~] = f16_longitudinal_model('LOFI');
        C_theta=[0,0,1,0,0];
        A_aug=[A, zeros(5,1); -C_theta, 0];
        B_aug=[B; 0, 0];
        p_des=[-2.1+2.14i -2.1-2.14i -0.8+0.6i -0.8-0.6i -10 p_int];
        K_aug=place(A_aug,B_aug,p_des);
        K_manual=K_aug(:,1:5);
        Ki_val=K_aug(:,6);

        % 写 base workspace (复用 f16_controller_design 的部分流程)
        r = f16_controller_design('LOFI', 'manual', K_manual);
        assignin('base','Ki_long',Ki_val);
        assignin('base','K_long',K_manual);

        % 构建仿真 (含积分)
        f16_build_simulink(['IntTest_' tag], true);
        assignin('base','fb_gain',1);
        f16_build_simulink(['IntTest_' tag], true);
        out = sim(['IntTest_' tag],'StopTime','30');
        d = out.yout{1}.Values.Data; t = out.tout;
        ss = t>=10; th = mean(d(ss,5))*180/pi;
        err = 3.0 - th;

        fprintf('θ_ss=%.2f° err=%.2f° Ki(2)=%.4f [%s]\n', th, err, Ki_val(2), ...
            iif(abs(err)<0.5,'OK',iif(abs(err)<1.0,'偏差','FAIL')));
        results_place_i(end+1) = struct('p_int',p_int,'Ki',Ki_val(2),...
            'theta_ss',th,'err',err,'data',d,'tout',t,'tag',tag,'status','OK');
        close_system(['IntTest_' tag],0);
    catch ME
        fprintf('CRASH: %s\n', ME.message);
        results_place_i(end+1) = struct('p_int',p_int,'Ki',NaN,...
            'theta_ss',NaN,'err',NaN,'data',[],'tout',[],'tag',tag,'status','CRASH');
    end
end

% --- lqr_i 测试: 扫描积分权重 ---
fprintf('\n--- lqr_i: 扫描积分权重 ---\n');
results_lqr_i = [];
for i = 1:length(test_lqr_i)
    Q_int = test_lqr_i(i);
    tag = sprintf('IntQ%.0f', Q_int);
    fprintf('  lqr_i Q_int=%.0f (Ki)... ', Q_int);

    try
        [A,B,~,~,dlef,fi,~] = f16_longitudinal_model('LOFI');
        C_theta=[0,0,1,0,0];
        A_aug=[A, zeros(5,1); -C_theta, 0];
        B_aug=[B; 0, 0];
        Q_aug=diag([0.1,1,100,10,0.1,Q_int]); R=diag([0.5,0.5]);
        K_aug=lqr(A_aug,B_aug,Q_aug,R);
        K_manual=K_aug(:,1:5);
        Ki_val=K_aug(:,6);

        r = f16_controller_design('LOFI', 'manual', K_manual);
        assignin('base','Ki_long',Ki_val);
        assignin('base','K_long',K_manual);

        f16_build_simulink(['IntTest_' tag], true);
        assignin('base','fb_gain',1);
        f16_build_simulink(['IntTest_' tag], true);
        out = sim(['IntTest_' tag],'StopTime','30');
        d = out.yout{1}.Values.Data; t = out.tout;
        ss = t>=10; th = mean(d(ss,5))*180/pi;
        err = 3.0 - th;

        fprintf('θ_ss=%.2f° err=%.2f° Ki(2)=%.4f [%s]\n', th, err, Ki_val(2), ...
            iif(abs(err)<0.5,'OK',iif(abs(err)<1.0,'偏差','FAIL')));
        results_lqr_i(end+1) = struct('Q_int',Q_int,'Ki',Ki_val(2),...
            'theta_ss',th,'err',err,'data',d,'tout',t,'tag',tag,'status','OK');
        close_system(['IntTest_' tag],0);
    catch ME
        fprintf('CRASH: %s\n', ME.message);
        results_lqr_i(end+1) = struct('Q_int',Q_int,'Ki',NaN,...
            'theta_ss',NaN,'err',NaN,'data',[],'tout',[],'tag',tag,'status','CRASH');
    end
end

%% ==== 汇总 ====
fprintf('\n=============================================\n');
fprintf('  积分控制测试汇总\n');
fprintf('=============================================\n');
fprintf('\n--- 基准 (纯比例) ---\n');
fprintf('  place: θ_ss=%.2f° err=%.2f°\n', th_place, err_place);
fprintf('  lqr:   θ_ss=%.2f° err=%.2f°\n', th_lqr, err_lqr);

fprintf('\n--- place_i 积分极点扫描 ---\n');
fprintf('  %-12s %8s %12s %10s %6s\n', '积分极点', 'Ki(2)', 'θ_ss', '稳态误差', '状态');
for i = 1:length(results_place_i)
    r = results_place_i(i);
    fprintf('  %-12s %8.4f %10.2f° %10.2f° %6s\n', ...
        num2str(r.p_int), r.Ki, r.theta_ss, r.err, r.status);
end

fprintf('\n--- lqr_i 积分权重扫描 ---\n');
fprintf('  %-12s %8s %12s %10s %6s\n', 'Q_int', 'Ki(2)', 'θ_ss', '稳态误差', '状态');
for i = 1:length(results_lqr_i)
    r = results_lqr_i(i);
    fprintf('  %-12s %8.4f %10.2f° %10.2f° %6s\n', ...
        num2str(r.Q_int), r.Ki, r.theta_ss, r.err, r.status);
end

%% ==== 绘图 ====
figure('Name','积分控制对比','Position',[30,30,1000,700]);

% 子图1: θ 响应对比 (不同积分极点)
subplot(2,2,1); hold on;
if exist('r_place','var')
    % 重新跑基准以获取数据 (或直接用已保存的)
    try
        r0 = f16_controller_design('LOFI','place');
        f16_build_simulink('IntTest_base');
        assignin('base','fb_gain',1);
        f16_build_simulink('IntTest_base');
        out0 = sim('IntTest_base','StopTime','30');
        d0 = out0.yout{1}.Values.Data;
        plot(out0.tout, d0(:,5)*180/pi, 'k-', 'LineWidth',1.2, 'DisplayName','place(无积分)');
        close_system('IntTest_base',0);
    catch; end
end
colors = {'r','b','g','m'};
for i = 1:length(results_place_i)
    if ~isempty(results_place_i(i).data)
        plot(results_place_i(i).tout, results_place_i(i).data(:,5)*180/pi, ...
            colors{i}, 'LineWidth',1.2, ...
            'DisplayName', sprintf('place_i p_i=%.1f', results_place_i(i).p_int));
    end
end
yline(3, 'g--', 'LineWidth',1.5, 'DisplayName','θ_{ref}=3°');
xlabel('时间(s)'); ylabel('θ(°)'); title('不同积分极点对比'); grid on; legend('Location','southeast');

% 子图2: θ 响应对比 (不同LQR积分权重)
subplot(2,2,2); hold on;
if exist('out0','var')
    plot(out0.tout, d0(:,5)*180/pi, 'k-', 'LineWidth',1.2, 'DisplayName','lqr(无积分)');
end
for i = 1:length(results_lqr_i)
    if ~isempty(results_lqr_i(i).data)
        plot(results_lqr_i(i).tout, results_lqr_i(i).data(:,5)*180/pi, ...
            colors{i}, 'LineWidth',1.2, ...
            'DisplayName', sprintf('lqr_i Q_I=%.0f', results_lqr_i(i).Q_int));
    end
end
yline(3, 'g--', 'LineWidth',1.5, 'DisplayName','θ_{ref}=3°');
xlabel('时间(s)'); ylabel('θ(°)'); title('不同LQR积分权重对比'); grid on; legend('Location','southeast');

% 子图3: 积分极点 vs Ki、稳态误差
subplot(2,2,3); hold on;
yyaxis left;
p_vals = [results_place_i.p_int]; ki_vals = [results_place_i.Ki];
plot(p_vals, abs(ki_vals), 'b-o', 'LineWidth',1.2);
ylabel('|Ki(2)| (对数)'); set(gca,'YScale','log');
yyaxis right;
err_vals = [results_place_i.err];
plot(p_vals, abs(err_vals), 'r-s', 'LineWidth',1.2);
ylabel('稳态误差 |e| (°)');
xlabel('积分极点位置'); title('积分极点→Ki(2)与稳态误差'); grid on;
xline(-0.5,'g--','推荐'); legend('|Ki(2)|','稳态误差','推荐');

% 子图4: 积分权重 vs Ki、稳态误差
subplot(2,2,4); hold on;
yyaxis left;
q_vals = [results_lqr_i.Q_int]; ki_vals2 = [results_lqr_i.Ki];
plot(q_vals, abs(ki_vals2), 'b-o', 'LineWidth',1.2);
ylabel('|Ki(2)|');
yyaxis right;
err_vals2 = [results_lqr_i.err];
plot(q_vals, abs(err_vals2), 'r-s', 'LineWidth',1.2);
ylabel('稳态误差 |e| (°)');
xlabel('LQR积分权重 Q_I'); title('积分权重→Ki(2)与稳态误差'); grid on;
xline(10,'g--','推荐'); legend('|Ki(2)|','稳态误差','推荐');

sgtitle('F-16 积分控制参数调试对比 (LOFI 模型)');

%% ==== 保存结果 ====
save(fullfile(this_dir,'results_integral.mat'), ...
    'results_place_i','results_lqr_i','test_place_i','test_lqr_i',...
    'th_place','err_place','th_lqr','err_lqr');
fprintf('\n  结果已保存到 results_integral.mat\n');
fprintf('=============================================\n');

function s=iif(c,t,f); if c; s=t; else; s=f; end; end
