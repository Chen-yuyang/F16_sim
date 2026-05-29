%% f16_demo.m — F-16 俯仰角闭环控制 完整演示 (含伺服作动器)
%==========================================================================
%  功能: 一键运行 F-16 俯仰角控制的完整流程
%        (加载模型 → 稳定性分析 → 设计控制器 → 构建 Simulink →
%         开环仿真 → 闭环仿真 → 绘图对比)
%  用法:
%    直接运行: >> f16_demo
%    切换模型: 修改下方 MODEL 变量 ('LOFI' 或 'HIFI')
%    切换方法: 修改下方 METHOD 变量
%  控制律:
%    u(油门,升降舵) = u_base + fb_gain * K_long * (x_ref_long - x_long)
%    其中 x_long = [V, α, θ, q, Pow]' 是纵向 5 维状态子集
%    含积分时升降舵额外 + Ki_long(2) * ∫(θ_ref - θ)dt
%  伺服作动器:
%    Rate Limiter(±60°/s) → 一阶 Transfer Fcn(20.2/(s+20.2)) → Saturation
%    模拟真实液压舵机的速率限制和延迟特性
%==========================================================================

clear; clc;  % 清空工作空间和命令窗口, 确保无残留变量干扰

% --- 路径设置 ---
% fileparts(mfilename('fullpath')) 获取当前脚本所在目录的完整路径
% 本例中 = 'e:\...\claude\lab3_0525_2'
this_dir = fileparts(mfilename('fullpath'));
% 上两级目录 = 'e:\...\claude\' 的上一级 = 'e:\...\FC_SimCode_1'
% 这是项目根目录, F16_dyn.mexw64 和 aerodata/ 都在这里
proj_root = fileparts(fileparts(this_dir));

% 将各目录加入 MATLAB 搜索路径, 确保所有函数和数据文件可访问
% 注意路径优先级: 最后添加的优先级最高 (MATLAB 会优先搜索后添加的路径)
addpath(this_dir);                              % 本目录脚本 (最高优先级, 后加)
addpath(proj_root);                             % 项目根 → F16_dyn.mexw64 (S-Function)
addpath(fullfile(proj_root,'aerodata'));        % 气动数据目录 → HIFI .dat 文件
addpath(fullfile(proj_root,'trae','lab1_0429')); % lab1 数据 → lab1_matrices.mat
cd(proj_root);  % 必须 cd 到根目录, 因为 HIFI 模型的 .dat 文件路径基于当前目录

%% ====== 用户配置区 ======
% 用户可在此区域自由修改参数, 无需修改下方代码
MODEL  = 'HIFI';      % 选择气动模型: 'LOFI'(低保真)或'HIFI'(高保真)
                      %   LOFI: 硬编码气动系数, 查表带钳位, 仿真快
                      %   HIFI: .dat 风洞数据 + getHyperCube 多维插值, 更真实
METHOD = 'place';     % 控制器设计方法:
                      %   'place'   → 极点配置 (默认, 指定闭环极点位置)
                      %   'lqr'     → LQR 最优控制 (通过 Q/R 权重优化)
                      %   'manual'  → 手动指定 K 矩阵
                      %   'place_i' → 极点配置 + 积分控制 (消除稳态误差)
                      %   'lqr_i'   → LQR + 积分控制 (消除稳态误差)
USE_EXACT = false;    % true=使用精确平衡解方案 (求解 Ax_d+Bu_d=0 得到参考状态)
                      % false=使用简单方案 (仅将配平 θ 改为目标值, 其余不变)

% 判断当前方法是否需要积分控制 (place_i 或 lqr_i)
% 积分控制需要 Simulink 模型包含积分器模块 (∫(θ_ref-θ)dt)
use_int = any(strcmpi(METHOD,{'place_i','lqr_i'}));

% 手动 K 矩阵 (仅在 METHOD='manual' 时使用)
% 格式: 2×5 矩阵, 对应 [δth, δe] × [V, α, θ, q, Pow]
% 默认值: 油门通道全零, 升降舵通道 θ→δe=-0.8, q→δe=+0.1
manual_K  = [0 0 0 0 0; 0 0 -0.8 0.1 0];
% =========================

% 打印演示标题和配置信息
fprintf('========================================\n');
fprintf('  F-16 俯仰角闭环控制 — 完整演示\n');
fprintf('  模型: %s | 方法: %s\n', MODEL, METHOD);
fprintf('========================================\n\n');

%% 步骤1: 加载纵向线性模型
% f16_longitudinal_model 从 lab1_matrices.mat 中加载 A(5×5), B(5×2)
% 以及配平状态 x_trim(13×1), 配平控制 u_trim(4×1) 等
fprintf('=== 步骤1: 加载纵向线性模型 ===');
[A, B, x_trim, u_trim, dlef, fi, label] = f16_longitudinal_model(MODEL);
% 输出说明:
%   A(5×5) — 状态矩阵 (V, α, θ, q, Pow 之间的耦合关系)
%   B(5×2) — 控制矩阵 (δth 油门, δe 升降舵对各状态的影响)
%   x_trim(13×1) — 配平状态 (全 13 维, 包含所有状态)
%   u_trim(4×1)  — 配平控制 [δth; δe; δa; δr]
%   dlef — 前缘襟翼偏转角 (配平值)
%   fi   — HIFI 标志 (0=LOFI, 1=HIFI)
%   label — 模型名称字符串

%% 步骤2: 开环稳定性分析
% 计算 A 矩阵的特征值, 判断开环系统是否稳定
% 输出: 特征值实部/虚部, 自然频率 ω_n, 阻尼比 ζ, 时间常数 τ
fprintf('=== 步骤2: 开环稳定性分析 ===');
f16_stability_analysis(A, label);

%% 步骤3: 控制器设计
% f16_controller_design 负责:
%   1. 计算状态反馈增益 K(2×5)
%   2. 计算参考状态 x_ref_long(5×1) 和前馈基线 u_base(4×1)
%   3. 所有变量写入 MATLAB base workspace (Simulink 通过变量名读取)
%   4. 含积分控制时生成 Ki_long(2×1) 积分增益
fprintf('=== 步骤3: 控制器设计 ===');
if strcmpi(METHOD,'manual')
    % manual 方法需要传入手动指定的 K 矩阵 [2×5]
    % varargin 中传入 manual_K
    r = f16_controller_design(MODEL, 'manual', manual_K);
else
    % 其他方法 (place/lqr/place_i/lqr_i) 不需要额外参数
    % 如果需要精确平衡解方案, 额外传入 'exact' 参数
    if USE_EXACT
        r = f16_controller_design(MODEL, METHOD, 'exact');
    else
        r = f16_controller_design(MODEL, METHOD);
    end
end
% r 是结果结构体, 包含 A, B, K, Ki_long, x_ref, 等各种设计结果

%% 步骤4: 构建 Simulink 模型
% f16_build_simulink 使用 add_block/add_line 等命令
% 以编程方式 (Programmatic API) 构建 Simulink 模型
% 第二个参数 use_int 控制是否添加积分控制模块
fprintf('=== 步骤4: 构建 Simulink 模型 ===\n');
f16_build_simulink('F16_FinalDemo', use_int);

%% 步骤5: 开环仿真 (fb_gain=0, 控制器不起作用)
% assignin('base',...) 将变量写入 MATLAB base workspace
% Simulink 模型运行时从 base workspace 读取变量值
% fb_gain=0 → Product 模块输出全零 → 纯开环, 飞机按配平控制飞行
fprintf('\n=== 步骤5: 开环仿真 (fb_gain=0) ===\n');
assignin('base','fb_gain',0);
load_system('F16_FinalDemo');  % 加载模型到内存 (可能尚未加载)
try
    % sim() 执行仿真, 'StopTime','30' 表示仿真 30 秒
    % yout{1} 是第一个 Outport 的输出数据
    % Values.Data 是数值矩阵 (时间序列 × 13 个状态)
    % tout 是时间向量
    ol = sim('F16_FinalDemo','StopTime','30');
    data_ol = ol.yout{1}.Values.Data;  % 开环仿真数据矩阵
    t_ol = ol.tout;                     % 时间向量
    % 打印开环起始和结束的俯仰角 (第5列是 θ, 单位弧度→度)
    fprintf('  ✅ 开环完成 θ=%.2f°→%.2f°\n',data_ol(1,5)*180/pi,data_ol(end,5)*180/pi);
catch ME
    % 如果仿真失败, 捕获异常并打印错误信息, 不中断流程
    fprintf('  ❌ 开环失败: %s\n',ME.message);
end

%% 步骤6: 闭环仿真 (fb_gain=1, 控制器激活)
fprintf('\n=== 步骤6: 闭环仿真 (fb_gain=1) ===\n');
assignin('base','fb_gain',1);  % 启用反馈: Product 模块允许 K 矩阵输出通过
% 重建模型以更新 fb_gain (fb_gain 在模型初始化时读取一次, 改了要重建)
f16_build_simulink('F16_FinalDemo', use_int);
try
    cl = sim('F16_FinalDemo','StopTime','30');
    data_cl = cl.yout{1}.Values.Data;  % 闭环仿真数据矩阵
    t_cl = cl.tout;                     % 时间向量

    % 计算稳态俯仰角: 取 t≥10s 之后的均值 (10s 后系统应进入稳态)
    ss_idx = t_cl >= 10;                % 稳态索引逻辑向量
    theta_ss = mean(data_cl(ss_idx,5))*180/pi;  % 稳态俯仰角 (度)
    err = 3.0 - theta_ss;               % 跟踪误差 = 目标值 - 实际值

    fprintf('  ✅ 闭环完成 θ=%.2f°→%.2f°  稳态=%.2f°  误差=%.2f°\n',...
        data_cl(1,5)*180/pi,data_cl(end,5)*180/pi,theta_ss,err);
    % 根据误差大小给出定性评价
    if abs(err) < 1.0, fprintf('  ✅ 跟踪良好!\n');
    elseif abs(err) < 2.0, fprintf('  ⚠ 有稳态误差\n');
    else fprintf('  ❌ 跟踪偏差较大\n'); end
catch ME
    fprintf('  ❌ 闭环失败: %s\n',ME.message);
end
close_system('F16_FinalDemo',0);  % 关闭模型, 0=不保存

%% 步骤7: 绘图对比 (开环 vs 闭环)
fprintf('\n=== 步骤7: 绘图对比 ===\n');
% 如果开环或闭环数据不存在 (仿真失败), 则不绘图
if ~exist('data_ol','var') && ~exist('data_cl','var'), return; end

% 创建新图窗, 设置名称和位置 [left, bottom, width, height]
figure('Name','F-16 Pitch Control','Position',[30,30,1200,800]);

% 配置 5 个子图的参数
% 每行: {子图行号, 状态索引, 缩放系数, y轴标签, 标题}
cfg = {1,5,180/pi,'\theta (°)','俯仰角';         % θ,  弧度→度
       2,3,180/pi,'\alpha (°)','迎角';           % α,  弧度→度
       3,1,1,'V_t (m/s)','速度';                 % V,  无量纲 (m/s)
       4,8,180/pi,'q (°/s)','俯仰角速率';         % q,  弧度/秒→度/秒
       5,12,-1,'高度 (m)','高度'};               % -h, 取负得高度 (m)

for i=1:5
    subplot(2,3,i); hold on;  % 2行3列布局, 第i个子图, 保持绘图
    c=cfg(i,:); idx=c{2}; sc=c{3};  % 提取状态索引和缩放系数

    % 绘制开环响应 (蓝色实线), 如果数据存在
    if exist('data_ol','var')
        plot(t_ol,sc*data_ol(:,idx),'b-','LineWidth',1.2,'DisplayName','开环');
    end
    % 绘制闭环响应 (红色实线), 如果数据存在
    if exist('data_cl','var')
        plot(t_cl,sc*data_cl(:,idx),'r-','LineWidth',1.2,'DisplayName','闭环');
    end
    % 俯仰角子图添加 θ_ref=3° 参考线 (绿色虚线)
    if idx==5, yline(3,'g--','LineWidth',1.5,'DisplayName','\theta_{ref}=3°'); end

    xlabel('时间 (s)'); ylabel(c{4}); title(c{5}); grid on; legend('Location','best');
end

% 第6个子图: 文本信息面板, 显示控制律、参数、关键结果
subplot(2,3,6);
text(0.1,0.65,sprintf(['控制律: u = u_{base} + fb\\_gain\\cdot K\\cdot(x_{ref}-x)\\newline'...
    '模型: %s | 方法: %s\\newline'...
    'K(2,3)=%.2f | fb\\_gain=0/1 切换\\newline\\newline'...
    '伺服: RateLim(±60°/s)+20.2/(s+20.2)\\newline'...
    'θ_{ref}=3° | 稳态误差=%.2f°'],...
    MODEL,METHOD,r.K(2,3),err),'FontSize',9,'Units','normalized');
axis off;  % 文本面板不需要坐标轴

% 总标题 (sgtitle = suptitle)
sgtitle(sprintf('F-16 俯仰角控制 (%s + %s + 伺服)',MODEL,METHOD));

fprintf('\n========================================\n  演示完成!\n========================================\n');
