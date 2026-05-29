%% design_controller.m — F-16 俯仰角控制器统一设计函数
%==========================================================================
%  功能:
%    加载 F-16 纵向线性模型 (A, B), 使用 Place 或 LQR 方法设计
%    状态反馈增益 K(2×5), 计算参考状态和前馈基线, 全部写入
%    MATLAB base workspace 供 Simulink 使用。
%
%  核心逻辑:
%    u(1:2) = u_base(1:2) + K_long * (x_ref_long - x_long)
%    其中 x_long = [V, α, θ, q, Pow]' 是纵向 5 维子集
%
%  两种基线方案:
%    1. 旧方案 (默认): u_base=u_trim, x_ref 仅改 θ→3°
%       问题: Ax_ref+Bu_trim≠0 → 稳态残差 → 比例控制有稳态误差
%    2. 新方案 (use_exact_eq=true): u_base=u_d, x_ref=x_d
%       求解 Ax_d+Bu_d=0, 理论上无稳态误差
%       但 x_d 在非线性模型上不一定可行 (线性近似在大偏差处失效)
%
%  输入参数:
%    model_type   — 'LOFI'(默认)或'HIFI'
%    method       — 'place'(默认)或'lqr'
%    use_exact_eq — true=精确平衡解, false=简单方案 (默认)
%
%  输出:
%    result — 结构体, 包含所有设计结果
%
%  写入 base workspace 的变量:
%    init_x(13×1), init_u_base(4×1), init_dlef, fi_flag_Simulink,
%    K_long(2×5), x_ref_long(5×1), fb_gain(0)
%
%  参考:
%    f16_controller_design.m — 更完整的版本 (支持积分控制, 更详细注释)
%    build_model.m — 使用本函数输出的变量构建 Simulink 模型
%==========================================================================

function result = design_controller(model_type, method, use_exact_eq)
% 输入参数处理: 设置默认值
if nargin<1, model_type='LOFI'; end   % 默认使用低保真气动模型
if nargin<2, method='place'; end       % 默认使用极点配置法
if nargin<3, use_exact_eq=false; end   % 默认使用简单参考方案

% --- 路径设置 ---
% fileparts(mfilename('fullpath')): 获取本函数所在目录
this_dir = fileparts(mfilename('fullpath'));
% 上两级目录 = 项目根目录
proj_root = fileparts(fileparts(fileparts(this_dir)));
addpath(genpath(proj_root));  % 递归添加所有子目录到搜索路径
cd(proj_root);  % 切换工作目录到项目根

% --- 加载线性模型 ---
% lab1_matrices.mat 由 lab1_step1_trim_and_linearize.m 生成
% 包含: 配平状态/控制, A/B 矩阵 (LOFI + HIFI)
mat = fullfile(proj_root,'trae','lab1_0429','lab1_matrices.mat');

switch upper(model_type)
    case 'LOFI'
        % LOFI 模型: C 数组硬编码气动系数, 查表带钳位 (永不越界)
        % A_longitude_lo(5×5), B_longitude_lo(5×2)
        % trim_state_lo(13×1), trim_thrust_lo(1×1), trim_control_lo(3×1)
        % dLEF_lo(标量) — 前缘襟翼配平偏角
        S=load(mat,'A_longitude_lo','B_longitude_lo','trim_state_lo',...
                 'trim_thrust_lo','trim_control_lo','dLEF_lo');
        A=S.A_longitude_lo; B=S.B_longitude_lo;
        x_trim=S.trim_state_lo;
        u_trim=[S.trim_thrust_lo;S.trim_control_lo];  % [δth; δe; δa; δr]
        dlef=S.dLEF_lo; fi=0;  % fi=0 → LOFI 气动模型

    case 'HIFI'
        % HIFI 模型: .dat 风洞数据 + getHyperCube 多维插值
        % 边界严格检查, 越界直接崩溃 (不返回外插值)
        S=load(mat,'A_longitude_hi','B_longitude_hi','trim_state_hi',...
                 'trim_thrust_hi','trim_control_hi','dLEF_hi');
        A=S.A_longitude_hi; B=S.B_longitude_hi;
        x_trim=S.trim_state_hi;
        u_trim=[S.trim_thrust_hi;S.trim_control_hi];
        dlef=S.dLEF_hi; fi=1;  % fi=1 → HIFI 气动表
end

% --- 提取纵向状态子集 ---
% F-16 全状态 13 维: [V, β, α, φ, θ, ψ, p, q, r, xE, yE, -h, Pow]
% 纵向相关状态索引: 1=V, 3=α, 5=θ, 8=q, 13=Pow
idx=[1,3,5,8,13]; xl=x_trim(idx);  % 提取 5 维纵向子集
theta_ref=3*pi/180;  % 俯仰角参考: 3° → 弧度

% --- 精确平衡解 (新方案) ---
% 求解 [A B; C_V 0; C_θ 0] * [xd; ud] = [0; V_trim; θ_ref]
% 约束: (1) Ax_d+Bu_d=0 (线性模型中的平衡点)
%       (2) V=200 m/s (速度不变)
%       (3) θ=3° (目标俯仰角)
% 构造 7×7 增广矩阵:
%   前5行: [A, B] — 状态方程 Ax+Bu=0
%   第6行: [1,0,0,0,0, 0,0] — V=V_trim
%   第7行: [0,0,1,0,0, 0,0] — θ=θ_ref
Aeq=[A,B; 1,0,0,0,0,0,0; 0,0,1,0,0,0,0];
sol=Aeq\[zeros(5,1);xl(1);theta_ref];  % 求解 7 个未知数 [xd; ud]
xd=sol(1:5); ud=sol(6:7);  % xd=5维状态, ud=2维控制
% 嵌入 13 维全状态 (横侧向状态沿用配平值)
x_d_full=x_trim; x_d_full([1,3,5,8,13])=xd;
% 4 维控制 [δth; δe; δa; δr], 横侧向通道沿用配平值
u_d_full=[ud(1);ud(2);u_trim(3);u_trim(4)];

% --- 简单参考状态 (旧方案) ---
% 仅将配平状态的 θ 改为目标值, q 置零, 其余不变
% x_ref 与配平状态的差异 → 产生稳态残差
x_ref=xl; x_ref(3)=theta_ref; x_ref(4)=0;

% --- 设计状态反馈增益 K ---
switch lower(method)
    case 'place'
        % 极点配置法: 直接指定闭环极点位置
        % 期望极点 (5 个):
        %   -2.1±2.14i: 短周期模态, ω_n=3.0, ζ=0.70, t_s≈1.9s
        %   -0.8±0.6i:  长周期 (沉浮) 模态, ω_n=1.0, ζ=0.80
        %   -10:         发动机模态, 时间常数 0.1s
        K=place(A,B,[-2.1+2.14i -2.1-2.14i -0.8+0.6i -0.8-0.6i -10]);

    case 'lqr'
        % LQR 最优控制: 通过权重矩阵间接优化
        % Q = diag([V, α, θ, q, Pow])
        %   Q(3,3)=100: θ 跟踪是最主要目标, 权重最大
        % R = diag([δth, δe]) = diag([0.5, 0.5])
        %   限制控制能量消耗
        K=lqr(A,B,diag([0.1,1,100,10,0.1]),diag([0.5,0.5]));
end

% --- 选择基线方案 ---
if use_exact_eq
    % 新方案: 使用精确平衡解
    init_x_val=x_d_full;   % 初始状态 = 精确平衡解 (13 维)
    baseline_u=u_d_full;   % 前馈基线 = 精确平衡控制 (4 维)
    ref_x=xd;              % 参考状态 = 精确平衡解 (5 维)
    label=[upper(method) '-new'];
else
    % 旧方案: 使用配平值 + 简单修改
    init_x_val=x_trim;     % 初始状态 = 配平态 (13 维)
    baseline_u=u_trim;     % 前馈基线 = 配平控制 (4 维)
    ref_x=x_ref;           % 参考状态 = 仅改 θ 的简单方案 (5 维)
    label=[upper(method) '-old'];
end

% --- 写入 MATLAB base workspace ---
% Simulink 模型通过变量名从 base workspace 读取参数
% 这些变量由 build_model.m 构建的 Simulink 模型引用
assignin('base','init_x',init_x_val);        % 13×1 初始状态
assignin('base','init_u_base',baseline_u);   % 4×1 前馈基线
assignin('base','init_dlef',dlef);           % 前缘襟翼 (标量)
assignin('base','fi_flag_Simulink',fi);      % HIFI 标志 (0/1)
assignin('base','K_long',K);                 % 2×5 反馈增益
assignin('base','x_ref_long',ref_x);         % 5×1 参考状态
assignin('base','fb_gain',0);                % 初始为开环

% 创建升降舵扰动信号 (仅当不存在时)
% 格式: [时间, 幅值] 两列矩阵, From Workspace 模块读取
if ~evalin('base','exist(''elevator_disturb'',''var'')')
    td=[0 1 1 2 2 5 5 6 6 30]; amp=5*pi/180;
    assignin('base','elevator_disturb',[td',[0;0;amp;amp;-amp;-amp;0;0;0;0]]);
end

% --- 输出摘要 ---
fprintf('%s | K(2,3)=%.2f | init theta=%.1f° | ref theta=%.1f°\n',...
    [model_type '-' label], K(2,3), init_x_val(5)*180/pi, ref_x(3)*180/pi);
% K(2,3) 表示 θ→δe 的增益:
%   负值: θ 偏高→升降舵上偏 (低头)→正确负反馈
%   正值: 极性反了! (需要检查传感器/作动器方向)

% --- 构建输出结构体 ---
result=struct('A',A,'B',B,'K',K,'xd',xd,'ud',ud,'x_ref',x_ref,...
    'x_d_full',x_d_full,'u_d_full',u_d_full,'x_trim',x_trim,'u_trim',u_trim,...
    'dlef',dlef,'fi',fi,'model',model_type,'method',method,...
    'use_exact_eq',use_exact_eq,'label',label,'init_x',init_x_val,...
    'baseline_u',baseline_u,'ref_x',ref_x);
end
