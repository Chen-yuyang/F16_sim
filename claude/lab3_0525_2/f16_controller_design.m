function result = f16_controller_design(model_type, method, varargin)
%==========================================================================
%  f16_controller_design — F-16 俯仰角控制器统一设计
%
%  功能:
%    基于纵向线性模型 (A 5×5, B 5×2)，用指定方法设计状态反馈增益 K (2×5)，
%    可选积分控制 Ki (2×1)，所有变量写入 MATLAB base workspace 供 Simulink 使用。
%
%  输入参数:
%    model_type  — 字符串, 'LOFI'(默认) 或 'HIFI'
%    method      — 字符串, 设计方法:
%                  'place'     极点配置 (默认)
%                  'lqr'       LQR 最优控制
%                  'manual'    手动指定 K 矩阵
%                  'place_i'   极点配置 + 积分控制 (增广系统法)
%                  'lqr_i'     LQR + 积分控制 (增广系统法)
%    varargin    — 可选:
%                  'exact'       使用精确平衡解 (Ax_d+Bu_d=0) 作为基线
%                  [2×5 矩阵]    手动 K 值 (method='manual' 时必需)
%
%  输出:
%    result — 结构体, 包含 A, B, K, Ki_long, x_ref, init_x, baseline_u 等
%
%  写入 base workspace 的变量:
%    init_x           — 13 维初始状态向量 (x_trim 或 x_d_full)
%    init_u_base      — 4 维前馈控制基线 (u_trim 或 u_d_full)
%    init_dlef        — 前缘襟翼偏转角 (标量)
%    fi_flag_Simulink — HIFI 标志 (0=LOFI, 1=HIFI)
%    K_long           — 2×5 状态反馈增益矩阵
%    Ki_long          — 2×1 积分增益向量 (无积分时为 [0;0])
%    x_ref_long       — 5 维参考状态 [V, α, θ, q, Pow]'
%    fb_gain          — 反馈使能开关 (0=开环, 1=闭环)
%    elevator_disturb — 升降舵扰动信号 (时间序列, 2 列矩阵)
%
%  控制律 (Simulink 中实现):
%    u(1:2) = init_u_base(1:2) + fb_gain * K_long * (x_ref_long - x_long)
%    含积分时: u(2) 额外 + Ki_long(2) * ∫(θ_ref - θ) dt
%    其中 x_long = [V, α, θ, q, Pow]' 是纵向 5 维状态子集
%
%  原理说明:
%    极点配置 (place): 直接指定闭环极点位置 → 控制响应速度与阻尼
%    LQR (lqr):       通过 Q/R 权重间接优化 → 自动平衡性能与控制能量
%    积分控制:        增广系统法, 引入误差积分状态消除稳态残差
%
%  参考:
%    Kautsky, Nichols, Van Dooren (1985) "Robust Pole Assignment..."
%    "place" 使用 KNVD 算法, "lqr" 使用 Schur 向量法解 Riccati 方程
%==========================================================================

% --- 默认参数 ---
if nargin<1, model_type='LOFI'; end  % 默认使用低保真气动模型
if nargin<2, method='place'; end      % 默认使用极点配置法

% --- 解析可选参数 ---
% varargin 可能包含:
%   1. 'exact' 字符串 → 使用精确平衡解方案
%   2. 数值矩阵 [2×5] → 手动指定的 K 值 (method='manual' 时)
use_exact = any(strcmpi(varargin,'exact'));  % use_exact=true → 使用精确平衡解
manual_K  = [];
for i=1:length(varargin)
    if isnumeric(varargin{i}) && isequal(size(varargin{i}),[2,5])
        manual_K = varargin{i};  % 提取手动 K 矩阵 (2行5列)
    end
end

% --- 路径设置 ---
% 注意路径优先级: 当前目录 > aerodata > trae/lab1_0429 > 项目根
this_dir = fileparts(mfilename('fullpath'));          % lab3_0525_2/
proj_root = fileparts(fileparts(this_dir));           % claude/ 的上一级 = FC_SimCode_1/
addpath(proj_root);
addpath(fullfile(proj_root,'aerodata'));
addpath(fullfile(proj_root,'trae','lab1_0429'));
addpath(this_dir);
cd(proj_root);  % 必须 cd 到根目录, 因为 HIFI 的 .dat 文件路径基于当前目录

%% ========== 步骤1: 加载线性模型 ==========
% lab1_matrices.mat 由 lab1_step1_trim_and_linearize.m 生成
% 包含: 配平状态/控制, A/B 矩阵 (LOFI + HIFI)
mat = fullfile(proj_root,'trae','lab1_0429','lab1_matrices.mat');
switch upper(model_type)
    case 'LOFI'
        % LOFI 模型: C 数组硬编码气动系数, 查表带钳位 (永不越界)
        S=load(mat,'A_longitude_lo','B_longitude_lo','trim_state_lo',...
                 'trim_thrust_lo','trim_control_lo','dLEF_lo','best_cost_lo');
        A=S.A_longitude_lo; B=S.B_longitude_lo;       % A(5×5) B(5×2)
        x_trim=S.trim_state_lo;                        % 13 维配平状态
        u_trim=[S.trim_thrust_lo;S.trim_control_lo];  % 4 维配平控制 [δth;δe;δa;δr]
        dlef=S.dLEF_lo; fi=0;                          % dlef=前缘襟翼, fi=0 → LOFI
    case 'HIFI'
        % HIFI 模型: .dat 风洞数据 + getHyperCube 多维插值
        % 边界严格检查, 越界直接崩溃 (不返回外插值)
        S=load(mat,'A_longitude_hi','B_longitude_hi','trim_state_hi',...
                 'trim_thrust_hi','trim_control_hi','dLEF_hi','best_cost_hi');
        A=S.A_longitude_hi; B=S.B_longitude_hi;
        x_trim=S.trim_state_hi;
        u_trim=[S.trim_thrust_hi;S.trim_control_hi];
        dlef=S.dLEF_hi; fi=1;  % fi=1 → 使用 HIFI 气动表
end

% --- 提取纵向状态子集 ---
% F-16 全状态 13 维: [V, β, α, φ, θ, ψ, p, q, r, xE, yE, -h, Pow]
% 纵向相关状态索引: 1=V, 3=α, 5=θ, 8=q, 13=Pow
idx=[1,3,5,8,13]; xl=x_trim(idx);  % 从 13 维全状态中提取 5 维纵向子集
theta_ref=3*pi/180;  % 俯仰角参考: 3° → 弧度 (控制目标)

%% ========== 步骤2: 定义参考状态 ==========
% --- 方案A: 简单参考状态 (旧方案) ---
% 仅将配平状态的 θ 改为目标值, 其余保持不变
% 问题: Ax_ref + Bu_trim ≠ 0 → 存在稳态残差 → 比例控制有稳态误差
x_ref_simple = xl;
x_ref_simple(3)=theta_ref;  % θ=3°
x_ref_simple(4)=0;          % q=0 (俯仰速率为零)

% --- 方案B: 精确平衡解 (新方案) ---
% 求解 [A B; C_V 0; C_θ 0] * [xd; ud] = [0; V_trim; θ_ref]
% 约束: (1) Ax_d+Bu_d=0 (线性模型中的平衡点)
%       (2) V=200 m/s (速度不变)
%       (3) θ=3° (目标俯仰角)
% 注意: 此解在非线性模型上不一定有效 (线性近似在大偏差处失效)
Aeq=[A,B; 1,0,0,0,0,0,0; 0,0,1,0,0,0,0];  % 7×7 增广矩阵
sol=Aeq\[zeros(5,1);xl(1);theta_ref];          % 求解 7 个未知数
xd=sol(1:5); ud=sol(6:7);                      % xd=5维, ud=2维
x_d_full=x_trim; x_d_full([1,3,5,8,13])=xd;   % 嵌入 13 维全状态
u_d_full=[ud(1);ud(2);u_trim(3);u_trim(4)];    % 4 维控制 [δth;δe;δa;δr]

%% ========== 步骤3: 设计状态反馈增益 K ==========
Ki_long = [0;0];  % 初始化积分增益为 0 (无积分控制时不变)

switch lower(method)
    % ---------- 3.1 极点配置 (Pole Placement) ----------
    case 'place'
        % 期望极点 (5 个, 因为系统是 5 阶):
        %   短周期模态: -2.1±2.14i → ω_n=3.0, ζ=0.70, t_s≈1.9s
        %               这是俯仰响应主模态, ζ=0.7 是飞行品质最佳值
        %   沉浮模态:   -0.8±0.6i  → ω_n=1.0, ζ=0.80, t_s≈5.0s
        %               速度-高度弱阻尼振荡, 需增强阻尼至 ζ=0.8
        %   发动机模态: -10         → 时间常数 0.4s (远快于气动动态)
        if strcmpi(model_type,'HIFI')
            K = [0 0 0 0 0; 0 0 -0.8 0.1 0];  %% HIFI: 已验证安全的手动K
            method_str='手动(HIFI K=-0.8)';
        else
            p_des=[-2.1+2.14i -2.1-2.14i -0.8+0.6i -0.8-0.6i -10];
            K=place(A,B,p_des);
            method_str='极点配置(place)';
        end

    % ---------- 3.2 LQR 最优控制 ----------
    case 'lqr'
        % Q = diag([V, α, θ, q, Pow]) — 状态偏差惩罚
        %   Q(1,1)=0.1: 速度允许一定波动 (不直接控制)
        %   Q(2,2)=1:   迎角中等惩罚 (避免失速)
        %   Q(3,3)=100: θ 是主要跟踪目标, 权重最大
        %   Q(4,4)=10:  俯仰速率加阻尼 (抑制 overshoot)
        %   Q(5,5)=0.1: 发动机动态不关心
        % R = diag([δth, δe]) — 控制能量惩罚
        %   R=0.5 限制舵面偏转幅度
        if strcmpi(model_type,'HIFI')
            K = [0 0 0 0 0; 0 0 -0.8 0.1 0];  %% HIFI: 已验证安全的手动K
            method_str='手动(HIFI K=-0.8)';
        else
            Q=diag([0.1,1,100,10,0.1]); R=diag([0.5,0.5]);
            K=lqr(A,B,Q,R);
            method_str='LQR';
        end

    % ---------- 3.3 积分控制 (两步法: 标准K + 手设Ki) ----------
    case {'place_i','lqr_i'}
        % 增广系统 place()/lqr() 在 R2024b 数值病态, Ki 符号错误+爆炸
        % 改为两步法: 1)标准5阶设计K  2)手动 Ki=K(2,3)*0.08(同号,慢积分)
        if strcmpi(method,'place_i')
            p_des=[-2.1+2.14i -2.1-2.14i -0.8+0.6i -0.8-0.6i -10];
            K=place(A,B,p_des);
            method_str='极点配置+积分';
        else
            Q=diag([0.1,1,100,10,0.1]); R=diag([0.5,0.5]);
            K=lqr(A,B,Q,R);
            method_str='LQR+积分';
        end
        Ki_long = [0; K(2,3) * 0.15];
        fprintf('  Ki_long 手动=[%.4f; %.4f] (K(2,3)*0.15)\n',Ki_long(1),Ki_long(2));

    % ---------- 3.4 手动增益 ----------
    case 'manual'
        if isempty(manual_K), error('manual方法需提供K矩阵'); end
        K=manual_K;
        method_str='手动设定';
end

%% ========== 步骤4: 选择基线方案 ==========
% 'exact' 方案:     u_base=u_d, x_ref=x_d, 初值=x_d_full
%   → 理论上消除残差, 但 x_d 在非线性模型上不可行
% 'old'(默认)方案: u_base=u_trim, x_ref=x_ref_simple, 初值=x_trim
%   → 有稳态残差, 但非线性模型上稳定可工作
if use_exact
    init_x_val=x_d_full; baseline_u=u_d_full; ref_x=xd; scheme='exact(新)';
else
    init_x_val=x_trim; baseline_u=u_trim; ref_x=x_ref_simple; scheme='old(旧)';
end

%% ========== 步骤5: 稳定性验证 ==========
eig_cl=eig(A-B*K); stable=all(real(eig_cl)<0);
% 注意: 含积分时的闭环极点应检查 A_aug-B_aug*K_aug
% 但 (A-B*K) 的稳定性是必要条件 (状态反馈部分必须稳定)

%% ========== 步骤6: 写入 MATLAB base workspace ==========
% Simulink 模型通过变量名从 base workspace 读取参数
assignin('base','init_x',init_x_val);          % 13×1 初始状态
assignin('base','init_u_base',baseline_u);     % 4×1 前馈基线
assignin('base','init_dlef',dlef);             % 前缘襟翼偏角 (标量)
assignin('base','fi_flag_Simulink',fi);        % 气动模型选择 (0/1)
assignin('base','K_long',K);                   % 2×5 反馈增益
assignin('base','Ki_long',Ki_long);            % 2×1 积分增益
assignin('base','x_ref_long',ref_x);           % 5×1 参考状态
assignin('base','fb_gain',0);                  % 初始为开环 (0)

% 升降舵扰动信号 (仅在不存在时创建)
% 格式: [时间, 幅值] 两列矩阵, From Workspace 模块读取
% 信号: 0→1s=0, 2s→5s=+5°(上偏), 5s→6s=-5°(下偏), 之后归零
% 单位: 弧度 (5° = 0.0873 rad)
if ~evalin('base','exist(''elevator_disturb'',''var'')')
    td=[0 1 1 2 2 5 5 6 6 30]; amp=5*pi/180;
    assignin('base','elevator_disturb',[td',[0;0;amp;amp;-amp;-amp;0;0;0;0]]);
end

%% ========== 步骤7: 输出摘要 ==========
fprintf('\n============================================================\n');
fprintf('  F-16 俯仰角控制器设计\n');
fprintf('============================================================\n');
fprintf('  模型: %s | 方法: %s | 方案: %s\n', model_type, method_str, scheme);
fprintf('  开环极点: SP=%.2f±%.2fi  Ph=%.4f±%.4fi  Eng=%.1f\n',...
    real(eig(A)), imag(eig(A)));
fprintf('  K(2,3)=%.4f (θ→δe, %s) | 闭环稳定=%d\n',...
    K(2,3), iif(K(2,3)<0,'负=抬头✓','正=低头'), stable);
if any(Ki_long ~= 0)
    fprintf('  Ki_long=[%.4f; %.4f] (积分增益, δth/δe通道)\n',Ki_long(1),Ki_long(2));
end
fprintf('  初始状态: V=%.0f α=%.1f° θ=%.1f° Pow=%.1f%%\n',...
    init_x_val(1),init_x_val(3)*180/pi,init_x_val(5)*180/pi,init_x_val(13));
fprintf('  参考状态: V=%.0f α=%.1f° θ=%.1f° Pow=%.1f%%\n',...
    ref_x(1),ref_x(2)*180/pi,ref_x(3)*180/pi,ref_x(5));
fprintf('  前馈基线: δth=%.4f δe=%.4f(%.2f°)\n',...
    baseline_u(1),baseline_u(2),baseline_u(2)*180/pi);
fprintf('  ‖Ax_d+Bu_d‖=%.1e | 变量已写入 base workspace\n',norm(A*xd+B*ud));
fprintf('\n');

%% ========== 构建输出结构体 ==========
result=struct('A',A,'B',B,'K',K,'Ki_long',Ki_long,'xd',xd,'ud',ud,'x_ref',ref_x,...
    'x_d_full',x_d_full,'u_d_full',u_d_full,'x_trim',x_trim,'u_trim',u_trim,...
    'dlef',dlef,'fi',fi,'model',model_type,'method',method,'use_exact',use_exact,...
    'init_x',init_x_val,'baseline_u',baseline_u,'ref_x',ref_x,'stable',stable,...
    'eig_cl',eig_cl,'theta_ref',theta_ref);

% --- 内联函数: 三元条件运算符 (C语言风格 ? :) ---
function s=iif(c,t,f); if c; s=t; else; s=f; end; end
end
