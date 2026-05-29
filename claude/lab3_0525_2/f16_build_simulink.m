function f16_build_simulink(model_name, use_integral)
%==========================================================================
%  f16_build_simulink — 构建 F-16 纵向闭环 Simulink 模型
%
%  功能:
%    用 MATLAB 脚本程序化构建一个 Simulink 模型, 包含:
%      - F-16 非线性动力学 S-Function (F16_dyn, 3端口连续)
%      - 状态反馈控制器 (K_gain × 误差信号)
%      - 升降舵伺服作动器 (Rate Limiter + 一阶低通)
%      - 可选积分控制路径 (消除稳态误差)
%      - 开环/闭环切换 (Product 块 × fb_gain)
%      - 升降舵扰动注入 (From Workspace)
%    生成 .slx 文件保存到 test_models/ 目录下。
%
%  输入参数:
%    model_name   — 字符串, 模型名称 (默认 'F16_ClosedLoop')
%    use_integral — 逻辑值, true=含积分路径, false=纯比例反馈 (默认 false)
%
%  依赖的 base workspace 变量 (由 f16_controller_design 写入):
%    init_x       — 13 维初始状态 (F16_dyn 的 S-Function 参数)
%    init_u_base  — 4 维前馈控制基线 [δth;δe;δa;δr]
%    init_dlef    — 前缘襟翼偏转角 (标量)
%    fi_flag_Simulink — 气动模型标志 (0=LOFI, 1=HIFI)
%    K_long       — 2×5 状态反馈增益矩阵
%    Ki_long      — 2×1 积分增益向量 (仅 use_integral=true 时需要)
%    x_ref_long   — 5 维参考状态 [V, α, θ, q, Pow]'
%    fb_gain      — 反馈使能 (0=开环, 1=闭环)
%    elevator_disturb — 升降舵扰动 (时间序列, 可选)
%
%  模型架构 (信号流从左到右):
%
%    信号源列               伺服列           F16列        控制器列
%    ┌──────────┐          ┌────────┐     ┌──────────┐   ┌──────────┐
%    │thrust_base├─(+)→sat─▶│        │     │          │   │ state_sel│
%    │elev_base ├─(+)→RL→SS→sat→    │     │  F16_dyn │   │ err_sum  │
%    │ail_base  │──────────▶│ctrl_mux├───▶│ (C MEX)  ├──▶│ K_gain   │
%    │rud_base  │──────────▶│        │     │          │   │ fb_enable│
%    │elev_dist │─→(+)/     │        │     │          │   │ fb_demux │
%    └──────────┘          └────────┘     └──────────┘   └─────┬────┘
%         ↑                                               ↑     │
%         │          积分路径(可选)                        │     │
%         │    ┌─────────────────┐                        │     │
%         └────┤←theta_integrator←int_err←theta_sel       │     │
%              │←Ki_gain                                  │     │
%              └──────────────────────────────────────────┘     │
%                                                                │
%    输出列                                                     │
%    ┌──────────┐                                                │
%    │states_out│←─────────────────────────────────────────←─────┘
%    │scope_demux│→ Scope_V, Scope_alpha, Scope_theta, Scope_q, Scope_h
%    └──────────┘
%
%    注: RL=Rate Limiter(±60°/s), SS=State-Space(20.2/(s+20.2))
%        sat=Saturation(δth∈[0,1], δe∈[-0.44,+0.44]rad)
%        (+)=Sum 求和块
%
%  各模块详解 (按信号流顺序):
%
%   1. thrust_base     — Constant: 油门前馈基线 init_u_base(1)
%   2. elev_base       — Constant: 升降舵前馈基线 init_u_base(2)
%   3. ail_base        — Constant: 副翼前馈基线 init_u_base(3) (横向, 纵向冻结)
%   4. rud_base        — Constant: 方向舵前馈基线 init_u_base(4) (横向, 纵向冻结)
%   5. elev_dist       — From Workspace: 升降舵扰动注入, 读取 elevator_disturb
%   6. sum_thrust      — Sum: 油门基线 + 油门反馈 (2输入, ++)
%   7. sum_elev        — Sum: 升降舵基线 + 扰动 + 反馈 + 积分 (4输入, ++++)
%   8. sat_thrust      — Saturation: 油门限幅 [0,1] (物理约束)
%   9. elev_rate       — Rate Limiter: 升降舵速率 ±60°/s (±1.047 rad/s)
%  10. elev_servo      — Transfer Fcn: 一阶伺服 20.2/(s+20.2) (τ≈0.05s)
%  11. sat_elev        — Saturation: 升降舵限幅 ±0.44 rad (±25°)
%  12. ctrl_mux        — Mux: 4 路控制合并 [δth;δe;δa;δr] → F16_dyn
%  13. F16_dyn         — S-Function: F16 非线性动力学 (3 端口)
%  14. state_sel       — Selector: 从 13 维全状态提取纵向5维 [V,α,θ,q,Pow]
%  15. err_sum         — Sum: 参考状态 - 实际状态 (x_ref - x)
%  16. K_gain          — Gain: 矩阵乘法 K_long * error (2×5 × 5×1 = 2×1)
%  17. fb_enable       — Product: fb_gain × 反馈信号 (开环/闭环切换)
%  18. fb_demux        — Demux: 2 路反馈 [油门通道; 升降舵通道]
%  19. theta_sel       — Selector: 从 13 态选 θ(索引5) (仅积分路径)
%  20. int_err         — Sum: θ_ref - θ (仅积分路径)
%  21. theta_integrator— Integrator: ∫(θ_ref - θ)dt (仅积分路径)
%  22. Ki_gain         — Gain: Ki_long(2) × 积分值 (仅积分路径)
%  23. states_out      — Out1: 13 维状态输出到 MATLAB
%  24. scope_demux     — Demux: 13 态分解后送 Scope
%  25. Scope_V/alpha/theta/q/h — 5 个 Scope 显示关键状态
%
%  使用方法:
%    >> f16_controller_design('LOFI','place');  % 设计控制器, 写入变量
%    >> f16_build_simulink('MyModel');           % 构建模型
%    >> assignin('base','fb_gain',1);            % 切换为闭环
%    >> f16_build_simulink('MyModel');           % 重建使 fb_gain 生效
%    >> out = sim('MyModel');                     % 运行仿真
%
%  积分控制用法:
%    >> f16_controller_design('LOFI','place_i'); % 含积分的设计
%    >> f16_build_simulink('MyModel', true);      % 含积分路径的模型
%    >> assignin('base','fb_gain',1);
%    >> f16_build_simulink('MyModel', true);
%    >> out = sim('MyModel');
%==========================================================================

% --- 默认参数 ---
if nargin<1, model_name='F16_ClosedLoop'; end  % 默认模型名
if nargin<2, use_integral=false; end            % 默认不含积分

% --- 路径设置 ---
this_dir = fileparts(mfilename('fullpath'));          % lab3_0525_2/
proj_root = fileparts(fileparts(this_dir));           % FC_SimCode_1/
out_dir  = fullfile(this_dir,'test_models');          % 输出到 test_models/
if ~exist(out_dir,'dir'), mkdir(out_dir); end
addpath(proj_root);
addpath(fullfile(proj_root,'aerodata'));
addpath(this_dir);
cd(proj_root);  % HIFI 需要 cd 到根以解析相对路径的 .dat 文件

% --- 检查必需的 base workspace 变量 ---
vars={'init_x','init_u_base','init_dlef','fi_flag_Simulink','K_long','x_ref_long','fb_gain'};
if use_integral, vars{end+1}='Ki_long'; end  % 积分模式需要 Ki_long
for i=1:numel(vars)
    if ~evalin('base',sprintf('exist(''%s'',''var'')',vars{i}))
        error('缺少变量: %s — 请先运行 f16_controller_design',vars{i});
    end
end

% 升降舵扰动信号: 如果不存在则创建默认值
% 5° 幅度的 doublet 脉冲: 2s→5s 正偏, 5s→6s 负偏, 之后归零
% 用于测试系统对扰动的抑制能力
if ~evalin('base','exist(''elevator_disturb'',''var'')')
    td=[0 1 1 2 2 5 5 6 6 30]; amp=5*pi/180;
    assignin('base','elevator_disturb',[td',[0;0;amp;amp;-amp;-amp;0;0;0;0]]);
end

%% ==== 清理旧模型 ====
% 避免模型名冲突: 删除已加载的同名模型和已存在的 .slx/.slxc 文件
if bdIsLoaded(model_name), close_system(model_name,1); end
for ext={'.slx','.slxc'}
    f=fullfile(out_dir,[model_name ext{1}]); if exist(f,'file'), delete(f); end
    % 同时清理项目根目录下的同名文件 (避免 load/sim 时 shadowing 警告)
    f_root=fullfile(proj_root,[model_name ext{1}]); if exist(f_root,'file'), delete(f_root); end
end

%% ==== 创建新模型 ====
new_system(model_name,'Model');  % 创建空模型 (默认为离散求解器)

% --- 求解器设置 ---
% ode4 = 四阶龙格-库塔 (固定步长), 适用于飞行仿真
% FixedStep=0.01s = 100Hz, 够捕捉 50ms 的伺服动态
set_param(model_name,'Solver','ode4','FixedStep','0.01','StopTime','30');
% 输出设置: 保存状态、输出、时间, via Out1 块返回 workspace
set_param(model_name,'SaveState','on','SaveOutput','on','SaveTime','on',...
    'ReturnWorkspaceOutputs','on');
% 禁止未连接输出的警告 (Scope 块可能未连接)
set_param(model_name,'UnconnectedOutputMsg','none');

%% ==== 布局参数 ====
% Y 是垂直定位游标, DY 是行间距
% 所有模块用 [left, top, right, bottom] 格式定位
Y=35; DY=50;

%% ====== 列1: 信号源 (x=50~160) ======
% 所有基线信号使用 Constant 块 (无时间变化, 采样时间 inf)
% 前馈值在仿真开始前由 base workspace 确定, 仿真中不变

% --- 油门基线: init_u_base(1) ---
add_block('simulink/Sources/Constant',[model_name '/thrust_base'],...
    'Position',[50,Y,160,Y+30]);
set_param([model_name '/thrust_base'],'Value','init_u_base(1)','SampleTime','inf');
Y=Y+DY;

% --- 升降舵基线: init_u_base(2) ---
add_block('simulink/Sources/Constant',[model_name '/elev_base'],...
    'Position',[50,Y,160,Y+30]);
set_param([model_name '/elev_base'],'Value','init_u_base(2)','SampleTime','inf');
Y=Y+DY;

% --- 副翼基线: init_u_base(3) (横向通道, 纵向控制中保持配平值) ---
add_block('simulink/Sources/Constant',[model_name '/ail_base'],...
    'Position',[50,Y,160,Y+30]);
set_param([model_name '/ail_base'],'Value','init_u_base(3)','SampleTime','inf');
Y=Y+DY;

% --- 方向舵基线: init_u_base(4) (横向通道, 纵向控制中保持配平值) ---
add_block('simulink/Sources/Constant',[model_name '/rud_base'],...
    'Position',[50,Y,160,Y+30]);
set_param([model_name '/rud_base'],'Value','init_u_base(4)','SampleTime','inf');
Y=Y+DY+20;

% --- 前缘襟翼: dlef (与速度和迎角相关的自动缝翼) ---
add_block('simulink/Sources/Constant',[model_name '/dlef'],...
    'Position',[50,Y,160,Y+30]);
set_param([model_name '/dlef'],'Value','init_dlef','SampleTime','inf');
Y=Y+DY;

% --- 气动模型标志: fi_flag_Simulink (0=LOFI, 1=HIFI) ---
% 该信号直接传入 F16_dyn S-Function 的第三端口
add_block('simulink/Sources/Constant',[model_name '/fi_flag'],...
    'Position',[50,Y,160,Y+30]);
set_param([model_name '/fi_flag'],'Value','fi_flag_Simulink','SampleTime','inf');
Y=Y+DY;

% --- 升降舵扰动: From Workspace ---
% 从 base workspace 读取 elevator_disturb 变量
% 格式: 2列矩阵 [时间, 幅值], 线性插值
add_block('simulink/Sources/From Workspace',[model_name '/elev_dist'],...
    'Position',[50,Y,160,Y+30]);
set_param([model_name '/elev_dist'],'VariableName','elevator_disturb',...
    'SampleTime','0.01','OutputAfterFinalValue','Setting to zero');
Y=Y+DY+20;

% --- 参考状态: x_ref_long ---
% 5×1 常数向量 [V_ref; α_ref; θ_ref; q_ref; Pow_ref]
add_block('simulink/Sources/Constant',[model_name '/x_ref'],...
    'Position',[50,Y,160,Y+40]);
set_param([model_name '/x_ref'],'Value','x_ref_long','SampleTime','inf');

% --- 反馈使能开关: fb_gain ---
% 0=开环 (纯前馈, 无反馈), 1=闭环 (前馈+反馈)
% 使用 Product 块实现乘法使能, 比 Manual Switch 可靠
add_block('simulink/Sources/Constant',[model_name '/fb_gain'],...
    'Position',[50,Y+60,160,Y+90]);
set_param([model_name '/fb_gain'],'Value','fb_gain','SampleTime','inf');

%% ====== 列2: 求和+伺服作动器 (x=250~520) ======

% --- 油门求和: sum_thrust ---
% 2 输入: thrust_base(1) + 反馈(2)
add_block('simulink/Math Operations/Sum',[model_name '/sum_thrust'],...
    'Position',[250,35,280,70]);
set_param([model_name '/sum_thrust'],'Inputs','++','IconShape','round');

% --- 油门限幅: sat_thrust ---
% 物理约束: 油门只能在 [0, 1] 之间 (0%~100%)
add_block('simulink/Discontinuities/Saturation',[model_name '/sat_thrust'],...
    'Position',[320,35,360,70]);
set_param([model_name '/sat_thrust'],'UpperLimit','1','LowerLimit','0');

% --- 升降舵求和: sum_elev ---
% 输入端口数取决于是否含积分:
%   use_integral=false: 3 输入 '+++' (基线+扰动+反馈)
%   use_integral=true:  4 输入 '++++' (基线+扰动+反馈+积分)
% ★ BUG FIX: 之前无条件设 '++++', 非积分模式第4端口悬空→Simulink报错
% 注意: 端口顺序由 Simulink 的排序规则决定 (从上到下, 从左到右)
add_block('simulink/Math Operations/Sum',[model_name '/sum_elev'],...
    'Position',[250,85,280,140]);
if use_integral
    set_param([model_name '/sum_elev'],'Inputs','++++','IconShape','round');
else
    set_param([model_name '/sum_elev'],'Inputs','+++','IconShape','round');
end

% ====== 伺服作动器 (用基础模块搭建 G(s)=20.2/(s+20.2)) ======
% ★ R2024b 的 Transfer Fcn 块删除了所有初始条件参数,
%   导致仿真起始舵面在 0(非配平值) → 瞬态越界 → F16_dyn 崩溃。
%   改用基础模块(Integrator+Gain+Sum)搭建, Integrator 的
%   InitialCondition 参数在所有 MATLAB 版本上都存在且有效。
%
%   传递函数: G(s) = 20.2/(s+20.2)  →  ẏ = -20.2y + 20.2u
%   搭建方式: y = ∫(20.2·u - 20.2·y) dt
%   信号流:
%     elev_rate → servo_K_fwd(×20.2) → servo_sum(+) → servo_int(∫,IC=trim) → sat_elev
%                                       servo_sum(-) ← servo_K_fbk(×20.2) ←──┘
%
% --- servo_K_fwd: 前向增益 20.2 ---
add_block('simulink/Math Operations/Gain',[model_name '/servo_K_fwd'],...
    'Position',[400,85,440,115]);
set_param([model_name '/servo_K_fwd'],'Gain','20.2');

% --- servo_sum: 前向 - 反馈 = ẏ ---
add_block('simulink/Math Operations/Sum',[model_name '/servo_sum'],...
    'Position',[465,85,490,120]);
set_param([model_name '/servo_sum'],'Inputs','+-','IconShape','round');

% --- servo_int: 积分器 (初始条件=配平升降舵值) ---
% 将 trim elevator 值从 workspace 读出, 用数值字符串设置 IC
% (不用符号引用, 避免 R2024b 运行时解析问题)
init_u_val = evalin('base','init_u_base');
elev_ic = init_u_val(2);
add_block('simulink/Continuous/Integrator',[model_name '/servo_int'],...
    'Position',[515,85,555,130]);
set_param([model_name '/servo_int'],'InitialCondition',num2str(elev_ic,16));
fprintf('  伺服 IC = %.6f rad (%.2f°)\n', elev_ic, elev_ic*180/pi);

% --- servo_K_fbk: 反馈增益 20.2 (从输出反馈到求和负端) ---
add_block('simulink/Math Operations/Gain',[model_name '/servo_K_fbk'],...
    'Position',[465,155,505,185]);
set_param([model_name '/servo_K_fbk'],'Gain','20.2');

% --- 升降舵限幅: sat_elev ---
% 物理约束: F-16 升降舵最大偏转 ±25° = ±0.44 rad
% 超过这个值, 气动数据表外推无效 (HIFI 会直接崩溃)
add_block('simulink/Discontinuities/Saturation',[model_name '/sat_elev'],...
    'Position',[480,85,520,140]);
set_param([model_name '/sat_elev'],'UpperLimit','0.44','LowerLimit','-0.44');

%% ====== 积分路径 (可选, use_integral=true) ======
% 积分控制用于消除稳态误差:
% 当 θ 达到稳态但 θ ≠ θ_ref 时, 积分项 ∫(θ_ref-θ)dt 不断增大,
% 推动升降舵持续偏转直到 θ = θ_ref, 误差为零。
%
% 信号流:
%   F16_dyn 的 13 维输出 → theta_sel (提取 θ, 索引5)
%   → int_err (θ_ref - θ) → theta_integrator (∫dt)
%   → Ki_gain (乘以 Ki_long(2)) → sum_elev 端口4
%
% 注意: 只有升降舵通道有积分 (Ki_long(2)),
%       油门通道的积分 Ki_long(1) 未使用。
if use_integral
    % --- theta_sel: 从 13 维状态提取 θ (索引5) ---
    % F16_dyn 输出 13 维向量: [V,β,α,φ,θ,ψ,p,q,r,xE,yE,-h,Pow]
    % θ 是第 5 个元素
    add_block('simulink/Signal Routing/Selector',[model_name '/theta_sel'],...
        'Position',[870,160,920,190]);
    set_param([model_name '/theta_sel'],...
        'InputPortWidth','13','Indices','5','IndexOptions','Index vector (dialog)');

    % --- int_err: 误差计算 θ_ref - θ ---
    % 两个输入: θ_ref(正端) - θ(负端)
    % θ_ref 来自 x_ref_long(3) 常量
    % θ 来自 theta_sel 的选择输出
    add_block('simulink/Math Operations/Sum',[model_name '/int_err'],...
        'Position',[940,160,970,190]);
    set_param([model_name '/int_err'],'Inputs','+-','IconShape','round');

    % --- theta_integrator: 误差积分 ---
    % 输出 x_I = ∫(θ_ref - θ) dt, 初始条件为 0
    % 只要 θ ≠ θ_ref, 积分值持续变化, 驱动积分控制
    add_block('simulink/Continuous/Integrator',[model_name '/theta_integrator'],...
        'Position',[1000,160,1030,190]);
    set_param([model_name '/theta_integrator'],'InitialCondition','0');

    % --- Ki_gain: 积分增益 ---
    % 将积分值乘以 Ki_long(2) (升降舵通道的积分增益)
    % Ki_long(2) 由 f16_controller_design 在 method='place_i'/'lqr_i' 时计算
    % 典型值: place_i(p_int=-0.5) → Ki(2)≈-2~-5
    %         lqr_i(Q_int=10)    → Ki(2)≈-3~-6
    % 负号: 因为 θ 偏低时积分项应为正 (抬头)
    add_block('simulink/Math Operations/Gain',[model_name '/Ki_gain'],...
        'Position',[1060,160,1100,190]);
    set_param([model_name '/Ki_gain'],'Gain','Ki_long(2)');

    % --- Ki_enable: 积分反馈使能开关 (Product 块) ---
    % ★ 关键: 积分路径必须也受 fb_gain 控制!
    % 比例路径通过 fb_enable(Product) 被 fb_gain 门控,
    % 但原设计中 Ki_gain 直连 sum_elev, 导致 fb_gain=0(开环)时
    % 积分器仍在运行, 累积误差驱动舵面 → 发散崩溃。
    % 加此 Product 后, fb_gain=0 时积分输出为 0, 开环仿真正确。
    add_block('simulink/Math Operations/Product',[model_name '/Ki_enable'],...
        'Position',[1130,160,1160,190]);
    set_param([model_name '/Ki_enable'],'Inputs','**','Multiplication','Element-wise(.*)');
end

%% ====== 列3: Mux + F16 动力学 (x=570~810) ======

% --- ctrl_mux: 4 路控制信号合并 ---
% 输入: [δth; δe; δa; δr], 输出 4 维向量到 F16_dyn
% δa/δr 保持配平值不变 (纵向控制中横向冻结)
add_block('simulink/Signal Routing/Mux',[model_name '/ctrl_mux'],...
    'Position',[570,30,610,180]);
set_param([model_name '/ctrl_mux'],'Inputs','4','DisplayOption','bar');

% --- F16_dyn: F-16 非线性动力学 S-Function ---
% C MEX S-Function, 3 个输入端口:
%   端口1: 4 维控制向量 [δth; δe; δa; δr]
%   端口2: 标量 dlef (前缘襟翼)
%   端口3: 标量 fi_flag (0=LOFI, 1=HIFI)
% 输出: 13 维状态向量 (见上方 theta_sel 的注释)
% 参数: init_x — 13 维初始状态 (S-Function 参数通过 Parameters 传入)
% 注意: S-Function 使用 CONTINUOUS_SAMPLE_TIME,
%       在 mdlDerivatives 中计算 13 个状态的导数
add_block('simulink/User-Defined Functions/S-Function',[model_name '/F16_dyn'],...
    'Position',[690,30,810,230]);
set_param([model_name '/F16_dyn'],'FunctionName','F16_dyn','Parameters','init_x');

%% ====== 列4: 控制器 (x=880~1260) ======

% --- state_sel: 从 13 维全状态提取纵向 5 维 ---
% 纵向控制需要的状态: [V(1), α(3), θ(5), q(8), Pow(13)]
% 索引向量 [1,3,5,8,13] 对应这 5 个状态
% 横侧向状态 (β,φ,ψ,p,r) 被忽略
add_block('simulink/Signal Routing/Selector',[model_name '/state_sel'],...
    'Position',[880,40,950,110]);
set_param([model_name '/state_sel'],'InputPortWidth','13',...
    'Indices','[1,3,5,8,13]','IndexOptions','Index vector (dialog)');

% --- err_sum: 参考 - 实际 = 误差 ---
% 计算 x_ref_long - x_long
% 正端(+): x_ref (来自 x_ref 常量块)
% 负端(-): x (来自 state_sel)
% 输出 5 维误差向量
add_block('simulink/Math Operations/Sum',[model_name '/err_sum'],...
    'Position',[1000,50,1030,110]);
set_param([model_name '/err_sum'],'Inputs','+-','IconShape','round');

% --- K_gain: 状态反馈增益矩阵 ---
% 执行矩阵乘法: u_fb = K_long * error (2×5 × 5×1 = 2×1)
% K_long(1,:) → δth 反馈 (油门)
% K_long(2,:) → δe 反馈 (升降舵)
% 关键增益 K(2,3): θ 误差 → 升降舵
%   K(2,3) < 0: θ 偏低 → 负误差 → 负×负 = 正δe → 抬头 ✓
%   K(2,3) > 0: θ 偏低 → 正δe? → 低头 ✗ (方向错误)
add_block('simulink/Math Operations/Gain',[model_name '/K_gain'],...
    'Position',[1080,50,1130,110]);
set_param([model_name '/K_gain'],'Gain','K_long','Multiplication','Matrix(K*u)');

% --- fb_enable: 反馈使能开关 (Product 块) ---
% 实现: 反馈信号 × fb_gain
%   fb_gain=0: 乘积=0 → 开环 (纯前馈)
%   fb_gain=1: 乘积=反馈 → 闭环 (前馈+反馈)
% 使用 Product 块而非 Manual Switch 的原因:
%   Manual Switch 的 'sw' 参数在 sim() 调用时被编译锁定,
%   set_param() 无法在 sim() 前更新——这是 lab3_0520 失败的根本原因
% Product 块的参数在仿真开始前由 MATLAB 求值, 每次 sim() 都是新的
add_block('simulink/Math Operations/Product',[model_name '/fb_enable'],...
    'Position',[1160,60,1190,100]);
set_param([model_name '/fb_enable'],'Inputs','**','Multiplication','Element-wise(.*)');

% --- fb_demux: 2 路反馈分离 ---
% 将 2×1 反馈向量分离为:
%   端口1: δth 反馈 (→ sum_thrust)
%   端口2: δe 反馈 (→ sum_elev)
add_block('simulink/Signal Routing/Demux',[model_name '/fb_demux'],...
    'Position',[1230,60,1260,110]);
set_param([model_name '/fb_demux'],'Outputs','2','DisplayOption','bar');

%% ====== 输出部分 ======

% --- states_out: 状态输出到 MATLAB ---
% 通过 Out1 块将 13 维状态向量返回 MATLAB
% 在 sim() 的输出中通过 out.yout{1}.Values.Data 访问
add_block('simulink/Sinks/Out1',[model_name '/states_out'],...
    'Position',[880,280,920,315]);

% --- scope_demux: 13 维状态分解 ---
% 将 13 维状态分解为 13 路标量, 分别送入各 Scope
add_block('simulink/Signal Routing/Demux',[model_name '/scope_demux'],...
    'Position',[990,280,1020,490]);
set_param([model_name '/scope_demux'],'Outputs','13','DisplayOption','bar');

% --- 5 个 Scope: 显示关键状态 ---
scopes={'V','alpha','theta','q','h'}; SY=280;
for i=1:5
    add_block('simulink/Sinks/Scope',[model_name '/Scope_' scopes{i}],...
        'Position',[1060,SY,1110,SY+30]);
    set_param([model_name '/Scope_' scopes{i}],'OpenAtSimulationStart','off');
    SY=SY+40;
end

%% ==== 连线 ====
% 所有连线使用 add_line, 格式: '源模块/端口号', '目标模块/端口号'

% --- 油门通道 ---
add_line(model_name,'thrust_base/1','sum_thrust/1');     % 基线 → 求和
add_line(model_name,'sum_thrust/1','sat_thrust/1');      % 求和 → 限幅
add_line(model_name,'sat_thrust/1','ctrl_mux/1');        % 限幅 → Mux端口1

% --- 升降舵通道 (伺服 → 限幅) ---
% 伺服从 trim 初始值开始, 无瞬态
add_line(model_name,'elev_base/1','sum_elev/1');         % 基线 → 求和(端口1)
add_line(model_name,'elev_dist/1','sum_elev/2');         % 扰动 → 求和(端口2)
add_line(model_name,'sum_elev/1','servo_K_fwd/1');      % 求和 → 伺服前向增益
add_line(model_name,'servo_K_fwd/1','servo_sum/1');     % 前向增益 → 求和(+)
add_line(model_name,'servo_int/1','servo_K_fbk/1');     % 积分器输出 → 反馈增益
add_line(model_name,'servo_K_fbk/1','servo_sum/2');     % 反馈增益 → 求和(-)
add_line(model_name,'servo_sum/1','servo_int/1');       % 求和 → 积分器
add_line(model_name,'servo_int/1','sat_elev/1');        % 伺服输出 → 限幅
add_line(model_name,'sat_elev/1','ctrl_mux/2');          % 限幅 → Mux端口2

% --- 横向通道 (纵向控制中冻结) ---
add_line(model_name,'ail_base/1','ctrl_mux/3');          % 副翼 → Mux端口3
add_line(model_name,'rud_base/1','ctrl_mux/4');          % 方向舵 → Mux端口4

% --- F16 动力学 ---
add_line(model_name,'ctrl_mux/1','F16_dyn/1');           % 4 控 → F16端口1
add_line(model_name,'dlef/1','F16_dyn/2');               % dlef → F16端口2
add_line(model_name,'fi_flag/1','F16_dyn/3');            % fi_flag → F16端口3

% --- 状态输出路由 ---
add_line(model_name,'F16_dyn/1','state_sel/1');          % 13态 → 状态选择器
add_line(model_name,'F16_dyn/1','states_out/1');         % 13态 → Out1 (到MATLAB)
add_line(model_name,'F16_dyn/1','scope_demux/1');        % 13态 → Scope分解

% --- 积分路径连线 (仅 use_integral=true) ---
if use_integral
    % theta_sel: 从 F16 输出中提取 θ (第5个状态)
    add_line(model_name,'F16_dyn/1','theta_sel/1');
    % θ → 积分误差 (负端)
    add_line(model_name,'theta_sel/1','int_err/2');
    % θ_ref 常量 (从 x_ref_long(3) 取值)
    add_block('simulink/Sources/Constant',[model_name '/theta_ref_val'],...
        'Position',[940,200,980,220]);
    set_param([model_name '/theta_ref_val'],'Value','x_ref_long(3)','SampleTime','inf');
    % θ_ref → 积分误差 (正端)
    add_line(model_name,'theta_ref_val/1','int_err/1');
    % 误差 → 积分器 → Ki 增益 → Ki_enable(×fb_gain) → 升降舵求和 (端口4)
    add_line(model_name,'int_err/1','theta_integrator/1');
    add_line(model_name,'theta_integrator/1','Ki_gain/1');
    add_line(model_name,'Ki_gain/1','Ki_enable/1');
    add_line(model_name,'fb_gain/1','Ki_enable/2');
    add_line(model_name,'Ki_enable/1','sum_elev/4');
end

% --- 反馈控制路径 ---
add_line(model_name,'x_ref/1','err_sum/1');              % 参考 → 误差(正端)
add_line(model_name,'state_sel/1','err_sum/2');           % 状态 → 误差(负端)
add_line(model_name,'err_sum/1','K_gain/1');              % 误差 → 增益
add_line(model_name,'K_gain/1','fb_enable/1');            % 增益 → 使能(端口1)
add_line(model_name,'fb_gain/1','fb_enable/2');           % fb_gain → 使能(端口2)
add_line(model_name,'fb_enable/1','fb_demux/1');          % 使能输出 → 分解
add_line(model_name,'fb_demux/1','sum_thrust/2');         % δth反馈 → 油门求和
add_line(model_name,'fb_demux/2','sum_elev/3');           % δe反馈 → 升降舵求和(端口3)

% --- Scope 连线 ---
% scope_demux 端口对应: 1=V, 3=α, 5=θ, 8=q, 12=-h
add_line(model_name,'scope_demux/1','Scope_V/1');
add_line(model_name,'scope_demux/3','Scope_alpha/1');
add_line(model_name,'scope_demux/5','Scope_theta/1');
add_line(model_name,'scope_demux/8','Scope_q/1');
add_line(model_name,'scope_demux/12','Scope_h/1');

%% ==== 自动布局 + 保存 ====
try Simulink.BlockDiagram.arrangeSystem(model_name,'FullLayout','true'); catch; end
save_system(model_name,fullfile(out_dir,[model_name '.slx']));
fprintf('  模型已保存: test_models/%s.slx',model_name);
if use_integral, fprintf(' [含积分控制]'); end
fprintf('\n');
end
