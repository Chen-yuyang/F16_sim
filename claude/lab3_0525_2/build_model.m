%% build_model.m — 以编程方式 (Programmatic API) 构建 F-16 闭环 Simulink 模型
%==========================================================================
%  功能:
%    使用 Simulink 的 add_block/add_line/set_param 等命令,
%    以 MATLAB 代码完全自动化地构建闭环仿真模型, 不依赖 .slx 文件模板。
%
%  模型结构:
%    信号源(列1) → 求和+作动器伺服(列2) → F16_dyn S-Function(列3) → 控制器(列4)
%
%  控制律实现:
%    u(1:2) = init_u_base(1:2) + fb_gain * K_long * (x_ref_long - x_long)
%    fb_gain=0 → 开环 (仅基线控制, 无反馈)
%    fb_gain=1 → 闭环 (反馈全通)
%    Product 模块实现 0/1 切换 (×0 断开, ×1 导通)
%
%  伺服作动器链 (升降舵通道):
%    sum_elev → Rate Limiter(±60°/s) → Transfer Fcn(20.2/(s+20.2)) → Saturation(±0.44rad)
%    Rate Limiter: 模拟真实液压舵机的速率限制 (物理约束)
%    Transfer Fcn: 模拟伺服阀的电气-机械响应延迟
%    Saturation: 模拟舵面偏转角度限位
%
%  输入参数:
%    model_name — Simulink 模型名称 (字符串), 默认 'F16_ClosedLoop'
%  前置条件:
%    base workspace 中必须存在以下变量:
%      init_x(13×1), init_u_base(4×1), init_dlef(标量),
%      fi_flag_Simulink(0/1), K_long(2×5), x_ref_long(5×1), fb_gain(0/1)
%  输出:
%    在 test_models/ 下生成 <model_name>.slx 文件
%==========================================================================

function build_model(model_name)
% 输入参数: model_name — 模型名称, 默认 'F16_ClosedLoop'
if nargin<1, model_name='F16_ClosedLoop'; end

% --- 路径设置 ---
% mfilename('fullpath') 获取本函数的完整路径
% build_model.m 在 lab3_0525_2/ 目录下
this_dir = fileparts(mfilename('fullpath'));
% 上两级目录 = 项目根目录 FC_SimCode_1/
proj_root = fileparts(fileparts(fileparts(this_dir)));
% 模型文件保存目录: test_models/
out_dir  = fullfile(this_dir,'test_models');
% 如果 test_models 目录不存在, 创建它
if ~exist(out_dir,'dir'), mkdir(out_dir); end
addpath(genpath(proj_root));  % 递归添加项目所有子目录到搜索路径
cd(proj_root);  % 切换工作目录到项目根

% --- 检查 base workspace 的必要变量 ---
% Simulink 模型运行时从 base workspace 读取变量值
% 如果缺少任一变量, 仿真会报错
vars={'init_x','init_u_base','init_dlef','fi_flag_Simulink','K_long','x_ref_long','fb_gain'};
for i=1:numel(vars)
    % evalin('base',...) 在 base workspace 中执行 MATLAB 表达式
    if ~evalin('base',sprintf('exist(''%s'',''var'')',vars{i}))
        error('缺少变量: %s',vars{i});
    end
end

% 检查/创建升降舵扰动信号
% elevator_disturb 是一个两列矩阵: [时间, 幅值]
% 用于模拟外界扰动对升降舵的影响 (验证控制器的抗干扰能力)
% 格式: From Workspace 模块读取, 时间序列插值
if ~evalin('base','exist(''elevator_disturb'',''var'')')
    % 信号定义:
    %   0→1s:  0 (无扰动)
    %   2→5s:  +5° (=0.0873rad, 升降舵上偏, 低头力矩)
    %   5→6s:  -5° (下偏, 抬头力矩)
    %   6→30s: 0 (归零)
    td=[0 1 1 2 2 5 5 6 6 30];  % 时间节点
    amp=5*pi/180;  % 幅值 5° → 弧度
    % 构建 [时间, 幅值] 两列矩阵
    assignin('base','elevator_disturb',[td',[0;0;amp;amp;-amp;-amp;0;0;0;0]]);
end

% --- 清理旧模型 ---
% 如果模型已加载到内存, 关闭并删除 (避免冲突)
if bdIsLoaded(model_name), close_system(model_name,1); end  % 1=删除文件
% 删除已有的 .slx 和 .slxc (Simulink 缓存) 文件
for ext={'.slx','.slxc'}
    f=fullfile(out_dir,[model_name ext{1}]);
    if exist(f,'file'), delete(f); end
end

% --- 创建新模型 ---
% new_system 创建空白模型, 'Model' 表示普通 Simulink 模型
new_system(model_name,'Model');

% --- 设置求解器参数 ---
% ode4: 四阶 Runge-Kutta 定步长求解器 (精度较高, 适合实时仿真)
% FixedStep=0.01: 固定步长 10ms (100Hz), 适合 F-16 的气动动态
% StopTime=30: 仿真时长 30 秒
% SaveOutput=on: 保存输出端口数据到工作空间
set_param(model_name,'Solver','ode4','FixedStep','0.01','StopTime','30');
set_param(model_name,'SaveState','on','SaveOutput','on','SaveTime','on','ReturnWorkspaceOutputs','on');

%% ========== 布局参数 ==========
% Y: 纵向坐标起始 (像素), DY: 每行垂直间距
% Simulink 模块位置为 [left, top, right, bottom] 四元组
Y=35; DY=50;

%% ---- 列1: 信号源 (Sources) ----
% Constant 模块: 输出恒值信号, 从 base workspace 读取
% 注意: init_u_base(4×1) = [δth; δe; δa; δr] 是配平控制量

% 油门基线 (δth): 从 init_u_base(1) 读取
add_block('simulink/Sources/Constant',[model_name '/thrust_base'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/thrust_base'],'Value','init_u_base(1)','SampleTime','inf'); Y=Y+DY;

% 升降舵基线 (δe): 从 init_u_base(2) 读取
add_block('simulink/Sources/Constant',[model_name '/elev_base'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/elev_base'],'Value','init_u_base(2)','SampleTime','inf'); Y=Y+DY;

% 副翼基线 (δa): 从 init_u_base(3) 读取 (纵向控制中不变)
add_block('simulink/Sources/Constant',[model_name '/ail_base'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/ail_base'],'Value','init_u_base(3)','SampleTime','inf'); Y=Y+DY;

% 方向舵基线 (δr): 从 init_u_base(4) 读取
add_block('simulink/Sources/Constant',[model_name '/rud_base'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/rud_base'],'Value','init_u_base(4)','SampleTime','inf'); Y=Y+DY+20;
% ↑ 额外 20px 间距, 与下面 F16_dyn 的直接输入区分

% 前缘襟翼 (dlef): F16_dyn S-Function 的第二个输入
% dlef 从配平数据中获取, 在仿真过程中保持不变
add_block('simulink/Sources/Constant',[model_name '/dlef'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/dlef'],'Value','init_dlef','SampleTime','inf'); Y=Y+DY;

% HIFI 标志: F16_dyn 的第三个输入 (0=LOFI, 1=HIFI)
add_block('simulink/Sources/Constant',[model_name '/fi_flag'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/fi_flag'],'Value','fi_flag_Simulink','SampleTime','inf'); Y=Y+DY;

% 升降舵扰动信号: 从 base workspace 的 elevator_disturb 变量读取
% OutputAfterFinalValue='Setting to zero': 超出时间范围后输出零
add_block('simulink/Sources/From Workspace',[model_name '/elev_dist'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/elev_dist'],'VariableName','elevator_disturb','SampleTime','0.01','OutputAfterFinalValue','Setting to zero'); Y=Y+DY+20;

% 参考状态 (x_ref_long): 5×1 向量 [V_ref; α_ref; θ_ref; q_ref; Pow_ref]
add_block('simulink/Sources/Constant',[model_name '/x_ref'],'Position',[50,Y,160,Y+40]);
set_param([model_name '/x_ref'],'Value','x_ref_long','SampleTime','inf');

% 反馈使能增益 (fb_gain): 0=开环, 1=闭环
% Product 模块将 K_gain 输出乘以 fb_gain, 实现 OL/CL 切换
add_block('simulink/Sources/Constant',[model_name '/fb_gain'],'Position',[50,Y+60,160,Y+90]);
set_param([model_name '/fb_gain'],'Value','fb_gain','SampleTime','inf');

%% ---- 列2: 求和 + 作动器伺服 (Actuator Dynamics) ----
% 油门通道: 基线 + 反馈 → 饱和限幅
% 油门是标量, 不需要伺服作动器 (发动机响应比舵面慢得多, 但在此简化)
% sum_thrust: 油门基线 + 油门反馈 (来自控制器 δth 通道)
add_block('simulink/Math Operations/Sum',[model_name '/sum_thrust'],'Position',[250,35,280,70]);
set_param([model_name '/sum_thrust'],'Inputs','++','IconShape','round');  % 两个输入求和
% sat_thrust: 油门饱和限幅 [0,1], 发动机油门只能在 0%~100% 之间
add_block('simulink/Discontinuities/Saturation',[model_name '/sat_thrust'],'Position',[320,35,360,70]);
set_param([model_name '/sat_thrust'],'UpperLimit','1','LowerLimit','0');

% 升降舵通道: 基线 + 扰动 + 反馈 → 伺服作动器 → 饱和
% sum_elev: 升降舵基线 + 扰动 + 升降舵反馈 (三个输入求和)
add_block('simulink/Math Operations/Sum',[model_name '/sum_elev'],'Position',[250,85,280,130]);
set_param([model_name '/sum_elev'],'Inputs','+++','IconShape','round');  % 三个输入

% ★ 作动器伺服 — Rate Limiter (速率限制器):
%   模拟真实液压舵机的最大偏转速率
%   ±60°/s = ±1.047 rad/s: F-16 升降舵的典型速率限制
%   如果控制指令变化太快, Rate Limiter 会将其限制在 ±60°/s 以内
%   这防止了控制器输出过大阶跃导致舵机无法跟随
add_block('simulink/Discontinuities/Rate Limiter',[model_name '/elev_rate'],'Position',[320,85,360,130]);
set_param([model_name '/elev_rate'],'RisingSlewRate','1.047','FallingSlewRate','-1.047','SampleTimeMode','continuous');

% ★ 作动器伺服 — 一阶传递函数 (Transfer Fcn):
%   G(s) = 20.2/(s+20.2), 时间常数 τ = 1/20.2 ≈ 0.05s
%   带宽 ω_bw ≈ 20.2 rad/s ≈ 3.2 Hz: 舵机响应速度
%   物理含义: 电气指令→液压阀→舵面偏转的完整伺服响应
%   初始条件设为 init_u_base(2): 使仿真起始时刻舵面在配平位置
add_block('simulink/Continuous/Transfer Fcn',[model_name '/elev_servo'],'Position',[390,85,440,130]);
set_param([model_name '/elev_servo'],'Numerator','[20.2]','Denominator','[1 20.2]','InitialConditions','init_u_base(2)');

% 升降舵饱和: 物理限位 ±0.44rad (±25°)
% F-16 升降舵最大偏转范围约为 ±25°, 超出此限位会导致机械碰撞
add_block('simulink/Discontinuities/Saturation',[model_name '/sat_elev'],'Position',[470,85,510,130]);
set_param([model_name '/sat_elev'],'UpperLimit','0.44','LowerLimit','-0.44');

%% ---- 列3: Mux + F16_dyn S-Function ----
% ctrl_mux: 将 4 个控制信号合成一个向量
% 输入顺序: [δth, δe, δa, δr] → 对应 F16_dyn 的第一个输入端口
add_block('simulink/Signal Routing/Mux',[model_name '/ctrl_mux'],'Position',[560,30,600,180]);
set_param([model_name '/ctrl_mux'],'Inputs','4','DisplayOption','bar');  % 4输入1输出

% F16_dyn: F-16 非线性气动模型 S-Function (编译自 Fortran)
%   第一个输入: 4 维控制向量 [δth, δe, δa, δr]
%   第二个输入: 前缘襟翼偏转角 dlef (标量)
%   第三个输入: HIFI 标志 (0=LOFI, 1=HIFI)
%   参数 'init_x': 13 维初始状态向量
%   输出: 13 维状态向量 [V, β, α, φ, θ, ψ, p, q, r, xE, yE, -h, Pow]'
add_block('simulink/User-Defined Functions/S-Function',[model_name '/F16_dyn'],'Position',[680,30,800,230]);
set_param([model_name '/F16_dyn'],'FunctionName','F16_dyn','Parameters','init_x');

%% ---- 列4: 控制器 (Feedback Controller) ----
% state_sel (Selector): 从 13 维全状态中提取纵向 5 维子集
% 索引 [1,3,5,8,13]: V(速度), α(迎角), θ(俯仰角), q(俯仰速率), Pow(发动机功率)
% 这是控制器需要的状态变量, 其他状态 (如 β, φ, ψ 等横侧向) 被忽略
add_block('simulink/Signal Routing/Selector',[model_name '/state_sel'],'Position',[870,40,940,110]);
set_param([model_name '/state_sel'],'InputPortWidth','13','Indices','[1,3,5,8,13]','IndexOptions','Index vector (dialog)');

% err_sum: 计算误差 e = x_ref_long - x_long
% 输入端口: '+' = x_ref_long, '-' = x_long (负反馈)
% 控制律: u_fb = K_long * e
add_block('simulink/Math Operations/Sum',[model_name '/err_sum'],'Position',[990,50,1020,110]);
set_param([model_name '/err_sum'],'Inputs','+-','IconShape','round');

% K_gain: 状态反馈增益矩阵 (2×5)
% Multiplication='Matrix(K*u)': 矩阵乘法, 输出 2 维向量
% K_long 由 f16_controller_design 计算并写入 base workspace
add_block('simulink/Math Operations/Gain',[model_name '/K_gain'],'Position',[1070,50,1120,110]);
set_param([model_name '/K_gain'],'Gain','K_long','Multiplication','Matrix(K*u)');

% ★ fb_enable (Product): 反馈使能开关
% 功能: K_gain 的输出 × fb_gain
%   fb_gain=0: 乘积为零 → 开环 (控制器被屏蔽)
%   fb_gain=1: 乘积不变 → 闭环 (控制器全通)
% 使用 Product 而不是 Manual Switch 的原因:
%   Manual Switch 的切换参数在编译时固定, 无法运行时修改
%   Product 可以通过修改 base workspace 的 fb_gain 变量实时切换
%   (实际仿真中仍需重建模型, 但参数修改更方便)
add_block('simulink/Math Operations/Product',[model_name '/fb_enable'],'Position',[1150,60,1180,100]);
set_param([model_name '/fb_enable'],'Inputs','**','Multiplication','Element-wise(.*)');

% fb_demux: 将 2 维反馈控制向量分解为油门反馈和升降舵反馈
% 输出1 → δth 通道 (加到 sum_thrust)
% 输出2 → δe 通道 (加到 sum_elev)
add_block('simulink/Signal Routing/Demux',[model_name '/fb_demux'],'Position',[1220,60,1250,110]);
set_param([model_name '/fb_demux'],'Outputs','2','DisplayOption','bar');

%% ---- 输出和示波器 ----
% states_out (Out1): 将 13 维状态输出到 MATLAB 工作空间
% sim() 的返回值通过 Outport 收集数据
add_block('simulink/Sinks/Out1',[model_name '/states_out'],'Position',[870,280,910,315]);

% scope_demux: 将 13 维状态分解为 13 个标量信号
% 分别连到 5 个示波器, 方便观察关键状态
add_block('simulink/Signal Routing/Demux',[model_name '/scope_demux'],'Position',[980,280,1010,490]);
set_param([model_name '/scope_demux'],'Outputs','13','DisplayOption','bar');

% 创建 5 个示波器, 分别显示 V, α, θ, q, h
% Scope 模块: 在仿真过程中实时显示波形 (可不开)
scopes={'V','alpha','theta','q','h'}; SY=280;  % 起始纵坐标
for i=1:5
    add_block('simulink/Sinks/Scope',[model_name '/Scope_' scopes{i}],'Position',[1050,SY,1100,SY+30]);
    set_param([model_name '/Scope_' scopes{i}],'OpenAtSimulationStart','off'); SY=SY+40;
end

%% ========== 连接信号线 ==========
% 使用 add_line 将各模块的输出端口连接到输入端口
% 格式: add_line(model, 'src_block/port', 'dst_block/port')

% --- 油门路径: thrust_base → sum_thrust → sat_thrust → ctrl_mux(1) ---
add_line(model_name,'thrust_base/1','sum_thrust/1');   % 基线 → 求和
add_line(model_name,'sum_thrust/1','sat_thrust/1');    % 求和 → 饱和
add_line(model_name,'sat_thrust/1','ctrl_mux/1');      % 饱和 → Mux(油门)

% --- 升降舵路径: elev_base → sum_elev → rate → servo → sat → ctrl_mux(2) ---
add_line(model_name,'elev_base/1','sum_elev/1');       % 基线 → 求和
add_line(model_name,'elev_dist/1','sum_elev/2');        % 扰动 → 求和
add_line(model_name,'sum_elev/1','elev_rate/1');       % 求和 → 速率限制器
add_line(model_name,'elev_rate/1','elev_servo/1');     % 速率限制 → 伺服传递函数
add_line(model_name,'elev_servo/1','sat_elev/1');      % 伺服 → 饱和限幅
add_line(model_name,'sat_elev/1','ctrl_mux/2');         % 饱和 → Mux(升降舵)

% --- 副翼/方向舵: 直接到 Mux (无伺服, 纵向控制不变) ---
add_line(model_name,'ail_base/1','ctrl_mux/3');         % 副翼基线 → Mux
add_line(model_name,'rud_base/1','ctrl_mux/4');         % 方向舵基线 → Mux

% --- F16_dyn 输入连接 ---
add_line(model_name,'ctrl_mux/1','F16_dyn/1');          % 4 维控制 → F16_dyn
add_line(model_name,'dlef/1','F16_dyn/2');               % 前缘襟翼 → F16_dyn
add_line(model_name,'fi_flag/1','F16_dyn/3');            % HIFI 标志 → F16_dyn

% --- F16_dyn 输出分发 ---
add_line(model_name,'F16_dyn/1','state_sel/1');         % 13 维状态 → Selector
add_line(model_name,'F16_dyn/1','states_out/1');         % 13 维状态 → Out1
add_line(model_name,'F16_dyn/1','scope_demux/1');        % 13 维状态 → 示波器

% --- 控制器路径: x_ref → err_sum → K_gain → fb_enable → fb_demux ---
add_line(model_name,'x_ref/1','err_sum/1');             % x_ref → 误差(+)
add_line(model_name,'state_sel/1','err_sum/2');         % x_meas → 误差(-)
add_line(model_name,'err_sum/1','K_gain/1');            % 误差 → K 矩阵
add_line(model_name,'K_gain/1','fb_enable/1');          % 反馈 → Product
add_line(model_name,'fb_gain/1','fb_enable/2');          % 使能 → Product
add_line(model_name,'fb_enable/1','fb_demux/1');         % Product → Demux
add_line(model_name,'fb_demux/1','sum_thrust/2');        % δth 反馈 → 油门求和
add_line(model_name,'fb_demux/2','sum_elev/3');          % δe 反馈 → 升降舵求和

% --- 示波器连接: scope_demux 各通道 → Scope ---
add_line(model_name,'scope_demux/1','Scope_V/1');       % 通道1 → V
add_line(model_name,'scope_demux/3','Scope_alpha/1');   % 通道3 → α
add_line(model_name,'scope_demux/5','Scope_theta/1');   % 通道5 → θ
add_line(model_name,'scope_demux/8','Scope_q/1');       % 通道8 → q
add_line(model_name,'scope_demux/12','Scope_h/1');      % 通道12 → h (高度)

% --- 自动布局 ---
% Simulink 自动排列模块位置 (使连线更整齐)
% 如果排列失败 (低版本 Simulink 不支持), 忽略错误
try Simulink.BlockDiagram.arrangeSystem(model_name,'FullLayout','true'); catch; end

% --- 保存模型 ---
% 保存到 test_models/<model_name>.slx
save_system(model_name,fullfile(out_dir,[model_name '.slx']));
fprintf('  模型已保存: test_models/%s.slx\n',model_name);
end
