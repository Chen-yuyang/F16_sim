% ================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     Extract the longitudal and lateral 
%     direction matrices.  These system matrices 
%     will be used to create pole-zero mapping
%     and the bode plots of each to each control
%     input.
% Author: Richard S. Russell
% 从完整的非线性动力学中提取纵向（longitudinal）
% 与横侧向（lateral）线性状态空间模型。这些矩阵可用于
% 后续的极点-零点图、Bode 图分析
% 并进行控制器设计（如极点配置、LQR）。
% ================================================
clear all;
clc ;

global fi_flag_Simulink

newline = sprintf('\n');

% Trim aircraft to desired altitude and velocity
%
altitude = input('Enter the altitude for the simulation (m)  :  ');
velocity = input('Enter the velocity for the simulation (m/s):  ');

% Initial guess for trim
%
beta = 0;                % 侧滑角β (rad)
dth = 0.2;               % thrust, newton（有误）
                         % 油门杆位置，无量纲，范围[0,1]
elevator =  -2*pi/180;   % elevator, rad 升降舵偏角 (rad) 初始猜测 -2°（轻微低头）
alpha = 10*pi/180;       % AOA, rad 攻角
rudder =0;               % rudder angle, rad 方向舵偏角
aileron =0;              % aileron, rad 副翼偏角

% % %% Find trim for Hifi model at desired altitude and velocity
% % %%
% % disp('Trimming High Fidelity Model:');
% % fi_flag_Simulink = 1;
% % 
% % [trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(beta, elevator, alpha, aileron, rudder, dth, altitude, velocity);
% % % For simulink 
% % 
% % init_x = trim_state_hi;
% % init_u = [trim_thrust_hi;trim_control_hi];
% % init_dlef = dLEF;
% % 
% % %% Find the state space model for the hifi model at the desired alt and vel.
% % %%
% % trim_state_lin = trim_state_hi;
%trim_thrust_lin = trim_thrust_hi;
%trim_control_lin = trim_control_hi;
% % 
% % [A_hi,B_hi,C_hi,D_hi] = linmod('F16_openloop_linearization', trim_state_lin, [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % Find trim for Lofi model at desired altitude and velocity
% % %
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(beta, elevator, alpha, aileron, rudder, dth, altitude, velocity);
%% Find the state space model for the Lofi model at the desired alt and vel.
%% 设置线性化工作点
trim_state_lin = trim_state_lo; 
trim_thrust_lin = trim_thrust_lo; 
trim_control_lin = trim_control_lo;
init_x = trim_state_lo;
init_u = [trim_thrust_lo;trim_control_lo];
init_dlef=dLEF;


[A_lo,B_lo,C_lo,D_lo] = linmod('F16_openloop_linearization', trim_state_lin, [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

% linmod obtains linear models from systems of ordinary differential equations（常微分方程） described as Simulink models and
% returns the linear model in state-space form, A, B, C, D, which describes the linearized input-output relationship
% linmod：MATLAB内置函数
% 从Simulink模型F16_openloop_linearization在指定工作点线性化
% 返回状态空间矩阵：A_lo 状态矩阵;B_lo 控制矩阵;C_lo 输出矩阵;D_lo 前馈矩阵


%% 创建整体状态空间系统与增广矩阵
%% Make state space model
%% 构造状态空间模型对象，方便系统分析、连接与绘图
% 调用的是MATLAB Control System Toolbox中的ss函数
% ss是“state space”的缩写
% 可以直接用统一的分析函数
% 可以直接用+、*或series,parallel,feedback等命令把多个系统连接起来
% 许多控制工具箱函数（如 lqr、place、kalman）可以直接接受ss对象作为输入
% 而不需要手动传入A, B矩阵
% SS_hi = ss(A_hi,B_hi,C_hi,D_hi);
SS_lo = ss(A_lo,B_lo,C_lo,D_lo);


%% Make MATLAB matrix
%%
% mat_hi = [A_hi B_hi; C_hi D_hi];
mat_lo = [A_lo B_lo; C_lo D_lo];


%% 提取纵向初始状态与横侧向初始状态
% 根据 trim_F16.m 中的状态向量定义，将工作点状态拆分为纵向和横侧向两组：
%   完整状态 x = [V, beta, alpha, phi, theta, psi, p, q, r, 0, 0, -alt, pow]
%   索引：      1   2     3      4    5      6    7  8  9  10 11  12    13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Directional %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 纵向状态（5维）：[V, alpha, theta, q, pow]
%   V(1)     — 空速 (m/s)
%   alpha(3) — 迎角/攻角 (rad)
%   theta(5) — 俯仰角 (rad)
%   q(8)     — 俯仰角速率 (rad/s)
%   pow(13)  — 发动机功率参数 (无量纲)
%  MATLAB的矩阵索引语法
% 从配平状态向量 trim_state_lo 中提取出与纵向运动相关的状态分量
% 第一个参数 [1 3 5 8 13]：行索引向量，指定要提取哪些行
init_longitude_x=trim_state_lo([1 3 5 8 13],:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lateral Directional %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 横侧向状态（5维）：[beta, phi, psi, p, r]
%   beta(2) — 侧滑角 (rad)
%   phi(4)  — 滚转角 (rad)
%   psi(6)  — 偏航角 (rad)
%   p(7)    — 滚转角速率 (rad/s)
%   r(9)    — 偏航角速率 (rad/s)
init_lateral_x=trim_state_lo([2 4 6 7 9],:);%beta,phi,psi，p,r

%% Select the components that make up the longitude A matrix
%%
%see trim_F16.m, 
%trim_state=xu(1:13);
%trim_thrust=uu(1);
%trim_control=[uu(2);uu(3);uu(4)];
%dLEF = uu(5);
% UX0 = [beta; elevator; alpha; aileron; rudder; dth];
%xu = [velocity UX0(1) UX0(3) phi*(pi/180) theta*pi/180 psi*(pi/180) p q r 0 0 -altitude pow]';
%uu = [UX0(6) UX0(2) UX0(4) UX0(5) dLEF fi_flag_Simulink]';
%According to the above info, 
% state x=[velocity,beta,alpha,phi,theta,psi,p,q,r,0,0,-altitude,pow];
% control u=[thrust,elevator,aileron,rudder];
% 综上，线性化系统为 13 状态 × 4 控制：
%   state    x = [V, beta, alpha, phi, theta, psi, p, q, r, 0, 0, -alt, pow]
%   control  u = [thrust, elevator, aileron, rudder]
%               索引 14      15        16       17

%% 提取A矩阵（状态矩阵）
%%
% sel(M, rows, cols) 为自定义子矩阵提取函数
% 作用：从增广矩阵 mat_lo = [A B; C D] 中抽取指定行列
% 从增广矩阵中提取纵向状态对应的 A 矩阵（5×5）

%A_longitude_lo = sel(mat_lo,[1 3 5 8 12 13], [1 3 5 8 12 13]);

A_longitude_lo = sel(mat_lo,[1 3 5 8 13], [1 3 5 8 13]);
% 纵向状态矩阵 5×5：仅保留 V, alpha, theta, q, pow 五个状态之间的耦合关系
% 被忽略的状态（高度/侧向）对纵向的影响视为扰动，简化设计

A_lateral_lo = sel(mat_lo,[2 4 6 7 9], [2 4 6 7 9]);
% 横侧向状态矩阵 5×5：保留 beta, phi, psi, p, r 五个状态之间的耦合关系


%% Select the components that make up the longitude B matrix
%% 提取B矩阵（控制矩阵）
% B矩阵描述控制输入如何影响状态变量的变化率

%B_longitude_lo = sel(mat_lo,[1 3 5 8 12 13], [14 15]);

B_longitude_lo = sel(mat_lo,[1 3 5 8 13], [14 15]);
% 纵向控制矩阵 5×2：控制量为 [油门 thrust, 升降舵 elevator]
%   第1列 — 油门对纵向状态的影响（速度/俯仰响应）
%   第2列 — 升降舵对纵向状态的影响（俯仰操纵）

B_lateral_lo = sel(mat_lo,[2 4 6 7 9], [16 17]);
% 横侧向控制矩阵 5×2：控制量为 [副翼 aileron, 方向舵 rudder]
%   第1列 — 副翼对横侧向状态的影响（滚转操纵）
%   第2列 — 方向舵对横侧向状态的影响（偏航操纵/协调转弯）


%% Select the components that make up the longitude C matrix
%% 提取C矩阵（输出矩阵）
% C 矩阵描述状态变量到量测/评价输出的映射关系
% 输出y可能是状态？

%C_longitude_lo = sel(mat_lo,[14 16 18 21 25 26], [1 3 5 8 12 13]);

C_longitude_lo = sel(mat_lo,[14 16 18 21 26], [1 3 5 8 13]);
% 纵向输出矩阵 5×5：输出的物理含义需结合 Simulink 模型中的 output port 定义
C_lateral_lo = sel(mat_lo,[15 17 19 20 22],[2 4 6 7 9]);

%% Select the components that make up the longitude D matrix
%% 提取D矩阵（前馈/直通矩阵）

%D_longitude_lo = sel(mat_lo,[14 16 18 21 25 26], [14 15]);
D_longitude_lo = sel(mat_lo,[14 16 18 21 26], [14 15]);
D_lateral_lo = sel(mat_lo,[15 17 19 20 22],[16 17]);

% 可选的：构造完整状态空间对象以便后续分析
%SS_long_lo = ss(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);


%% Make longitudal direction SYSTEM matrix
% %%
% % sys_long_hi = pck(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
% sys_long_lo = pck(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);
% pck是旧版Robust Control Toolbox中的函数，作用与ss类似，将四个矩阵打包成一个系统变量
% 本脚本中前面已经用 SS_lo = ss(...) 构建了整体系统，这里注释掉避免重复。


%Q=[0 0 0 0;0 500 0 0;0 0 500 0;0 0 0 0];
%R=[1 0;0 1];
%Klqr=lqr(A_longitude_lo,B_longitude_lo,Q,R);
% LQR（线性二次型调节器） 通过选择状态权重矩阵 Q 和控制权重矩阵 R，最小化性能指标 
% J解出最优状态反馈增益 Klqr。

P=[-5.6+4.2i,-5.6-4.2i,-1,-0.8,-1];
K=place(A_longitude_lo,B_longitude_lo,P);
%xd=[200;-0.5263;0.0349;0];

%Xd_Lateral_lo=[0.3  0.05  0.01  0.01  0.01]';
%Ud_Lateral_lo=[init_u(3) init_u(4)]';
P_lateral_lo=[-0.4;-0.4;-1.2;-1.2;-0.8];
K_lateral=place(A_lateral_lo,B_lateral_lo,P_lateral_lo);

%[x,y,z]=solve('-0.2158*x+0.0105*y+0.0309*z=0','-26.4913*x-34.2117*y+6.5983*z=0','8.2850*x-1.5291*y-3.2287*z=0','x','y','z')
