function [cost, Xdot, xu, uu] = trim_fun(UX0)
%=====================================================
%      F16 nonlinear model trim cost function
%   for longitudinal motion, steady level flight   对于纵向运动，稳定水平飞行
% (cost = sum of weighted squared state derivatives)
%
% Original authors: T. Keviczky, Richard S. Russell
% Date:   April 29, 2002, November 7th, 2002
%
%  File made suitable for trimming S-function dynamics
%
%  L. Sonneveldt May 2006
% 
% 整个函数的目标是计算飞机 纵向、稳定水平飞行 时的配平代价
% 它接收一组自由参数（侧滑角、升降舵、攻角、副翼、方向舵、油门）
% 调用 F-16 非线性模型计算出当前状态下的所有状态导数
% 然后对导数进行加权平方求和，得到一个标量代价
%=====================================================
% 输入 UX0 为待优化的参数向量（6×1），输出有四个：
% cost：代价（标量，所有状态导数的加权平方和）
% Xdot：状态导数向量（13×1）
% xu：状态向量（13×1）
% uu：控制向量（7×1）
%=====================================================

% 声明三个全局变量，它们由主脚本在调用 fminsearch 前设定：
% altitude：目标配平高度（m）
% velocity：目标配平速度（m/s）
% fi_flag_Simulink：模型保真度标志（0=低 fidelity，1=高 fidelity）
global altitude velocity fi_flag_Simulink

%%
% 这部分的目的是将优化变量约束在 F-16 物理舵面和气动角的允许范围内
% 防止优化过程中出现不合理甚至危险的飞行状态。
% Implementing limits:
% 对 UX0(1)（侧滑角 β）限幅，范围 ±30°
if UX0(1) > 30*pi/180
    UX0(1) = 30*pi/180;
elseif UX0(1) < -30*pi/180
    UX0(1) = -30*pi/180;
end;

% 对 UX0(2)（升降舵偏角 elevator）限幅，范围 ±25°
% elevator limits
if UX0(2) > 25*pi/180
    UX0(2) = 25*pi/180;
elseif UX0(2) < -25*pi/180
    UX0(2) = -25*pi/180;
end;

% 对 UX0(3)（攻角 α）限幅。根据模型保真度不同，允许范围不同：
% 低 fidelity 模型：-10° ~ 45°
% 高 fidelity 模型：-20° ~ 90°
% angle of attack limits
if (fi_flag_Simulink == 0)
  if UX0(3) > 45*pi/180
    UX0(3) = 45*pi/180;
  elseif UX0(3) < -10*pi/180
    UX0(3) = -10*pi/180;
  end
elseif (fi_flag_Simulink == 1)
  if UX0(3) > 90*pi/180
    UX0(3) = 90*pi/180;
  elseif UX0(3) < -20*pi/180
    UX0(3) = -20*pi/180;
  end
end

% 对 UX0(4)（副翼偏角 aileron）限幅，范围 ±21.5°
%  Aileron limits
if UX0(4) > 21.5*pi/180
    UX0(4) = 21.5*pi/180;
elseif UX0(4) < -21.5*pi/180
    UX0(4) = -21.5*pi/180;
end;

% 对 UX0(5)（方向舵偏角 rudder）限幅，范围 ±30°
% Rudder limits
if UX0(5) > 30*pi/180
    UX0(5) = 30*pi/180;
elseif UX0(5) < -30*pi/180
    UX0(5) = -30*pi/180;
end;


if (fi_flag_Simulink == 1)
    rho0 = 1.225;                                     % 国际标准大气在海平面（高度 0 m）时的空气密度，单位 kg/m³
    temp = 288.15-altitude*0.0065;                    % 计算得到的当前高度处的大气温度（K），温度随海拔升高减小
    rho = rho0*exp(-9.80665/287.05/temp*altitude);    % 计算得到的当前高度处的大气密度（kg/m³）
    qbar = 0.5*rho*velocity^2;                        % 动压，计算气动力和力矩的基本量
    ps = rho/rho0*101325;                             % 通过密度比估计的当前高度的静压
    
    % 这一经验公式根据攻角和飞行状态（动压/静压比）计算前缘襟翼（leading-edge flap）的偏转角度
    % 单位为度。最后乘 pi/180 将结果转回弧度
    % F-16 的前缘襟翼在高攻角时会自动向下偏转以改善失速特性。此公式模拟了飞行控制系统自动调节前缘襟翼偏度的调度律。
    dLEF = (1.38*UX0(3)*180/pi - 9.05*qbar/ps + 1.45)*pi/180; 
elseif (fi_flag_Simulink == 0)
    dLEF = 0.0;
end

% 对 UX0(6)（油门杆位置 dth）限幅，范围 [0,1]
% dth limits
if UX0(6) > 1
    UX0(6) = 1;
elseif UX0(6) < 0
    UX0(6) = 0;
end;

% 确保前缘襟翼偏角在 [0°, 25°] 范围内，防止物理上不合理
% Verify that the calculated leading edge flap
% have not been violated.
if (dLEF > 25*pi/180)
    dLEF = 25*pi/180;
elseif (dLEF < 0)
    dLEF = 0;
end;
%%


% Initialize the other variables
% 这里尝试使用四元数，UX0(3)表示攻角
% 这个四元数表示绕y轴旋转
% q0 = cos(UX0(3)/2);
% q1 = 0;
% q2 = sin(UX0(3)/2);
% q3 = 0;

% 配平时假设飞机处于无滚转（机翼水平）的对称稳态飞行
% 绝大多数配平情况确实要求滚转角为 0（除非是定常盘旋）
phi=0;              
%theta=0.349;该值对应度数为20度，是否太大?
theta=0.0349;       % 俯仰角
psi=0;              % 偏航角
% 配平状态要求飞机无转动加速度，即所有角速率导数应为零
% 此处目标为直线稳定水平飞行，其他飞行状态可能不为0
p = 0; 
q = 0; 
r = 0;
% tgear是一个油门杆位置到发动机功率的转换函数，pow是转换后的功率百分比
pow = tgear(UX0(6));  % taken from Ying Huo's model
%%
   
tu = 0;
%xu = [velocity UX0(1) UX0(3) phi*(pi/180) theta*pi/180 psi*(pi/180) p q r 0 0 -altitude pow]';
%  真空速 侧滑角(rad)  攻角   滚转 俯仰  偏航 ... 北向位置 东向位置 负的高度 发动机功率百分比
xu = [velocity UX0(1) UX0(3) phi theta psi p q r 0 0 -altitude pow]'; % 加上了转置，变为列向量
% 油门杆位置  升降舵  副翼  方向舵  前缘襟翼 
uu = [UX0(6) UX0(2) UX0(4) UX0(5) dLEF fi_flag_Simulink]';
% 以字符串 'derivs' 为命令调用函数 F16_trim。输出状态导数
dx = feval('F16_trim', tu, xu, uu, 'derivs');
% 将导数赋给输出变量 Xdot，便于外部检查配平质量
Xdot = dx;

% Create weight function
weight = [  2            ...%Vt_dot
            10           ...%beta_dot
            10           ...%alpha_dot
            10           ...%phi_dot
            10           ...%theta_dot
            10           ...%psi_dot
            10           ...%p_dot
            10           ...%q_dot
            10           ...%r_dot
            0            ...%x_dot
            0            ...%y_dot
            5            ...%z_dot
            50            ...%pow_dot
            ];

cost = weight*(Xdot.*Xdot);