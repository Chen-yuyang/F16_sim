function [trim_state, trim_thrust, trim_control, dLEF, xu] = trim_F16(beta, elevator, alpha, aileron, rudder, dth, alt,vel)
%================================================
%     F16 nonlinear model trimming routine
%  for longitudinal motion, steady level flight 对于纵向运动，稳定水平飞行
%
% Author: T. Keviczky
% Date:   April 29, 2002
%
%
%      Added addtional functionality. 添加了附加功能
%      This trim function can now trim at three 此配平功能现在可以在三个附加飞行条件下配平
%      additional flight conditions
%         -  Steady Turning Flight given turn rate       给定转弯率的稳定转弯飞行
%         -  Steady Pull-up flight - given pull-up rate  稳定上拉飞行
%         -  Steady Roll - given roll rate               稳定滚转飞行
%
% Coauthor: Richard S. Russell
% Date:     November 7th, 2002
%
% 注释说明该函数最早用于纵向稳定水平飞行的配平，后续扩充了更多飞行条件
%
% 输入：配平初始猜测值 beta, elevator, alpha, aileron, 
% rudder, dth，目标高度 alt，目标速度 vel。
% 输出：trim_state：配平后的 13 维状态向量。
% trim_thrust：配平后油门杆位置。
% trim_control：配平后操纵面偏角 [升降舵; 副翼; 方向舵]。
% dLEF：前缘襟翼偏角。
% xu：配平时的完整状态向量（可能与 trim_state 重复，便于调试）
%================================================

global altitude velocity fi_flag_Simulink Ma  % Ma马赫数，未在代码中使用，留作扩展
global  phi_weight theta_weight psi_weight    % 代价函数中欧拉角导数的权重
altitude = alt;
velocity = vel;

% 早期可能试图用结构体组织输出，但后来直接使用向量形式，故注释掉。
% state = struct('trim_state');
% dth = struct('trim_thrust');
% control = struct('trim_control');
% dlef = struct('dLEF');

%alpha = alpha*pi/180;  %convert to radians

% OUTPUTS: trimmed values for states and controls
% INPUTS:  guess values for thrust, elevator, alpha  (assuming steady level flight)

% Initial Guess for free parameters
% UX0 = [thrust; elevator; alpha; ail; rud];  % free parameters: two control values & angle of attack
UX0 = [beta; elevator; alpha; aileron; rudder; dth];

% Initialize some varibles
%
% phi = 0; psi = 0;
% q0 = cos(UX0(3)/2);
% q1 = 0;
% q2 = sin(UX0(3)/2);
% q3 = 0;
% 角速率初始设为 0（水平飞行默认角速率为 0）
p = 0; q = 0; r = 0;
phi_weight = 10; theta_weight = 10; psi_weight = 10;

disp('At what flight condition would you like to trim the F-16?');
disp('1.  Steady Wings-Level Flight.');  % 稳定水平飞行
disp('2.  Steady Turning Flight.');
disp('3.  Steady Pull-Up Flight.');
disp('4.  Steady Roll Flight.');
FC_flag = input('Your Selection:  ');

switch FC_flag
    case 1
        % do nothing
    case 2
        r = input('Enter the turning rate (deg/s):  ');
        psi_weight = 0;
    case 3
        q = input('Enter the pull-up rate (deg/s):  ');
        theta_weight = 0;
    case 4    
        p = input('Enter the Roll rate    (deg/s):  ');
        phi_weight = 0;
    otherwise
        disp('Invalid Selection')
%        break;
end

% Initializing optimization options and running optimization:
% TolFun: 设定优化目标函数的精度
% TolX:     设定被优化参数的精度
OPTIONS = optimset('TolFun',1e-10,'TolX',1e-10,'MaxFunEvals',5e+04,'MaxIter',1e+04);

iter = 1;
feval('F16_trim', [], [], [], 'lincompile');% https://www.ajpsp.com/zuoye/3687515 ，该网页上有人问了同样的问题，lincompile属于未公开的feval参数，表示： compile the model for linearization (used by linmod)
load_system('F16_trim');%调入内存，基本原理？
while iter == 1
   % feval('F16_trim', [], [], [], 'lincompile');% https://www.ajpsp.com/zuoye/3687515 ，该网页上有人问了同样的问题，lincompile属于未公开的feval参数，表示： compile the model for linearization (used by linmod)
   % load_system('F16_trim');%调入内存，基本原理？
    
    [UX,FVAL,EXITFLAG,OUTPUT] = fminsearch('trim_fun',UX0,OPTIONS);
   
    [cost, Xdot, xu, uu] = trim_fun(UX);
    
    disp('Trim Values and Cost:');
    disp(['cost   = ' num2str(cost)])
    disp(['dth    = ' num2str(uu(1)) ' -'])  
    disp(['elev   = ' num2str(uu(2)*180/pi) ' deg'])
    disp(['ail    = ' num2str(uu(3)*180/pi) ' deg'])
    disp(['rud    = ' num2str(uu(4)*180/pi) ' deg'])
    disp(['alpha  = ' num2str(xu(3)*180/pi) ' deg'])
    disp(['dLEF   = ' num2str(uu(5)*180/pi) ' deg'])
    disp(['Vel.   = ' num2str(xu(1)) ' m/s']) 
    disp(['pow    = ' num2str(xu(13)) ' %']) 

    flag = input('Continue trim rountine iterations? (y/n):  ','s'); 
    if flag == 'n'
        iter = 0;
    end
    UX0 = UX;
  
end

% 它并不会关闭模型或卸载模型本身
% 只是退出 lincompile 状态，让模型回到普通的未编译待仿真状态
 feval('F16_trim', [], [], [], 'term'); % 终止这个编译模式，释放编译占用的临时资源。
% 该命令会关闭模型窗口，并从内存中卸载模型 F16_trim
% 释放所有与模型相关的资源（模块、信号、缓存等）
% 如果后续不再需要该模型，调用 close_system 是清理内存的好习惯
% close_system('F16_trim');

% trim_fun 输入UX0和输出uu不要搞混
trim_state=xu(1:13);
trim_thrust=uu(1);
trim_control=[uu(2);uu(3);uu(4)];
dLEF = uu(5);

