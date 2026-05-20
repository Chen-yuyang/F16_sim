%=====================================================
%     Matlab Script File used to run the 
%     non-linear F-16 Simulation. The results
%     will also be plotted.
%
%   Author: Richard S. Russell
%
%  File made suitable for F-16 S-function dynamics.
%  - The simulink file "F16_trim.mdl" is used together
%    with the m-files "trim_fun.m" and "tgear.m"
%    to trim the aircraft.
%  - The simulink file "F16_openloop" is used to 
%    simulate the open loop response of the F-16 model.
%
%  L. Sonneveldt May 2006
%
%=====================================================

close all;
clear all;
clc;

global altitude velocity fi_flag_Simulink

%% Ask user which simulation to run.
%%
newline = sprintf('\n');
disp('Which model would you like to use to trim the aircraft:')
disp('  1. Low Fidelity F-16 Trim')
disp('  2. High Fidelity F-16 Trim')
fi_flag = input('Your Selection:  ');
disp(newline);
disp(newline);

%% Determine from flag the correct simulation.
%%
if fi_flag == 1;
  fi_type = 'lofi';
  fi_flag_Simulink = 0;
elseif fi_flag == 2;
  fi_type = 'hifi';
  fi_flag_Simulink = 1;
else
  disp('Invalid selection');
 % break;
end

%% Trim aircraft to desired altitude and velocity
%%
altitude = input('Enter the altitude for the simulation (m)  :  ');
velocity = input('Enter the velocity for the simulation (m/s):  ');

%% Initial Conditions for trim routine.
%% The following values seem to trim to most
%% flight condition.  If the F16 does not trim
%% Change these values.
beta = 0;                  % 侧滑角β (rad) - 假设无侧风，完美直飞
elevator = -2*pi/180;       % 升降舵偏角, elevator, rad
alpha = 10*pi/180;         % 攻角, α, AOA, rad
rudder = 0;                % 方向舵, rudder angle, rad
aileron = 0;               % 副翼, aileron, rad
dth = 0.2;                 % 油门杆位置 (0 到 1 之间)，0.2 代表 20% 油门

% Initial Guess for free parameters
% 将所有待优化参数堆成一个列向量 UX0，作为 fminsearch 的起点。
% 向量顺序：侧滑角、升降舵、攻角、副翼、方向舵、油门。
UX0 = [beta; elevator; alpha; aileron; rudder; dth];

% Initializing optimization options and running optimization:
% 这是配置 MATLAB 内置函数 fminsearch 的参数。
% TolFun（目标函数值的终止容差） 和 TolX（自变量变化的终止容差）是收敛精度 
% (1e-10 代表极高的精度，小数点后10位不变化才算找到)。
% MaxFunEvals 和 MaxIter 是防止死循环的，最多计算 50000 次，最多迭代 10000 次。
OPTIONS = optimset('TolFun',1e-10,'TolX',1e-10,'MaxFunEvals',5e+04,'MaxIter',1e+04);

iter = 1;
while iter == 1
    
    % lincompile 是一种“预编译”命令。
    % 以 'lincompile' 为参数调用 F16_trim 函数
    % 该命令会编译 Simulink 模型中的线性化端口，使后续反复调用模型计算导数时速度更快
    % 配平过程中，fminsearch 会成千上万次地调用模型计算状态导数。
    % 如果每次调用都重新打开、初始化 Simulink 模型，速度会慢得令人发指。
    % lincompile 把模型冻结在内存中，准备好进行高速的数学计算而不推进仿真时间。
    feval('F16_trim', [], [], [], 'lincompile');
    load_system('F16_trim'); % 显式加载Simulink模型F16_trim，以防未被加载。
   
    % 使用 Nelder-Mead 单纯形算法（fminsearch）寻找一组自由参数 UX，使代价函数 trim_fun(UX) 达到最小。
    % FVAL: 在UX处计算得到的代价函数值，即trim_fun(UX)的返回值。理想配平时该值应接近0。
    % EXITFLAG: 优化退出标志，指示算法终止的原因：
    % – 1：函数收敛于解（满足 TolFun 或 TolX）
    % – 0：达到最大迭代次数或函数计算次数
    % – -1：算法被输出函数停止
    % 配平成功通常希望为 1。
    % OUTPUT: 结构体，包含优化过程的详细信息：
    % OUTPUT.iterations – 迭代次数
    % OUTPUT.funcCount – 函数计算次数
    % OUTPUT.algorithm – 使用的算法（'Nelder-Mead simplex direct search'）
    % OUTPUT.message – 退出信息文本
    % 可用于诊断优化是否顺利。
    % 优化器只使用它的第一个返回值作为要最小化的目标，其余返回值一律被忽略
    [UX,FVAL,EXITFLAG,OUTPUT] = fminsearch('trim_fun',UX0,OPTIONS);
   
    % 拿到最优解 UX 后，再放进 trim_fun 跑最后一次，
    % 把最终的代价(cost, 应与FVAL相同)、状态导数(Xdot，应该全是0)、状态变量(xu)和控制变量(uu)提取出来。
    % trim_fun 是纯确定性的——给定相同的 UX，永远返回相同的输出
    [cost, Xdot, xu, uu] = trim_fun(UX);
    
    % 在屏幕上打印出找到的平衡点数据
    disp('Trim Values and Cost:');
    disp(['cost   = ' num2str(cost)])                    % 越接近0越好
    disp(['dth    = ' num2str(uu(1)) ' -'])              % 油门
    disp(['elev   = ' num2str(uu(2)*180/pi) ' deg'])     % 升降舵
    disp(['ail    = ' num2str(uu(3)*180/pi) ' deg'])     % 副翼
    disp(['rud    = ' num2str(uu(4)*180/pi) ' deg'])     % 方向舵
    disp(['alpha  = ' num2str(xu(3)*180/pi) ' deg'])     % 攻角
    disp(['dLEF   = ' num2str(uu(5)*180/pi) ' deg'])     % 前缘襟翼角度（高保真模型自动计算）
    disp(['Vel.   = ' num2str(xu(1)) ' m/s'])            % 实际速度
    disp(['pow    = ' num2str(xu(13)) ' %'])             % 发动机实际功率百分比

    flag = input('Continue trim rountine iterations? (y/n):  ','s'); 
    if flag == 'n'
        iter = 0;
    end

    % 解除 Simulink 模型的编译锁定状态释放内存
    feval('F16_trim', [], [], [], 'term');
    % 将刚才找到的最佳值作为下一次循环的起点 (如果用户选择 y)
    UX0 = UX;
end

% For simulink:
% 将配平得到的初始状态、控制量提取出来，供后续 Simulink 仿真（如 F16_openloop）使用。
init_x = xu(1:13);
init_u = uu(1:4);
init_dlef = uu(5);
