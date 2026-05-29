%% f16_longitudinal_model.m — F-16 纵向线性模型加载函数
%==========================================================================
%  功能:
%    从 lab1_matrices.mat 中加载 F-16 纵向线性模型数据,
%    包括 A(5×5) 状态矩阵, B(5×2) 控制矩阵, 配平状态/控制,
%    以及前缘襟翼偏角, HIFI 标志等。
%    同时输出模型的物理含义: 配平状态值, 关键气动导数等。
%
%  线性模型来源:
%    lab1_step1_trim_and_linearize.m 在 H=5000m, V=200m/s 配平点
%    对 F-16 六自由度非线性模型做小扰动线性化得到。
%
%  纵向状态 (5 维):
%    x_long = [V(速度), α(迎角), θ(俯仰角), q(俯仰速率), Pow(发动机功率)]'
%
%  纵向控制 (2 维):
%    u_long = [δth(油门), δe(升降舵)]'
%
%  A 矩阵的物理含义 (以重力/气动/推力项的耦合关系):
%    A(1,:) = X_*: 速度导数的各项系数 (阻力/推力方向)
%    A(2,:) = Z_*: 迎角导数的各项系数 (升力方向)
%    A(3,:):  θ 导数为 0/0/0/1/0, 即 θ = ∫q dt (纯运动学)
%    A(4,:) = M_*: 俯仰力矩导数的各项系数 (纵轴力矩)
%    A(5,:):  发动机功率动态 (一阶滞后)
%
%  B 矩阵的物理含义:
%    B(:,1): δth(油门) → 直接影响 V(推力) 和 Pow(油门响应)
%    B(:,2): δe(升降舵) → 直接影响 M(俯仰力矩), 间接影响 α
%
%  输入参数:
%    model_type — 字符串:
%      'LOFI'(默认) — 低保真模型: 硬编码气动系数数组, 查表带钳位
%      'HIFI'       — 高保真模型: .dat 风洞数据 + getHyperCube 多维插值
%
%  输出参数:
%    A       — 5×5 状态矩阵 (连续时间, s^-1)
%    B       — 5×2 控制矩阵 (连续时间)
%    x_trim  — 13×1 配平状态向量 (全状态)
%    u_trim  — 4×1 配平控制向量 [δth; δe; δa; δr]
%    dlef    — 配平点前缘襟翼偏转角 (标量, 弧度)
%    fi      — HIFI 模型标志: 0=LOFI, 1=HIFI
%    label   — 模型名称字符串 (用于显示)
%
%  用法:
%    [A, B, x_trim, u_trim, dlef, fi, label] = f16_longitudinal_model('LOFI');
%    eig(A) 查看开环极点, 判断稳定性
%
%  参考:
%    f16_controller_design.m — 使用本函数的输出设计控制器
%    f16_stability_analysis.m — 使用本函数的 A 矩阵分析稳定性
%    lab1_step1_trim_and_linearize.m — 生成 lab1_matrices.mat 的源脚本
%==========================================================================

function [A, B, x_trim, u_trim, dlef, fi, label] = f16_longitudinal_model(model_type)
% 输入参数处理: 默认使用 LOFI 模型
if nargin<1, model_type='LOFI'; end

% --- 路径设置 ---
% fileparts(mfilename('fullpath')): 获取本 .m 文件所在目录
this_dir = fileparts(mfilename('fullpath'));
% 上两级目录: FC_SimCode_1/ (项目根)
proj_root = fileparts(fileparts(this_dir));
% 添加必要的路径到 MATLAB 搜索路径
addpath(proj_root);  % 项目根 → F16_dyn.mexw64, aerodata/
addpath(fullfile(proj_root,'trae','lab1_0429'));  % lab1 数据
addpath(this_dir);  % 本目录
cd(proj_root);  % 必须 cd 到根目录, HIFI .dat 路径基于当前目录

% lab1_matrices.mat 的完整路径
mat = fullfile(proj_root,'trae','lab1_0429','lab1_matrices.mat');

% --- 加载模型数据 ---
switch upper(model_type)
    case 'LOFI'
        % LOFI: 从 .mat 文件加载低保真模型
        % A_longitude_lo(5×5):  低保真状态矩阵
        % B_longitude_lo(5×2):  低保真控制矩阵
        % trim_state_lo(13×1):  配平状态 (LOFI)
        % trim_thrust_lo(1×1):  配平推力 (油门标量)
        % trim_control_lo(3×1): 配平舵面 [δe; δa; δr]
        % dLEF_lo(1×1):         配平前缘襟翼偏角
        S=load(mat,'A_longitude_lo','B_longitude_lo','trim_state_lo',...
                 'trim_thrust_lo','trim_control_lo','dLEF_lo');
        A=S.A_longitude_lo; B=S.B_longitude_lo;
        x_trim=S.trim_state_lo;
        % 组合 4 维控制向量: [δth; δe; δa; δr]
        u_trim=[S.trim_thrust_lo;S.trim_control_lo];
        dlef=S.dLEF_lo;
        fi=0;  % LOFI 标志
        label='LOFI (低 fidelity, 硬编码气动系数)';

    case 'HIFI'
        % HIFI: 从 .mat 文件加载高保真模型
        % 气动系数来自 .dat 风洞数据, 通过 getHyperCube 多维插值
        S=load(mat,'A_longitude_hi','B_longitude_hi','trim_state_hi',...
                 'trim_thrust_hi','trim_control_hi','dLEF_hi');
        A=S.A_longitude_hi; B=S.B_longitude_hi;
        x_trim=S.trim_state_hi;
        u_trim=[S.trim_thrust_hi;S.trim_control_hi];
        dlef=S.dLEF_hi;
        fi=1;  % HIFI 标志
        label='HIFI (高 fidelity, .dat 风洞插值)';
end

% --- 提取纵向状态子集 ---
% 全状态 13 维 → 纵向 5 维
% 索引: [1=V, 3=α, 5=θ, 8=q, 13=Pow]
idx=[1,3,5,8,13]; xl=x_trim(idx);

% --- 打印模型信息 (方便用户理解) ---
fprintf('\n============================================================\n');
fprintf('  F-16 纵向线性模型 — %s\n', label);
fprintf('  飞行条件: H=5000m, V=200m/s\n');
fprintf('============================================================\n');

% 配平状态值 (带单位转换: 弧度→度)
fprintf('\n  【配平状态】 V=%.1fm/s  α=%.2f°  θ=%.2f°  q=%.4f  Pow=%.1f%%\n',...
    xl(1),xl(2)*180/pi,xl(3)*180/pi,xl(4),xl(5));
% 配平控制值 (带单位转换)
fprintf('  【配平控制】 油门=%.4f  δe=%.4frad(%.2f°)  δa=%.4f  δr=%.4f  dLEF=%.4f\n',...
    u_trim(1),u_trim(2),u_trim(2)*180/pi,u_trim(3),u_trim(4),dlef);

% A 矩阵 (逐行打印)
% 行含义: 1=V̇, 2=α̇, 3=θ̇, 4=q̇, 5=Ṗow
fprintf('\n  A 矩阵 (5×5, 行=V̇ α̇ θ̇ q̇ Ṗow):\n');
for i=1:5, fprintf('  %+.4f',A(i,:)); fprintf('\n'); end

% 关键气动导数 (影响飞行品质的最主要参数)
% X_α = A(1,2): 迎角变化对速度的影响 (阻力随迎角变化)
% M_α = A(4,2): 迎角变化对俯仰力矩的影响 (纵向静稳定性)
%               负值 → 静稳定 (迎角增加→低头力矩→恢复)
% M_q = A(4,4): 俯仰阻尼导数 (抵抗俯仰速率)
%               负值 → 正阻尼 (防止俯仰振荡发散)
% M_δe = B(4,2): 升降舵效率 (舵面偏转产生的俯仰力矩)
fprintf('\n  关键导数: X_α=%.2f  M_α=%.2f  M_q=%.2f  M_δe=%.2f\n',...
    A(1,2),A(4,2),A(4,4),B(4,2));

% B 矩阵 (逐行打印)
% 列1: δth 对各状态的影响
% 列2: δe 对各状态的影响
fprintf('  B 矩阵 (5×2, 列=δth δe):\n');
for i=1:5, fprintf('  %+.4f  %+.4f\n',B(i,1),B(i,2)); end
fprintf('\n');
end
