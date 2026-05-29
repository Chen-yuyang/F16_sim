function result = f16_lateral_controller(model_type, method, varargin)
% 横侧向控制器设计: [β,φ,ψ,p,r] → [δa,δr]
% 方法: 'place'/'lqr'/'manual'/'place_i'/'lqr_i'
% 目标: φ跟踪参考, β→0 (协调转弯)
if nargin<1, model_type='LOFI'; end
if nargin<2, method='place'; end
manual_K=[]; use_exact=false;
for i=1:length(varargin)
    if isnumeric(varargin{i}) && isequal(size(varargin{i}),[2,5]), manual_K=varargin{i}; end
    if strcmpi(varargin{i},'exact'), use_exact=true; end
end

this_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(fileparts(this_dir));
addpath(proj_root); addpath(fullfile(proj_root,'trae','lab1_0429')); addpath(this_dir);
cd(proj_root);

mat = fullfile(proj_root,'trae','lab1_0429','lab1_matrices.mat');
switch upper(model_type)
    case 'LOFI'
        S=load(mat,'A_lateral_lo','B_lateral_lo','trim_state_lo','trim_thrust_lo','trim_control_lo','dLEF_lo');
        A=S.A_lateral_lo; B=S.B_lateral_lo; x_trim=S.trim_state_lo;
        u_trim=[S.trim_thrust_lo;S.trim_control_lo]; dlef=S.dLEF_lo; fi=0;
    case 'HIFI'
        S=load(mat,'A_lateral_hi','B_lateral_hi','trim_state_hi','trim_thrust_hi','trim_control_hi','dLEF_hi');
        A=S.A_lateral_hi; B=S.B_lateral_hi; x_trim=S.trim_state_hi;
        u_trim=[S.trim_thrust_hi;S.trim_control_hi]; dlef=S.dLEF_hi; fi=1;
end

idx_lat=[2,4,6,7,9];
xl_lat = x_trim(idx_lat);  % [β,φ,ψ,p,r]

% 参考: φ=5° (小滚转角测试), β=0 (协调), ψ=free
phi_ref = 5*pi/180;
x_ref_lat = xl_lat; x_ref_lat(2)=phi_ref; x_ref_lat(1)=0;  % φ=5°, β=0

% 设计 K (2×5)
Ki_lat = [0;0];
switch lower(method)
    case 'place'
        p_des=[-2.5+2.5i -2.5-2.5i -3.0 -0.5 -0.5];  % 荷兰滚加快, 螺旋/滚转稳定
        K=place(A,B,p_des);
    case 'lqr'
        Q=diag([10,100,10,1,1]); R=diag([0.5,0.5]);  % 重点φ,β
        K=lqr(A,B,Q,R);
    case {'place_i','lqr_i'}
        C_phi=[0,1,0,0,0];  % 输出φ用于积分
        A_aug=[A, zeros(5,1); -C_phi, 0]; B_aug=[B; 0, 0];
        if strcmpi(method,'place_i')
            K_aug=place(A_aug,B_aug,[p_des,-1.5]);
        else
            K_aug=lqr(A_aug,B_aug,diag([10,100,10,1,1,100]),diag([0.5,0.5]));
        end
        K=K_aug(:,1:5); Ki_lat=K_aug(:,6);
    case 'manual'
        if isempty(manual_K), error('manual需提供K(2×5)'); end
        K=manual_K;
    case 'pi'
        % 简化为PI: 仅φ反馈到副翼, β抑制到方向舵
        K = [0, 0.3, 0, 0, 0;   % δa = 0.3·(φ_ref-φ) + ...
             0.1, 0, 0, 0, 0.1]; % δr = 0.1·β + 0.1·r
        Ki_lat = [0; 0.05];       % δr积分
end

eig_cl=eig(A-B*K); stable=all(real(eig_cl)<0);

% 写入workspace
assignin('base','init_x',x_trim);
assignin('base','init_u_base',u_trim);
assignin('base','init_dlef',dlef);
assignin('base','fi_flag_Simulink',fi);
assignin('base','K_lat',K);
assignin('base','Ki_lat',Ki_lat);
assignin('base','x_ref_lat',x_ref_lat);
assignin('base','fb_gain',0);
if ~evalin('base','exist(''elevator_disturb'',''var'')')
    assignin('base','elevator_disturb',[0 30;0 0]');
end

fprintf('\n============================================================\n');
fprintf('  横侧向控制器设计 — %s | %s\n',model_type,upper(method));
fprintf('  开环极点: '); fprintf('%.2f%+.2fi ',real(eig(A)),imag(eig(A))); fprintf('\n');
fprintf('  K(1,2)=%.3f (φ→δa) | K(2,1)=%.3f (β→δr)\n',K(1,2),K(2,1));
fprintf('  闭环稳定=%d | φ_ref=%.0f° | β_ref=0°\n',stable,phi_ref*180/pi);
fprintf('============================================================\n\n');

result=struct('A',A,'B',B,'K',K,'Ki_lat',Ki_lat,'x_ref_lat',x_ref_lat,...
    'x_trim',x_trim,'u_trim',u_trim,'dlef',dlef,'fi',fi,'model',model_type,...
    'method',method,'stable',stable,'eig_cl',eig_cl,'phi_ref',phi_ref);
end
