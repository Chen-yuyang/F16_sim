function [A_lat, B_lat, x_trim, u_trim, dlef, fi, label] = f16_lateral_model(model_type)
% 加载 F-16 横侧向线性模型 (LOFI/HIFI)
% 状态: [β, φ, ψ, p, r]  控制: [δa, δr]
if nargin<1, model_type='LOFI'; end
this_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(fileparts(this_dir));
addpath(proj_root); addpath(fullfile(proj_root,'trae','lab1_0429')); addpath(this_dir);
cd(proj_root);
mat = fullfile(proj_root,'trae','lab1_0429','lab1_matrices.mat');
switch upper(model_type)
    case 'LOFI'
        S=load(mat,'A_lateral_lo','B_lateral_lo','trim_state_lo','trim_thrust_lo','trim_control_lo','dLEF_lo');
        A_lat=S.A_lateral_lo; B_lat=S.B_lateral_lo; x_trim=S.trim_state_lo;
        u_trim=[S.trim_thrust_lo;S.trim_control_lo]; dlef=S.dLEF_lo; fi=0; label='LOFI';
    case 'HIFI'
        S=load(mat,'A_lateral_hi','B_lateral_hi','trim_state_hi','trim_thrust_hi','trim_control_hi','dLEF_hi');
        A_lat=S.A_lateral_hi; B_lat=S.B_lateral_hi; x_trim=S.trim_state_hi;
        u_trim=[S.trim_thrust_hi;S.trim_control_hi]; dlef=S.dLEF_hi; fi=1; label='HIFI';
end
idx=[2,4,6,7,9]; xl=x_trim(idx);
fprintf('\n============================================================\n');
fprintf('  F-16 横侧向线性模型 — %s\n',label);
fprintf('============================================================\n');
fprintf('  状态: β=%.4f° φ=%.2f° ψ=%.2f° p=%.4f r=%.4f\n',...
    xl(1)*180/pi,xl(2)*180/pi,xl(3)*180/pi,xl(4),xl(5));
fprintf('  控制: δa=%.4f δr=%.4f\n',u_trim(3),u_trim(4));
fprintf('\n  A_lat (5×5, [β φ ψ p r]):\n');
for i=1:5, fprintf('  %+.4f',A_lat(i,:)); fprintf('\n'); end
fprintf('  B_lat (5×2, [δa δr]):\n');
for i=1:5, fprintf('  %+.4f  %+.4f\n',B_lat(i,1),B_lat(i,2)); end
fprintf('  特征值: '); fprintf('%.2f%+.2fi ',real(eig(A_lat)),imag(eig(A_lat))); fprintf('\n\n');
end
