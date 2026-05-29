%% f16_lateral_demo.m — F-16 横侧向闭环控制 完整演示
%  纵向冻结: δth/δe 保持配平值
%  横侧向反馈: β,φ,ψ,p,r → δa,δr
%  目标: φ 跟踪 5°
clear; clc;

%% 配置
MODEL='LOFI'; METHOD='place';  % place/lqr/manual/pi

this_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(fileparts(this_dir));
addpath(this_dir); addpath(proj_root);
addpath(fullfile(proj_root,'aerodata')); addpath(fullfile(proj_root,'trae','lab1_0429'));
cd(proj_root);

fprintf('========================================\n');
fprintf('  F-16 横侧向闭环控制 — 完整演示\n');
fprintf('  模型: %s | 方法: %s\n',MODEL,METHOD);
fprintf('========================================\n\n');

%% 步骤1: 加载横侧向模型
fprintf('=== 步骤1: 加载横侧向线性模型 ===');
[A,B,x_trim,u_trim,dlef,fi,label] = f16_lateral_model(MODEL);

%% 步骤2: 设计横侧向控制器
fprintf('=== 步骤2: 设计横侧向控制器 ===');
r = f16_lateral_controller(MODEL, METHOD);

%% 步骤3: 构建 Simulink 模型
fprintf('=== 步骤3: 构建横侧向 Simulink 模型 ===\n');
f16_build_lateral_sim('LatTest');

%% 步骤4: 开环仿真
fprintf('\n=== 步骤4: 开环仿真 (fb_gain=0) ===\n');
assignin('base','fb_gain',0);
try
    ol=sim('LatTest','StopTime','30');
    dol=ol.yout{1}.Values.Data;
    fprintf('  ✅ OL: φ=%.2f→%.2f° β=%.4f→%.4f°\n',...
        dol(1,4)*180/pi,dol(end,4)*180/pi,dol(1,2)*180/pi,dol(end,2)*180/pi);
catch ME, fprintf('  ❌ OL失败: %s\n',ME.message); end

%% 步骤5: 闭环仿真
fprintf('\n=== 步骤5: 闭环仿真 (fb_gain=1) ===\n');
assignin('base','fb_gain',1);
f16_build_lateral_sim('LatTest');
try
    cl=sim('LatTest','StopTime','30');
    dcl=cl.yout{1}.Values.Data;
    ss=cl.tout>=10;
    phi_ss=mean(dcl(ss,4))*180/pi;
    fprintf('  ✅ CL: φ=%.2f→%.2f° ss=%.2f° (目标5°) err=%.2f°\n',...
        dcl(1,4)*180/pi,dcl(end,4)*180/pi,phi_ss,5.0-phi_ss);
    if abs(5.0-phi_ss)<2.0, fprintf('  ✅ 跟踪良好!\n');
    else fprintf('  ⚠ 跟踪偏差\n'); end
catch ME, fprintf('  ❌ CL失败: %s\n',ME.message); end
close_system('LatTest',0);

%% 步骤6: 绘图
fprintf('\n=== 步骤6: 绘图 ===\n');
if ~exist('dol','var') && ~exist('dcl','var'), return; end

figure('Name','F-16 Lateral Control','Position',[30,30,1200,800]);
cfg={1,2,180/pi,'β (°)','侧滑角'; 2,4,180/pi,'φ (°)','滚转角';
     3,6,180/pi,'ψ (°)','偏航角'; 4,7,180/pi,'p (°/s)','滚转速率';
     5,9,180/pi,'r (°/s)','偏航速率'};
for i=1:5
    subplot(2,3,i); hold on; c=cfg(i,:); idx=c{2}; sc=c{3};
    if exist('dol','var'), plot(cl.tout,sc*dol(:,idx),'b-','LineWidth',1.2,'DisplayName','开环'); end
    if exist('dcl','var'), plot(cl.tout,sc*dcl(:,idx),'r-','LineWidth',1.2,'DisplayName','闭环'); end
    if idx==4, yline(5,'g--','LineWidth',1.5,'DisplayName','φ_{ref}=5°'); end
    xlabel('时间(s)'); ylabel(c{4}); title(c{5}); grid on; legend('Location','best');
end
subplot(2,3,6);
text(0.1,0.6,sprintf(['横侧向控制: δa,δr←K_{lat}·(x_{ref}-x)\\newline'...
    '模型:%s | 方法:%s\\newline纵向:δth/δe=trim(冻结)\\newline'...
    '目标:φ→5° β→0°\\newlineK(1,2)=%.2f(φ→δa) K(2,1)=%.2f(β→δr)'],...
    MODEL,METHOD,r.K(1,2),r.K(2,1)),'FontSize',9,'Units','normalized');
axis off;
sgtitle(sprintf('F-16 横侧向控制 (%s+%s, 纵向冻结)',MODEL,METHOD));
fprintf('\n========================================\n  演示完成!\n========================================\n');
