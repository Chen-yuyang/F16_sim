function f16_build_lateral_sim(model_name)
% 构建横侧向闭环 Simulink 模型 (纵向冻结)
%  控制律: u_lat = u_lat_trim + fb_gain * K_lat * (x_ref_lat - x_lat)
%  纵向: δth=trim, δe=trim (冻结, 无反馈)
if nargin<1, model_name='F16_Lateral_CL'; end

this_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(fileparts(this_dir));
out_dir  = fullfile(this_dir,'test_models');
if ~exist(out_dir,'dir'), mkdir(out_dir); end
addpath(proj_root); addpath(fullfile(proj_root,'aerodata')); addpath(this_dir); cd(proj_root);

vars={'init_x','init_u_base','init_dlef','fi_flag_Simulink','K_lat','x_ref_lat','fb_gain'};
for i=1:numel(vars)
    if ~evalin('base',sprintf('exist(''%s'',''var'')',vars{i}))
        error('缺少变量: %s',vars{i});
    end
end
if ~evalin('base','exist(''elevator_disturb'',''var'')')
    assignin('base','elevator_disturb',[0 30;0 0]');
end

if bdIsLoaded(model_name), close_system(model_name,1); end
for ext={'.slx','.slxc'}
    f=fullfile(out_dir,[model_name ext{1}]); if exist(f,'file'), delete(f); end
end
new_system(model_name,'Model');
set_param(model_name,'Solver','ode4','FixedStep','0.01','StopTime','30');
set_param(model_name,'SaveState','on','SaveOutput','on','SaveTime','on','ReturnWorkspaceOutputs','on');

%% 布局
Y=35; DY=50;

% 纵向源 (冻结)
add_block('simulink/Sources/Constant',[model_name '/thrust_trim'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/thrust_trim'],'Value','init_u_base(1)','SampleTime','inf'); Y=Y+DY;
add_block('simulink/Sources/Constant',[model_name '/elev_trim'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/elev_trim'],'Value','init_u_base(2)','SampleTime','inf'); Y=Y+DY;

% 横侧向源 (反馈修正)
add_block('simulink/Math Operations/Sum',[model_name '/sum_ail'],'Position',[250,Y-50,280,Y-20]);
set_param([model_name '/sum_ail'],'Inputs','++','IconShape','round');
add_block('simulink/Sources/Constant',[model_name '/ail_base'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/ail_base'],'Value','init_u_base(3)','SampleTime','inf'); Y=Y+DY;

add_block('simulink/Math Operations/Sum',[model_name '/sum_rud'],'Position',[250,Y-50,280,Y-20]);
set_param([model_name '/sum_rud'],'Inputs','++','IconShape','round');
add_block('simulink/Sources/Constant',[model_name '/rud_base'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/rud_base'],'Value','init_u_base(4)','SampleTime','inf'); Y=Y+DY+20;

add_block('simulink/Sources/Constant',[model_name '/dlef'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/dlef'],'Value','init_dlef','SampleTime','inf'); Y=Y+DY;
add_block('simulink/Sources/Constant',[model_name '/fi_flag'],'Position',[50,Y,160,Y+30]);
set_param([model_name '/fi_flag'],'Value','fi_flag_Simulink','SampleTime','inf'); Y=Y+DY+20;

% 横侧向参考
add_block('simulink/Sources/Constant',[model_name '/x_ref_lat'],'Position',[50,Y,160,Y+40]);
set_param([model_name '/x_ref_lat'],'Value','x_ref_lat','SampleTime','inf');
add_block('simulink/Sources/Constant',[model_name '/fb_gain'],'Position',[50,Y+60,160,Y+90]);
set_param([model_name '/fb_gain'],'Value','fb_gain','SampleTime','inf');

% Mux + F16
add_block('simulink/Signal Routing/Mux',[model_name '/ctrl_mux'],'Position',[380,20,420,160]);
set_param([model_name '/ctrl_mux'],'Inputs','4','DisplayOption','bar');
add_block('simulink/User-Defined Functions/S-Function',[model_name '/F16_dyn'],'Position',[520,20,640,200]);
set_param([model_name '/F16_dyn'],'FunctionName','F16_dyn','Parameters','init_x');

% 控制器
add_block('simulink/Signal Routing/Selector',[model_name '/state_sel'],'Position',[720,40,790,110]);
set_param([model_name '/state_sel'],'InputPortWidth','13','Indices','[2,4,6,7,9]','IndexOptions','Index vector (dialog)');
add_block('simulink/Math Operations/Sum',[model_name '/err_sum'],'Position',[840,50,870,110]);
set_param([model_name '/err_sum'],'Inputs','+-','IconShape','round');
add_block('simulink/Math Operations/Gain',[model_name '/K_gain'],'Position',[920,50,970,110]);
set_param([model_name '/K_gain'],'Gain','K_lat','Multiplication','Matrix(K*u)');
add_block('simulink/Math Operations/Product',[model_name '/fb_enable'],'Position',[1000,60,1030,100]);
set_param([model_name '/fb_enable'],'Inputs','**','Multiplication','Element-wise(.*)');
add_block('simulink/Signal Routing/Demux',[model_name '/fb_demux'],'Position',[1070,60,1100,110]);
set_param([model_name '/fb_demux'],'Outputs','2','DisplayOption','bar');

% 输出
add_block('simulink/Sinks/Out1',[model_name '/states_out'],'Position',[720,280,760,315]);
add_block('simulink/Signal Routing/Demux',[model_name '/scope_demux'],'Position',[830,280,860,490]);
set_param([model_name '/scope_demux'],'Outputs','13','DisplayOption','bar');
scopes={'beta','phi','psi','p','r'}; SY=280;
for i=1:5
    add_block('simulink/Sinks/Scope',[model_name '/Scope_' scopes{i}],'Position',[900,SY,950,SY+30]);
    set_param([model_name '/Scope_' scopes{i}],'OpenAtSimulationStart','off'); SY=SY+40;
end

%% 连线
add_line(model_name,'thrust_trim/1','ctrl_mux/1');
add_line(model_name,'elev_trim/1','ctrl_mux/2');
add_line(model_name,'ail_base/1','sum_ail/1');
add_line(model_name,'sum_ail/1','ctrl_mux/3');
add_line(model_name,'rud_base/1','sum_rud/1');
add_line(model_name,'sum_rud/1','ctrl_mux/4');
add_line(model_name,'ctrl_mux/1','F16_dyn/1');
add_line(model_name,'dlef/1','F16_dyn/2');
add_line(model_name,'fi_flag/1','F16_dyn/3');
add_line(model_name,'F16_dyn/1','state_sel/1');
add_line(model_name,'F16_dyn/1','states_out/1');
add_line(model_name,'F16_dyn/1','scope_demux/1');
add_line(model_name,'x_ref_lat/1','err_sum/1');
add_line(model_name,'state_sel/1','err_sum/2');
add_line(model_name,'err_sum/1','K_gain/1');
add_line(model_name,'K_gain/1','fb_enable/1');
add_line(model_name,'fb_gain/1','fb_enable/2');
add_line(model_name,'fb_enable/1','fb_demux/1');
add_line(model_name,'fb_demux/1','sum_ail/2');   % 反馈→副翼
add_line(model_name,'fb_demux/2','sum_rud/2');   % 反馈→方向舵
add_line(model_name,'scope_demux/2','Scope_beta/1');
add_line(model_name,'scope_demux/4','Scope_phi/1');
add_line(model_name,'scope_demux/6','Scope_psi/1');
add_line(model_name,'scope_demux/7','Scope_p/1');
add_line(model_name,'scope_demux/9','Scope_r/1');

try Simulink.BlockDiagram.arrangeSystem(model_name,'FullLayout','true'); catch; end
save_system(model_name,fullfile(out_dir,[model_name '.slx']));
fprintf('  横侧向模型已保存: test_models/%s.slx\n',model_name);
end
