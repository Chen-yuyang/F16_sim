%% run_tests.m — 含伺服作动器的全组合测试脚本
%==========================================================================
%  功能: 自动遍历 LOFI/HIFI × place/lqr × 旧方案/新方案 共 4 种组合
%        每种组合运行开环+闭环仿真, 记录稳态误差, 汇总结果
%  用途:
%    1. 回归测试: 修改代码后一键验证所有组合是否仍可工作
%    2. 方案对比: 快速比较不同方案 (模型/方法/基线) 的跟踪性能
%    3. 参数筛选: 为后续调试提供基准数据
%  用法:
%    >> run_tests          % 运行全部 4 组测试
%  前置条件:
%    需先运行 design_controller 和 build_model 生成 .slx 文件
%    但本脚本会自动调用它们 (不依赖手动步骤)
%  输出:
%    results.mat — 保存所有测试结果的结构体
%==========================================================================

clear; clc;  % 清空工作空间和命令窗口

% --- 路径设置 ---
% fileparts(mfilename('fullpath')) 获取当前 .m 文件所在目录
this_dir = fileparts(mfilename('fullpath'));
% 上两级目录 = FC_SimCode_1/ (项目根目录)
proj_root = fileparts(fileparts(fileparts(this_dir)));
addpath(genpath(proj_root));  % genpath 递归添加所有子目录到搜索路径
cd(proj_root);  % 切换工作目录到项目根 (HIFI .dat 文件路径基于当前目录)

% --- 测试用例定义 ---
% 每行: {模型类型, 控制方法, 是否使用精确平衡解, 测试标签}
% 一共 4 种组合, 覆盖 LOFI/HIFI × place/lqr × old/new
tests = {
    'LOFI','place',false,'T1_LP_old';   % T1: LOFI + place + 旧方案 (简单参考)
    'LOFI','place',true, 'T2_LP_new';   % T2: LOFI + place + 新方案 (精确平衡)
    'LOFI','lqr',  false,'T3_LL_old';   % T3: LOFI + lqr + 旧方案
    'HIFI','place',false,'T4_HP_old';   % T4: HIFI + place + 旧方案 ★关键
};  % T4 是关键测试: HIFI 模型带伺服作动器能否稳定运行?

% 存储结果的 cell 数组 (每个元素是一个结构体)
results = cell(size(tests,1),1);
fprintf('===== 含伺服作动器的闭环测试 =====\n\n');

% --- 主循环: 遍历所有测试用例 ---
for t = 1:size(tests,1)
    % 提取当前测试用例的参数字段
    mdl_type = tests{t,1};  % 模型类型: 'LOFI' 或 'HIFI'
    method = tests{t,2};     % 控制方法: 'place' 或 'lqr'
    use_exact = tests{t,3};  % 是否使用精确平衡解: true/false
    mdl = tests{t,4};        % Simulink 模型名称 (标签)

    fprintf('[%d/4] %s-%s... ',t,mdl_type,upper(method));

    try
        % === 步骤1: 控制器设计 ===
        % design_controller 执行:
        %   1. 加载模型 (A, B)
        %   2. 计算 K (place/lqr)
        %   3. 写入 base workspace (init_x, K_long, x_ref_long 等)
        r = design_controller(mdl_type, method, use_exact);

        % === 步骤2: 构建 Simulink 模型 ===
        % build_model 使用 add_block/add_line 构建 .slx 文件
        % 保存在 test_models/<mdl>.slx
        build_model(mdl);

        % === 步骤3: 开环仿真 (fb_gain=0) ===
        % 开环: 控制器输出被 Product 模块屏蔽, 飞机按配平控制飞行
        % 用于对比: 开环时飞机应保持配平状态 (θ≈2°)
        assignin('base','fb_gain',0);  % 反馈增益设为 0 (开环)
        ol = sim(mdl,'StopTime','30');  % 运行仿真 30 秒
        ol_data = ol.yout{1}.Values.Data;  % 提取仿真数据 (时间序列 × 13 状态)

        % === 步骤4: 闭环仿真 (fb_gain=1) ===
        % 闭环: 控制器输出通过 Product 模块, 形成负反馈
        % fb_gain=1 时 K_long*(x_ref_long - x_long) 被完全加到控制输入上
        assignin('base','fb_gain',1);  % 反馈增益设为 1 (闭环)
        build_model(mdl);  % 重建模型 (因为 fb_gain 变量更新了)
        cl = sim(mdl,'StopTime','30');
        cl_data = cl.yout{1}.Values.Data;

        % === 步骤5: 计算稳态指标 ===
        % 取 t≥10s 之后的俯仰角均值作为稳态值
        % 10s 足够系统进入稳态 (短周期 t_s≈2s, 长周期 t_s≈5s)
        ss = cl.tout >= 10;  % 稳态索引 (逻辑向量)
        th_ol = mean(ol_data(ss,5))*180/pi;  % 开环稳态俯仰角 (度)
        th_cl = mean(cl_data(ss,5))*180/pi;  % 闭环稳态俯仰角 (度)
        err = 3.0 - th_cl;  % 跟踪误差 = 目标 θ_ref=3° - 实际稳态 θ

        % 判定: 误差 < 2° 为通过 (PASS), 否则失败 (FAIL)
        if abs(err) < 2.0, st = 'PASS'; else st = 'FAIL'; end
        fprintf('OL=%.2f° CL=%.2f° err=%.2f° [%s]\n',th_ol,th_cl,err,st);

        % 将结果打包到结构体, 存入 results cell 数组
        r.ol_data=ol_data; r.cl_data=cl_data;  % 原始仿真数据
        r.th_ol=th_ol; r.th_cl=th_cl;          % 稳态角度
        r.err=err; r.status=st;                 % 误差和判定
        results{t}=r;
        close_system(mdl,0);  % 关闭 Simulink 模型, 不保存

    catch ME
        % 如果任何步骤失败, 记录 CRASH 状态和错误信息
        fprintf('CRASH: %s\n',ME.message);
        results{t}=struct('status','CRASH','error',ME.message);
        try close_system(mdl,0); catch; end  % 尝试关闭模型, 忽略关闭错误
    end
end

%% 汇总输出
fprintf('\n===== 汇总 =====\n');
% 打印表头: 配置名称、开环 θ、闭环 θ、跟踪误差
fprintf('%-20s %8s %8s %8s\n','配置','OL_θ','CL_θ','误差');
for t=1:size(tests,1)
    r=results{t};
    if isfield(r,'status') && ~strcmp(r.status,'CRASH')
        % 正常完成: 打印数值
        fprintf('%-20s %7.2f° %7.2f° %7.2f° [%s]\n',...
            tests{t,4},r.th_ol,r.th_cl,r.err,r.status);
    else
        % 崩溃: 打印 CRASH
        fprintf('%-20s %8s\n',tests{t,4},'CRASH');
    end
end

% 保存结果到 .mat 文件, 供后续分析使用
save(fullfile(this_dir,'results.mat'),'results','tests');
fprintf('\n结果已保存\n');
