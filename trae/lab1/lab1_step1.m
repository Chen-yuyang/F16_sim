function lab1_step1()
%==========================================================================
%  实验项目1 - 步骤1：F-16纵向线性化模型 A/B 矩阵提取
%
%  飞行条件: 高度 5000m, 速度 200m/s
%  输出:     纵向A矩阵(5x5), B矩阵(5x2), 特征值分析
%
%  使用方法: 在MATLAB中直接运行 lab1_step1()
%  结果保存在: trae/lab1/step1_result.txt
%
%  作者: Trae IDE 自动生成
%  日期: 2026-04-17
%==========================================================================

    clc;

    this_file = mfilename('fullpath');
    proj_root = fileparts(fileparts(fileparts(this_file)));
    result_dir = fileparts(this_file);
    result_file = fullfile(result_dir, 'step1_result.txt');

    cd(proj_root);

    model_trim = 'F16_trim';
    model_lin  = 'F16_openloop_linearization';

    fid = fopen(result_file, 'w');
    fprintf(fid, '============================================================\n');
    fprintf(fid, '  F-16 纵向线性化模型提取 - 实验项目1 步骤1\n');
    fprintf(fid, '  飞行条件: H = 5000 m, V = 200 m/s\n');
    fprintf(fid, '  时间: %s\n', datestr(now));
    fprintf(fid, '============================================================\n\n');

    global fi_flag_Simulink phi_weight theta_weight psi_weight altitude velocity
    fi_flag_Simulink = 0;
    phi_weight = 10; theta_weight = 10; psi_weight = 10;
    altitude = 5000;
    velocity = 200;

    OPTIONS = optimset('TolFun',1e-10,'TolX',1e-10,'MaxFunEvals',5000,'MaxIter',1000,'Display','final');

    fprintf('=== 步骤1/4: 加载并编译F16_trim模型 ===\n');
    fprintf(fid, '[步骤1] 加载并编译F16_trim模型...\n');
    load_system(model_trim);
    feval('F16_trim', [], [], [], 'lincompile');
    fprintf(fid, '  模型编译完成\n\n');

    beta = 0; dth = 0.2;
    elevator = -2*pi/180;
    alpha = 10*pi/180;
    rudder = 0; aileron = 0;
    UX0 = [beta; elevator; alpha; aileron; rudder; dth];

    phi=0; theta=0.0349; psi=0;
    p=0; q=0; r=0;
    pow_val = tgear(dth);
    dLEF = 0.0;

    xu_test = [velocity, beta, alpha, phi, theta, psi, p, q, r, 0, 0, -altitude, pow_val]';
    uu_test = [dth, elevator, aileron, rudder, dLEF, fi_flag_Simulink]';

    try
        dx = feval('F16_trim', 0, xu_test, uu_test, 'derivs');
        fprintf('  derivs调用成功, dx维度=%d\n', length(dx));
        fprintf(fid, '  derivs调用成功, 状态向量验证通过 (dim=%d)\n\n', length(dx));
    catch ME
        fprintf(fid, '  ERROR: %s\n', ME.message);
        fclose(fid);
        error('模型derivs调用失败: %s', ME.message);
    end

    fprintf('\n=== 步骤2/4: 配平迭代 (fminsearch单纯形法) ===\n');
    fprintf(fid, '[步骤2] 配平迭代寻找平衡状态...\n');
    fprintf(fid, '  初始猜测: beta=0, elev=-2deg, alpha=10deg, dth=0.2\n');
    fprintf(fid, '  使用Nelder-Mead单纯形法优化...\n\n');

    [UX,FVAL,EXITFLAG,OUTPUT] = fminsearch('trim_fun', UX0, OPTIONS);
    [best_cost, Xdot, best_xu, best_uu] = trim_fun(UX);

    feval('F16_trim', [], [], [], 'term');

    fprintf('  -> 配平完成! 迭代次数: %d, 函数调用: %d\n', OUTPUT.iterations, OUTPUT.funcCount);
    fprintf('  -> 最终代价 cost=%.4e, 退出标志=%d\n', best_cost, EXITFLAG);
    fprintf(fid, '  配平完成! 迭代次数=%d, 函数调用=%d\n', OUTPUT.iterations, OUTPUT.funcCount);
    fprintf(fid, '  最终代价 cost=%.4e, 退出标志=%d\n\n', best_cost, EXITFLAG);

    xu = best_xu; uu = best_uu; cost = best_cost;
    trim_state_lo = xu(1:13);
    trim_thrust_lo = uu(1);
    trim_control_lo = [uu(2);uu(3);uu(4)];
    dLEF = uu(5);

    fprintf(fid, '\n---------- 配平结果 ----------\n');
    fprintf(fid, '  目标高度      = %d m\n', altitude);
    fprintf(fid, '  目标速度      = %d m/s\n', velocity);
    fprintf(fid, '  配平代价 cost  = %.10e\n', cost);
    fprintf(fid, '  V   = %.4f m/s\n', xu(1));
    fprintf(fid, '  beta= %.8f rad (%.4f deg)\n', xu(2), xu(2)*180/pi);
    fprintf(fid, '  alpha = %.6f rad (%.4f deg)\n', xu(3), xu(3)*180/pi);
    fprintf(fid, '  phi  = %.8f rad\n', xu(4));
    fprintf(fid, '  theta= %.6f rad (%.4f deg)\n', xu(5), xu(5)*180/pi);
    fprintf(fid, '  psi  = %.8f rad\n', xu(6));
    fprintf(fid, '  altitude = %.4f m\n', -xu(12));
    fprintf(fid, '  power= %.4f %%\n', xu(13));
    fprintf(fid, '  thrust = %.6f\n', uu(1));
    fprintf(fid, '  elevator = %.6f rad (%.4f deg)\n', uu(2), uu(2)*180/pi);
    fprintf(fid, '  aileron = %.6f rad (%.4f deg)\n', uu(3), uu(3)*180/pi);
    fprintf(fid, '  rudder  = %.6f rad (%.4f deg)\n', uu(4), uu(4)*180/pi);
    fprintf(fid, '  dLEF    = %.6f rad (%.4f deg)\n', uu(5), uu(5)*180/pi);

    fprintf('\n=== 步骤3/4: 线性化提取A/B矩阵 ===\n');
    fprintf(fid, '\n[步骤3] 调用linmod进行线性化...\n');
    load_system(model_lin);
    assignin('base', 'fi_flag_Simulink', fi_flag_Simulink);
    assignin('base', 'init_x', trim_state_lo);
    assignin('base', 'init_u', [trim_thrust_lo; trim_control_lo]);
    [A_lo, B_lo, C_lo, D_lo] = linmod(model_lin, trim_state_lo, [trim_thrust_lo; trim_control_lo]);

    idx = [12, 1, 3, 8, 5];
    A_sub = A_lo(idx, idx);
    B_sub = B_lo(idx, 1:2);
    S = diag([-1, 1, 1, 1, 1]);
    A_long = S * A_sub * S;
    B_long = S * B_sub;

    state_names = {'H(m)', 'V(m/s)', 'alpha(rad)', 'q(rad/s)', 'theta(rad)'};
    ctrl_names = {'thrust', 'elevator(rad)'};

    fprintf(fid, '\n========== 纵向 A 矩阵 (5 x 5) ==========\n');
    fprintf(fid, '  状态变量顺序:\n');
    for i = 1:5
        fprintf(fid, '    [%d] %s\n', i, state_names{i});
    end
    fprintf(fid, '\n           ');
    for j = 1:5, fprintf(fid, '%14s', state_names{j}); end
    fprintf(fid, '\n');
    for i = 1:5
        fprintf(fid, '  %-8s ', state_names{i});
        for j = 1:5, fprintf(fid, '%14.6e', A_long(i,j)); end
        fprintf(fid, '\n');
    end

    fprintf(fid, '\n========== 纵向 B 矩阵 (5 x 2) ==========\n');
    fprintf(fid, '  控制变量顺序:\n');
    for i = 1:2
        fprintf(fid, '    [%d] %s\n', i, ctrl_names{i});
    end
    fprintf(fid, '\n           ');
    for j = 1:2, fprintf(fid, '%14s', ctrl_names{j}); end
    fprintf(fid, '\n');
    for i = 1:5
        fprintf(fid, '  %-8s ', state_names{i});
        for j = 1:2, fprintf(fid, '%14.6e', B_long(i,j)); end
        fprintf(fid, '\n');
    end

    fprintf('\n=== 步骤4/4: 特征值分析 ===\n');
    eig_long = eig(A_long);

    fprintf(fid, '\n========== 特征值与模态分析 ==========\n');
    fprintf(fid, '  共%d个特征值(对应5阶纵向系统):\n\n', length(eig_long));

    sp_count = 0;
    phugoid_count = 0;

    for i = 1:length(eig_long)
        re = real(eig_long(i)); im = imag(eig_long(i));
        if abs(im) > 1e-10
            wn = abs(eig_long(i));
            zeta = -re / wn;
            T = 2*pi / abs(im);
            if wn > 0.3
                mode_name = '短周期模态 (Short-Period)';
                sp_count = sp_count + 1;
            else
                mode_name = '长周期模态 (Phugoid)';
                phugoid_count = phugoid_count + 1;
            end
            fprintf(fid, '  lambda(%d) = %+.6e %+.6ei\n', i, re, im);
            fprintf(fid, '    模态类型 : %s\n', mode_name);
            fprintf(fid, '    自然频率 wn  = %.6f rad/s (%.4f Hz)\n', wn, wn/(2*pi));
            fprintf(fid, '    阻尼比   zeta = %.6f\n', zeta);
            fprintf(fid, '    周期     T    = %.4f s\n', T);
            if zeta >= 0 && zeta < 1
                fprintf(fid, '    稳定性   : 欠阻尼振荡衰减 (稳定)\n');
            elseif zeta >= 1
                fprintf(fid, '    稳定性   : 过阻尼 (稳定)\n');
            else
                fprintf(fid, '    稳定性   : 不稳定!\n');
            end
        else
            tau = -1/re;
            fprintf(fid, '  lambda(%d) = %+.6e (实根)\n', i, re);
            fprintf(fid, '    时间常数 tau = %.4f s\n', tau);
            if re < 0
                fprintf(fid, '    稳定性       : 稳定 (指数衰减)\n');
            else
                fprintf(fid, '    稳定性       : 不稳定 (指数发散!)\n');
            end
        end
        fprintf(fid, '\n');
    end

    fprintf(fid, '============================================================\n');
    fprintf(fid, '  分析完毕\n');
    fprintf(fid, '============================================================\n');
    fclose(fid);

    fprintf('\n============================================================\n');
    fprintf('  完成! 结果已保存到:\n    %s\n', result_file);
    fprintf('============================================================\n\n');

    format short e
    fprintf('--- A矩阵 ---\n'); disp(A_long);
    fprintf('--- B矩阵 ---\n'); disp(B_long);
    fprintf('--- 特征值 ---\n'); disp(eig_long);

end
