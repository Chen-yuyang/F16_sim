function lab1_step2_eigenvalue_analysis()
%==========================================================================
%  实验1 - 步骤2：特征值分析与模态识别
%  对 LOFI 和 HIFI 的纵向+横侧向线性模型分别进行特征值分析
%  识别飞行模态并对比两种保真度下的模态差异
%
%  【背景 - 为何分析特征值】
%  线性化后的系统 Δẋ = A·Δx 的解由 A 矩阵的特征值决定：
%    x(t) = Σ c_i · v_i · e^{λ_i t}
%  其中 λ_i = σ ± jω 为特征值，v_i 为特征向量
%
%  特征值与飞行品质的直接关系：
%    λ = σ + jω  →  时间响应 e^{σt} · sin(ωt)
%    σ < 0: 稳定（振荡衰减）  |  σ > 0: 不稳定（振荡发散）
%    ω = 0: 非振荡模态（实根）  |  ω ≠ 0: 振荡模态（复共轭对）
%
%  【飞行模态分类】
%  纵向模态（5阶 → 2对复根 + 1实根）：
%    短周期 (Short-Period): α 和 q 的耦合快速振荡
%    长周期/浮沉 (Phugoid): V 和 θ 的慢速振荡（动能↔势能交替）
%    功率模态 (Pow): 发动机惯性环节，通常为实根 λ ≈ -1
%
%  横侧向模态（5阶 → 1对复根 + 2实根 + 1零根）：
%    荷兰滚 (Dutch Roll): β, p, r 耦合振荡
%    螺旋 (Spiral): φ, ψ 的极慢漂移（可能不稳定但增长极慢）
%    滚转收敛 (Roll Subsidence): p 的快速阻尼
%    航向中立 (Heading Neutral): ψ 积分器，λ=0，中立稳定
%
%  【MIL-F-8785C 飞行品质标准】
%    短周期阻尼 ζ_sp ≥ 0.35 (Level 1), ≥ 0.25 (Level 2), ≥ 0.15 (Level 3)
%    长周期阻尼 ζ_ph ≥ 0.04 (Level 1)
%    荷兰滚阻尼 ζ_dr ≥ 0.19 (Level 1), ≥ 0.08 (Level 2)
%    滚转收敛时间常数 τ_roll ≤ 1.0s (Level 1)
%
%  输入: lab1_matrices.mat (由步骤1生成，含 LOFI+HIFI)
%  输出: lab1_eigenvalue_result.txt, lab1_eigen.mat
%==========================================================================

    %% ===================== 初始化与加载数据 =====================
    clc;
    fprintf('========================================\n');
    fprintf('  实验1 - 步骤2：特征值与模态识别\n');
    fprintf('  分析: LOFI + HIFI\n');
    fprintf('========================================\n\n');

    this_file = mfilename('fullpath');
    proj_root = fileparts(fileparts(fileparts(this_file)));
    result_dir = fileparts(this_file);
    cd(proj_root);

    result_mat = fullfile(result_dir, 'lab1_matrices.mat');
    result_txt = fullfile(result_dir, 'lab1_eigenvalue_result.txt');

    if ~exist(result_mat, 'file')
        error('未找到 lab1_matrices.mat，请先运行步骤1。');
    end

    % 从步骤1的结果中加载纵向和横侧向 A 矩阵
    % 这些矩阵是 5×5 的方阵，来自 LOFI 和 HIFI 两套模型
    %
    % 【命名约定】
    %   _lo = LOFI (Low Fidelity, 低保真度模型)
    %   _hi = HIFI (High Fidelity, 高保真度模型)
    %   例如: A_longitude_lo = LOFI 的纵向状态矩阵
    %         eig_lat_hi     = HIFI 的横侧向特征值
    load(result_mat, ...
        'A_longitude_lo', 'B_longitude_lo', 'A_lateral_lo', 'B_lateral_lo', ...
        'A_longitude_hi', 'B_longitude_hi', 'A_lateral_hi', 'B_lateral_hi');

    fprintf('  已加载: A_longitude_lo(5x5), A_lateral_lo(5x5)\n');
    fprintf('           A_longitude_hi(5x5), A_lateral_hi(5x5)\n\n');

    fid = fopen(result_txt, 'w');
    fprintf(fid, '============================================================\n');
    fprintf(fid, '  F-16 特征值分析 -  实验1 (LOFI vs HIFI)\n');
    fprintf(fid, '  时间: %s\n', datestr(now));
    fprintf(fid, '============================================================\n\n');

    %% ===================== Part A: LOFI 纵向模态 =====================
    % 注：_lo = LOFI（低保真度），_hi = HIFI（高保真度），下同
    % 纵向 5 阶系统 → 5 个特征值
    % 典型分布：2 个复共轭对 + 1 个实根
    %   λ_sp = -ζ_sp·ω_n,sp ± j·ω_n,sp·√(1-ζ_sp²)  [短周期 Short-Period]
    %   λ_ph = -ζ_ph·ω_n,ph ± j·ω_n,ph·√(1-ζ_ph²)  [长周期/浮沉 Phugoid]
    %   λ_pow ≈ -1                                      [功率 Power]
    %
    %  后缀 _sp / _ph / _dr 只是标明属于哪个模态，ζ 都是阻尼比的意思
    %
    % 预计 LOFI 短周期阻尼 ζ_sp ≈ 0.49 (Level 1)
    fprintf('--- Part A: LOFI 纵向模态 ---\n');
    fprintf(fid, '================== LOFI 纵向模态 ==================\n\n');
    % --- eig 函数说明 ---
    % eig(A) 计算方阵 A 的所有特征值，返回一个列向量
    %   数学定义: 特征值 λ 满足 det(A - λI) = 0，即 |A - λI| = 0
    %   对于 n×n 矩阵，共有 n 个特征值（含重根）
    %
    %   特征值 λ 一般为复数: λ = σ + jω
    %     σ = real(λ): 实部 → 决定衰减/发散速率 (σ<0 稳定, σ>0 不稳定)
    %       注: 阻尼比 ζ = -σ/ω_n，ζ>0 则衰减，ζ<0 则发散
    %
    %     ω = imag(λ): 虚部 → 阻尼振荡频率 ω_d (Damped Natural Frequency)
    %       也就是你实际能观察到的振荡频率
    %       它与无阻尼自然频率 ω_n 的关系: ω_d = ω_n · √(1-ζ²)
    %       |ω| = 0 为非振荡实模态, |ω| ≠ 0 为振荡复模态
    %       取绝对值 |ω| 是因为复根成对出现 (σ±jω)，周期只需正频率
    %       imag() = MATLAB 函数，提取复数的虚部，如 imag(3+4j) = 4
    %
    %   复特征值总是成对出现（共轭对）: 若 a+jb 是特征值，则 a-jb 也是
    %   实矩阵不可能有单独的复特征值
    %
    % 完整调用形式 [V, D] = eig(A):
    %   满足 A·V = V·D，即 A = V·D·V⁻¹ (特征分解)
    %     V: 特征向量矩阵 (n×n)，每一列 v_i 是对应于 λ_i 的特征向量
    %     D: 特征值对角矩阵 (n×n)，D(i,i) = λ_i
    %   物理含义: 特征向量 v_i 描述第 i 个模态下各状态量的相对幅值与相位
    %    例如纵向短周期模态的特征向量中 |v_α|/|v_q| 反映了 α 和 q 的耦合程度
    %
    % MATLAB eig 使用 QR 迭代算法 (LAPACK *GEV 系列)，对一般实矩阵自动处理复特征值
    %------------------------------------------------------------------
    eig_long_lo = eig(A_longitude_lo);
    identify_modes_longitudinal(eig_long_lo, fid);
    fprintf(fid, '\n');

    %% ===================== Part B: LOFI 横侧向模态 =====================
    % 横侧向 5 阶系统 → 5 个特征值
    % 状态变量: β(侧滑角), p(滚转角速率), r(偏航角速率),
    %           φ(滚转角), ψ(偏航角)
    % 典型分布：1 个复共轭对(荷兰滚) + 1 个大实根(滚转收敛)
    %           + 1 个小实根(螺旋) + 1 个零根(航向中立)
    %   λ_dr  = -ζ_dr·ω_n,dr ± j·ω_n,dr·√(1-ζ_dr²)  [荷兰滚 Dutch Roll]
    %   λ_roll = -1/τ_roll       [滚转收敛 Roll Subsidence (τ 很小 < 1s)]
    %   λ_sp  = -1/τ_sp          [螺旋 Spiral (τ 很大, 可达 100s+)]
    %   λ_heading = 0            [航向中立 Heading Neutral (ψ 积分器)]
    %
    %  后缀：_dr = Dutch Roll, _sp(横侧向) = Spiral, _roll = Roll Subsidence
    %
    % 【关于零根 λ=0 的说明】
    %   全状态 F-16 横侧向模型包含偏航角 ψ。但气动力只取决于相对气流(如 β)，
    %   不依赖于绝对航向 ψ，因此 A 矩阵中 ψ 列全为 0，产生 λ=0。
    %   这个根不叫螺旋，而是"航向积分器"——飞机不"认北"。
    %   参考: Stevens, Lewis & Johnson "Aircraft Control and Simulation" 3rd ed.
    %
    % 预计 LOFI 荷兰滚阻尼 ζ_dr ≈ 0.095 (Level 2)
    % 螺旋模态可能接近中性稳定 (λ ≈ 0 或略正)
    % 滚转收敛 τ ≈ 0.41s (Level 1)
    fprintf('--- Part B: LOFI 横侧向模态 ---\n');
    fprintf(fid, '================== LOFI 横侧向模态 ==================\n\n');
    eig_lat_lo = eig(A_lateral_lo);
    identify_modes_lateral(eig_lat_lo, fid);
    fprintf(fid, '\n');

    %% ===================== Part C: HIFI 纵向模态 =====================
    % HIFI 与 LOFI 的纵向模态预计有显著差异（原因见步骤1）：
    %   M_α(HIFI) ≈ -12.6 vs -2.2 (LOFI)
    %   → 短周期自然频率 ω_n,sp ∝ √(-M_α) 将大幅增加
    %   → 但阻尼比 ζ_sp ∝ -real(λ)/ω_n 将显著下降
    %      LOFI ζ_sp ≈ 0.49 → 预计 HIFI ζ_sp ≈ 0.19 (Level 3)
    %   这意味着增稳系统（SAS）对于 HIFI 模型是必需的，而非可选
    fprintf('--- Part C: HIFI 纵向模态 ---\n');
    fprintf(fid, '================== HIFI 纵向模态 ==================\n\n');
    eig_long_hi = eig(A_longitude_hi);
    identify_modes_longitudinal(eig_long_hi, fid);
    fprintf(fid, '\n');

    %% ===================== Part D: HIFI 横侧向模态 =====================
    % HIFI 横侧向预计与 LOFI 差异较小 (< 10%)
    % 因为 dLEF 在 xz 对称面内偏转，主要影响纵向气动
    % 模态分类同 Part B：荷兰滚 + 滚转收敛 + 螺旋 + 航向中立(λ=0)
    fprintf('--- Part D: HIFI 横侧向模态 ---\n');
    fprintf(fid, '================== HIFI 横侧向模态 ==================\n\n');
    eig_lat_hi = eig(A_lateral_hi);
    identify_modes_lateral(eig_lat_hi, fid);
    fprintf(fid, '\n');

    %% ===================== Part E: LOFI vs HIFI 对比 =====================
    % 逐特征值对比 LOFI 和 HIFI，并提取关键模态参数（短周期、荷兰滚等）
    % 这是理解保真度差异对飞行品质评估影响的核心
    fprintf('--- Part E: LOFI vs HIFI 对比 ---\n');
    fprintf(fid, '================== LOFI vs HIFI 模态对比 ==================\n\n');
    compare_modes(fid, eig_long_lo, eig_long_hi, '纵向');
    compare_modes(fid, eig_lat_lo,  eig_lat_hi,  '横侧向');

    %% ===================== 计算特征向量并保存 =====================
    % 特征向量矩阵 V 和特征值对角矩阵 D 满足：A·V = V·D
    % V 的列是特征向量，D 的对角元素是特征值
    % 特征向量显示每个特征值对应的运动形态，用于理解模态的物理含义
    [V_long_lo, D_long_lo] = eig(A_longitude_lo);
    [V_lat_lo,  D_lat_lo]  = eig(A_lateral_lo);
    [V_long_hi, D_long_hi] = eig(A_longitude_hi);
    [V_lat_hi,  D_lat_hi]  = eig(A_lateral_hi);

    fprintf(fid, '\n============================================================\n');
    fprintf(fid, '  步骤2完成 (LOFI + HIFI)\n');
    fprintf(fid, '============================================================\n');
    fclose(fid);

    % 控制台打印摘要
    fprintf('\n--- LOFI 纵向特征值 ---\n'); disp(eig_long_lo);
    fprintf('\n--- HIFI 纵向特征值 ---\n'); disp(eig_long_hi);
    fprintf('\n--- LOFI 横侧向特征值 ---\n'); disp(eig_lat_lo);
    fprintf('\n--- HIFI 横侧向特征值 ---\n'); disp(eig_lat_hi);

    % 保存结果变量命名同前：_lo=LOFI, _hi=HIFI
    save(fullfile(result_dir, 'lab1_eigen.mat'), ...
        'eig_long_lo', 'eig_lat_lo', 'eig_long_hi', 'eig_lat_hi', ...
        'V_long_lo', 'D_long_lo', 'V_lat_lo', 'D_lat_lo', ...
        'V_long_hi', 'D_long_hi', 'V_lat_hi', 'D_lat_hi');

    fprintf('\n========================================\n');
    fprintf('  步骤2完成! LOFI+HIFI 结果已保存到:\n');
    fprintf('    %s\n', result_txt);
    fprintf('========================================\n');

end


%% ===================== 子函数 =====================

function identify_modes_longitudinal(eigvals, fid)
    % 识别纵向模态（短周期 + 长周期 + 功率实根）
    %
    % 【核心逻辑】
    %   对于复特征值 λ = σ ± jω，用自然频率 ω_n = |λ| 辨别：
    %     ω_n > 0.3 rad/s → 短周期（快速振荡，约 1.5-3.5 rad/s）
    %     ω_n < 0.3 rad/s → 长周期/浮沉（慢速振荡，约 0.06-0.07 rad/s）
    %   分离阈值 0.3 rad/s 在此飞行条件下适用（短周期 ω_n ≈ 1.6-3.5）
    %
    % 【关键参数】
    %   阻尼比 ζ = -σ/ω_n  (0<ζ<1 为欠阻尼，ζ=0 无阻尼，ζ≥1 过阻尼)
    %   自然频率 ω_n = |λ| = √(σ²+ω²)
    %   周期 T = 2π/ω_d
    %     ω_d = |imag(λ)| = 阻尼振荡频率（就是 λ 虚部的绝对值）
    %     注意区分: ω_d ≠ ω_n，ω_n = |λ| = √(σ²+ω²) 是无阻尼自然频率
    %   时间常数 τ = 1/(ζ·ω_n)
    %
    % 本实验预计结果：
    %   LOFI:  ζ_sp ≈ 0.49 (Level 1),  ω_n,sp ≈ 1.65,  ζ_ph ≈ 0.11
    %   HIFI:  ζ_sp ≈ 0.19 (Level 3!), ω_n,sp ≈ 3.52,  ζ_ph ≈ 0.09
    fprintf(fid, '特征值共 %d 个 (5阶纵向系统):\n\n', length(eigvals));
    n_sp = 0; n_ph = 0; n_real = 0;
    for i = 1:length(eigvals)
        if abs(imag(eigvals(i))) > 1e-8
            if abs(eigvals(i)) > 0.3, n_sp = n_sp + 1; else, n_ph = n_ph + 1; end
        else
            n_real = n_real + 1;
        end
    end
    fprintf(fid, '模态分类: 短周期 %d个(%d对)  长周期 %d个(%d对)  实根 %d个\n\n', ...
        n_sp, n_sp/2, n_ph, n_ph/2, n_real);

    for i = 1:length(eigvals)
        lambda = eigvals(i); re = real(lambda); im = imag(lambda);
        fprintf(fid, '  特征值 (%d): ', i);
        if abs(im) > 1e-8
            % 复特征值 → 振荡模态
            wn = abs(lambda); zeta = -re/wn; T = 2*pi/abs(im);
            tau = 1/(zeta*wn); sigma = -zeta*wn;
            mode_name = cond(wn > 0.3, '短周期模态 (Short-Period)', '长周期模态 (Phugoid)');
            fprintf(fid, 'lambda = %+.6e %c %.6e j\n', re, sign_str(im), abs(im));
            fprintf(fid, '      模态: %s\n', mode_name);
            fprintf(fid, '      omega_n=%.6f rad/s (%.4f Hz)\n', wn, wn/(2*pi));
            fprintf(fid, '      zeta=%.6f  T=%.4fs  tau=%.4fs\n', zeta, T, tau);
            fprintf(fid, '      稳定性: %s\n', stab_str(zeta, re));
        else
            % 实特征值 → 非振荡模态（通常为功率环节或退化模态）
            tau = -1/re;
            fprintf(fid, 'lambda = %+.6e (实根)  tau=%.4fs  稳定性: %s\n', ...
                re, tau, stab_str(2, re));
        end
        fprintf(fid, '\n');
    end
    fprintf(fid, '纵向总结: 短周期=(alpha,q)耦合振荡  长周期=动能<->势能交替\n\n');
end

function identify_modes_lateral(eigvals, fid)
    % 识别横侧向模态（荷兰滚 + 滚转收敛 + 螺旋 + 航向中立）
    %
    % 全状态 F-16 横侧向模型为 5 阶系统，包含 5 个状态：
    %   β(侧滑角), p(滚转角速率), r(偏航角速率), φ(滚转角), ψ(偏航角)
    %
    % 【复根 → 荷兰滚 (Dutch Roll)】
    %   β-p-r 三自由度的耦合振荡运动
    %   典型 ω_n ≈ 2.8-3.0 rad/s, ζ ≈ 0.09-0.1
    %
    % 【大实根 → 滚转收敛 (Roll Subsidence)】
    %   主要由 L_p (滚转阻尼导数) 决定，λ ≈ L_p (通常 -2 ~ -3)
    %   物理含义：副翼偏转后滚转角速率的收敛快慢
    %
    % 【小实根（接近零但不为零）→ 螺旋 (Spiral)】
    %   由 L_β·N_r - L_r·N_β 等耦合项决定，τ 可达 100s+
    %   物理含义：倾斜后飞机是否自动改平
    %
    % 【λ ≈ 0 → 航向中立模态 (Heading Neutral)】
    %   由偏航角 ψ 引入——气动力不依赖于绝对航向
    %   对应 A 矩阵中 ψ 列全为零，产生严格零特征值
    %   物理含义：飞机不"认北"，航向改变后无气动恢复力
    %
    % 预计结果：
    %   LOFI:  ζ_dr ≈ 0.095, ω_n,dr ≈ 3.03, τ_roll ≈ 0.41s, τ_spiral ≈ 101s
    %   HIFI:  ζ_dr ≈ 0.094, ω_n,dr ≈ 2.82, τ_roll ≈ 0.44s, τ_spiral ≈ 65s
    %   → 横侧向差异显著小于纵向（dLEF 对横向影响有限）
    fprintf(fid, '特征值共 %d 个 (5阶横侧向系统):\n\n', length(eigvals));
    n_osc = 0; n_real = 0;
    for i = 1:length(eigvals)
        if abs(imag(eigvals(i))) > 1e-8, n_osc = n_osc + 1; else, n_real = n_real + 1; end
    end
    fprintf(fid, '荷兰滚: %d个共轭根(%d对)  ', n_osc, n_osc/2);
    fprintf(fid, '实根: %d个 (航向中立+螺旋+滚转收敛)\n\n', n_real);

    for i = 1:length(eigvals)
        lambda = eigvals(i); re = real(lambda); im = imag(lambda);
        fprintf(fid, '  特征值 (%d): ', i);
        if abs(im) > 1e-8
            % 【荷兰滚】唯一的横侧向振荡模态
            wn = abs(lambda); zeta = -re/wn; T = 2*pi/abs(im);
            t_half = log(2)/(zeta*wn);
            fprintf(fid, 'lambda = %+.6e %c %.6e j\n', re, sign_str(im), abs(im));
            fprintf(fid, '      模态: 荷兰滚 (Dutch Roll)\n');
            fprintf(fid, '      omega_n=%.6f  zeta=%.6f  T=%.4fs  t1/2=%.4fs\n', wn, zeta, T, t_half);
            fprintf(fid, '      稳定性: %s\n', stab_str(zeta, re));
        elseif abs(re) < 1e-8
            % 【航向中立】λ ≈ 0，由偏航角 ψ 积分器产生
            fprintf(fid, 'lambda = %+.6e (实根)\n', re);
            fprintf(fid, '      模态: 航向中立模态 (Heading Neutral)\n');
            fprintf(fid, '      说明: ψ 积分器，气动力不依赖绝对航向\n');
            fprintf(fid, '      稳定性: 中立稳定 (临界)\n');
        else
            % 【实根】按幅值区分螺旋和滚转收敛
            tau = 1/abs(re);
            if abs(re) > 0.5
                % 大实根 → 滚转收敛
                fprintf(fid, 'lambda = %+.6e (实根)  tau=%.4fs\n', re, tau);
                fprintf(fid, '      模态: 滚转收敛 (Roll Subsidence)\n');
                fprintf(fid, '      说明: 主要由 L_p 滚转阻尼导数决定\n');
                fprintf(fid, '      稳定性: %s\n', stab_str(2, re));
            else
                % 小实根 → 螺旋模态
                fprintf(fid, 'lambda = %+.6e (实根)  tau=%.4fs\n', re, tau);
                fprintf(fid, '      模态: 螺旋模态 (Spiral)\n');
                fprintf(fid, '      说明: 倾斜后飞机是否自动改平\n');
                fprintf(fid, '      稳定性: %s\n', stab_str(2, re));
            end
        end
        fprintf(fid, '\n');
    end
    fprintf(fid, '横侧向总结: 荷兰滚=(beta,p,r)耦合振荡  滚转收敛=快速滚转阻尼\n');
    fprintf(fid, '          螺旋=(phi,psi)极慢漂移  航向中立=ψ积分器(λ=0)\n\n');
end

function compare_modes(fid, e_lo, e_hi, label)
    % 对比 LOFI vs HIFI 特征值，提取关键模态参数差异
    %
    % 排序策略：按自然频率 |λ| 升序排列（低频→高频）
    % 确保 LOFI 和 HIFI 的特征值一一对应（因为解的拓扑结构相同）
    %
    % 纵向对比重点：
    %   短周期: LOFI vs HIFI 的 ω_n 和 ζ 差异（预计 HIFI ω_n↑ ζ↓）
    %   长周期: 两套模型差异较小（主要由重力/运动学决定，不依赖气动数据）
    %
    % 横侧向对比重点：
    %   荷兰滚: ω_n 和 ζ 的差异（预计 < 10%）
    %   滚转收敛: 时间常数 τ 的差异（预计 < 10%）
    fprintf(fid, '[%s] LOFI vs HIFI 特征值对比\n', label);
    fprintf(fid, '%-6s %-28s %-28s %-10s\n', '序号', 'LOFI', 'HIFI', '差异');
    fprintf(fid, '%s\n', repmat('-', 1, 75));

    elo = sort_by_wn(e_lo); ehi = sort_by_wn(e_hi);

    for i = 1:length(elo)
        l_lo = elo(i); l_hi = ehi(i);
        delta_real = real(l_lo) - real(l_hi);
        delta_imag = imag(l_lo) - imag(l_hi);
        fprintf(fid, '(%d)   %+8.4f %+8.4fj   %+8.4f %+8.4fj   Δ=(%+.2e,%+.2e)\n', ...
            i, real(l_lo), imag(l_lo), real(l_hi), imag(l_hi), delta_real, delta_imag);
    end

    if strcmp(label, '纵向')
        % 短周期：按 ω_n 排序后的第 3 个特征值（中频，索引 3 = |λ| 第三小）
        % 长周期：第 1 个（低频最小）
        wn_lo = abs(elo(3)); zeta_lo = -real(elo(3))/abs(elo(3));
        wn_hi = abs(ehi(3)); zeta_hi = -real(ehi(3))/abs(ehi(3));
        fprintf(fid, '\n  短周期: LOFI ωn=%.3f ζ=%.4f  |  HIFI ωn=%.3f ζ=%.4f\n', wn_lo, zeta_lo, wn_hi, zeta_hi);
        wn_lo_p = abs(elo(1)); zeta_lo_p = -real(elo(1))/abs(elo(1));
        wn_hi_p = abs(ehi(1)); zeta_hi_p = -real(ehi(1))/abs(ehi(1));
        fprintf(fid, '  长周期: LOFI ωn=%.4f ζ=%.4f  |  HIFI ωn=%.4f ζ=%.4f\n', wn_lo_p, zeta_lo_p, wn_hi_p, zeta_hi_p);
    else
        % 横侧向：5 个特征值 → 荷兰滚 + 滚转收敛 + 螺旋 + 航向中立(λ=0)
        %
        % 识别策略：
        %   1) 荷兰滚：找最大 |imag|（唯一的复共轭对）
        %   2) 滚转收敛：找最负实根（最快的实模态衰减）
        %   3) 航向中立：最接近 0 的实根（|re| 最小）
        %   4) 螺旋模态：剩下的那个实根

        % 1) 荷兰滚
        [~, idx_lo] = max(abs(imag(elo))); [~, idx_hi] = max(abs(imag(ehi)));
        wn_lo = abs(elo(idx_lo)); zeta_lo = -real(elo(idx_lo))/abs(elo(idx_lo));
        wn_hi = abs(ehi(idx_hi)); zeta_hi = -real(ehi(idx_hi))/abs(ehi(idx_hi));
        fprintf(fid, '\n  荷兰滚: LOFI ωn=%.3f ζ=%.4f  |  HIFI ωn=%.3f ζ=%.4f\n', wn_lo, zeta_lo, wn_hi, zeta_hi);

        % 2) 滚转收敛（最负实根）
        rr_lo = min(real(elo)); rr_hi = min(real(ehi));
        fprintf(fid, '  滚转收敛: LOFI λ=%.3f τ=%.3fs  |  HIFI λ=%.3f τ=%.3fs\n', ...
            rr_lo, -1/rr_lo, rr_hi, -1/rr_hi);

        % 逐根标记已识别的模态，以便定位航向中立即螺旋
        used_lo = false(length(elo), 1); used_hi = false(length(ehi), 1);
        used_lo(idx_lo) = true;          used_hi(idx_hi) = true;          % 荷兰滚占2个
        [~, rridx_lo] = min(real(elo));  [~, rridx_hi] = min(real(ehi));
        used_lo(rridx_lo) = true;        used_hi(rridx_hi) = true;       % 滚转收敛占1个

        % 3) 航向中立：未被标记的实根中 |real| 最小的
        re_lo = real(elo); re_hi = real(ehi);
        cand_lo = re_lo(~used_lo); cand_hi = re_hi(~used_hi);
        [~, hidx_lo] = min(abs(cand_lo)); [~, hidx_hi] = min(abs(cand_hi));
        h_lo = cand_lo(hidx_lo); h_hi = cand_hi(hidx_hi);
        fprintf(fid, '  航向中立: LOFI λ=%.6f  |  HIFI λ=%.6f\n', h_lo, h_hi);

        % 4) 螺旋模态：剩下的最后一个实根
        % 在 used 中标记航向中立的原始索引
        orig_idx_lo = find(~used_lo); orig_idx_hi = find(~used_hi);
        used_lo(orig_idx_lo(hidx_lo)) = true; used_hi(orig_idx_hi(hidx_hi)) = true;
        sp_lo = real(elo(~used_lo)); sp_hi = real(ehi(~used_hi));
        if ~isempty(sp_lo) && ~isempty(sp_hi)
            tau_sp_lo = -1/sp_lo; tau_sp_hi = -1/sp_hi;
            fprintf(fid, '  螺旋模态: LOFI λ=%.4f τ=%.1fs  |  HIFI λ=%.4f τ=%.1fs\n', ...
                sp_lo, tau_sp_lo, sp_hi, tau_sp_hi);
        end
    end
    fprintf(fid, '\n');
end

function sorted = sort_by_wn(e)
    % 按自然频率 (|λ|) 升序排序
    % 保证 LOFI 和 HIFI 的特征值一一可比
    [~, idx] = sort(abs(e));
    sorted = e(idx);
end

function val = cond(tst, a, b)
    % 三元条件运算符的 MATLAB 替代
    % cond(条件, 真值, 假值) 等价于 ? : 运算符
    if tst, val = a; else, val = b; end
end

function s = stab_str(zeta_or_mode, re)
    % 判断稳定性描述
    % 调用方式 1: stab_str(zeta, re)  → 复根的阻尼分析
    % 调用方式 2: stab_str(2, re)     → 实根的稳定性分析 (mode=2 为标记)
    if nargin == 2 && zeta_or_mode == 2
        if re < 0
            s = '稳定(指数衰减)';
        elseif abs(re) < 1e-8
            s = '中立稳定(临界)';
        else
            s = '不稳定(指数发散)';
        end
    else
        if zeta_or_mode > 0 && zeta_or_mode < 1
            s = '欠阻尼(稳定,振荡收敛)';
        elseif zeta_or_mode >= 1
            s = '过阻尼(稳定)';
        else
            s = '发散不稳定!';
        end
    end
end

function s = sign_str(val)
    % 格式化符号字符串（用于显示复数的虚部符号）
    if val >= 0, s = '+'; else, s = '-'; end
end
