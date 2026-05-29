function lab1_step1_trim_and_linearize()
%==========================================================================
%  实验1 - 步骤1：F-16 配平与线性化
%  同时对 LOFI (低 fidelity) 和 HIFI (高 fidelity) 气动模型进行配平和线性化
%  从 13 阶全状态空间模型中提取纵向（5阶）和横侧向（5阶）子模型
%
%  【背景】
%  F-16 的非线性动力学 ẋ = f(x, u) 需要在平衡点附近线性化为
%  Δẋ = A·Δx + B·Δu 的形式，才能使用线性系统理论分析稳定性。
%  配平 = 找到使 ẋ=0 的控制量 u₀ 和状态 x₀。
%  线性化 = 用一阶泰勒展开近似 f 在 (x₀,u₀) 附近的局部行为。
%
%  飞行条件: 高度 H = 5000 m, 速度 V = 200 m/s
%
%  【变量生命周期】
%  本文件是 MATLAB 函数（以 function 开头），而非脚本（.m 无 function）。
%  函数内所有局部变量（A_lo, B_lo, trim_state_lo, mat_lo 等）在函数执行
%  完毕后自动销毁，不会留在 base workspace 中。
%  运行结束后工作区仅保留：
%    1) global 声明的全局变量（fi_flag_Simulink 等）
%    2) assignin('base', ...) 写入的变量（init_x, init_u, init_dlef）
%  如需在运行后查看中间变量，可在代码中设断点调试，或从保存的 .mat 文件中加载。
%  相比之下，FindF16Dynamics.m 是脚本（无 function 关键字），其中所有变量
%  在运行后都会留在 base workspace 中。
%
%  【LOFI vs HIFI】
%  - LOFI: 简化气动数据（dLEF=0°），数据硬编码在 C 数组中，线性插值
%  - HIFI: 完整风洞数据（dLEF 自动计算 ≈ 2°），从 .dat 文件读取，多维插值
%  dLEF 差异 → 翼型弯度不同 → 气动中心位置不同 → 矩阵元素不同
%
%  输出:
%     lab1_trim_result.txt  - LOFI+HIFI 配平结果 + A/B 矩阵
%     lab1_matrices.mat     - LOFI+HIFI 的 A/B/C/D 矩阵
%==========================================================================

    %% ===================== 初始化：飞行条件与全局变量 =====================
    clc;
    fprintf('========================================\n');
    fprintf('  实验1 - 步骤1：F-16 配平与线性化\n');
    fprintf('  飞行条件: H=5000m, V=200m/s\n');
    fprintf('  模型: LOFI + HIFI\n');
    fprintf('========================================\n\n');

    % 自动定位项目根目录（本文件位于 trae/lab1_0429/ 下）
    this_file = mfilename('fullpath');
    proj_root = fileparts(fileparts(fileparts(this_file)));
    result_dir = fileparts(this_file);
    cd(proj_root);

    result_txt = fullfile(result_dir, 'lab1_trim_result.txt');
    result_mat = fullfile(result_dir, 'lab1_matrices.mat');

    % 打开文本结果文件，写入文件头
    fid = fopen(result_txt, 'w');
    fprintf(fid, '============================================================\n');
    fprintf(fid, '  F-16 配平与线性化结果 - 实验1\n');
    fprintf(fid, '  飞行条件: H = 5000 m, V = 200 m/s\n');
    fprintf(fid, '  时间: %s\n', datestr(now));
    fprintf(fid, '============================================================\n\n');

    % --- 全局变量说明 ---
    % fi_flag_Simulink: 模型选择开关 (0=LOFI, 1=HIFI)
    %   该变量通过 assignin 传递给 Simulink 模型，F16_dyn.c 内部根据此标志
    %   切换 lofi/hifi 两套气动数据插值路径
    % phi/theta/psi_weight: 配平代价函数的权重 (见 trim_fun.m)
    % altitude/velocity: 飞行条件
    global fi_flag_Simulink phi_weight theta_weight psi_weight altitude velocity
    phi_weight = 10; theta_weight = 10; psi_weight = 10;
    altitude = 5000;
    velocity = 200;

    %% ===================== 配平求解器配置 =====================
    % fminsearch 使用 Nelder-Mead 单纯形法（直接搜索，不需梯度）
    % 优化目标：trim_fun 返回的加权状态变化率平方和 J = Σ w_i · ẋ_i²
    % 通过最小化 J 找到平衡点 (x₀, u₀)
    OPTIONS = optimset('TolFun', 1e-10, 'TolX', 1e-10, 'MaxFunEvals', 5000, ...
                       'MaxIter', 1000, 'Display', 'final');

    % --- 配平初始猜测 ---
    % UX = [beta; elevator; alpha; aileron; rudder; dth]
    % 这些是 trim_fun.m 中使用的自由变量
    % beta=0（对称飞行）、elevator=-2°（初始低头力矩配平）、alpha=10°（初始迎角）
    % aileron=0、rudder=0（对称飞行）、dth=0.2（初始油门）
    beta = 0; dth = 0.2;
    elevator = -2*pi/180;
    alpha = 10*pi/180;
    rudder = 0; aileron = 0;
    UX0 = [beta; elevator; alpha; aileron; rudder; dth];

    %% ===================== 状态/控制索引定义 =====================
    % F-16 全状态向量 (13 维)：
    %   x = [V, β, α, φ, θ, ψ, p, q, r, xE, yE, -h, Pow]
    %       北向位置 东向位置  高度   功率
    %   不需要（位置环）
    % 控制向量 (4 维+1)：
    %   u = [throttle, elevator, aileron, rudder, dLEF]
    %
    % 气动-运动解耦原理：
    %   飞机关于 xz 平面对称。小扰动下，纵向运动（对称面内）不会激励
    %   横侧向运动（对称面外），反之亦然。因此 A 矩阵可近似为块对角。
    %
    % 纵向状态 (5 个):  V, α, θ, q, Pow    → 速度、迎角、俯仰角、俯仰角速率、发动机功率
    % 纵向控制 (2 个):  thrust, elevator    → 推力、升降舵
    % 横侧向状态 (5 个): β, φ, ψ, p, r     → 侧滑角、滚转角、偏航角、滚转角速率、偏航角速率
    % 横侧向控制 (2 个): aileron, rudder    → 副翼、方向舵
    %
    % 索引对应 linmod 输出的 13 状态 + 4 控制 = 17×17 矩阵
    % linmod 输出矩阵列顺序 = [13 states; 4 controls]
    idx_long_state = [1, 3, 5, 8, 13];   % V, α, θ, q, Pow
    idx_long_ctrl  = [14, 15];            % thrust, elevator
    idx_lat_state  = [2, 4, 6, 7, 9];    % β, φ, ψ, p, r
    idx_lat_ctrl   = [16, 17];            % aileron, rudder

    % 行/列标签（用于格式化输出矩阵）
    long_sn = {'V(m/s)', 'alpha(rad)', 'theta(rad)', 'q(rad/s)', 'pow'};
    long_cn = {'thrust', 'elevator(rad)'};
    lat_sn  = {'beta(rad)', 'phi(rad)', 'psi(rad)', 'p(rad/s)', 'r(rad/s)'};
    lat_cn  = {'aileron(rad)', 'rudder(rad)'};


    %% ===================== LOFI (Low Fidelity) =====================
    % LOFI 特点：
    %   1. 气动系数硬编码在 C 数组中，一维/二维线性插值
    %   2. dLEF ≡ 0°（前缘襟翼始终收起，不考虑自动偏转）
    %   3. 气动模型简单：CX/Cm 仅依赖 α,δe；CY 用解析公式
    %   4. 无 LEF 修正、无分离流修正
    % → 适用于快速原型验证，但纵向气动导数精度有限
    fprintf('==================== LOFI (Low Fidelity) ====================\n');
    fprintf(fid, '==================== LOFI (Low Fidelity) ====================\n\n');

    fi_flag_Simulink = 0;

    %% --- LOFI 阶段1：配平 ---
    % 配平流程：
    %   1. feval('F16_trim',[],[],[],'lincompile') → 编译/初始化 Simulink 模型
    %   2. load_system → 加载模型到内存
    %   3. fminsearch('trim_fun', UX0) → Nelder-Mead 搜索最优 UX
    %   4. trim_fun(UX) 内部：
    %      a. 从 UX 解析 free variables: beta, elevator, alpha, aileron, rudder, dth
    %      b. 设置状态 x = [V, β, α, φ, θ, ψ, p, q, r, 0,0,-h, Pow]
    %      c. 调用 feval('F16_trim', t, x, u, 'derivs') 得到 ẋ
    %      d. 返回加权平方和 J = Σ w_i · ẋ_i²
    %   5. 最优 UX → trim_fun 再调一次，提取配平状态 xu 和控制 uu
    %
    % 其中 important: trim_fun 中使用的 alpha 和 dLEF 的约束上下界
    % 在 LOFI 和 HIFI 中不同：
    %   LOFI:  alpha ∈ [-10°, 20°],  dLEF = 0 (固定)
    %   HIFI:  alpha ∈ [-20°, 90°],  dLEF = f(α, qbar, ps) (自动计算)
    fprintf('--- LOFI 阶段1/4: 配平 ---\n');
    fprintf(fid, '[LOFI 阶段1] 配平\n');
    fprintf(fid, '  使用 Nelder-Mead 单纯形法 (fminsearch)\n\n');

    feval('F16_trim', [], [], [], 'lincompile');
    load_system('F16_trim');
    [UX, FVAL, EXITFLAG, OUTPUT] = fminsearch('trim_fun', UX0, OPTIONS);
    [best_cost_lo, Xdot_lo, xu_lo, uu_lo] = trim_fun(UX);
    feval('F16_trim', [], [], [], 'term');    % 清理 Simulink 模型

    fprintf('  LOFI 配平: iter=%d, cost=%.4e, exit=%d\n', ...
        OUTPUT.iterations, best_cost_lo, EXITFLAG);% 输出提示到命令行
    fprintf(fid, '  配平完成! 迭代次数: %d, 函数调用: %d\n', ...
        OUTPUT.iterations, OUTPUT.funcCount);
    fprintf(fid, '  最终代价 cost = %.4e, 退出标志 = %d\n\n', best_cost_lo, EXITFLAG);

    % 提取配平结果
    % xu_lo(1:13): 13 维状态向量
    % uu_lo: [throttle, elevator, aileron, rudder, dLEF]
    %   uu_lo(1): 油门杆位置 (0~1)
    %   uu_lo(2:4): 升降舵/副翼/方向舵偏角 (rad)
    %   uu_lo(5): 前缘襟翼偏角 (LOFI 中恒为 0)
    trim_state_lo = xu_lo(1:13);
    trim_thrust_lo = uu_lo(1);
    trim_control_lo = [uu_lo(2); uu_lo(3); uu_lo(4)];
    dLEF_lo = uu_lo(5);

    print_trim_state(fid, 'LOFI', xu_lo, uu_lo);

    disp(['    LOFI alpha=' num2str(xu_lo(3)*180/pi) ...
        ' deg, elev=' num2str(uu_lo(2)*180/pi) ' deg, cost=' num2str(best_cost_lo)]);

    %% --- LOFI 阶段2：线性化 ---
    % linmod 原理（数值扰动法）：
    %   在配平点 (x₀, u₀) 处，非线性系统 ẋ = f(x,u) 泰勒展开（忽略高阶项）：
    %     Δẋ ≈ ∂f/∂x · Δx + ∂f/∂u · Δu = A·Δx + B·Δu
    %
    %   求 A 的第 j 列 (∂f/∂xⱼ)：扰动状态 xⱼ，控制量冻结在 u₀
    %     Δxⱼ = 1e-5 · (1 + |xⱼ|)
    %     A(:,j) = [f(xⱼ+Δxⱼ, u₀) - f(xⱼ, u₀)] / Δxⱼ
    %   求 B 的第 j 列 (∂f/∂uⱼ)：扰动控制 uⱼ，状态量冻结在 x₀
    %     Δuⱼ = 1e-5 · (1 + |uⱼ|)
    %     B(:,j) = [f(x₀, uⱼ+Δuⱼ) - f(x₀, uⱼ)] / Δuⱼ
    %   重复 13+4 = 17 次差分，拼成完整的 A(13×13) 和 B(13×4) 矩阵
    %
    %   这是航空领域标准做法：气动数据是查表插值的，没有封闭解析表达式，
    %   无法求解析偏导，因此用数值扰动法在 Simulink 黑箱层面做线性化。
    %   FindF16Dynamics.m（2002 年原版）和 trim_F16.m 均使用完全相同的
    %   linmod('F16_openloop_linearization', ...) 方式，仅提取子矩阵时
    %   原版用 sel() 函数，本版改用更直观的直接索引 A(idx_state, idx_state)。
    %
    % F16_openloop_linearization.slx 是线性化专用 Simulink 模型
    %   assignin('base', ...) 将变量写入 MATLAB base workspace，是脚本→Simulink 的数据桥梁：
    %     Simulink 模型直接从 base workspace 读取变量作为模块参数（如状态初值、增益等），
    %     而普通赋值 = 只在当前函数/脚本的局部工作空间中有效，Simulink 无法访问。
    %   本文件中 assignin 仅出现在此线性化步骤（LOFI+HIFI 共 8 处），
    %     其他文件（FindF16Dynamics.m、runF16model.m）虽也用同名变量，但通过直接赋值 = 使用。
    fprintf('\n--- LOFI 阶段2/4: 线性化 ---\n');
    fprintf(fid, '\n[LOFI 阶段2] 线性化 (linmod)\n\n');

    load_system('F16_openloop_linearization');
    assignin('base', 'fi_flag_Simulink', fi_flag_Simulink);  % 模型选择: 0=LOFI, 1=HIFI → 传给 Simulink 工作空间, F16_dyn.c 据此切换气动数据路径
    assignin('base', 'init_x', trim_state_lo);               % 初始状态向量 x₀ (13维): [V, β, α, φ, θ, ψ, p, q, r, xE, yE, -h, Pow] — linmod 在此状态附近做数值扰动线性化
    assignin('base', 'init_u', [trim_thrust_lo; trim_control_lo]);  % 初始控制向量 u₀ (4维): [throttle(0~1); elevator(rad); aileron(rad); rudder(rad)]
    assignin('base', 'init_dlef', dLEF_lo);                  % 前缘襟翼偏角 (rad): LOFI=0, HIFI≈2° — 作为单独参数传入 C 函数的气动查表模块
    [A_lo, B_lo, C_lo, D_lo] = linmod('F16_openloop_linearization', ...
        trim_state_lo, [trim_thrust_lo; trim_control_lo]);
    close_system('F16_openloop_linearization', 0);
    mat_lo = [A_lo, B_lo; C_lo, D_lo];

    fprintf(fid, '  线性化完成: A(%dx%d) B(%dx%d)\n\n', ...
        size(A_lo,1), size(A_lo,2), size(B_lo,1), size(B_lo,2));

    %% --- LOFI 阶段3：提取纵向矩阵 (5阶) ---
    % 从 13 阶全矩阵中提取纵向子矩阵
    % 纵向 A 矩阵 (5×5) 包含以下气动导数：
    %   A(1,:) → V̇ 方程:  X_V(速度阻尼), X_α(迎角-速度耦合), -g·cosθ₀(重力项), X_q(俯仰阻尼), X_Pow(推力)
    %   A(2,:) → α̇ 方程:  Z_V, Z_α, Z_θ≈0, Z_q≈1(几何关系: α̇≈q), Z_Pow
    %   A(3,:) → θ̇ 方程:  运动学恒等式 θ̇=q, 故仅有 (3,4)=1
    %   A(4,:) → q̇ 方程:  M_V, M_α(静稳定性!), M_q(俯仰阻尼), ...
    %   A(5,:) → Ṗow 方程: 发动机惯性环节, 通常 (5,5)≈-1
    %
    % M_α 是最关键的纵向稳定性参数：负值表示机头恢复力（静稳定飞机）
    fprintf('--- LOFI 阶段3/4: 纵向矩阵 ---\n');
    A_longitude_lo = mat_lo(idx_long_state, idx_long_state);
    B_longitude_lo = mat_lo(idx_long_state, idx_long_ctrl);
    fprintf(fid, '[LOFI 阶段3] 纵向矩阵\n\n');
    print_matrix(fid, '纵向 A 矩阵 (5x5)', A_longitude_lo, long_sn, long_sn);
    print_matrix(fid, '纵向 B 矩阵 (5x2)', B_longitude_lo, long_sn, long_cn);

    %% --- LOFI 阶段4：提取横侧向矩阵 (5阶) ---
    % 横侧向 A 矩阵 (5×5) 包含以下气动导数：
    %   A(1,:) → β̇ 方程:  Y_β(方向稳定性), Y_φ≈g/V₀(重力耦合), Y_p, Y_r≈-1(运动耦合)
    %   A(2,:) → φ̇ 方程:  运动学 φ̇=p, 故 (2,4)=1
    %   A(3,:) → ψ̇ 方程:  运动学 ψ̇=r, 故 (3,5)=1
    %   A(4,:) → ṗ 方程:  L_β(上反角效应!), L_p(滚转阻尼), L_r(交叉导数)
    %   A(5,:) → ṙ 方程:  N_β(方向静稳定性!), N_p(交叉导数), N_r(偏航阻尼)
    %
    % L_β 是最关键的横侧向参数：负值表示有上反角效应（侧滑时产生恢复滚转力矩）
    % N_β 是方向静稳定性：正值表示风标稳定性（侧滑时机头转向侧滑方向）
    fprintf('--- LOFI 阶段4/4: 横侧向矩阵 ---\n');
    A_lateral_lo = mat_lo(idx_lat_state, idx_lat_state);
    B_lateral_lo = mat_lo(idx_lat_state, idx_lat_ctrl);
    fprintf(fid, '[LOFI 阶段4] 横侧向矩阵\n\n');
    print_matrix(fid, '横侧向 A 矩阵 (5x5)', A_lateral_lo, lat_sn, lat_sn);
    print_matrix(fid, '横侧向 B 矩阵 (5x2)', B_lateral_lo, lat_sn, lat_cn);

    disp('  LOFI 纵向 A:'), disp(A_longitude_lo);
    disp('  LOFI 横侧向 A:'), disp(A_lateral_lo);


    %% ===================== HIFI (High Fidelity) =====================
    % HIFI 特点：
    %   1. 从 .dat 文件读取完整风洞数据，多维线性插值 (mexndinterp.c)
    %   2. dLEF 自动计算: dLEF = f(α, qbar, ps) = (1.38α - 9.05·qbar/ps + 1.45)°
    %      本工况下 dLEF ≈ 2°，这意味着前缘襟翼偏转 2°，改变了翼型弯度
    %   3. 包含 LEF 修正项、分离流修正、完整的交叉导数
    %   4. 气动模型全面：所有系数都是多维查表 (α,β,δe 等)
    %
    % HIFI vs LOFI 差异来源：
    %   1. f 本身不同（气动数据不同）
    %   2. 配平点不同（dLEF 导致配平舵面偏角不同）
    %   → A/B 矩阵必然不同，尤其是纵向气动导数
    fprintf('\n\n==================== HIFI (High Fidelity) ====================\n');
    fprintf(fid, '\n\n==================== HIFI (High Fidelity) ==================\n\n');

    fi_flag_Simulink = 1;

    %% --- HIFI 阶段5：配平 ---
    % 与 LOFI 阶段1 流程相同，但 fi_flag=1
    % trim_fun 内部差异：
    %   - dLEF 自动计算（而非硬编码为 0）
    %   - alpha 约束从 [-10°,20°] 变为 [-20°,90°]
    %   - 气动导数查询路径不同
    %
    % 代价函数在 HIFI 中高维搜索更复杂，可能无法达到 TolFun→EXITFLAG=0
    fprintf('--- HIFI 阶段5/8: 配平 ---\n');
    fprintf(fid, '[HIFI 阶段5] 配平\n');
    fprintf(fid, '  dLEF 自动计算 (f(alpha, qbar, ps))\n\n');

    feval('F16_trim', [], [], [], 'lincompile');
    load_system('F16_trim');
    [UX, FVAL, EXITFLAG, OUTPUT] = fminsearch('trim_fun', UX0, OPTIONS);
    [best_cost_hi, Xdot_hi, xu_hi, uu_hi] = trim_fun(UX);
    feval('F16_trim', [], [], [], 'term');

    fprintf('  HIFI 配平: iter=%d, cost=%.4e, exit=%d\n', ...
        OUTPUT.iterations, best_cost_hi, EXITFLAG);
    fprintf(fid, '  配平完成! 迭代次数: %d, 函数调用: %d\n', ...
        OUTPUT.iterations, OUTPUT.funcCount);
    fprintf(fid, '  最终代价 cost = %.4e, 退出标志 = %d\n\n', best_cost_hi, EXITFLAG);

    trim_state_hi = xu_hi(1:13);
    trim_thrust_hi = uu_hi(1);
    trim_control_hi = [uu_hi(2); uu_hi(3); uu_hi(4)];
    dLEF_hi = uu_hi(5);

    print_trim_state(fid, 'HIFI', xu_hi, uu_hi);

    disp(['    HIFI alpha=' num2str(xu_hi(3)*180/pi) ...
        ' deg, elev=' num2str(uu_hi(2)*180/pi) ' deg, dLEF=' num2str(uu_hi(5)*180/pi) ...
        ' deg, cost=' num2str(best_cost_hi)]);

    %% --- HIFI 阶段6：线性化 ---
    % 与 LOFI 阶段2 流程相同，但使用 HIFI 的配平点
    % fi_flag=1 → Simulink 模型内部走 F16_dyn.c 的 HIFI 分支
    fprintf('\n--- HIFI 阶段6/8: 线性化 ---\n');
    fprintf(fid, '\n[HIFI 阶段6] 线性化 (linmod)\n\n');

    load_system('F16_openloop_linearization');
    assignin('base', 'fi_flag_Simulink', fi_flag_Simulink);  % 模型选择: 0=LOFI, 1=HIFI
    assignin('base', 'init_x', trim_state_hi);               % 初始状态向量 x₀ (13维)
    assignin('base', 'init_u', [trim_thrust_hi; trim_control_hi]);  % 初始控制向量 u₀ (4维)
    assignin('base', 'init_dlef', dLEF_hi);                  % 前缘襟翼偏角 (rad), HIFI 中自动计算≈2°
    [A_hi, B_hi, C_hi, D_hi] = linmod('F16_openloop_linearization', ...
        trim_state_hi, [trim_thrust_hi; trim_control_hi]);
    close_system('F16_openloop_linearization', 0);
    mat_hi = [A_hi, B_hi; C_hi, D_hi];

    fprintf(fid, '  线性化完成: A(%dx%d) B(%dx%d)\n\n', ...
        size(A_hi,1), size(A_hi,2), size(B_hi,1), size(B_hi,2));

    %% --- HIFI 阶段7：纵向矩阵 ---
    % 与 LOFI 阶段3 使用完全相同的索引，但矩阵元素值不同
    % 关键差异预测（实际见输出）：
    %   M_α(HIFI) ≈ -12.6 vs M_α(LOFI) ≈ -2.2 → 静稳定性增大 5.8 倍
    %     因为 dLEF=2° 使气动中心后移，增加了纵向静稳定裕度
    %   X_α(HIFI) ≈ -2.1 vs X_α(LOFI) ≈ +4.5 → 符号反转！
    %     LOFI 中迎角增大→速度增加（物理上可疑），HIFI 修正为迎角增大→减速（合理）
    fprintf('--- HIFI 阶段7/8: 纵向矩阵 ---\n');
    A_longitude_hi = mat_hi(idx_long_state, idx_long_state);
    B_longitude_hi = mat_hi(idx_long_state, idx_long_ctrl);
    fprintf(fid, '[HIFI 阶段7] 纵向矩阵\n\n');
    print_matrix(fid, '纵向 A 矩阵 (5x5)', A_longitude_hi, long_sn, long_sn);
    print_matrix(fid, '纵向 B 矩阵 (5x2)', B_longitude_hi, long_sn, long_cn);

    %% --- HIFI 阶段8：横侧向矩阵 ---
    % 横侧向差异预计较小（< 10-15%）：
    % dLEF 主要改变翼型弯度（位于 xz 对称面内），对横向气动影响有限
    fprintf('--- HIFI 阶段8/8: 横侧向矩阵 ---\n');
    A_lateral_hi = mat_hi(idx_lat_state, idx_lat_state);
    B_lateral_hi = mat_hi(idx_lat_state, idx_lat_ctrl);
    fprintf(fid, '[HIFI 阶段8] 横侧向矩阵\n\n');
    print_matrix(fid, '横侧向 A 矩阵 (5x5)', A_lateral_hi, lat_sn, lat_sn);
    print_matrix(fid, '横侧向 B 矩阵 (5x2)', B_lateral_hi, lat_sn, lat_cn);

    disp('  HIFI 纵向 A:'), disp(A_longitude_hi);
    disp('  HIFI 横侧向 A:'), disp(A_lateral_hi);


    %% ===================== 保存结果 =====================
    % 保存所有变量到 .mat 文件（供 step2 加载分析）
    % 同时关闭 .txt 文件
    fprintf(fid, '\n============================================================\n');
    fprintf(fid, '  步骤1完成 (LOFI + HIFI)\n');
    fprintf(fid, '============================================================\n');
    fclose(fid);

    save(result_mat, ...
        'trim_state_lo', 'trim_thrust_lo', 'trim_control_lo', 'dLEF_lo', ...
        'A_lo', 'B_lo', 'C_lo', 'D_lo', ...
        'A_longitude_lo', 'B_longitude_lo', ...
        'A_lateral_lo', 'B_lateral_lo', ...
        'trim_state_hi', 'trim_thrust_hi', 'trim_control_hi', 'dLEF_hi', ...
        'A_hi', 'B_hi', 'C_hi', 'D_hi', ...
        'A_longitude_hi', 'B_longitude_hi', ...
        'A_lateral_hi', 'B_lateral_hi', ...
        'xu_lo', 'uu_lo', 'best_cost_lo', ...
        'xu_hi', 'uu_hi', 'best_cost_hi');

    fprintf('\n========================================\n');
    fprintf('  步骤1完成! LOFI + HIFI 结果已保存到:\n');
    fprintf('    %s\n', result_txt);
    fprintf('    %s\n', result_mat);
    fprintf('========================================\n');

end


%% ===================== 子函数 =====================

function print_trim_state(fid, label, xu, uu)
    % 打印配平状态到文本文件
    % 将弧度转换为角度以利于阅读
    % xu: 13 维状态向量
    %   xu(1)  → V      空速 (m/s)
    %   xu(2)  → β      侧滑角 (rad)
    %   xu(3)  → α      迎角 (rad)
    %   xu(4)  → φ      滚转角 (rad)
    %   xu(5)  → θ      俯仰角 (rad)
    %   xu(6)  → ψ      偏航角 (rad)
    %   xu(7)  → p      滚转角速率 (rad/s)
    %   xu(8)  → q      俯仰角速率 (rad/s)
    %   xu(9)  → r      偏航角速率 (rad/s)
    %   xu(10) → xE     东向位置 (m)
    %   xu(11) → yE     北向位置 (m)
    %   xu(12) → -h     负高度 (m) — 注意符号!
    %   xu(13) → Pow    发动机功率 (%)
    % uu: 控制向量
    %   uu(1) → throttle    油门杆位置 (0~1)
    %   uu(2) → elevator    升降舵偏角 (rad)
    %   uu(3) → aileron     副翼偏角 (rad)
    %   uu(4) → rudder      方向舵偏角 (rad)
    %   uu(5) → dLEF        前缘襟翼偏角 (rad)
    fprintf(fid, '---------- %s 配平状态 ----------\n', label);
    fprintf(fid, '  V      = %.4f m/s\n', xu(1));
    fprintf(fid, '  beta   = %.6f rad (%.4f deg)\n', xu(2), xu(2)*180/pi);
    fprintf(fid, '  alpha  = %.6f rad (%.4f deg)\n', xu(3), xu(3)*180/pi);
    fprintf(fid, '  phi    = %.6f rad (%.4f deg)\n', xu(4), xu(4)*180/pi);
    fprintf(fid, '  theta  = %.6f rad (%.4f deg)\n', xu(5), xu(5)*180/pi);
    fprintf(fid, '  psi    = %.6f rad (%.4f deg)\n', xu(6), xu(6)*180/pi);
    fprintf(fid, '  p      = %.6f rad/s\n', xu(7));
    fprintf(fid, '  q      = %.6f rad/s\n', xu(8));
    fprintf(fid, '  r      = %.6f rad/s\n', xu(9));
    fprintf(fid, '  alt    = %.4f m\n', -xu(12));
    fprintf(fid, '  power  = %.4f %%\n', xu(13));
    fprintf(fid, '\n');
    fprintf(fid, '---------- %s 配平控制量 ----------\n', label);
    fprintf(fid, '  thrust   = %.6f\n', uu(1));
    fprintf(fid, '  elevator = %.6f rad (%.4f deg)\n', uu(2), uu(2)*180/pi);
    fprintf(fid, '  aileron  = %.6f rad (%.4f deg)\n', uu(3), uu(3)*180/pi);
    fprintf(fid, '  rudder   = %.6f rad (%.4f deg)\n', uu(4), uu(4)*180/pi);
    fprintf(fid, '  dLEF     = %.6f rad (%.4f deg)\n', uu(5), uu(5)*180/pi);
    fprintf(fid, '\n');
end

function print_matrix(fid, title, M, row_names, col_names)
    % 打印矩阵到文本文件，带行/列标签（便于与原理指南对照）
    % 使用科学计数法显示矩阵元素
    n = size(M, 1); m = size(M, 2);
    fprintf(fid, '--- %s ---\n', title);
    fprintf(fid, '%14s ', '');
    for j = 1:m, fprintf(fid, '%14s ', col_names{j}); end
    fprintf(fid, '\n');
    for i = 1:n
        fprintf(fid, '%14s ', row_names{i});
        for j = 1:m, fprintf(fid, '%14.6e ', M(i,j)); end
        fprintf(fid, '\n');
    end
    fprintf(fid, '\n');
end
