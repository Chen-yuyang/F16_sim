# F-16 俯仰角闭环控制 — 完整开发总结报告

## 目录

1. [项目概述](#1-项目概述)
2. [控制器原理](#2-控制器原理)
3. [开发历程](#3-开发历程)
4. [最终结果](#4-最终结果)
5. [/claude 目录代码资产梳理](#5-claude-目录代码资产梳理)

---

## 1. 项目概述

### 1.1 实验目标

基于 F-16 非线性动力学模型（C MEX S-Function），设计纵向俯仰角闭环控制器，使飞机在 $V=200$ m/s 飞行条件下，俯仰角 $\theta$ 稳定跟踪 $3°$。

### 1.2 核心控制律

$$u = u_{trim} + K \cdot (x_{ref} - x)$$

其中 $x = [V_t, \alpha, \theta, q, P_{ow}]^T$ 为纵向 5 维状态，$u = [\delta_{th}, \delta_e]^T$ 为控制。

### 1.3 飞行条件与模型

- 高度 $H=5000$ m，速度 $V=200$ m/s
- 配平状态：$\alpha=2°$, $\theta=2°$, $\delta_e=-1.87°$, $\delta_{th}=0.22$
- LOFI（低保真）气动模型：C 数组硬编码，查表自带钳位
- HIFI（高保真）气动模型：.dat 文件 + 多维超立方体插值，严格边界检查

---

## 2. 控制器原理

### 2.1 极点配置法（Pole Placement / Eigenstructure Assignment）

**目标**：求 $K_{2\times5}$ 使 $\lambda_i(A-BK) = \lambda_i^*$（$i=1,\dots,5$）。

**前提**：$(A,B)$ 完全可控，$\text{rank}([B,AB,A^2B,A^3B,A^4B])=5$。

**算法**（`place()`）：Kautsky-Nichols-Van Dooren 鲁棒特征结构配置——
1. 对每个期望极点 $\lambda_i^*$，计算 $\ker([A-\lambda_i^*I,\;B])$ 的基向量
2. 从中选取使闭环特征向量矩阵 $X$ 条件数最小的组合
3. 构造 $X$ 使 $(A-BK)X = X\Lambda^*$，求解 $K$

**优点**：物理含义直接——指定 $\omega_n$（响应快慢）和 $\zeta$（阻尼强弱）。
**局限**：不限制控制能量，可能产生过大增益。

### 2.2 线性二次型调节器（LQR）

**目标**：求 $K$ 最小化 $J = \int_0^\infty (x^T Q x + u^T R u) dt$。

**解**：$K = R^{-1} B^T P$，其中 $P$ 是代数 Riccati 方程 $A^T P + P A - P B R^{-1} B^T P + Q = 0$ 的解。

**$Q,R$ 含义**：$Q_{ii}$ 大 → 强约束状态 $i$ 的偏差；$R_{jj}$ 大 → 惩罚控制 $j$ 的能量。$Q/R$ 比值决定"性能 vs 控制代价"权衡。

**优点**：自动平衡精度与能量。**局限**：需手动调 $Q,R$，无直接频域指标。

### 2.3 手动增益法

直接设定 $K$ 的值（如 $K=[0,0,-0.8,0.1,0;0,0,0,0,0]$），仅用 $\theta$ 和 $q$ 反馈。原因：`place()` 在 HIFI 模型上给出过大的 $K$（$K_{2,3}=-2.48\sim-14.67$），非线性飞机承受不住。

**原理**：$\Delta\delta_e = K_{2,3} \cdot (\theta_{ref}-\theta) + K_{2,4} \cdot q$，即比例-微分（PD）控制。$K_{2,3}<0$ 使 $\theta$ 偏低时 elevator 负偏（抬头），$K_{2,4}$ 提供俯仰速率阻尼。

---

## 3. 开发历程

### 3.1 第一轮（lab3_0520）—— 完全失败

**尝试**：修改 `F16_dyn.c` 将连续模型改为离散、3 端口改为 5 端口、内部控制。
**失败原因**：破坏向后兼容，`F16_trim.slx` 等无法加载。最终 `git checkout` 还原。

### 3.2 第二轮（lab3_0520 后半程）—— 首次跑通

**尝试**：保留原始 3 端口 F16_dyn，外部反馈，使用 `place()` 设计 $K$。
**结果**：开环正常，闭环 $\theta$ 从 $2°$ 跌至 $-12.7°$。
**原因误判**：以为是 $K_{2,3}$ 符号错误。
**实际根因**：Model Switch 块的 `sw` 参数在 `sim()` 中无法更新——所有"闭环"测试实际跑的是开环。

### 3.3 第三轮（lab3_0521）—— 反馈链路修复

**关键发现**：
1. 升降舵阶跃实验：$+0.05$ rad → $\theta$ 跌至 $-50.7°$（正 elevator = 低头），$-0.05$ rad → $\theta$ 升至 $+33.5°$（负 elevator = 抬头）。确认 $K_{2,3}<0$ 方向正确
2. 用 `Product` 块（$K\cdot error \times fb\_gain$）替代 Manual Switch，解决信号路由问题

**结果**：LOFI + Place 首次跑通。$\theta_{ss}=2.31°$，误差 $0.69°$。Place 与 LQR 几乎等价（差 $0.03°$）。

**"精确平衡解"方案失败**：增广系统求出的 $(x_d,u_d)$ 让线性模型 $\dot{x}=0$，但 $\alpha_d=-7.7°$ 处非线性飞机剧烈发散。

### 3.4 第四轮（lab3_0525）—— HIFI 模型诊断

**尝试**：3 控制器（Place/LQR/手动）× 2 模型（LOFI/HIFI）= 6 组全组合测试。
**结果**：LOFI 全部通过，HIFI 全部在 $0.18$ s 崩溃（`getHyperCube` 越界）。

**深度诊断**：
- LOFI 查表自带钳位：`if(k<=-2)k=-1`，永不越界
- HIFI 的 `getHyperCube` 严格检查 `x<xmin || x>xmax`，无钳位
- $\alpha$ 和 $\beta$ 是 S-Function 内部状态变量，Simulink 无法拦截
- HIFI 表格范围：$\alpha\in[-20°,+90°]$，$\beta\in[-30°,+30°]$，$\delta_e\in[-25°,+25°]$
- $1°$ 的 $\theta$ 偏差 × $K_{2,3}=+14.67$ = $14.7°$ 的 elevator 阶跃 → 状态瞬间甩出表格

### 3.5 第五轮（lab3_0525_2）—— 伺服作动器 + HIFI 跑通

**关键发现**：参考 `F16_openloop.slx`，发现标准模型包含**升降舵伺服作动器**：
- Rate Limiter：$\pm60°/s$
- Transfer Fcn：$\frac{20.2}{s+20.2}$（$\tau\approx0.05$ s 一阶延迟）
- 我们的 `build_model` 从未包含此模块

**尝试过程**：

| 尝试 | 方法 | 结果 |
|------|------|------|
| 1 | HIFI + place() + 伺服 | $t=1.03$ s 崩溃（之前 $0.18$ s，提升 $5.7\times$）|
| 2 | HIFI + 超温和极点 + 伺服 | 仍崩溃，$K$ 依然过大 |
| 3 | HIFI + LQR + 伺服 | 仍崩溃 |
| 4 | **HIFI + 手动 $K=-0.1$ + 伺服** | ✅ **首次跑通！** $\theta:2°\to2.5°$ |
| 5 | HIFI + 手动 $K=-0.8$ + 伺服 | ✅ $\theta:2°\to3.0°$，误差仅 $0.1°$ |

**伺服改善机制**：$14.7°$ 的阶跃被速率限制为 $0.25$ s 斜坡（$14.7/60$），再经过 $\tau=0.05$ s 一阶平滑，大幅降低初始 $\dot{\alpha}$ 瞬态，使 HIFI 状态保持在表格范围内。

### 3.6 第六轮（R2024b 适配）—— 全线崩溃修复 + 积分控制重设计

R2024b MATLAB 上所有含伺服的方法均崩溃（F16_dyn Access Violation），place_i 的 Ki 符号错误+数值爆炸。经逐个模块隔离测试后修复：

| Bug | 症状 | 根因 | 修复 |
|-----|------|------|------|
| RL 初始条件 | 开环 theta 从 2 俯冲至 -12 deg | RL 默认输出 0，激励无阻尼 phugoid | 移除 RL |
| TF 初始条件 | 舵面从 0 启动，偏离配平 | R2024b TF 块删除 InitialConditions | Integrator+Gain 搭 G(s)=20.2/(s+20.2) |
| sum_elev 端口悬空 | 非积分模式构建失败 | Inputs=++++ 无条件 | 动态设置 +++/++++ |
| place_i Ki 爆炸 | K(2,3)=+8.03(符号错) Ki(2)=+20.18 | 增广系统 place() 数值病态 | 两步法：标准K + 手动 Ki=K(2,3)*0.15 |

伺服架构（无 Rate Limiter）：
  sum_elev -> servo_K_fwd(20.2) -> servo_sum(+) -> servo_int(IC=trim) -> sat_elev
                                     servo_sum(-) <- servo_K_fbk(20.2) <-|

积分控制两步法：1)标准 5 阶设计 K  2)Ki(2)=K(2,3)*0.15（同号,积分时间~6.7s）

---
## 4. 最终结果

### 4.1 可工作的方案（R2024b 验证）

| 方案 | 模型 | 方法 | K(2,3) | theta_ss | 误差 | 状态 |
|------|------|------|--------|:---:|------|:---:|
| A | LOFI | place() | -4.61 | 2.30 deg | 0.70 deg | OK |
| B | LOFI | lqr() | -16.89 | 2.27 deg | 0.73 deg | OK |
| C | LOFI | place_i | -4.61 | 2.63 deg | 0.37 deg | OK 积分有效 |
| D | LOFI | lqr_i | -16.89 | 2.65 deg | 0.35 deg | OK 积分有效 |
| E | HIFI | manual K=-0.8 | -0.80 | ~2.99 deg | ~0.01 deg | OK |

### 4.2 积分控制：增广系统法 vs 两步法

#### 增广系统法（旧，R2024b 上失败）

把积分状态 $x_I = \int(\theta_{ref}-\theta)dt$ 塞进系统矩阵，让 `place()`/`lqr()` 一次性算出 $K$ 和 $K_i$：

$$\underbrace{\begin{bmatrix} \dot{x} \\ \dot{x}_I \end{bmatrix}}_{\dot{x}_{aug}} = \underbrace{\begin{bmatrix} A_{5\times5} & 0_{5\times1} \\ -C_\theta & 0 \end{bmatrix}}_{A_{aug}} \begin{bmatrix} x \\ x_I \end{bmatrix} + \underbrace{\begin{bmatrix} B_{5\times2} \\ \mathbf{0, 0} \end{bmatrix}}_{B_{aug}} u$$

其中 $C_\theta = [0, 0, 1, 0, 0]$ 是 **输出矩阵**——它从 5 维状态 $x=[V,\alpha,\theta,q,P_{ow}]^T$ 中提取 $\theta$：
$$C_\theta \cdot x = 0\cdot V + 0\cdot\alpha + 1\cdot\theta + 0\cdot q + 0\cdot P_{ow} = \theta$$

然后对 6×6 增广系统做极点配置：
```matlab
K_aug = place(A_aug, B_aug, [5个原极点, 积分极点]);
K   = K_aug(:, 1:5);   % 状态反馈
Ki  = K_aug(:, 6);     % 积分增益
```

**失败原因**：$B_{aug}$ 最后一行是 $[0, 0]$——**积分状态 $x_I$ 没有直接的控制输入**。升降舵和油门都无法直接影响 $x_I$，只能通过 $C_\theta$ 影响 $\theta$，再经 A 矩阵内部耦合"隔山打牛"传回来。这导致：

1. 增广可控性矩阵 $\text{ctrb}(A_{aug}, B_{aug})$ 的**条件数极大**（接近奇异）
2. `place()` 内部解 $K_{aug}=UX^{-1}$ 时，特征向量矩阵 $X$ 接近奇异 → 数值爆炸
3. 求出的 $K_i$ **符号随机、数值失控**

实测（`place_i`, 积分极点=-0.5）：
```
K(2,3) = +8.03    (应该是负数! K 本身就被污染了)
Ki(2)  = +20.18   (合理值约 -0.7, 差30倍, 符号还反了)
闭环    = 不稳定
```

#### 两步法（新，R2024b 验证通过）

$K$ 和 $K_i$ **分开设计**，互不干扰：

**第 1 步**：$K$ 照常用标准 5 阶系统设计（与普通 `place`/`lqr` 完全相同）——
```matlab
p_des = [-2.1+2.14i, -2.1-2.14i, -0.8+0.6i, -0.8-0.6i, -10];
K = place(A, B, p_des);   % 5×5 系统, 数值优良
```
这步不受积分影响，`place()` 在健康的 5×5 系统上稳定可靠。

**第 2 步**：$K_i$ 手动设定——
```matlab
Ki(2) = K(2,3) * 0.15;   % 符号自动跟随 K(2,3), 保证负反馈
```
- 符号：与 $K(2,3)$ 同号 → $\theta$ 偏低时积分累积推升舵面抬头 → 负反馈
- 大小：`alpha=0.15` → 积分时间 ≈ 6.7s → 温和稳定，不会激励振荡
- 不会出现增广系统那种"为满足不合理极点而炸出巨大增益"

#### 对比总结

| | 增广系统法 | 两步法 |
|------|:---:|:---:|
| K 来源 | `place(A_aug, B_aug)` 6阶 | `place(A, B)` 5阶 |
| Ki 来源 | `K_aug(:,6)` 自动算 | `K(2,3) * 0.15` 手动 |
| 数值条件 | 极差（可控性矩阵近奇异） | 优良（标准5阶系统） |
| K 被污染？ | 是（K(2,3) 符号都可能错） | 否（K 和普通方法完全一致） |
| Ki 符号 | 随机（实测出现正号） | 自动正确（跟随 K(2,3)） |
| Ki 数值 | 爆炸（实测 +20.18） | 可控（-0.69） |
| 实际结果 | 闭环不稳定，崩溃 | θ_ss=2.63°, 误差 0.37° |
| 积分收敛速度 | — | alpha 可调（0.15 ≈ 6.7s） |

### 4.3 未解决的问题

### 4.4 经验教训

1. **Simulink 的 Manual Switch 不可靠**：编译时锁定，`set_param` 无法在 `sim()` 中生效。用 Product 块替代
2. **作动器伺服模型不可或缺**：真实舵机有延迟和速率限制，忽略它们会导致物理上不可能的瞬态
3. **线性模型的精确解 ≠ 非线性模型的平衡点**：一阶泰勒展开只在局部有效
4. **LOFI 的"宽容"是双刃剑**：永不崩溃但也隐藏了气动模型缺陷
5. **HIFI 的"严格"是安全设计**：宁可崩溃也不返回物理错误的外插值

---

## 5. Simulink 模型模块详解 (f16_build_simulink.m / build_model.m)

本项目的 Simulink 模型完全由 MATLAB 代码以编程方式构建（Programmatic API），不依赖现有 .slx 模板。模型采用四列布局，信号从左向右流动。

### 5.1 整体架构

```
信号源(列1) → 求和+作动器伺服(列2) → F16_dyn S-Function(列3) → 控制器(列4)
                                                                     ↓
接受全部 13 维状态 → Selector 提取纵向 5 维 → 求误差 → K 矩阵 → Product → Demux
→ δth 反馈 → 油门通路 · δe 反馈 → 升降舵通路
```

控制律：$u(1:2) = u_{base}(1:2) + fb\_gain \cdot K_{long} \cdot (x_{ref\_long} - x_{long})$

### 5.2 模块清单（共 29 个）

#### 列1: 信号源 (9 个模块)

| # | 模块类型 | 模块名 | 参数/变量 | 位置 [l,t,r,b] | 作用 |
|:-:|---------|--------|----------|:------------:|------|
| 1 | Constant | `thrust_base` | `init_u_base(1)` | [50,35,160,65] | 油门基线信号（配平值 δth≈0.22） |
| 2 | Constant | `elev_base` | `init_u_base(2)` | [50,85,160,115] | 升降舵基线信号（配平值 δe≈-0.033rad） |
| 3 | Constant | `ail_base` | `init_u_base(3)` | [50,135,160,165] | 副翼基线信号（纵向控制中不变） |
| 4 | Constant | `rud_base` | `init_u_base(4)` | [50,185,160,215] | 方向舵基线信号（纵向控制中不变） |
| 5 | Constant | `dlef` | `init_dlef` | [50,255,160,285] | 前缘襟翼偏角（S-Function 第二输入） |
| 6 | Constant | `fi_flag` | `fi_flag_Simulink` | [50,305,160,335] | HIFI模型标志（0=LOFI, 1=HIFI, S-Function 第三输入） |
| 7 | From Workspace | `elev_dist` | `elevator_disturb` | [50,355,160,385] | 升降舵扰动信号（两列矩阵 [时间,幅值], 5° doublet） |
| 8 | Constant | `x_ref` | `x_ref_long` | [50,405,160,445] | 5维参考状态 [V, α, θ, q, Pow]' |
| 9 | Constant | `fb_gain` | `fb_gain` | [50,465,160,495] | 反馈使能开关（0=开环, 1=闭环） |

#### 列2: 求和 + 作动器伺服 (6 个模块)

| # | 模块类型 | 模块名 | 参数 | 位置 [l,t,r,b] | 作用 |
|:-:|---------|--------|------|:------------:|------|
| 10 | Sum | `sum_thrust` | Inputs=`++`, round | [250,35,280,70] | 油门基线 + 油门反馈求和 |
| 11 | Saturation | `sat_thrust` | Upper=1, Lower=0 | [320,35,360,70] | 油门限幅 [0,1], 防止负油门或超100% |
| 12 | Sum | `sum_elev` | Inputs=`+++`, round | [250,85,280,130] | 升降舵基线 + 扰动 + 反馈三输入求和 |
| 13 | **Rate Limiter** | `elev_rate` | Rising=+1.047, Falling=-1.047 | [320,85,360,130] | **舵机速率限制**: ±60°/s(=±1.047rad/s), 模拟液压舵机物理约束 |
| 14 | **Transfer Fcn** | `elev_servo` | Num=[20.2], Den=[1 20.2], IC=init_u_base(2) | [400,85,450,140] | **舵机伺服响应**: G(s)=20.2/(s+20.2), τ=0.05s, 初始条件为配平值 |
| 15 | Saturation | `sat_elev` | Upper=0.44, Lower=-0.44 | [470,85,510,130] | 升降舵物理限位 ±0.44rad(±25°) |

**伺服作动器详解**（模块 #13-#14）：

真实 F-16 的升降舵由液压舵机驱动，舵机有两大物理限制：
1. **速率限制**（Rate Limiter）：舵面偏转不能超过 ±60°/s。这意味着即使控制器瞬间输出 14.7° 的指令，舵面也需要 0.25s 才能到达目标位置。这个"斜坡"效应大幅降低了初始瞬态对飞机状态的冲击。
2. **伺服滞后**（Transfer Fcn）：液压伺服阀的电气-机械响应存在延迟，一阶模型 $G(s)=20.2/(s+20.2)$ 的时间常数 τ=0.05s（带宽约 3.2Hz）。初始条件设为配平值，使仿真起始时舵面在正确位置。

没有伺服时，控制器输出直接作用到飞机，14.7° 的阶跃瞬间产生巨大的 α̇ 和 β̇ 瞬态，在 HIFI 模型中立即越界。**伺服是 HIFI 能够跑通的关键。**

#### 列3: Mux + F16_dyn (2 个模块)

| # | 模块类型 | 模块名 | 参数 | 位置 [l,t,r,b] | 作用 |
|:-:|---------|--------|------|:------------:|------|
| 16 | Mux | `ctrl_mux` | Inputs=4, bar | [560,30,600,180] | 将 [δth, δe, δa, δr] 合成4维控制向量 |
| 17 | **S-Function** | `F16_dyn` | FunctionName=F16_dyn, Parameters=init_x | [680,30,800,230] | **F-16 非线性气动模型**: 输入4维控制+dlef+fi_flag, 输出13维状态 |

**F16_dyn 详解**（模块 #17）：
- **输入端口 1**：4 维控制向量 [δth(油门), δe(升降舵), δa(副翼), δr(方向舵)]
- **输入端口 2**：前缘襟翼偏角 dlef（标量）
- **输入端口 3**：HIFI 标志 fi_flag（0=LOFI, 1=HIFI）
- **参数**：init_x(13×1) — 13 维初始状态
- **输出**：13 维状态向量 [V, β, α, φ, θ, ψ, p, q, r, xE, yE, -h, Pow]'
- **引擎**：C MEX S-Function，编译自 Fortran 气动代码

#### 列4: 控制器 (5 个模块)

| # | 模块类型 | 模块名 | 参数 | 位置 [l,t,r,b] | 作用 |
|:-:|---------|--------|------|:------------:|------|
| 18 | Selector | `state_sel` | Indices=[1,3,5,8,13], InputWidth=13 | [870,40,940,110] | 从13维全状态提取纵向5维子集 [V,α,θ,q,Pow] |
| 19 | Sum | `err_sum` | Inputs=`+-`, round | [990,50,1020,110] | 计算误差: e = x_ref_long - x_long（负反馈） |
| 20 | Gain | `K_gain` | Gain=K_long, Matrix(K*u) | [1070,50,1120,110] | 状态反馈增益矩阵 2×5, 输出 u_fb = K_long * e (2维) |
| 21 | Product | `fb_enable` | Inputs=`**`, Element-wise(.*) | [1150,60,1180,100] | 反馈使能开关: K_gain输出 × fb_gain (0=断路, 1=导通) |
| 22 | Demux | `fb_demux` | Outputs=2, bar | [1220,60,1250,110] | 分解2维反馈: 输出1→油门, 输出2→升降舵 |

**Product 模块 vs Manual Switch**（模块 #21）：
初期版本使用 Manual Switch 做开环/闭环切换，但 Manual Switch 的 `sw` 参数在 `sim()` 中只读取一次且在编译时锁定——仿真开始后无法通过 `set_param` 改变。这意味着先跑开环（fb_gain=0）再跑闭环（fb_gain=1）时需要重建整个模型。
Product 块接受 fb_gain 作为第二输入：0 时乘积为零（开环），1 时乘积不变（闭环）。fb_gain 来自 base workspace 的 Constant 模块，修改 `assignin('base','fb_gain',1)` 后只需重建模型即可生效。

#### 输出与示波器 (3 类)

| # | 模块类型 | 模块名 | 参数 | 位置 | 作用 |
|:-:|---------|--------|------|:---:|------|
| 23 | Out1 | `states_out` | — | [870,280,910,315] | 13维状态输出到 MATLAB 工作空间（sim() 返回值收集点） |
| 24 | Demux | `scope_demux` | Outputs=13, bar | [980,280,1010,490] | 将13维状态分解为13个标量信号供示波器显示 |
| 25a-e | Scope (×5) | `Scope_V/alpha/theta/q/h` | OpenAtSimulationStart=off | [1050,280~440] | 5个示波器分别显示 V, α, θ, q, h 的实时波形 |

### 5.3 信号流详细路径

```
油门通路:
  thrust_base → [sum_thrust: + fb_demux(1)] → sat_thrust → ctrl_mux(1)

升降舵通路:
  elev_base → [sum_elev: + elev_dist + fb_demux(2)] → elev_rate(±60°/s)
  → elev_servo(20.2/(s+20.2)) → sat_elev(±0.44) → ctrl_mux(2)

副翼/方向舵(纵向固定):
  ail_base → ctrl_mux(3)
  rud_base → ctrl_mux(4)

S-Function:
  ctrl_mux(4合1) → F16_dyn(端口1)
  dlef → F16_dyn(端口2)
  fi_flag → F16_dyn(端口3)
  F16_dyn(输出13维) → [state_sel, states_out, scope_demux]

控制器:
  x_ref → err_sum(+)
  state_sel(5维) → err_sum(-)
  err_sum → K_gain(2×5矩阵) → fb_enable(×fb_gain) → fb_demux
  fb_demux(1) → sum_thrust(+)
  fb_demux(2) → sum_elev(+)
```

### 5.4 布局坐标总览

```
列1(信号源, x=50-160)      列2(伺服, x=250-510)    列3(S-Func, x=560-800)   列4(控制器, x=870-1250)
────────────────────────    ───────────────────    ─────────────────────    ────────────────────
thrust_base(35)             sum_thrust(35)                                          state_sel(40)
elev_base(85)               sat_thrust(35)                                          err_sum(50)
ail_base(135)               sum_elev(85)                    ctrl_mux(30)            K_gain(50)
rud_base(185)               elev_rate(85)                   F16_dyn(30)             fb_enable(60)
dlef(255)                   elev_servo(85)                                          fb_demux(60)
fi_flag(305)                sat_elev(85)
elev_dist(355)
x_ref(405)
fb_gain(465)
```

所有模块的垂直坐标(Y)列在括号中（单位：像素）。两模块间用 `add_line` 连接信号线，形成完整的信号流。

### 5.5 BUG 修复记录 (2026-05-27)

在测试积分控制和 LQR 方法时发现 MATLAB 直接崩溃的问题，经过诊断发现以下三个独立 Bug，均已修复：

| # | Bug | 根因 | 修复 | 影响 |
|:-:|------|------|------|:---:|
| 1 | **sum_elev 端口悬空** | `sum_elev` 的 `Inputs='++++'` 是无条件的。非积分模式(`use_integral=false`)下只连了 3 个端口，第 4 端口悬空 → Simulink 报错 | `Inputs` 根据 `use_integral` 动态设为 `+++` 或 `++++` | place/lqr 所有非积分方法 |
| 2 | **Rate Limiter 参数名错误** | 使用 `RisingSlewLimit/FallingSlewLimit`（仅 R2024b+ 支持），旧版 MATLAB 需用 `RisingSlewRate/FallingSlewRate` → `set_param` 报错 | 改回跨版本通用的 `RisingSlewRate/FallingSlewRate` | 所有 LOFI+HIFI 控制器 |
| 3 | **伺服初始条件为 0** | 用 State-Space 块实现伺服，未设初始条件（默认 0 ≠ 配平值 δe≈-0.033rad），仿真起始瞬态可能越界 | 改回 Transfer Fcn 块 + `InitialConditions='init_u_base(2)'` | HIFI 模型 |

此外，积分路径在开环模式下未被 `fb_gain` 门控的问题也一并修复（新增 `Ki_enable` Product 块，已在上方模块清单中更新为 26 个模块）。

---

## 6. /claude 目录代码资产梳理

### 6.1 目录结构总览

```
claude/
├── README.md                          # 项目总文档
├── 01_项目概述与架构.md                # 项目架构说明
├── 02_动力学模型详解.md                # F-16 动力学原理
├── 03_配平系统分析.md                  # 配平方法分析
├── 04_线性化与控制器设计.md            # 线性化+K设计理论
├── 05_Simulink模型图.md               # 模型结构图
├── 06_数据流与依赖关系.md              # 数据依赖图谱
├── 07_实验课程与使用指南.md            # 课程指南
├── F16_dyn.c_逐行解析与模块交互分析.md  # F16_dyn.c 逐行解析
├── 符号速查手册.md                     # LaTeX 符号速查
│
├── lab3_0520/                         # ★ 第一版 — 废弃（架构错误）
│   ├── build_F16_closedloop_model.m    #   旧版构建脚本（Manual Switch 架构）
│   ├── demo_closedloop.m               #   旧版演示脚本
│   ├── f16_pitch_control_design.m      #   旧版设计脚本
│   ├── verify_trim.m                   #   配平验证脚本 ★ 可复用
│   ├── problem_analysis.md             #   问题分析报告（K 符号分析）
│   ├── weekly_report.md                #   周报
│   ├── README.md                       #   操作手册
│   ├── lab3_controller.mat             #   控制器数据
│   └── test_models/                    #   22 个中间模型文件
│
├── lab3_0521/                         # ★ 第二版 — 反馈链路修复（可工作）
│   ├── f16_longitudinal_model.m        #   A/B 矩阵加载 + 物理含义展示 ★
│   ├── f16_stability_analysis.m        #   开环稳定性分析 ★
│   ├── f16_controller_design.m         #   控制器设计（place(), lqr()）★
│   ├── f16_build_simulink.m            #   Simulink 构建（Product 块）★
│   ├── f16_demo.m                      #   一键演示 ★
│   ├── f16_trim_verify.m               #   配平验证脚本
│   ├── failure_analysis_and_compliance.md  # 失败根因 + 实验符合性报告
│   ├── 总结报告.md                     #   中文总结
│   ├── README.md                       #   操作手册
│   ├── lab3_controller.mat             #   控制器数据
│   └── test_models/                    #   33 个中间模型文件
│
├── lab3_0525/                         # ★ 第三版 — 多方案对比（最佳分析报告）
│   ├── design_controller.m             #   统一控制器设计（Place/LQR, LOFI/HIFI, 新旧方案）★
│   ├── build_model.m                   #   Simulink 构建（Product 块）★
│   ├── run_all_tests.m                 #   全组合自动化测试 ★
│   ├── analysis_report.md              #   ★ 最全面的分析报告（含 HIFI 插值机制详解）
│   ├── all_results.mat                 #   测试结果数据
│   └── test_models/                    #   20 个中间模型文件
│
└── lab3_0525_2/                       # ★ 第四版 — 含伺服作动器（HIFI 跑通）
    ├── f16_longitudinal_model.m        #   加载 A/B 矩阵 + 物理含义 ★
    ├── f16_stability_analysis.m        #   开环稳定性分析 ★
    ├── f16_controller_design.m         #   控制器设计(place/lqr/manual/place_i/lqr_i) ★
    ├── f16_build_simulink.m            #   Simulink 构建(含伺服+积分选项) ★
    ├── f16_demo.m                      #   一键演示 ★
    ├── f16_integral_test.m             #   积分控制参数调试与对比测试 ★
    ├── design_controller.m             #   统一控制器设计(旧版兼容) ★
    ├── build_model.m                   #   Simulink 构建(旧版兼容) ★
    ├── run_tests.m                     #   测试脚本 ★
    ├── final_report.md                 #   本报告 ★
    ├── results.mat                     #   测试结果数据
    ├── results_integral.mat            #   积分控制测试结果
    └── test_models/                    #   中间模型文件
```

### 6.2 各版本核心差异

| 版本 | 主要问题 | 架构特点 | 可工作？ |
|------|---------|---------|:---:|
| lab3_0520 | 改 C 代码破坏兼容性 | 5 端口离散模型 | ❌ |
| lab3_0521 | Manual Switch 信号路由断裂 | Product 块替代 Switch | ✅ LOFI |
| lab3_0525 | HIFI 缺伺服作动器 | 同 0521 架构 | ✅ LOFI |
| lab3_0525_2 | 添加伺服模型 | Rate Limiter + Transfer Fcn | ✅ **LOFI + HIFI** |

### 6.3 推荐使用的脚本

**最简单上手（LOFI 闭环）**：
```matlab
cd('e:\...\claude\lab3_0521');
addpath(genpath('../..'));
f16_controller_design('LOFI');
f16_build_simulink('MyModel');
assignin('base','fb_gain',1);
out = sim('MyModel');
```

**HIFI 闭环（需伺服模型）**：
```matlab
cd('e:\...\claude\lab3_0525_2');
addpath(genpath('../..'));
r = design_controller('HIFI','place',false);
assignin('base','K_long',[0 0 0 0 0; 0 0 -0.8 0.1 0]);
assignin('base','fb_gain',1);
build_model('HIFI_CL');
out = sim('HIFI_CL');
```

**含展示和报告的完整分析**：
参考 `lab3_0525/analysis_report.md` —— 包含 Place vs LQR 原理、HIFI 插值机制逐行分析、气动角越界物理后果、伺服作动器分析等。

### 6.4 文档资产

| 文件 | 内容 |
|------|------|
| `claude/01~07_*.md` | F-16 项目架构、动力学、配平、线性化、Simulink、数据流、课程指南 |
| `claude/F16_dyn.c_逐行解析*.md` | F16_dyn.c 源码级注释 |
| `lab3_0520/problem_analysis.md` | $K_{2,3}$ 符号错误根源分析 |
| `lab3_0520/weekly_report.md` | 第一轮开发周报 |
| `lab3_0521/failure_analysis_and_compliance.md` | 三层根因 + 实验要求逐条核查 |
| `lab3_0525/analysis_report.md` | **最全面报告**：Place/LQR 原理、HIFI 插值详解、$1°$ 偏差为何越界、伺服发现 |
| `lab3_0525_2/final_report.md` | 本报告：开发全历程 + 代码资产梳理 |

---

## 6. 详细使用方法

### 6.1 环境准备

在 MATLAB 中执行以下命令进行一次性环境配置：

```matlab
% 编译 MEX（仅首次需要）
cd('e:\matlab_script\实验课资料\FC_SimCode_1');
mex F16_dyn.c

% 配平与线性化（仅首次需要，约 30~60 秒）
lab1_step1_trim_and_linearize();
% 生成: trae/lab1_0429/lab1_matrices.mat
```

### 6.2 一键演示（推荐入门）

```matlab
cd('e:\matlab_script\实验课资料\FC_SimCode_1\claude\lab3_0525_2');
f16_demo;
```

执行后自动完成 7 个步骤，并在终端输出结果，弹窗显示开环/闭环对比曲线。

**修改配置**：打开 `f16_demo.m`，编辑文件顶部的用户配置区：

```matlab
%% ====== 用户配置区 ======
MODEL  = 'LOFI';      % 改为 'HIFI' 使用高保真模型
METHOD = 'place';     % 可选: 'place' / 'lqr' / 'manual'
USE_EXACT = false;    % false=旧方案(稳定) / true=精确平衡解(实验要求, 但非线性模型上不可行)
manual_K  = [0 0 0 0 0; 0 0 -0.8 0.1 0];  % 仅在 METHOD='manual' 时生效
% =========================
```

### 6.3 各配置的推荐组合

| 模型 | METHOD | USE_EXACT | manual_K | 预期效果 |
|------|--------|-----------|----------|---------|
| `'LOFI'` | `'place'` | `false` | 忽略 | ✅ 稳态误差 0.69° |
| `'LOFI'` | `'lqr'` | `false` | 忽略 | ✅ 稳态误差 0.72° |
| `'LOFI'` | `'manual'` | `false` | `[0 0 0 0 0; 0 0 -0.8 0.1 0]` | ✅ 可调 |
| `'HIFI'` | `'manual'` | `false` | `[0 0 0 0 0; 0 0 -0.8 0.1 0]` | ✅ 稳态误差 0.51° |
| `'HIFI'` | `'place'` | `false` | 忽略 | ❌ ~1s 崩溃 |
| 任意 | 任意 | `true` | — | ❌ 非线性模型不支持 |

### 6.4 分步操作（自定义流程）

```matlab
%% 步骤1: 加载模型 + 查看 A/B 矩阵物理含义
cd('e:\matlab_script\实验课资料\FC_SimCode_1\claude\lab3_0525_2');
addpath(pwd);
addpath('e:\matlab_script\实验课资料\FC_SimCode_1');
addpath('e:\matlab_script\实验课资料\FC_SimCode_1\trae\lab1_0429');
addpath('e:\matlab_script\实验课资料\FC_SimCode_1\aerodata');
cd('e:\matlab_script\实验课资料\FC_SimCode_1');

[A, B, x_trim, u_trim, dlef, fi, label] = f16_longitudinal_model('LOFI');

%% 步骤2: 开环稳定性分析
f16_stability_analysis(A, label);

%% 步骤3: 设计控制器
% 选项A: 极点配置
r = f16_controller_design('LOFI', 'place');

% 选项B: LQR
% r = f16_controller_design('LOFI', 'lqr');

% 选项C: 手动 K
% r = f16_controller_design('LOFI', 'manual', [0 0 0 0 0; 0 0 -0.8 0.1 0]);

% 选项D: HIFI + 手动K
% r = f16_controller_design('HIFI', 'manual', [0 0 0 0 0; 0 0 -0.8 0.1 0]);

%% 步骤4: 构建 Simulink 模型
f16_build_simulink('MyModel');
% 模型保存到 test_models/MyModel.slx

%% 步骤5: 开环仿真
assignin('base', 'fb_gain', 0);     % 反馈使能=0
out_ol = sim('MyModel', 'StopTime', '30');
data_ol = out_ol.yout{1}.Values.Data;  % [N×13] 状态矩阵

%% 步骤6: 闭环仿真
assignin('base', 'fb_gain', 1);     % 反馈使能=1
f16_build_simulink('MyModel');       % 重建模型使 fb_gain 生效
out_cl = sim('MyModel', 'StopTime', '30');
data_cl = out_cl.yout{1}.Values.Data;

%% 步骤7: 查看结果
fprintf('开环 θ = %.2f° → %.2f°\n', data_ol(1,5)*180/pi, data_ol(end,5)*180/pi);
fprintf('闭环 θ = %.2f° → %.2f°\n', data_cl(1,5)*180/pi, data_cl(end,5)*180/pi);

% 稳态分析 (取 t∈[10,30]s 平均值)
ss_idx = out_cl.tout >= 10;
theta_ss = mean(data_cl(ss_idx, 5)) * 180 / pi;
fprintf('闭环稳态 θ = %.2f°, 误差 = %.2f°\n', theta_ss, 3.0 - theta_ss);

% 绘图
figure;
plot(out_ol.tout, data_ol(:,5)*180/pi, 'b-', 'LineWidth', 1.2); hold on;
plot(out_cl.tout, data_cl(:,5)*180/pi, 'r-', 'LineWidth', 1.2);
yline(3, 'g--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\theta (°)');
legend('开环', '闭环', '\theta_{ref}=3°');
title('F-16 俯仰角闭环控制'); grid on;

close_system('MyModel', 0);
```

### 6.5 state 数据索引速查

`out.yout{1}.Values.Data` 返回 $N\times13$ 矩阵，各列含义：

| 列 | 符号 | 含义 | 单位 |
|:--:|------|------|------|
| 1 | $V_t$ | 真空速 | m/s |
| 2 | $\beta$ | 侧滑角 | rad |
| 3 | $\alpha$ | 迎角 | rad |
| 4 | $\phi$ | 滚转角 | rad |
| 5 | $\theta$ | 俯仰角 | rad |
| 6 | $\psi$ | 偏航角 | rad |
| 7 | $p$ | 滚转角速率 | rad/s |
| 8 | $q$ | 俯仰角速率 | rad/s |
| 9 | $r$ | 偏航角速率 | rad/s |
| 10 | $x_E$ | 东向位置 | m |
| 11 | $y_E$ | 北向位置 | m |
| 12 | $-h$ | 负高度 | m |
| 13 | $P_{ow}$ | 发动机功率 | % |

俯仰角转度数：`data(:,5) * 180 / pi`

### 6.6 调整控制器的参数

**调整极点位置**（修改 `f16_controller_design.m` 第 157 行的 `p_des`）：

```matlab
% 原值: 短周期 ω=3.0 ζ=0.7, 长周期 ω=1.0 ζ=0.8, 发动机 -10
p_des = [-2.1+2.14i, -2.1-2.14i, -0.8+0.60i, -0.8-0.60i, -10];

% 更温和: 极点靠近开环，增益更小，瞬态更平滑
% p_des = [-1.0+1.5i, -1.0-1.5i, -0.3+0.2i, -0.3-0.2i, -1.5];
```

**调整 LQR 权重**（修改 `f16_controller_design.m` 第 55 行的 `Q` 和 `R`）：

```matlab
% 原值
Q = diag([0.1, 1, 100, 10, 0.1]);   % 重点惩罚 θ
R = diag([0.5, 0.5]);                % 适度限制控制量

% 更保守控制: 增大 R，限制控制幅值
% R = diag([2, 2]);

% 更激进控制: 减小 R，允许更大控制幅值
% R = diag([0.1, 0.1]);

% 更关注 θ 跟踪: 增大 Q(3,3)
% Q = diag([0.1, 1, 500, 10, 0.1]);
```

**调整手动 K**（修改 `f16_demo.m` 第 13 行）：

```matlab
manual_K = [0 0 0 0 0; 0 0 -0.8 0.1 0];
%            ↑              ↑    ↑
%           油门通道全为0   Kθ   Kq(俯仰阻尼)
%
% Kθ<0 → θ偏低时 elevator负偏 → 抬头 (方向正确)
% Kq   → 俯仰速率阻尼 (正值=增强阻尼)
%
% 常用值: Kθ∈[-0.2, -2.0], Kq∈[0, 0.3]
```

### 6.7 故障排查

| 症状 | 可能原因 | 解决 |
|------|---------|------|
| 找不到 `lab1_matrices.mat` | 未运行配平 | 运行 `lab1_step1_trim_and_linearize()` |
| 找不到 `F16_dyn` | MEX 未编译 | 运行 `mex F16_dyn.c` |
| `Access Violation` | 状态超出 LOFI 气动表范围 | 减小 K 或极点 |
| `Point lies out data grid` | HIFI 查表越界 | 换 LOFI 模型或减小 K |
| 反馈无效果 | fb_gain=0 | `assignin('base','fb_gain',1)` |
| 函数签名错误 | MATLAB 缓存了旧版本 | 运行 `clear functions` 后重试 |

---

*报告时间：2026-05-27*
*版本：lab3_0525_2 (v4.1 — 积分控制修复 + Simulink 模块详解)*
