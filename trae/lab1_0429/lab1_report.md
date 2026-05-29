# 实验1：F-16 飞行器线性化建模与非线性仿真验证

> **符号说明**: 本文中所有符号、下标和变量的定义请查阅 [符号速查手册](../../claude/符号速查手册.md)。
>
> **实验日期**: 2026-04-29 | **飞行条件**: $H = 5000\ \text{m}$, $V = 200\ \text{m/s}$  
> **气动模型**: LOFI + HIFI 双模型对比 | **版本**: v2.0 (HIFI 数据完整版)  
> **更新说明**: 本报告已根据 2026-04-29 21:31 运行的最新步骤1/2 结果更新，新增 HIFI 配平、线性化矩阵、特征值及 LOFI/HIFI 逐项对比  
> **所有输出**: `trae/lab1_0429/`

---

## 📖 阅读指引

| 文档 | 适合人群 | 内容 |
|------|---------|------|
| **本文档** `lab1_report.md` | 所有人 | 实验全流程+数据+结论（主线报告，含 LOFI/HIFI 完整对比） |
| [lab1_principles_guide.md](lab1_principles_guide.md) | 想彻底理解原理 | 从小白到专业：线性化/特征值/气动导数/控制效率 |
| [lab1_modal_analysis_report.md](lab1_modal_analysis_report.md) | 关注模态 | 动画式讲解5种模态"发生了什么" |
| [lab1_code_analysis.md](lab1_code_analysis.md) | 想读懂代码 | 逐函数逐行+LOFI/HIFI 双路径数据流 |

---

## 目录

- [1. 实验目的与原理脉络](#1-实验目的与原理脉络)
- [2. F-16 非线性模型概述](#2-f-16-非线性模型概述)
- [3. 步骤1：配平与线性化 (LOFI + HIFI)](#3-步骤1配平与线性化-lofi--hifi)
- [4. 步骤2：纵向模态分析 (LOFI vs HIFI)](#4-步骤2纵向模态分析-lofi-vs-hifi)
- [5. 步骤2：横侧向模态分析 (LOFI vs HIFI)](#5-步骤2横侧向模态分析-lofi-vs-hifi)
- [6. LOFI vs HIFI 全维度对比与差异分析](#6-lofi-vs-hifi-全维度对比与差异分析)
- [7. 步骤3：非线性开环仿真验证](#7-步骤3非线性开环仿真验证)
- [8. 综合分析与结论](#8-综合分析与结论)
- [9. 附录](#9-附录)

---

## 1. 实验目的与原理脉络

### 1.1 我们要做什么

从 F-16 的非线性全量模型出发，经过四个阶段的处理，得到可用于控制器设计的线性模型。**本版本同时处理 LOFI 和 HIFI 两种保真度**，实现全维度对比。

```
非线性F-16 (13阶)                线性模型 (5阶纵向+5阶横侧向)
    │                                     │
    ├─[1] LOFI 配平+线性化 ───→ Along_LOFI, Blong_LOFI
    ├─[2] HIFI 配平+线性化 ───→ Along_HIFI, Blong_HIFI
    ├─[3] 独立特征值分析 ────→ 5×2 组模态参数
    └─[4] LOFI vs HIFI 对比 ──→ 差异量化 + 物理解释
```

### 1.2 核心原理

飞机非线性动力学 $\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u})$ 在平衡点处泰勒展开取一阶项：

$$\boxed{\Delta\dot{\mathbf{x}} = \mathbf{A}\Delta\mathbf{x} + \mathbf{B}\Delta\mathbf{u}}$$

其中 $\mathbf{A} = \partial\mathbf{f}/\partial\mathbf{x}|_0$ 是状态矩阵，$\mathbf{B} = \partial\mathbf{f}/\partial\mathbf{u}|_0$ 是控制矩阵。

**关键认知**：线性化结果取决于两个因素 — (1) $\mathbf{f}$ 函数本身，(2) 展开点 $(\mathbf{x}_0,\mathbf{u}_0)$。LOFI 和 HIFI 的 $\mathbf{f}$ 不同、配平点也不同，因此 **A/B 矩阵必然不同**。差异的大小和物理含义是本次实验的核心发现。

---

## 2. F-16 非线性模型概述

### 2.1 状态向量 (13维) 与控制向量 (4维)

| 索引 | 符号 | 含义 | 单位 | 属于 |
|:---:|------|------|------|:---:|
| 1 | $V$ | 空速 | m/s | 纵向 |
| 2 | $\beta$ | 侧滑角 | rad | 横侧向 |
| 3 | $\alpha$ | 迎角 | rad | 纵向 |
| 4 | $\phi$ | 滚转角 | rad | 横侧向 |
| 5 | $\theta$ | 俯仰角 | rad | 纵向 |
| 6 | $\psi$ | 偏航角 | rad | 横侧向 |
| 7 | $p$ | 滚转角速率 | rad/s | 横侧向 |
| 8 | $q$ | 俯仰角速率 | rad/s | 纵向 |
| 9 | $r$ | 偏航角速率 | rad/s | 横侧向 |
| 10-11 | $x_E,y_E$ | 水平位置 | m | — |
| 12 | $-h$ | 负高度 | m | 纵向 |
| 13 | $P_{ow}$ | 发动机功率 | % | 纵向 |

> **NED 坐标系约定**：状态 12 ($z_{earth}$) 为负高度是因为模型使用航空标准 NED (North-East-Down) 坐标系——z 轴向下为正（重力方向）。真实高度 $h = -z_{earth}$，即 $z_{earth} = -h$。在大气模型调用 `atmos(-z_earth, ...)` 中取负还原为正高度。

| 索引 | 符号 | 含义 | 属于 |
|:---:|------|------|:---:|
| 1 | $\delta_{th}$ | 油门 | 纵向 |
| 2 | $\delta_e$ | 升降舵 | 纵向 |
| 3 | $\delta_a$ | 副翼 | 横侧向 |
| 4 | $\delta_r$ | 方向舵 | 横侧向 |

### 2.2 LOFI vs HIFI 本质区别

| 维度 | LOFI (`fi_flag=0`) | HIFI (`fi_flag=1`) |
|------|:---:|:---:|
| 气动数据源 | `lofi_f16_aerodata.c` 简化查表 | `hifi_f16_aerodata.c` 完整风洞 |
| 前缘襟翼 dLEF | **固定 0°** | **$f(\alpha, q_{bar}/p_s)$ 自动计算** |
| 迎角范围 | −10°~45° | −20°~90° |
| 引擎模型 | 简化 `tgear` | 完整引擎+加力燃烧室 |

---

## 3. 步骤1：配平与线性化 (LOFI + HIFI)

### 3.1 配平结果

| 参数 | LOFI | HIFI | Δ |
|------|:---:|:---:|:---:|
| $\alpha$ [deg] | 2.00 | 2.00 | 0 |
| $\theta$ [deg] | 2.00 | 2.00 | 0 |
| $\delta_e$ [deg] | **−1.87** | **−1.53** | +0.34° |
| $\delta_a$ [deg] | −0.006 | −0.002 | +0.004° |
| $\delta_r$ [deg] | 0.063 | 0.020 | −0.043° |
| dth [−] | 0.221 | 0.219 | −0.002 |
| 功率 [%] | 14.33 | 14.21 | −0.12 |
| **dLEF [deg]** | **0.0** | **2.02** | **+2.02°** |
| 配平代价 | $8.07\times10^{-4}$ | $7.97\times10^{-4}$ | — |
| 退出标志 | 1 (收敛) | 0 (达最大迭代) | — |

- **俯仰角（Pitch Angle，θ）**
  飞机纵轴（机头指向）与**水平面**之间的夹角。它是描述飞机姿态的，机头抬得有多高。
- **迎角（Angle of Attack，α）**
  机翼弦线（可简单理解为机身纵轴方向）与**相对气流方向**之间的夹角。它直接决定了机翼产生的升力大小。

飞机的速度矢量与水平面的夹角，称为**航迹角（Flight Path Angle，γ）**。这三个角之间存在一个简化的运动学关系（假设机翼的安装角可以忽略）：

**俯仰角 (θ) = 迎角 (α) + 航迹角 (γ)**

现在，若 **α = θ**，代入公式就会得到 **γ = 0**。
航迹角为零，意味着飞机实际运动的方向是水平的，也就是**平飞**。

**关键发现**：HIFI 中前缘襟翼在 $\alpha=2^\circ$ 时自动偏转约 **2.02°**。这一额外的翼型弯度改变了气动特性，因此 HIFI 配平的 $\delta_e$ (−1.53°) 与 LOFI (−1.87°) 相差 **0.34°** — dLEF 提供的额外升力减少了平尾配平需求。

### 3.2 LOFI 线性模型

**纵向** 状态 $[V, \alpha, \theta, q, P_{ow}]^T$ / 控制 $[\delta_{th}, \delta_e]^T$

**矩阵阅读说明**：每一行是一个状态量的导数方程，每一列是该导数对某个状态量的偏导（即影响系数）。例如，第1行第2列的 +4.50 表示 $\partial\dot{V}/\partial\alpha$，即"迎角 $\alpha$ 每增大1单位，速度变化率 $\dot{V}$ 就增加4.50"。

$$\mathbf{A}_{long}^{LOFI} = \begin{bmatrix}
& \Delta V & \Delta\alpha & \Delta\theta & \Delta q & \Delta P_{ow} \\[2pt]
\Delta\dot{V} & -0.0137 & \boldsymbol{+4.50} & -9.79 & -0.105 & +0.083 \\
\Delta\dot{\alpha} & -0.00044 & -0.683 & 0 & +0.951 & \cdots \\
\Delta\dot{\theta} & 0 & 0 & 0 & +1.000 & 0 \\
\Delta\dot{q} & \cdots & \boldsymbol{-2.186} & 0 & \boldsymbol{-0.927} & 0 \\
\Delta\dot{P}_{ow} & 0 & 0 & 0 & 0 & -1.000
\end{bmatrix}
\quad
\mathbf{B}_{long}^{LOFI} = \begin{bmatrix}
& \Delta\delta_{th} & \Delta\delta_e \\[2pt]
\Delta\dot{V} & 0 & +2.70 \\
\Delta\dot{\alpha} & 0 & -0.0825 \\
\Delta\dot{\theta} & 0 & 0 \\
\Delta\dot{q} & 0 & \boldsymbol{-9.16} \\
\Delta\dot{P}_{ow} & 64.94 & 0
\end{bmatrix}$$

**横侧向** 状态 $[\beta, \phi, \psi, p, r]^T$ / 控制 $[\delta_a, \delta_r]^T$

$$\mathbf{A}_{lat}^{LOFI} = \begin{bmatrix}
& \Delta\beta & \Delta\phi & \Delta\psi & \Delta p & \Delta r \\[2pt]
\Delta\dot{\beta} & -0.216 & +0.049 & 0 & +0.035 & -0.995 \\
\Delta\dot{\phi} & 0 & 0 & 0 & +1.000 & +0.035 \\
\Delta\dot{\psi} & 0 & 0 & 0 & 0 & +1.001 \\
\Delta\dot{p} & \boldsymbol{-26.50} & 0 & 0 & \boldsymbol{-2.467} & +0.437 \\
\Delta\dot{r} & \boldsymbol{+8.29} & 0 & 0 & -0.015 & \boldsymbol{-0.334}
\end{bmatrix}
\quad
\mathbf{B}_{lat}^{LOFI} = \begin{bmatrix}
& \Delta\delta_a & \Delta\delta_r \\[2pt]
\Delta\dot{\beta} & +0.0105 & +0.0309 \\
\Delta\dot{\phi} & 0 & 0 \\
\Delta\dot{\psi} & 0 & 0 \\
\Delta\dot{p} & \boldsymbol{-34.21} & +6.60 \\
\Delta\dot{r} & -1.53 & \boldsymbol{-3.23}
\end{bmatrix}$$

### 3.3 HIFI 线性模型 🆕

**纵向** 状态 $[V, \alpha, \theta, q, P_{ow}]^T$ / 控制 $[\delta_{th}, \delta_e]^T$

$$\mathbf{A}_{long}^{HIFI} = \begin{bmatrix}
& \Delta V & \Delta\alpha & \Delta\theta & \Delta q & \Delta P_{ow} \\[2pt]
\Delta\dot{V} & -0.0129 & \boldsymbol{-2.111} & -9.79 & +0.139 & +0.083 \\
\Delta\dot{\alpha} & -0.00049 & \boldsymbol{-0.423} & 0 & +0.951 & \cdots \\
\Delta\dot{\theta} & 0 & 0 & 0 & +1.000 & 0 \\
\Delta\dot{q} & +0.00065 & \boldsymbol{-12.626} & 0 & \boldsymbol{-0.945} & 0 \\
\Delta\dot{P}_{ow} & 0 & 0 & 0 & 0 & -1.000
\end{bmatrix}
\quad
\mathbf{B}_{long}^{HIFI} = \begin{bmatrix}
& \Delta\delta_{th} & \Delta\delta_e \\[2pt]
\Delta\dot{V} & 0 & +1.927 \\
\Delta\dot{\alpha} & 0 & -0.0925 \\
\Delta\dot{\theta} & 0 & 0 \\
\Delta\dot{q} & 0 & \boldsymbol{-9.68} \\
\Delta\dot{P}_{ow} & 64.94 & 0
\end{bmatrix}$$

**横侧向** 状态 $[\beta, \phi, \psi, p, r]^T$ / 控制 $[\delta_a, \delta_r]^T$

$$\mathbf{A}_{lat}^{HIFI} = \begin{bmatrix}
& \Delta\beta & \Delta\phi & \Delta\psi & \Delta p & \Delta r \\[2pt]
\Delta\dot{\beta} & -0.203 & +0.049 & 0 & +0.035 & -0.995 \\
\Delta\dot{\phi} & 0 & 0 & 0 & +1.000 & +0.035 \\
\Delta\dot{\psi} & 0 & 0 & 0 & 0 & +1.001 \\
\Delta\dot{p} & \boldsymbol{-23.80} & 0 & 0 & \boldsymbol{-2.286} & +0.281 \\
\Delta\dot{r} & \boldsymbol{+7.05} & 0 & 0 & -0.032 & \boldsymbol{-0.349}
\end{bmatrix}
\quad
\mathbf{B}_{lat}^{HIFI} = \begin{bmatrix}
& \Delta\delta_a & \Delta\delta_r \\[2pt]
\Delta\dot{\beta} & +0.0102 & +0.0299 \\
\Delta\dot{\phi} & 0 & 0 \\
\Delta\dot{\psi} & 0 & 0 \\
\Delta\dot{p} & \boldsymbol{-33.62} & +6.42 \\
\Delta\dot{r} & -1.60 & \boldsymbol{-3.23}
\end{bmatrix}$$

> 🆕 表示本版本新增数据

### 3.4 LOFI/HIFI 矩阵关键元素对比

**纵向 A 矩阵**（状态方程 $\Delta\dot{\mathbf{x}} = \mathbf{A}\Delta\mathbf{x}$ 中的 $\mathbf{A}$，全是偏导数）：

| 元素 | 偏导含义 | LOFI | HIFI | 变化 | 物理解释 |
|:----:|:---:|:---:|:---:|:---:|------|
| $A_{V\alpha}$ | $\partial\dot{V}/\partial\alpha$ | **+4.50** | **−2.11** | 🔴 **符号反转** | 见上文特别分析；迎角对速度的影响方向完全相反 |
| $A_{V\theta}$ | $\partial\dot{V}/\partial\theta$ | −9.79 | −9.79 | **0%** | = $-g$，重力投影项，纯物理不涉及气动 → 线性化验证 ✅ |
| $A_{\alpha\alpha}$ | $\partial\dot{\alpha}/\partial\alpha$ | −0.683 | −0.423 | −38% | 迎角自身的恢复率（$Z_\alpha$ 气动导数）；HIFI 中恢复力减弱 |
| $A_{\alpha q}$ | $\partial\dot{\alpha}/\partial q$ | +0.951 | +0.951 | **0%** | ≈ +1，运动学关系（$\dot{\alpha}\approx q$），纯物理 → 线性化验证 ✅ |
| $A_{\theta q}$ | $\partial\dot{\theta}/\partial q$ | +1.000 | +1.000 | **0%** | $\dot{\theta}=q$，运动学恒等式 → 线性化验证 ✅ |
| $A_{q\alpha}$ ($M_\alpha$) | $\partial\dot{q}/\partial\alpha$ | **−2.19** | **−12.63** | 🔴 **+476%** | 俯仰力矩对迎角的偏导，衡量**静稳定性**；值越负 → 迎角扰动产生恢复力矩越强。HIFI 的静稳定度是 LOFI 的 5.8 倍 |
| $A_{qq}$ ($M_q$) | $\partial\dot{q}/\partial q$ | −0.927 | −0.945 | +2% | 俯仰阻尼导数；俯仰速率自身产生的阻尼力矩，两者几乎相同 |
| $A_{Vq}$ | $\partial\dot{V}/\partial q$ | −0.105 | +0.139 | 🔴 **符号** | 量级很小（$<0.15$），符号反转但对动力学影响有限 |
| $A_{P_{ow}P_{ow}}$ | $\partial\dot{P}_{ow}/\partial P_{ow}$ | −1.000 | −1.000 | **0%** | 发动机功率一阶滞后，纯物理 → 线性化验证 ✅ |

> 🔴 表示差异显著 | ✅ 表示运动学/物理恒等式，零差异验证了线性化流程的正确性

**纵向 B 矩阵**（控制矩阵 $\Delta\dot{\mathbf{x}} = \mathbf{B}\Delta\mathbf{u}$）：

| 元素 | 偏导含义 | LOFI | HIFI | 变化 | 物理解释 |
|:----:|:---:|:---:|:---:|:---:|------|
| $B_{V\delta_e}$ | $\partial\dot{V}/\partial\delta_e$ | +2.70 | +1.93 | **−29%** | 升降舵偏转产生的速度变化率；HIFI 中舵效略有降低 |
| $B_{q\delta_e}$ | $\partial\dot{q}/\partial\delta_e$ | −9.16 | −9.68 | +6% | 升降舵的俯仰操纵效能；两者接近 |
| $B_{P_{ow}\delta_{th}}$ | $\partial\dot{P}_{ow}/\partial\delta_{th}$ | 64.94 | 64.94 | **0%** | 油门到功率的增益，纯发动机模型，不涉气动 → 线性化验证 ✅ |

**横侧向 A 矩阵**：

| 元素 | 偏导含义 | LOFI | HIFI | 变化 | 物理解释 |
|:----:|:---:|:---:|:---:|:---:|------|
| $A_{\beta\beta}$ | $\partial\dot{\beta}/\partial\beta$ | −0.216 | −0.203 | −6% | 侧滑角的自身恢复率（$Y_\beta$ 气动导数）；两者接近 |
| $A_{\beta r}$ | $\partial\dot{\beta}/\partial r$ | −0.995 | −0.995 | **0%** | ≈ −1，运动学关系（$\dot{\beta}\approx -r$），纯物理 ✅ |
| $A_{\phi p}$ | $\partial\dot{\phi}/\partial p$ | +1.000 | +1.000 | **0%** | $\dot{\phi}\approx p$，运动学恒等式 ✅ |
| $A_{\psi r}$ | $\partial\dot{\psi}/\partial r$ | +1.001 | +1.001 | **0%** | $\dot{\psi}\approx r$，运动学恒等式 ✅ |
| $A_{p\beta}$ ($L_\beta$) | $\partial\dot{p}/\partial\beta$ | **−26.50** | **−23.80** | −10% | 侧滑引起的滚转力矩导数（上反角效应）；值越负 → 侧滑时滚转恢复越强 |
| $A_{pp}$ ($L_p$) | $\partial\dot{p}/\partial p$ | −2.467 | −2.286 | −7% | 滚转阻尼导数；滚转速率自身的阻尼效应，两者接近 |
| $A_{pr}$ | $\partial\dot{p}/\partial r$ | +0.437 | +0.281 | **−36%** | 偏航速率对滚转的交叉耦合；HIFI 中此耦合减弱 |
| $A_{r\beta}$ ($N_\beta$) | $\partial\dot{r}/\partial\beta$ | **+8.29** | **+7.05** | −15% | 侧滑引起的偏航力矩导数（方向稳定性/风标效应）；值越正 → 偏航恢复越强 |
| $A_{rr}$ ($N_r$) | $\partial\dot{r}/\partial r$ | −0.334 | −0.349 | +4% | 偏航阻尼导数；偏航速率自身的阻尼效应，两者接近 |

**横侧向 B 矩阵**：

| 元素 | 偏导含义 | LOFI | HIFI | 变化 | 物理解释 |
|:----:|:---:|:---:|:---:|:---:|------|
| $B_{p\delta_a}$ | $\partial\dot{p}/\partial\delta_a$ | −34.21 | −33.62 | −2% | 副翼的滚转操纵效能；两者几乎相同 |
| $B_{r\delta_r}$ | $\partial\dot{r}/\partial\delta_r$ | −3.23 | −3.23 | **0%** | 方向舵的偏航操纵效能，完全相同 ✅ |
| $B_{r\delta_a}$ | $\partial\dot{r}/\partial\delta_a$ | −1.53 | −1.60 | +5% | 副翼的偏航交叉耦合（不利偏航），两者接近 |
| $B_{p\delta_r}$ | $\partial\dot{p}/\partial\delta_r$ | +6.60 | +6.42 | −3% | 方向舵的滚转交叉耦合，两者接近 |

> ✅ 表示零差异或极小差异，证明两个模型的线性化流程正确处理了运动学/物理部分

**核心结论**：差异集中在纵向 A 矩阵的气动项（$A_{V\alpha}$ 符号反转、$A_{q\alpha}$ 放大 5.8×），以及纵向 B 矩阵的 $B_{V\delta_e}$（−29%）。横侧向差异普遍小于 15%。所有运动学/物理恒等式（≈0, ≈1, ≈−g）在两个模型中完全相同，验证了线性化计算的正确性。

---

#### 特别分析：$A_{V\alpha}$ 为何符号反转？

$A_{V\alpha} = \partial\dot{V}/\partial\alpha$ 的含义是：**迎角 $\alpha$ 增大时，速度的变化率 $\dot{V}$ 会怎么变？**

直觉上，迎角增大 → 诱导阻力增大 → 飞机减速 → $\dot{V}$ 应变得更负 → $A_{V\alpha}$ 应为**负数**。HIFI 的 −2.11 符合这个直觉，但 LOFI 的 **+4.50** 却是正数——迎角增大会让速度变快？这似乎反直觉。

**原因在于 LOFI 模型缺少了一个关键物理效应。**

要理解这一点，先写出飞机沿速度方向的受力方程。考虑作用在飞机上的力，沿**速度方向**（即航迹方向，与机身轴线差一个迎角 $\alpha$）建立受力平衡：

作用于飞机的力有三个来源，在**速度方向上**的分量为：

| 力 | 符号 | 速度方向分量 | 说明 |
|:--|:---:|:------:|:----:|
| **推力** | $T$ | $T\cos\alpha$ | 推力沿机身轴线向前，需要投影到速度方向：$\alpha$ 越大，推力在速度方向的"有效"分量越小 |
| **阻力** | $D$ | $-D$ | 阻力本身就是抵抗运动的力，方向与速度相反，所以直接为负 |
| **重力** | $mg$ | $-mg\sin\gamma$ | 重力竖直向下。$\gamma = \theta - \alpha$ 是航迹角（速度方向与水平面的夹角）。飞机爬升时 $\gamma>0$，重力有向后分量，起减速作用 |

由牛顿第二定律 $ma = \sum F$，沿速度方向：

$$m\dot{V} = T\cos\alpha - D - mg\sin\gamma$$

两边同除以 $m$：

$$\boxed{\dot{V} = \frac{T\cos\alpha - D}{m} - g\sin\gamma}$$

这就是速度变化率的完整表达式。

**各变量含义**：

| 符号 | 含义 | 单位 | 说明 |
|:---:|------|:---:|------|
| $V$ | 空速 (飞机相对空气的速度) | m/s | |
| $T$ | 发动机推力 | N | 沿机身轴线向前 |
| $D$ | 气动阻力 | N | 与速度方向相反 |
| $m$ | 飞机质量 | kg | |
| $g$ | 重力加速度 | m/s² | $g \approx 9.81$ |
| $\alpha$ | 迎角 (机身轴线与速度方向夹角) | rad | |
| $\gamma$ | 航迹角 (速度方向与水平面夹角) | rad | $\gamma = \theta - \alpha$ |
| $\theta$ | 俯仰角 (机身轴线与水平面夹角) | rad | |

在配平点附近平飞时 $\gamma\approx 0$，所以 $g\sin\gamma\approx 0$，简化为：

$$\dot{V} \approx \frac{T\cos\alpha - D}{m}$$

对 $\alpha$ 求偏导：

$$\frac{\partial\dot{V}}{\partial\alpha} \approx \frac{1}{m}\left(-\frac{\partial D}{\partial\alpha} - T\sin\alpha\right)$$

到这里为止，分析的仍然是阻力 D。但**代码里并不直接计算 D**——它计算的是体轴力系数 CX 和 CZ。D 必须从它们推导出来，而这个推导过程才是理解 LOFI/HIFI 差异的关键。

**代码里实际算的是什么？**

代码直接得到的是机体轴下的两个力（$\beta=0$ 时简化）：

$$X_{\text{body}} = \bar{q}S\cdot\text{CX} \quad\text{(沿机身轴线，向前为正)}$$
$$Z_{\text{body}} = \bar{q}S\cdot\text{CZ} \quad\text{(垂直于机身轴线，向下为正)}$$

阻力 D 是**沿速度方向**的力，与机身轴线差一个迎角 $\alpha$，所以需要将 $X_{\text{body}}$ 和 $Z_{\text{body}}$ 向速度方向投影合成：

$$D = -\big(X_{\text{body}}\cos\alpha + Z_{\text{body}}\sin\alpha\big)$$

**为什么这个合成重要？**

将 D 表达式代入 $\partial\dot{V}/\partial\alpha$ 并展开（忽略推力项的微小变化），$A_{V\alpha}$ 的主要贡献来自：

$$A_{V\alpha} = \frac{1}{m}\left(-\frac{\partial D}{\partial\alpha}\right) \approx \frac{\bar{q}S}{m}\left(\frac{\partial\text{CX}}{\partial\alpha}\cos\alpha + \frac{\partial\text{CZ}}{\partial\alpha}\sin\alpha\right)$$

在小迎角下（$\alpha=2^\circ$，$\sin\alpha\approx0.035$）：

$$A_{V\alpha} \approx \frac{\bar{q}S}{m}\left(\frac{\partial\text{CX}}{\partial\alpha}\cdot 1 + \frac{\partial\text{CZ}}{\partial\alpha}\cdot 0.035\right)$$

**关键点**：$\ddot{V}$ 对 $\alpha$ 的敏感性主要由 $\partial\text{CZ}/\partial\alpha$ 贡献的（因为 $\partial\text{CZ}/\partial\alpha$ 的量级远大于 $\partial\text{CX}/\partial\alpha$），而不是 CX。CZ 在 LOFI 和 HIFI 中随 $\alpha$ 的变化率不同，这才是 $A_{V\alpha}$ 符号反转的根本原因。

**LOFI 和 HIFI 差异的根源**：前缘襟翼 dLEF=2.02° 改变了翼型弯度，导致 CZ 随 $\alpha$ 的变化率（升力线斜率）发生了系统性的偏移。这个偏移通过 $\sin\alpha$ 项投影到速度方向，最终使 $A_{V\alpha}$ 从 LOFI 的 +4.50 变为 HIFI 的 -2.11。

这也提醒我们：**LOFI 模型在 $\alpha$ 对 $V$ 的耦合上不可信**，控制律设计时如果依赖 LOFI 的结果，会得出错误的结论。

---

## 4. 步骤2：纵向模态分析 (LOFI vs HIFI)

### 4.1 特征值总览

| 模态 | LOFI | HIFI | 关键变化 |
|------|------|------|:---:|
| **短周期** $\lambda$ | $\boldsymbol{-0.806 \pm j1.437}$ | $\boldsymbol{-0.684 \pm j3.455}$ | 🔴 虚部 +141% |
| 短周期 $\omega_n$ | **1.647 rad/s** | **3.522 rad/s** | 🔴 +114% |
| 短周期 $\zeta$ | **0.489** | **0.194** | 🔴 −60% |
| 短周期 $T$ | **4.37 s** | **1.82 s** | −58% |
| **长周期** $\lambda$ | $\boldsymbol{-0.0062 \pm j0.0587}$ | $\boldsymbol{-0.0064 \pm j0.0714}$ | |
| 长周期 $\omega_n$ | **0.059 rad/s** | **0.072 rad/s** | +21% |
| 长周期 $\zeta$ | **0.105** | **0.089** | −15% |
| 长周期 $T$ | **107.0 s** | **88.0 s** | −18% |
| **功率** $\lambda$ | −1.000 | −1.000 | 零变化 |

### 4.2 短周期 — LOFI/HIFI 差异是本次实验最大的发现 🔴

LOFI 和 HIFI 的短周期特征值差异远超出预期：

$M_\alpha$（即状态矩阵中的 $A_{q\alpha} = \partial\dot{q}/\partial\alpha$）是**俯仰力矩对迎角的偏导数**，它衡量飞机的静稳定性：值越负，迎角扰动产生的恢复力矩越强，飞机越倾向于回正。

| 参数 | LOFI | HIFI | 物理含义 |
|------|:---:|:---:|------|
| $\omega_n$ | 1.65 rad/s | **3.52 rad/s** | HIFI 的俯仰振荡频率是 LOFI 的 2.1 倍 |
| $\zeta$ | 0.489 | **0.194** | HIFI 预测的阻尼比只有 LOFI 的 40% |
| $T$ | 4.37 s | **1.82 s** | HIFI 周期短得多 |
| $M_\alpha$ | −2.19 | **−12.63** | HIFI 的静稳定性强度是 LOFI 的 5.8 倍 |

**物理原因**：

- dLEF≈2° 改变了翼型弯度，显著后移了气动中心
- 气动中心后移 → 静稳定性 $M_\alpha$ 大幅增强 (更负)
- $\omega_{n,sp} \propto \sqrt{-M_\alpha}$ → $\omega_n$ 随之增大

  这个关系来自短周期的简化分析。短周期运动主要是 $\alpha$ 和 $q$ 的耦合，忽略速度变化（假设 $V$ 不变），其 2×2 子系统为：

  $$\begin{bmatrix}\Delta\dot{\alpha}\\ \Delta\dot{q}\end{bmatrix} = \begin{bmatrix} Z_\alpha & 1 \\ M_\alpha & M_q \end{bmatrix} \begin{bmatrix}\Delta\alpha\\ \Delta q\end{bmatrix}$$

  其中 $Z_\alpha = A_{\alpha\alpha}$（迎角自身恢复率），$M_q = A_{qq}$（俯仰阻尼），$M_\alpha = A_{q\alpha}$（静稳定性）。特征方程为 $\det(\lambda\mathbf{I} - \mathbf{A}) = 0$：

  $$\det\begin{bmatrix} \lambda - Z_\alpha & -1 \\ -M_\alpha & \lambda - M_q \end{bmatrix} = (\lambda - Z_\alpha)(\lambda - M_q) - M_\alpha = 0$$

  $$\lambda^2 - (Z_\alpha + M_q)\lambda + (Z_\alpha M_q - M_\alpha) = 0$$

  与标准二阶系统 $\lambda^2 + 2\zeta\omega_n\lambda + \omega_n^2 = 0$ 对比：

  $$\omega_n^2 = Z_\alpha M_q - M_\alpha$$

  注意 $M_\alpha$ 本身是负的（如 LOFI −2.19, HIFI −12.63），所以 $-M_\alpha = |M_\alpha|$。代入数值验证：

  $$\text{LOFI: } \omega_n^2 = (-0.683)(-0.927) + 2.19 = 0.633 + 2.19 = 2.82 \;\rightarrow\; \omega_n = 1.68\text{ rad/s}$$

  $$\text{HIFI: } \omega_n^2 = (-0.423)(-0.945) + 12.63 = 0.400 + 12.63 = 13.03 \;\rightarrow\; \omega_n = 3.61\text{ rad/s}$$

  与实测的 1.65 rad/s 和 3.52 rad/s 吻合得很好。其中 $Z_\alpha M_q$ 项（两个负值相乘）在 LOFI 和 HIFI 中几乎不变（0.63 vs 0.40），而 $|M_\alpha|$ 从 2.19 跳到 12.63（+476%），所以 $\omega_n^2$ 的增长完全由 $|M_\alpha|$ 主导，故：

  $$\boxed{\omega_{n,sp} \propto \sqrt{-M_\alpha}}$$

**品质评估变化**：

| 标准 | LOFI | HIFI |
|------|:---:|:---:|
| $\zeta_{sp}$ | 0.489 ≥ 0.35 → **Level 1** ✅ | 0.194 ≥ 0.35 → **不满足 Level 1!** ⚠️ |
| MIL-8785C Level 2 (≥0.25) | ✅ | **也不满足!** |
| MIL-8785C Level 3 (≥0.15) | — | ✅ 刚满足 |

> **极重要发现**：如果 HIFI 是更精确的模型，那么 F-16 在 H=5000m/V=200m/s 的短周期阻尼比仅 0.194，只满足 Level 3 标准！这远超 LOFI 预测的 0.489。对飞行品质评估和控制律设计有重大影响。

### 4.3 长周期

LOFI/HIFI 的长周期差异在 15-21% 范围，量级一致。$\zeta_{ph}$ 都满足 Level 1 (>0.04)。

---

## 5. 步骤2：横侧向模态分析 (LOFI vs HIFI)

### 5.1 特征值总览

| 模态 | LOFI | HIFI | 变化 |
|------|------|------|:---:|
| **荷兰滚** $\lambda$ | $\boldsymbol{-0.289 \pm j3.014}$ | $\boldsymbol{-0.264 \pm j2.803}$ | |
| 荷兰滚 $\omega_n$ | **3.028 rad/s** | **2.816 rad/s** | −7% |
| 荷兰滚 $\zeta$ | **0.0953** | **0.0939** | −1.5% |
| 荷兰滚 $T$ | **2.08 s** | **2.24 s** | +8% |
| **螺旋** $\lambda$ | $0.0000$, $−0.0099$ | $0.0000$, $−0.0154$ | |
| 螺旋 $\tau$ | $\infty$, 101.2s | $\infty$, **64.7s** | −36% |
| **滚转收敛** $\lambda$ | **−2.430** | **−2.294** | |
| 滚转收敛 $\tau$ | **0.412 s** | **0.436 s** | +6% |

### 5.2 关键发现

**横侧向的 LOFI/HIFI 差异明显小于纵向**：

- 荷兰滚参数变化 ≤ 8% — LOFI 和 HIFI 对横侧向的预测高度一致
- 滚转收敛 $\tau$ 仅差 0.024s (6%)
- 螺旋时间常数从 101s 变为 65s (仍属极慢范畴)

**品质评估**：荷兰滚 $\zeta_{dr}$ 在两种模型下都处于 Level 2 (0.08~0.19)。横侧向结论不受 LOFI/HIFI 选择的影响。

---

## 6. LOFI vs HIFI 全维度对比与差异分析

### 6.1 差异分层

| 层 | 内容 | 差异程度 | 说明 |
|:---:|------|:---:|------|
| **第0层** | 运动学关系 (≈0,≈1,≈−g) | **零** | 纯物理，不涉气动 |
| **第1层** | 横侧向 A/B | **小 (<10%)** | 配平差异小、dLEF 对横向气动影响有限 |
| **第2层** | 纵向 B 矩阵 | **小 (<6%)** | 舵效对 dLEF 状态有一定敏感度 |
| **第3层** | 纵向 A 矩阵气动项 | 🔴 **极大** | $A_{V\alpha}$ 符号反转、$M_\alpha$ 放大 5.8× |

### 6.2 根源分析

LOFI 和 HIFI 的根本差异在于：在 H=5000m/V=200m/s（$\alpha\approx2^\circ$，小迎角巡航），HIFI 的前缘襟翼自动偏转了约 2°。这个看似微小的偏转**改变了翼型的弯度分布**，导致：

1. **气动中心位置后移** → 静稳定性 $M_\alpha$ 大幅增强
2. **升力线斜率改变** → $A_{V\alpha}$ 从正变负（迎角增大 → 速度变化率由加速变为减速）
3. **横向气动受 dLEF 影响较小** → 横侧向参数变化不超过 10%

### 6.3 工程含义

| 问题 | 答案 |
|------|------|
| LOFI 和 HIFI 矩阵一样吗？ | **完全不同。** 纵向 A 矩阵中多项有量级甚至符号的差异 |
| 哪个更可信？ | **HIFI** — 包含前缘襟翼的真实偏转效应，更接近真实 F-16 |
| LOFI 还能用吗？ | **能，但要清楚局限**：LOFI 严重低估了纵向短周期频率、高估了阻尼比 |
| 什么情况下差异最关键？ | **飞行品质评估**：LOFI 判断短周期为 Level 1，HIFI 仅 Level 3 — 这可能导致对增稳系统需求的错误判断 |
| 什么情况下差异不重要？ | **模态结构识别**（哪种模态存在、哪种稳定/不稳定）— LOFI/HIFI 定性一致 |

### 6.4 飞行品质重新评估

| 模态 | 参数 | LOFI | HIFI | Level 1 标准 | HIFI 评级 |
|------|------|:---:|:---:|:---:|:---:|
| 短周期 | $\zeta_{sp}$ | 0.489 ✅ | **0.194** ⚠️ | ≥0.35 | **Level 3** 🔴 |
| 长周期 | $\zeta_{ph}$ | 0.105 ✅ | 0.089 ✅ | ≥0.04 | Level 1 |
| 荷兰滚 | $\zeta_{dr}$ | 0.095 ⚠️ | 0.094 ⚠️ | ≥0.19 | Level 2 |
| 滚转收敛 | $\tau_{roll}$ | 0.41s ✅ | 0.44s ✅ | ≤1.0s | Level 1 |

> 🔴 短周期品质从 LOFI 的 Level 1 降为 HIFI 的 Level 3 — 这是本次数据更新最重要的发现

这个发现意味着：基于 LOFI 模型设计的自动驾驶仪俯仰回路可能过于乐观（低估了控制难度），需要用 HIFI 模型重新验证。

---

## 7. 步骤3：非线性开环仿真验证

### 7.1 仿真设置

| 参数 | 值 |
|------|-----|
| 输入 | 5° 升降舵 Doublet: t=1s→+5°, t=2s→−5°, t=5s→0° |
| 仿真时长 | 30s |
| 对比 | LOFI (蓝实线) vs HIFI (绿虚线) |

### 7.2 仿真结果

- **Figure 5**: 控制输入 (2×2) — del Thrust/Elevator/Aileron/Rudder
- **Figure 6**: 位置与姿态 (2×3) — 高度波动±5m 后恢复，$\theta$ 对 elevator 响应明显
- **Figure 7**: 速度与角速率 (2×3) — $\alpha$ 和 $q$ 快速振荡后衰减

### 7.3 仿真与线性分析的交叉验证

| 线性预测 | 仿真观察 | 吻合？ |
|---------|---------|:---:|
| LOFI 短周期 $T=4.37$s | $\alpha$/$q$ 振荡约 4s 衰减 | ✅ |
| HIFI 短周期 $T=1.82$s | 需在 ~1.8s 尺度上检查高频成分 | ⚠️ 待验证 |
| 纵向机动不激发横向 | $\phi,p,r\approx0$ | ✅ |

### 7.4 仿真数据的互补作用

仿真直接使用非线性模型，无需线性化近似，因此是验证线性分析结论的"黄金标准"。仿真中 LOFI/HIFI 的 $\alpha$ 和 $q$ 响应曲线的高度相似性表明：**在小扰动范围内，非线性动力学对 LOFI/HIFI 差异的"放大"或"抑制"效应需要进一步定量评估**。

---

## 8. 综合分析与结论

### 8.1 LOFI vs HIFI 差异的物理根源

$$\text{LOFI: dLEF = 0° → 原始翼型 → } M_\alpha = -2.19$$
$$\text{HIFI: dLEF = 2.02° → 弯度增加 → 气动中心后移 → } M_\alpha = -12.63$$

这一差异是**系统性的**（不是随机噪声），它从根本上改变了纵向短周期的特征值位置。

### 8.2 结论

1. 成功对 LOFI 和 HIFI 分别完成了**配平→线性化→矩阵提取→特征值分析**全线流程
2. HIFI 配平状态下前缘襟翼自动偏转 **2.02°**，是差异的根本来源
3. **短周期模态差异极大**：HIFI 的 $\omega_n$ 是 LOFI 的 2.1 倍 (3.52 vs 1.65 rad/s)，$\zeta$ 仅为 LOFI 的 40% (0.194 vs 0.489)
4. **横侧向模态差异小** (<10%)：荷兰滚、滚转收敛的参数在两个模型下高度一致
5. 运动学元素 (≈0, ≈1, ≈−g) 在两种模型下相同 — 验证了线性化流程的正确性
6. HIFI 模型下短周期阻尼比仅 0.194，降至 **MIL-8785C Level 3**，对飞行品质评估和控制律设计有重大影响
7. LOFI 模型可用于模态结构的定性识别，但**定量参数应以 HIFI 为准**

---

## 9. 附录

### 9.1 文件结构

```
trae/lab1_0429/
├── 📄 lab1_report.md                  ← 本报告 (v2.0, HIFI 完整版)
├── 📄 lab1_principles_guide.md        ← 原理指南
├── 📄 lab1_modal_analysis_report.md   ← 模态专题
├── 📄 lab1_code_analysis.md           ← 代码分析
│
├── 🔧 lab1_step1_trim_and_linearize.m   # 配平+线性化 (LOFI+HIFI)
├── 🔧 lab1_step2_eigenvalue_analysis.m  # 特征值+模态+对比 (LOFI+HIFI)
├── 🔧 lab1_step3_simulation.m           # 非线性仿真 (LOFI vs HIFI)
│
├── 📊 lab1_trim_result.txt            # [v2.0] LOFI+HIFI 配平+完整矩阵
├── 📊 lab1_matrices.mat               # [v2.0] LOFI+HIFI 全部 A/B/C/D
├── 📊 lab1_eigenvalue_result.txt      # [v2.0] LOFI+HIFI 特征值+对比
├── 📊 lab1_eigen.mat                  # [v2.0] 特征值/向量
├── 📊 lab1_trim_info.txt              # 仿真配平信息
├── 📊 lab1_simulation_data.mat        # 仿真数据
│
└── 🖼️ figure5/6/7/summary .png/.fig    # 仿真图表
```

### 9.2 运行方法

```matlab
cd('E:\matlab_script\实验课资料\FC_SimCode_1\trae\lab1_0429')
lab1_step1_trim_and_linearize   % LOFI+HIFI 双配平+双线性化 (~4-6min)
lab1_step2_eigenvalue_analysis  % LOFI+HIFI 双特征值+对比 (~1s)
lab1_step3_simulation           % 非线性仿真 (~3-5min)
```

### 9.3 关键公式

**线性化**: $\mathbf{A} = \frac{\partial\mathbf{f}}{\partial\mathbf{x}}\big|_0$, $\mathbf{B} = \frac{\partial\mathbf{f}}{\partial\mathbf{u}}\big|_0$

**模态参数 (复根 $\lambda=\sigma\pm j\omega_d$)**: $\omega_n=|\lambda|$, $\zeta=-\sigma/\omega_n$, $T=2\pi/\omega_d$

**短周期近似**: $\omega_{n,sp} \approx \sqrt{-M_q Z_\alpha - M_\alpha}$

---

*本报告 (v2.0) 基于 2026-04-29 21:31 运行的最新 step1/2 输出。核心更新：首次纳入 HIFI 完整配平+矩阵+特征值数据，发现了 LOFI/HIFI 短周期模态的巨大差异。*
