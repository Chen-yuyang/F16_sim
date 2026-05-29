# 实验1 步骤1：F-16纵向线性化模型 A/B 矩阵提取

> 飞行条件：高度 5000m，马赫 0.59（速度 200m/s）
> 文档日期：2026-04-17

---

## 1. 这个脚本在干什么？

### 1.1 核心目标

**把一个复杂的非线性飞机模型，在特定飞行条件下，近似成一个线性方程组。**

```
非线性F-16模型（13阶）  →  线性模型（5阶纵向）  →  A矩阵 + B矩阵
```

这个线性模型（A/B矩阵）有什么用？
- **飞行品质分析**：判断飞机是否稳定、响应快慢
- **控制律设计**：用 LQR、PID 等线性方法设计自动驾驶仪
- **仿真加速**：用线性模型代替非线性模型，大幅提升实时仿真速度

### 1.2 四步流程总览

| 步骤 | 内容 | 关键技术 |
|------|------|---------|
| 步骤1 | 加载并编译Simulink模型 | `load_system` + `lincompile` |
| 步骤2 | 寻找平衡点（配平） | `fminsearch` 单纯形法 |
| 步骤3 | 在平衡点处线性化 | `linmod` |
| 步骤4 | 特征值分析 | `eig()` + 模态判别 |

---

## 2. 为什么需要这么长的代码？

### 2.1 直接运行项目根目录的代码行不行？

**不行。原因有以下几点：**

#### 原因1：Simulink模型必须"编译"才能被MATLAB调用

F-16模型不是普通的MATLAB脚本，而是一个 **Simulink S-Function 模块**：

```
F16_trim.slx  (Simulink模型)
    ↓ load_system() + lintool()
变成可调用的函数
    ↓ feval('F16_trim', t, x, u, 'derivs')
返回状态导数 ẋ = f(x, u)
```

**代码流程：**
```matlab
load_system('F16_trim');           % 加载模型到内存
feval('F16_trim', [], [], [], 'lincompile');  % 编译为可调用的C代码/MEX
% ... 运行配平和线性化 ...
feval('F16_trim', [], [], [], 'term');        % 清理编译产生的临时文件
```

**没有这步编译，直接 `feval` 会崩溃**——MATLAB根本不知道这个"函数"怎么调用。

#### 原因2：配平不是解方程，而是优化问题

很多人以为"找平衡点 = 解 ẋ=0 的方程"。实际上：

```
ẋ = f(x, u) = 0  ← 这是13个方程
x 是13维，u 是6维 → 方程数 < 未知数数量 → 无穷多解！
```

所以配平本质上是 **带约束的优化问题**：

```
min  cost(x,u) = Σ weight(i) × ẋ_i²
subject to: 物理限制（舵面偏度、油门范围等）
```

项目根目录的 `trim_fun.m` 实现了这个代价函数，但调用方式需要：
1. 先编译模型
2. 设置全局变量（`altitude`, `velocity`, `fi_flag_Simulink`）
3. 反复调用 `fminsearch`（每次调用都要执行一次完整的13阶非线性方程）

这就是为什么需要设置 `OPTIONS`、调用 `fminsearch`，而不是简单几行代码。

#### 原因3：linmod有特殊的模型要求

`linmod()` 函数对模型有严格要求：

```
F16_trim.slx    → 用于配平（13阶非线性）
F16_openloop_linearization.slx → 用于线性化（专用接口）
```

`F16_openloop_linearization` 是预先设置好线性化接口的模型，不能用 `F16_trim` 直接代替。

此外，linmod执行时需要在**MATLAB基础工作区**能找到变量，所以需要 `assignin('base', ...)`。

---

## 3. 代码逐段解析

### 3.1 路径设置与初始化（第12-42行）

```matlab
this_file = mfilename('fullpath');
proj_root = fileparts(fileparts(fileparts(this_file)));  % 三级父目录 → 项目根目录
result_dir = fileparts(this_file);                          % trae/lab1
result_file = fullfile(result_dir, 'step1_result.txt');
cd(proj_root);                                              % 切换到项目根目录！
```

**为什么要 cd 到项目根目录？**

因为 `F16_trim.slx`、`trim_fun.m`、`tgear.m` 等依赖文件都在根目录。如果当前目录不是根目录，Simulink找不到这些文件，会报"模型未加载"错误。

### 3.2 全局变量设置（第33-38行）

```matlab
global fi_flag_Simulink phi_weight theta_weight psi_weight altitude velocity
fi_flag_Simulink = 0;           % 0 = 非Simulink模式（非实时仿真）
phi_weight = 10;               % 滚转角速度权重
theta_weight = 10;             % 俯仰角速度权重
psi_weight = 10;               % 偏航角速度权重
altitude = 5000;               % 飞行高度 m
velocity = 200;                % 飞行速度 m/s
```

这些全局变量被 `trim_fun.m` 直接使用（用于计算气动系数和代价函数）。

### 3.3 模型编译（第44-51行）

```matlab
load_system(model_trim);
feval('F16_trim', [], [], [], 'lincompile');
```

**`lincompile` 做了什么？**
- 将Simulink模型翻译成C代码
- 编译成MEX（MATLAB Executable）文件
- 之后的 `feval(..., 'derivs')` 就能直接调用非线性方程组

### 3.4 配平优化（第62-82行）

```matlab
UX0 = [beta; elevator; alpha; aileron; rudder; dth];  % 6个待优化变量
[UX, FVAL, EXITFLAG, OUTPUT] = fminsearch('trim_fun', UX0, OPTIONS);
```

**UX0的6个分量物理含义：**

| 分量 | 符号 | 物理意义 | 初始值 |
|------|------|---------|--------|
| 1 | β | 侧滑角 (rad) | 0 |
| 2 | δ_e | 平尾偏度 (rad) | -2° |
| 3 | α | 迎角 (rad) | 10° |
| 4 | δ_a | 副翼偏度 (rad) | 0 |
| 5 | δ_r | 方向舵偏度 (rad) | 0 |
| 6 | dth | 油门开度 (0~1) | 0.2 |

**`fminsearch`（Nelder-Mead单纯形法）的优点：**
- 不需要梯度信息 → 适合黑盒优化
- MATLAB内置 → 稳定可靠
- 相比手动梯度下降，收敛快，不容易卡死

### 3.5 配平结果提取（第91-99行）

```matlab
xu = best_xu; uu = best_uu;
trim_state_lo = xu(1:13);           % 取前13个状态
trim_thrust_lo = uu(1);             % 推力
trim_control_lo = [uu(2); uu(3); uu(4)];  % elevator, aileron, rudder
```

**xu (13个状态) 的排列顺序：**

```
[ V,  β,  α,   φ,   θ,   ψ,   p,   q,   r,   x,   y,   h,  power ]
  1   2   3    4    5    6    7    8    9   10   11   12   13
```

**uu (6个控制) 的排列顺序：**

```
[ dth,  elevator,  aileron,  rudder,  dLEF,  fi_flag_Simulink ]
   1        2          3         4        5          6
```

### 3.6 线性化与状态筛选（第107-117行）

```matlab
idx = [12, 1, 3, 8, 5];   % 从13个状态中挑选5个纵向状态
A_sub = A_lo(idx, idx);   % 提取子矩阵
S = diag([-1, 1, 1, 1, 1]);  % 符号变换矩阵
A_long = S * A_sub * S;   % 修正符号约定
```

**为什么需要符号变换S？**

F-16的状态约定（高度h向下为正）和标准航空约定（h向上为正）相反：
- h_dot = -V·sin(θ-α) ≈ -V·(θ-α)

所以第12个状态（高度h）需要乘以 -1 来修正符号。

**idx映射关系：**

```
idx=[12, 1, 3, 8, 5]
原始13状态: [V, β, α, φ, θ, ψ, p, q, r, x, y, h, power]
                  ↓        ↓           ↓     ↑
筛选5状态:   h(12) V(1) α(3)    q(8)   θ(5)
                 ↓
纵向状态顺序: [H, V, α, q, θ]
```

### 3.7 特征值与模态分析（第155-198行）

```matlab
eig_long = eig(A_long);   % 求特征值

% 判断模态类型
if wn > 0.3
    '短周期模态 (Short-Period)'
else
    '长周期模态 (Phugoid)'
end
```

**短周期 vs 长周期模态的物理本质：**

```
短周期（高频 ≈ 1.6 rad/s）：
  主要涉及迎角α和俯仰角速度q的快速振荡
  本质：飞机绕重心的俯仰转动惯性 vs 气动恢复力矩
  飞行员感受：杆力振动，通常3-5秒内衰减

长周期（低频 ≈ 0.07 rad/s）：
  主要涉及速度V和俯仰角θ的缓慢交替
  本质：动能 ↔ 势能 的交替（飞机"飘上去-滑下来"）
  飞行员感受：几乎感觉不到，无需干预
```

---

## 4. 项目文件架构

```
FC_SimCode_1/
├── F16_trim.slx                      ← 非线性13阶F-16模型（Simulink）
├── F16_openloop_linearization.slx    ← 专用线性化接口模型
├── trim_fun.m                        ← 配平代价函数（被fminsearch调用）
├── tgear.m                           ← 油门开度 → 推力 查表函数
├── trim_F16.m                        ← 配平主函数
├── FindF16Dynamics.m                 ← 动力学查找表
├── F16_dyn.mexw64                    ← 编译后的C代码（MEX二进制）
│
└── trae/lab1/
    └── lab1_step1.m                  ← 本实验的主脚本
        └── step1_result.txt          ← 运行结果输出
```

**文件依赖关系：**

```
lab1_step1.m
    ├── load_system('F16_trim')
    │       └── 调用 F16_dyn.mexw64（C代码MEX）
    ├── fminsearch('trim_fun', UX0)
    │       └── trim_fun.m
    │               ├── feval('F16_trim', ..., 'derivs')
    │               └── tgear.m
    └── linmod('F16_openloop_linearization', xtrim, utrim)
            └── 读取 F16_openloop_linearization.slx
```

---

## 5. 关键发现与结果验证

### 5.1 配平合理性验证

```matlab
theta ≈ alpha   % θ = 2.00°, α = 2.00°
phi = psi = 0   % 无侧滑、无偏航
p = q = r = 0   % 无角速度
```

→ **完全符合定直平飞（steady level flight）的物理定义**

### 5.2 线性模型稳定性验证

```
所有特征值实部 < 0  →  线性模型是渐近稳定的
```

| 模态 | 阻尼比 ζ | 稳定性 | 典型周期 |
|------|---------|--------|---------|
| 短周期 | 0.489 | ✅ 稳定 | 4.37s |
| 长周期 | 0.076 | ✅ 稳定 | 94.7s |

### 5.3 B矩阵的物理意义

```
推力(thrust)列：全为0
→ 在该平衡点附近，油门变化对高度/速度无直接影响
（这是对的——定直平飞时，油门主要用于保持高度而非改变高度）

平尾(elevator)列：非零
→ 平尾是唯一的纵向控制面，支配V、α、q的响应
```

---

## 6. 常见错误与排查

| 错误现象 | 可能原因 | 解决方法 |
|---------|---------|---------|
| `derivs调用失败` | 模型未编译 | 先执行 `lincompile` |
| `变量fi_flag_Simulink无法识别` | linmod在base workspace查找变量 | `assignin('base', 'fi_flag_Simulink', 0)` |
| `模型未加载` | 当前目录不是项目根目录 | `cd(proj_root)` |
| 配平不收敛 | 初始猜测远离平衡点 | 调整UX0初始值 |
| `cost无法降低` | F16_dyn.mexw64未编译 | 在项目根目录执行 `mex F16_dyn.c` |

---

## 7. 为什么代码风格不好？

本项目的代码风格存在以下问题（不是你的错觉）：

### 7.1 空格代替逗号（危险写法）

```matlab
% 危险：空格可读性差，容易数错元素
xu = [velocity beta alpha phi theta psi p q r 0 0 -altitude pow_val]'

% 推荐：明确逗号分隔
xu = [velocity, beta, alpha, phi, theta, psi, p, q, r, 0, 0, -altitude, pow_val]'
```

### 7.2 变量命名不统一

```
trim_fun.m 中：  theta=0.0349（硬编码初始猜测）
lab1_step1.m中：theta=0.0349（同样硬编码）
```

这些magic number应该定义为常量或从函数返回值读取。

### 7.3 缺少函数注释

`trim_fun.m` 的核心注释居然是乱码：
```matlab
% thrust limits����ط�Ӣ��ע���д���ӦΪbeta
```
这是编码问题，原作者可能是非英语母语者。

---

## 8. 总结

`lab1_step1.m` 之所以需要这么多行代码，根本原因是：

```
F-16是一个Simulink S-Function模型
    ↓
无法直接用 MATLAB 脚本调用
    ↓
必须：加载 → 编译 → 优化配平 → 线性化 → 清理
    ↓
每一步都有特定的MATLAB/Simulink API
    ↓
代码自然就长了
```

**核心价值：** 这段代码把"学术研究级别"的飞行器线性化流程自动化了——从13阶非线性模型提取5阶纵向线性模型，这是现代飞机自动驾驶仪设计的必经之路。
