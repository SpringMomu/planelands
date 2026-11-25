# Planelands · 航天飞机自主返场：SCP + MPC 制导的教学型仿真

[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

把一架航天飞机（Orbiter 级，无动力）随机放在天空里的任意位置：高度 8–25 km、速度最高 Mach 2.6、机头朝向随意。
它要自己决定降落在哪条跑道、走哪条路，管理好能量，对准跑道，拉平、接地、放减速伞、刹车，最后停在跑道上。
整个过程没有人工干预。

这个仓库有两种读法：

- **当作一门课来读**：本 README 的主体是一套讲义，像导师带学生一样，从“滑翔机的能量账”讲起，
  一步步推导**序列凸优化（SCP）轨迹规划**和**模型预测控制（MPC）跟踪**，最后讲 Orbiter 式的进近拉平律。
  每个公式都对应 `sim/gnc/` 中的具体代码，参数取自代码本身，可以对照着读。
- **当作一个仿真器来玩**：6-DOF 分片气动、电传飞控、起落架、Dryden 湍流，加上 three.js 渲染的地形、体积云、雷暴、
  完整的 MEDS 玻璃座舱和 HUD。整个仿真可以完全跑在浏览器里（Pyodide），不需要服务器。

> 本项目与 NASA 及任何航天机构无关。机体几何、气动和航电都是根据公开资料重新建模的近似，不是真实飞行数据。

---

## 目录

- [快速开始](#快速开始)
- [讲义](#讲义)
  - [第 0 课 问题的形状：一架没有发动机的飞机](#第-0-课-问题的形状一架没有发动机的飞机)
  - [第 1 课 规划模型：3-DOF 质点 + 配平气动表](#第-1-课-规划模型3-dof-质点--配平气动表)
  - [第 2 课 能量管理与航迹几何：给 SCP 一个好的初值](#第-2-课-能量管理与航迹几何给-scp-一个好的初值)
  - [第 3 课 序列凸优化（SCP）](#第-3-课-序列凸优化scp)
  - [第 4 课 模型预测控制（MPC）](#第-4-课-模型预测控制mpc)
  - [第 5 课 进近与着陆：Orbiter 式剖面和拉平律](#第-5-课-进近与着陆orbiter-式剖面和拉平律)
  - [第 6 课 当规划失败时：降级模式](#第-6-课-当规划失败时降级模式)
  - [第 7 课 内环：电传飞控](#第-7-课-内环电传飞控)
  - [课后思考题](#课后思考题)
- [仿真的逼真程度](#仿真的逼真程度)
- [测试与结果](#测试与结果)
- [代码结构](#代码结构)
- [已知限制](#已知限制)
- [参考文献](#参考文献)
- [许可证](#许可证)

---

## 快速开始

### 本地运行（Python 后端）

```bash
pip install -r requirements.txt        # numpy, scipy, websockets, cvxpy (+ CLARABEL / ECOS), osqp
python server.py                       # http://localhost:8000   (WebSocket: 8765)
```

用 Chrome 或 Edge 打开 `http://localhost:8000`（需要 WebGL2，最好有独立显卡）。
首次启动会生成世界数据（约 1 分钟，缓存到 `web/data/`）和配平表（约 6 秒，缓存到 `sim/data/`）。

| 参数 | 说明 |
|---|---|
| `--guidance scp_mpc\|hold_glide` | 制导律。默认 `scp_mpc`；`hold_glide` 是机翼水平滑翔的占位制导，用来对比 |
| `--spawn feasible\|any` | 出生点。默认 `feasible`，只在能飞回跑道的能量带内随机出生；`any` 完全随机 |
| `--seed N` | 固定随机出生点 |
| `--http-port` / `--ws-port` | 端口 |

后端没有启动时，网页会自动进入“本地预览”模式。这个模式只有运动学，没有气动，用来单独调画面。

### 浏览器版：整个仿真跑在访问者的浏览器里

浏览器版没有后端进程，服务器只提供静态文件。同一套 Python 仿真代码（6-DOF、飞控、SCP/MPC）通过
[Pyodide](https://pyodide.org)（WebAssembly 版 CPython）在两个 Web Worker 里运行：

- `web/js/sim/sim_worker.js`：仿真主循环，60 Hz，对应 `server.py`；
- `web/js/sim/plan_worker.js`：SCP 规划，对应服务器版的规划进程。

Worker 和页面之间的 JSON 消息与 WebSocket 版完全相同，页面通过 `core/local_sim.js` 把 Worker 当成一个 socket 使用。

```bash
python server.py  →  http://localhost:8000/?sim=browser    # 开发：不构建，直接在浏览器里跑仿真（Pyodide 走 CDN）
python tools/build_web.py --serve 8090                     # 构建 dist/ 并在本机预览 http://localhost:8090
python tools/deploy_web.py <ssh-host> [--dry-run]          # 构建并经 ssh + tar 发布到 <host>:/var/www/planelands
                                                           # （tools/deploy_web.sh / .cmd 是它的启动脚本；nginx 配置在 deploy/）
```

<details>
<summary>浏览器版的缓存、体积和性能细节</summary>

- **缓存**：每次发布是一个以内容哈希命名的目录 `v/<ver>/`；大数据按内容寻址放在 `d/<sha>/`；Pyodide 自托管在 `pyodide/<版本>/`。
  这三类都设成 `immutable` 长期缓存，只有 `index.html` 和 `version.json` 每次访问都会重新验证。已打开的页面每 5 分钟检查一次新版本。
  发布时只上传有变化的文件，最后才切换 `index.html`，保留最近 5 个版本。
- **体积**：首次访问约下载 35 MB。其中地形和天气数据约 10 MB（高程图用平面预测 + 字节分组无损编码，13.3 MB 压到 7.4 MB），
  Pyodide 加 numpy/scipy/cvxpy/clarabel 约 25 MB。体积云的 3D 噪声在页面的 Worker 里生成，不下载。
- **性能**：WebAssembly 里的 Python 比原生慢约 3 倍。仿真每 tick 约 11 ms（预算 16.7 ms），一次 SCP 规划约 1–3 s，在独立 Worker 中运行。
  CPU 跟不上时仿真时间会变慢，但不会丢步。
- **与原生的差异**：Pyodide 里没有 OSQP，MPC 改用 Clarabel 解同一个 QP，结果相差小于 5e-4。
  cvxpy 默认的 COO 规范化后端在 wasm32 上会因 int64→int32 转换出错，所以改用 SciPy 后端。
</details>

### 操作

| 操作 | 按键 / 鼠标 |
|---|---|
| 视角：追随 / 驾驶舱 / 塔台 / 跑道旁 / 飞掠 / 自由 / 俯瞰 | `1`–`7`，或底部按钮 |
| 环视 / 缩放 | 鼠标拖动 / 滚轮（塔台、跑道旁视角滚轮是变焦） |
| 暂停、随机重新放置、隐藏界面 | `P`、`R`、`H` |
| 手动驾驶（CSS 操纵杆模式） | `W/S` 俯仰、`A/D` 滚转、`Q/E` 偏航、`Z/X` 减速板、`G` 起落架、`B` 刹车、`C` 减速伞 |
| 自由相机 | `WASD` + `Q/E` 升降，`Shift` 加速 |
| MEDS 显示器窗口：PFD / 导航 | `F` / `N` |

右侧面板可以调云、降水、雷暴、风和湍流、时间日期、曝光和画质。
“短五边演示”会把飞机直接放到 RWY33 入口前 1 km 的拉起圆弧末段，用来单独观察拉平、接地和滑跑。

---

# 讲义

> 这部分按上课的顺序写。建议先读第 0 课建立全局图景，再按需要深入。
> 公式里的符号尽量和代码一致，代码位置用 `文件:函数` 标出。

## 第 0 课 问题的形状：一架没有发动机的飞机

### 0.1 能量是唯一的货币

在讲任何算法之前，我们先把物理问题想清楚。轨道器返回大气层后**没有任何推力**。它能利用的只有两样东西：高度和速度。
我们把两者合成一个量，叫**能量高度**（单位质量的机械能除以 $g$）：

```math
E_h \;=\; h + \frac{V^2}{2g}
```

对它求时间导数，代入质点动力学 $\dot h = V\sin\gamma$、$\dot V = -D/m - g\sin\gamma$（先不考虑风），可以得到：

```math
\dot E_h \;=\; \dot h + \frac{V\dot V}{g} \;=\; V\sin\gamma + \frac{V}{g}\Big(-\frac{D}{m} - g\sin\gamma\Big) \;=\; -\frac{D\,V}{m g}
```

请记住这个结论：**能量高度只会减少，减少的速率由阻力决定。** 再换成对地面距离 $s$ 的导数。
在稳定滑翔中 $\dot s = V\cos\gamma$，并且 $L\cos\phi \approx mg\cos\gamma$（$\phi$ 是倾侧角），于是

```math
\frac{dE_h}{ds} \;=\; -\frac{D}{mg\cos\gamma} \;\approx\; -\frac{1}{(L/D)\cos\phi}
```

这就是整门课的出发点：**每飞 1 m 地面距离，就要花掉 $1/(L/D)$ m 的能量高度**；转弯时还要再除以一个 $\cos\phi$。
升阻比 $L/D$ 能在一个范围内调节：

- 以最大升阻比 $(L/D)_\text{max}$ 飞行，花得最省，航程最远；
- 打开减速板、增大迎角，可以让 $L/D$ 降到 $(L/D)_\text{min}$，花得最快。

所以，从当前状态飞到目标点（地面路径长度为 $S$），所需的能量落在一个**能量带**里：

```math
\Delta E_\text{min} = \int_0^S \frac{ds}{(L/D)_\text{max}} \;\;\le\;\; \Delta E_\text{avail} = E_{h,0} - E_{h,f} \;\;\le\;\; \Delta E_\text{max} = \int_0^S \frac{ds}{(L/D)_\text{min}}
```

能量太少（左边不成立），无论怎么飞都够不着跑道；能量太多（右边不成立），减速板全开也消耗不掉，只能绕路。
而且没有复飞的机会，**只有一次**。无动力着陆问题的本质，就是在这个能量带里找一条可飞的路。

> **停一下想想**：同样的能量过剩，为什么“绕一圈”比“开减速板”更有效？
> 提示：减速板只能把 $L/D$ 从 4.9 降到 2 左右，而航迹长度可以翻倍。

### 0.2 分层：把一个难问题拆成几个容易的

从 25 km 高空到停在跑道上，大约 5–10 分钟。直接对这整段做一个 6-DOF 最优控制问题既不现实，也没必要。
工程上的做法是**按时间尺度分层**，每一层只解决它那个尺度上的问题：

```
 规划层   SCP 序列凸优化           每 ~20 s 或事件触发；一次求解 0.2–3 s
          跑道 / HAC 选择 → Dubins 初值 → 自由终端时间 SCP → TAEM 基准轨迹 x̄(t), ū(t)
                │  基准轨迹（到 ALI）+ 进近着陆剖面（ALI → 接地 → 滑跑）
                ▼
 跟踪层   LTV-MPC，2.5 Hz，28 s 预测时域             进近段纵向：着陆纵向律，10 Hz
          输出：迎角 α、倾侧角 φ、减速板 δsb
                │
                ▼
 内环     电传飞控 FCS，50 Hz：α/β/μ 外环 → 角速率 → 动态逆 → 控制分配
          输出：4 片升降副翼、方向舵、体襟翼（减速板直通）
                │
                ▼
 被控对象 6-DOF 刚体 + 分片气动 + 起落架，100 Hz（近地 400 Hz）
```

这种结构有一个核心思想：**每一层都把下一层当成一个“理想化的执行机构”**。

- 规划层看到的飞机是一个 3-DOF 质点。它假设迎角和倾侧角可以直接设定，只要变化速率不超过限制；力矩平衡交给飞控处理。
- MPC 在规划给出的轨迹附近做线性化，处理规划层没有考虑到的风、湍流和模型误差。
- 飞控负责把“我要这个迎角和倾侧角”变成具体的舵面偏角。

接下来几课，我们自上而下逐层讲解。

### 0.3 飞行阶段

```
PLAN → TAEM（ACQ 截获 → HAC 航向对准圆 → PRF 五边前段）
     → A/L（OGS 外下滑道 → PREFLARE 预拉平 → IGS 内下滑道 → FLARE 最终拉平）
     → TOUCHDOWN（降前轮）→ ROLLOUT（减速伞、刹车、前轮转向）→ STOPPED
```

- **TAEM**（Terminal Area Energy Management，末端区域能量管理）：从当前位置飞到**进近着陆界面 ALI**，同时把能量调整到恰好合适。
  这一段由 SCP 规划，由 MPC 跟踪。
- **A/L**（Approach and Landing，进近着陆）：从 ALI 开始沿固定的几何剖面下滑并拉平接地。这段剖面是事先算好的，不需要在线优化。

坐标约定：世界坐标是 ENU（东、北、天），单位 m。航向 $\psi$ 从正北顺时针量起。
每个跑道端有自己的跑道坐标系：$x$ 沿着陆方向，入口处 $x=0$；$y$ 指向着陆方向的右侧。

---

## 第 1 课 规划模型：3-DOF 质点 + 配平气动表

> 代码：`sim/gnc/model.py`

### 1.1 状态、控制与运动方程

规划用的模型故意做得比真实飞机简单。状态和控制分别是

```math
x = [\,E,\ N,\ h,\ V,\ \gamma,\ \psi\,]^\top, \qquad u = [\,\alpha,\ \phi,\ \delta_{sb}\,]^\top
```

其中 $V$ 是真空速，$\gamma$ 和 $\psi$ 是**相对空气**的航迹角和航向，$\alpha$ 是迎角（度），$\phi$ 是倾侧角（弧度），$\delta_{sb}$ 是减速板开度（0–98.6°）。
运动方程为

```math
\begin{aligned}
\dot E &= V\cos\gamma\sin\psi + w_E(h) \\
\dot N &= V\cos\gamma\cos\psi + w_N(h) \\
\dot h &= V\sin\gamma \\
\dot V &= -\frac{D}{m} - g(h)\sin\gamma \\
\dot\gamma &= \frac{L\cos\phi - m g(h)\cos\gamma}{mV} \\
\dot\psi &= \frac{L\sin\phi}{mV\cos\gamma}
\end{aligned}
\qquad
\begin{aligned}
L &= \bar q S\, C_L(\alpha, M, \delta_{sb}, \text{gear}) \\
D &= \bar q S\, C_D(\alpha, M, \delta_{sb}, \text{gear}) \\
\bar q &= \tfrac12\rho(h)V^2,\quad M = V/a(h) \\
g(h) &= g_0\big(R_E/(R_E+h)\big)^2
\end{aligned}
```

- 大气用 US1976 标准大气（0–86 km，向量化实现）。
- 风 $w(h)$ 是平均风廓线：500 m 以下按 1/7 次幂律，$|w| = w_{10}\,(h_\text{AGL}/10)^{1/7}$；再往上保持不变。
- 质量 $m = 99.8$ t，参考面积 $S = 249.9\ \text{m}^2$。

### 1.2 为什么要用“配平后”的气动表

真实飞机的 $C_L$ 和 $C_D$ 不只取决于迎角，还取决于舵面偏角。而舵面偏角又是为了让俯仰力矩为零（配平）才偏的。
如果规划模型直接用“干净构型”的气动系数，就会漏掉**配平阻力**：升降副翼和体襟翼为了配平而偏转时产生的额外阻力。
在跨声速区，这部分阻力相当可观。

所以我们离线做一件事（`sim/trim.py`）：在网格

```math
\alpha \in [-4^\circ, 40^\circ]\ (\text{步长}\ 1^\circ) \;\times\; M \in \{0.2,\dots,3.0\}\ (21\ \text{点}) \;\times\; \delta_{sb}\in\{0,25,50,75,98.6\} \;\times\; \text{gear}\in\{0,1\}
```

的每个点上，用完整的分片气动模型求出 $C_m = 0$ 时的升降副翼和体襟翼偏角，并记录此时的 $C_L$、$C_D$，以及“这个点能否配平”。
规划模型做四维多线性插值（`PlanningModel.coeffs`）。所有运算都对任意前导维度向量化，后面计算 SCP 的雅可比矩阵时会大量用到这一点。

这张表还给出两个关键边界：

- **可用最大迎角** $\alpha_\text{max}(M)$：取“能配平的最大迎角减 1°”和“对称升降副翼不低于 −12°”中较小的一个。
  第二个条件为滚转和偏航留出至少 21° 的舵面行程。在 Mach 1.2–1.6 附近体襟翼已经饱和，配平全靠升降副翼，所以可用迎角只剩 9–12°。
- **最佳/最差升阻比** $(L/D)_\text{max}(M)$ 和 $(L/D)_\text{min}(M)$：第 2 课估算能量带时要用。

### 1.3 控制速率：超声速时转不动

倾侧角的变化速率并不是想多快就多快。超声速时垂尾被机身遮挡，方向舵效能很低，协调滚转很困难。
规划模型里的倾侧速率上限随 Mach 数变化：

```math
\dot\phi_\text{max}(M) = \text{interp}\big(M;\ [0.9,\,1.3,\,2.0,\,3.0] \mapsto [8,\,5,\,2.5,\,2]\ ^\circ/\text{s}\big)
```

迎角速率限制为 2°/s，减速板为 5°/s（实际作动器是 6.1°/s，这里故意留了余量）。

### 1.4 流映射（flow map）

SCP 和 MPC 都不直接使用 $\dot x = f(x,u)$，而是使用它在一个时间步上的积分结果，即**流映射**：

```math
x_{k+1} = \Phi\big(x_k,\ u_k,\ u_{k+1},\ \Delta t\big)
```

实现上，在区间内用 3 个 RK4 子步积分。控制量在区间内按一阶保持（FOH）线性插值：$u(t) = u_k + (u_{k+1}-u_k)\,t/\Delta t$。
`PlanningModel.flow` 一次调用就能对所有区间同时积分。

---

## 第 2 课 能量管理与航迹几何：给 SCP 一个好的初值

> 代码：`sim/gnc/geometry.py`

SCP 是**局部**方法：它只在初值附近改进，不会自己从“左转还是右转”“降 33 号还是 15 号跑道”这样的离散选择里跳出来。
所以在 SCP 之前，必须先用几何方法解决离散决策，并构造一条大致可行的初值轨迹。真实的航天飞机 TAEM 制导也是这么做的，
用的是 HAC（Heading Alignment Cone，本仿真里简化为圆柱）。

### 2.1 目标：ALI 与 HAC 的几何

取 `ApproachConfig` 的默认值：

| 量 | 值 | 说明 |
|---|---|---|
| 外下滑道角 $\gamma_\text{OGS}$ | −20° | 轻载 Orbiter 的数值 |
| OGS 瞄准点 | 入口前 1500 m | 真实 Orbiter 为 7500 ft，原因见第 5 课 |
| ALI 高度 | 跑道上方 3000 m | |
| ALI 位置 | $x_\text{ALI} = -1500 - 3000/\tan 20^\circ \approx -9742$ m | |
| ALI 速度 | 290 KEAS（≈149 m/s EAS） | |
| 五边前段（WP2 → ALI） | 2500 m 直线，WP2 在 $x \approx -12242$ m | |
| HAC 半径 | 5500 m | 两侧都可以 |

所以 SCP 的终端目标是一个**完整的 6 维状态**：

```math
x_f = \big[\,E_\text{ALI},\ N_\text{ALI},\ h_\text{rwy}+3000,\ V_\text{TAS}(290\,\text{KEAS}),\ -20^\circ,\ \psi_\text{rwy}\,\big]
```

### 2.2 Dubins CSC 航迹

从当前位置 $p_0$、航向 $\psi_0$ 到 WP2（航向等于跑道方向），水平面上最短的“转弯–直线–转弯”路径叫 Dubins CSC 路径：

- 第一个圆的半径 $r_1$ 由当前速度决定，取 45° 坡度的等速转弯半径 $r_1 = V^2/(g\tan 45^\circ)$，并限制在 3–40 km 之间；
- 第二个圆就是 HAC，半径 $r_\text{HAC}$；
- 两圆之间是公切线。

每个候选方案由以下组合确定：跑道端（33/15）× 第一个圆的转向（左/右）× HAC 侧（左/右）× HAC 上的额外圈数（0/1/2）。

**高速出生时的首转弯半径**：Mach 1.6、20 km 高度时，45° 坡度的转弯半径是 22.9 km，转回来要走 115 km，这显然是低估了飞机的能力。
实际上飞机在转弯过程中一边下滑，一边减速，动压大致不变而真空速下降，转弯半径 $\propto V^2$ 越转越小。
`effective_turn_radius` 把转弯分成 12 段积分，每段：

```math
n_\text{max} = \min\!\Big(\frac{C_{L,\max}\,\bar q S}{mg},\ 2\Big),\quad \phi = \min\!\big(\arccos(1/n_\text{max}),\ 55^\circ\big),\quad r = \frac{V^2}{g\tan\phi},\quad \Delta E_h = \frac{r\,\Delta\theta}{(L/D)_\text{max}\sqrt{\cos\phi}}
```

然后由新的 $E_h$ 和保持的 $\bar q$ 反解出 $h$ 和 $V$。这个“减速转弯半径”会作为额外的 $r_1$ 候选。

### 2.3 沿路径的能量预算

对每条候选路径，`path_energy_budget` 沿路径采样，估算所需能量的上下界：

1. 假设能量高度随路径长度线性下降，$E_h(s) = E_{h,0} + (E_{h,f}-E_{h,0})\,s/S$；
   动压在前 2/3 路径内从当前值线性过渡到 ALI 的值。由 $E_h$ 和 $\bar q$ 反解出 $h(s)$、$V(s)$ 和 $M(s)$：
   ```math
   h + \frac{\bar q}{\rho(h)\,g} = E_h \quad(\text{对 } h \text{ 二分})
   ```
2. 由路径曲率 $\kappa$ 求所需倾侧角 $\phi = \arctan(V^2|\kappa|/g)$。
3. 计入顺逆风，地速比为 $\eta = (V + w_\parallel)/V$。
4. 累加：
   ```math
   \Delta E_\text{min} = \sum \frac{\Delta s}{(L/D)_\text{max}(M)\,\sqrt{\cos\phi}\;\eta}, \qquad
   \Delta E_\text{max} = \sum \frac{\Delta s}{(L/D)_\text{min}(M)\;\eta\;(0.85\ \text{if turning})}
   ```

这里转弯的折减用了 $\sqrt{\cos\phi}$，比第 0 课推出的 $\cos\phi$ 温和一些。这是一个经验取值，因为转弯中速度和迎角会一起调整。

候选方案**可行**的条件是能量落在能量带内，并且两端各留余量：

```math
1.04\,\Delta E_\text{min} \;\le\; \Delta E_\text{avail} \;\le\; 0.97\,\Delta E_\text{max}
```

再定义**能量带位置**：

```math
b = \frac{\Delta E_\text{avail} - \Delta E_\text{min}}{\Delta E_\text{max} - \Delta E_\text{min}} \in [0, 1]
```

$b=0$ 表示必须全程以最大升阻比飞行，$b=1$ 表示必须全程减速板全开。我们的目标是 $b^\star = 0.35$，偏低一些，
因为阻力总可以增加，升阻比却不能超过最大值。在所有可行方案中，按下面的评分选最小（`plan_score`）：

```math
J_\text{route} = |b - 0.35| + 0.04\max(w_\text{tail}-2,\,0) + 0.05\,n_\text{loops} + 1.5\times10^{-6}\,S + \dots
```

各项分别代表：偏好逆风着陆、少绕圈、航迹短。拉长的航迹还要加上逆风暴露量和 HAC 半径偏离标称值的惩罚。

### 2.4 能量方向：先判断“多了还是少了”

如果没有一个标称方案可行，先问一个问题：**能量是多了还是少了？**（`energy_direction`）

- 只要有一条路线的 $\Delta E_\text{avail} > 0.97\,\Delta E_\text{max}$，就判为**过剩**，然后拉长航迹：
  - 对每种拓扑，把 HAC 半径乘以系数 $k \in [0.76, 2.2]$，先网格搜索再二分，使 $b$ 落在 0.35；圈数只是把搜索带到下一个区间，所以不同圈数之间没有空档；
  - 还不行就试 S 形转弯（偏出 60° 或 90°，偏出段长度连续搜索）和起始盘旋（半径从可飞的最小值开始连续搜索）；
  - 拉长后的航迹还要满足一个鲁棒性条件：逆风增强 30% 时仍然可达。
- 所有路线都不够时，判为**不足**，改用缩短的进近：降低 ALI、缩短五边前段、缩小 HAC。
- 如果把首转弯换成亚声速半径就够得着，判为**转弯受限**（TURN LIMITED）。超声速时升力只能提供约 1 g，飞机暂时转不回来，但能量并不缺。

为什么先判方向？早期版本先尝试缩短方案，结果高能量出生点常常被错误地放到低能量方案上，还报告 LOW ENERGY。顺序很重要。

### 2.5 构造初值轨迹

选定路线后，`initial_guess` 把几何路径变成一条完整的 $(X, U, t_f)$：

```math
\begin{aligned}
&h(s),\ V(s) \leftarrow (E_h(s),\ \bar q(s)) && \text{同 2.3} \\
&\gamma = \arctan(dh/ds), && \phi = \text{clip}\big(\arctan(V^2\kappa/g),\ \pm55^\circ\big) \\
&n = \cos\gamma/\cos\phi, && C_L = \frac{n\,mg}{\bar q S},\quad \alpha = C_L^{-1}(C_L, M, \delta_{sb}) \\
&\delta_{sb} = \text{clip}\Big(160\big(1 - \tfrac{(L/D)_\text{need}}{(L/D)_\text{max}}\big),\ 0,\ 70\Big), && (L/D)_\text{need} = S/\Delta E_\text{avail} \\
&t(s) = \int \frac{ds}{V\cos\gamma + w_\parallel}
\end{aligned}
```

最后按时间均匀重采样到 $N$ 个节点，首节点等于当前状态，末节点等于 ALI。

**一个实例**（下文 SCP 迭代日志用的就是它）：出生在 $(E,N) = (-30, -25)$ km、高度 15 km、$V=270$ m/s（Mach 0.91）、$\gamma = -6^\circ$、航向 045°。
`select_plan` 用 44 ms 选出 RWY15、右侧 HAC、0 圈，能量预算为
$\Delta E_\text{min} = 11.8$ km，$\Delta E_\text{avail} = 14.2$ km，$\Delta E_\text{max} = 19.1$ km，能量带位置 $b = 0.33$。
初值的飞行时间是 249 s，对应 $N = 35$ 个节点。

---

## 第 3 课 序列凸优化（SCP）

> 代码：`sim/gnc/scp.py`。方法属于 Szmuk、Açıkmeşe、Malyuta 等人发展的 PTR（penalized trust region）一类序列凸化，参考文献 [1][2]。

### 3.1 我们要解的连续时间问题

```math
\begin{aligned}
\min_{x(\cdot),\,u(\cdot),\,t_f}\quad & t_f \;+\; \text{(控制平滑与能耗正则项)} \\
\text{s.t.}\quad & \dot x = f(x, u), \qquad x(0) = x_0, \qquad x(t_f) = x_f \\
& \alpha_\text{min} \le \alpha \le \alpha_\text{max}(M), \quad |\phi| \le \phi_\text{max}(M), \quad 0 \le \delta_{sb} \le 98.6^\circ \\
& |\dot\alpha| \le 2^\circ/\text{s}, \quad |\dot\phi| \le \dot\phi_\text{max}(M), \quad |\dot\delta_{sb}| \le 5^\circ/\text{s} \\
& 4 \le \bar q(h,V) \le 15\ \text{kPa}, \qquad n_z(h,V,\alpha) \le 2.5\,g \\
& h \ge T(E,N) + 600\ \text{m}, \qquad -35^\circ \le \gamma \le 12^\circ, \qquad V \ge 70\ \text{m/s}
\end{aligned}
```

这个问题**非凸**，难点有三处：

1. 动力学 $f$ 是非线性的：三角函数、插值表、大气密度随高度呈指数变化；
2. 动压、过载、地形约束都是状态的非线性函数；
3. 终端时间 $t_f$ 也是决策变量，网格长度本身未知。

SCP 的思路是：**在当前猜测附近把问题线性化，得到一个凸子问题；求解后用新解作为新的猜测，反复迭代。**
关键在于如何让这个过程既收敛，又不被线性化误差带偏。下面逐步讲。

### 3.2 时间伸缩：把 $t_f$ 变成普通变量

令 $t = s\,\tau$，$\tau \in [0,1]$，其中 $s \equiv t_f$ 是一个标量决策变量。于是

```math
\frac{dx}{d\tau} = s\, f(x, u)
```

把 $\tau$ 等分成 $K = N-1$ 段，$\Delta\tau = 1/K$，每段的物理时长是 $s\,\Delta\tau$。这样问题的网格固定了，$s$ 只是一个普通变量。

### 3.3 离散化：一阶保持 + 精确流映射 + 有限差分雅可比

控制量取一阶保持（FOH）：$u(\tau) = (1-\lambda)u_k + \lambda u_{k+1}$，$\lambda \in [0,1]$。
设上一轮的解为 $(\bar x, \bar u, \bar s)$，把流映射在它附近做一阶展开：

```math
x_{k+1} \;\approx\; A_k x_k + B^-_k u_k + B^+_k u_{k+1} + S_k\, s + z_k
```

```math
A_k = \frac{\partial\Phi}{\partial x_k},\quad B^-_k = \frac{\partial\Phi}{\partial u_k},\quad B^+_k = \frac{\partial\Phi}{\partial u_{k+1}},\quad S_k = \Delta\tau\,\frac{\partial\Phi}{\partial(\Delta t)},\quad
z_k = \Phi(\bar x_k,\bar u_k,\bar u_{k+1},\bar s\Delta\tau) - A_k\bar x_k - B^-_k\bar u_k - B^+_k\bar u_{k+1} - S_k\bar s
```

教科书上的写法通常是对 $\dot x = f$ 线性化，再把 $A(t)$、$B(t)$ 和状态转移矩阵一起积分。
本项目用了一个更直接的办法：**对 RK4 流映射做中心差分**。

```math
A_k e_i \approx \frac{\Phi(\bar x_k + \varepsilon_i e_i,\,\cdot) - \Phi(\bar x_k - \varepsilon_i e_i,\,\cdot)}{2\varepsilon_i}
```

扰动量为 $\varepsilon_x = (2, 2, 2\ \text{m},\ 0.05\ \text{m/s},\ 10^{-4},\ 10^{-4}\ \text{rad})$，$\varepsilon_u = (0.02^\circ,\ 10^{-3}\ \text{rad},\ 0.1^\circ)$。
6 个状态、3+3 个控制和 1 个时间，共 13 个方向，每个方向正负各一次，加上基准点，一共 27 组。
$K$ 个区间 × 27 组全部堆成一个批次，**一次向量化调用就能算完**（`PlanningModel.flow_jac`）。49 段大约 10 ms。
气动是查表插值，这样做比推导解析雅可比简单得多，也不容易出错。

### 3.4 凸子问题

每一轮迭代解下面这个问题。所有量都先**缩放到 O(1)**（见 3.5），记号上的帽子省略不写：

```math
\begin{aligned}
\min\quad & \underbrace{w_{t}\,s}_{\text{飞行时间}}
 + \underbrace{w_{\Delta u}\tfrac{N}{40}\sum_k \lVert u_{k+1}-u_k\rVert_2^2}_{\text{控制平滑}}
 + \underbrace{w_\phi\tfrac{40}{N}\sum_k \phi_k^2 + w_{sb}\tfrac{40}{N}\sum_k \delta_{sb,k}}_{\text{少压坡度、少开减速板}} \\
& + \underbrace{w_\nu\sum_k\lVert\nu_k\rVert_1}_{\text{虚拟控制}}
 + \underbrace{\textstyle\sum_i w_{f,i}\,|\sigma_{f,i}|}_{\text{终端松弛}}
 + \underbrace{w_\text{soft}\textstyle\sum_k(\sigma_{q,k} + \sigma'_{q,k} + \sigma_{n,k} + 0.01\,\sigma_{T,k})}_{\text{软约束}} \\
& + \underbrace{w_\text{tr}\big(\lVert X-\bar X\rVert_F^2 + \lVert U-\bar U\rVert_F^2 + (s-\bar s)^2\big)}_{\text{信赖域惩罚}} \\[4pt]
\text{s.t.}\quad
& x_{k+1} = A_k x_k + B^-_k u_k + B^+_k u_{k+1} + S_k s + z_k + \nu_k && \text{线性化动力学 + 虚拟控制} \\
& x_0 = x(t_0), \qquad x_{N-1} - x_f = \sigma_f && \text{初始状态；终端（带松弛）} \\
& \alpha_{\text{lo},k} \le \alpha_k \le \alpha_{\text{hi},k}, \quad |\phi_k| \le \phi_{\text{max},k}, \quad 0 \le \delta_{sb,k} \le 98.6^\circ && \text{控制边界} \\
& |u_0 - u_\text{prev}| \le (3^\circ,\ 12^\circ,\ 8^\circ) && \text{与当前指令衔接} \\
& |u_{k+1} - u_k| \le \dot u_\text{max}\cdot s\,\Delta\tau && \text{速率约束（对 } s \text{ 仍是线性的！）} \\
& \bar q_k + q_h(h_k - \bar h_k) + q_V(V_k - \bar V_k) \in [\,4 - \sigma'_{q,k},\ 15 + \sigma_{q,k}\,]\ \text{kPa} && \text{动压（线性化，软）} \\
& \bar n_k + n_h\Delta h_k + n_V\Delta V_k + n_\alpha\Delta\alpha_k \le 2.5 + \sigma_{n,k} && \text{过载（线性化，软）} \\
& h_k \ge T(\bar E_k,\bar N_k) + \nabla T\cdot(E_k-\bar E_k,\ N_k-\bar N_k) + 600 - \sigma_{T,k} && \text{地形净空（线性化，软）} \\
& \gamma_k \in [-35^\circ, 12^\circ], \quad V_k \ge 70, \quad \max(0.25\bar s, 20) \le s \le 2000\ \text{s} \\
& \lVert x_k - \bar x_k\rVert_\infty \le r, \quad \lVert u_k - \bar u_k\rVert_\infty \le 2r, \quad |s - \bar s| \le r && \text{硬信赖域}
\end{aligned}
```

代码中的权重：$w_t = 1$，$w_{\Delta u} = 3$，$w_\phi = 0.05$，$w_{sb} = 0.002$，$w_\nu = 1000$，$w_\text{soft} = 50$，$w_\text{tr} = 1$，
$w_f = (300, 300, 300, 60, 60, 60)$。

下面逐项解释几个关键设计：

**(a) 虚拟控制 $\nu_k$。** 线性化后的约束可能与初始状态、终端状态互相矛盾，导致子问题**人为不可行**（artificial infeasibility）。
在动力学里加一个自由变量 $\nu_k$，并用很大的 L1 权重惩罚它，子问题就总是可行的。L1 范数是精确罚函数：
只要原问题可行且权重足够大，最优解里 $\nu$ 会恰好为零，而不只是接近零。

**(b) 信赖域。** 线性化只在 $(\bar x, \bar u, \bar s)$ 附近可信，所以要限制每一步走多远。本项目同时用了两种：

- 二次惩罚 $w_\text{tr}\lVert\cdot\rVert^2$：PTR 的标准做法，让步长成为优化的一部分；
- 硬性 L∞ 盒子 $r$：根据线性化质量自适应调节。

线性化质量用**非线性缺陷**（defect）来衡量：把子问题的解代回真正的非线性流映射，看节点之间接不接得上：

```math
d_k = \big(\Phi(x_k, u_k, u_{k+1}, s\Delta\tau) - x_{k+1}\big) \oslash D_x, \qquad d_\text{max} = \max_k \lVert d_k\rVert_\infty
```

调节规则：$d_\text{max} > 0.05$ 时 $r \leftarrow \max(0.5r,\ 0.02)$；$d_\text{max} < 0.01$ 时 $r \leftarrow \min(1.5r,\ 2)$；子问题求解失败时 $r \leftarrow 0.5r$。初值 $r_0 = 1$。

> **请注意区分 $\nu$ 和 $d$。** $\nu$ 衡量的是**线性化模型**里还差多少；$d$ 衡量的是**真实非线性模型**里还差多少。
> 下面的迭代日志里，第 1 步 $\nu = 0$，但 $d = 0.37$。线性模型认为一切完美，实际上差得很远。
> 收敛判据必须看 $d$。

**(c) 速率约束对 $s$ 是线性的。** $|u_{k+1}-u_k| \le \dot u_\text{max}\,s\,\Delta\tau$ 的右边是变量 $s$ 乘以常数，所以仍然是凸约束。
这是时间伸缩带来的一个好处：飞得越久，每段的时间越长，允许的控制变化也越大，这个关系在凸问题里可以直接表达。

**(d) 非线性路径约束要线性化，并且设为软约束。** 动压、过载和地形都在 $\bar x$ 处做一阶展开。例如

```math
\bar q(h,V) \approx \bar q + \underbrace{\tfrac12\bar V^2\frac{\partial\rho}{\partial h}}_{q_h}(h-\bar h) + \underbrace{\bar\rho\bar V}_{q_V}(V-\bar V)
```

它们都带有非负松弛变量 $\sigma$，以 $w_\text{soft}=50$ 计入代价。这样做是因为初值往往违反这些约束，硬约束会让第一轮直接不可行。
地形先做了 800 m 网格的最大值滤波和平滑（`TerrainField`），这样梯度不会因为山脊的尖角而剧烈跳变。

**(e) 终端约束也带松弛。** 终端误差 $\sigma_f$ 以加权 L1 惩罚。位置和高度的权重是 300，速度和角度是 60。
同时，终端节点的倾侧角限制为 3°：到达 ALI 时机翼基本要放平。

**(f) 起始控制量的处理。** 迎角上下界会自动放宽到“从当前指令按速率限制能够到达的范围”。
否则，如果当前迎角已经高于这个马赫数下的控制余量上限，第一轮子问题就会不可行。

### 3.5 缩放：凸优化求解器最在意的事

$E$ 和 $N$ 的量级是 $10^4$ m，$\gamma$ 的量级是 $10^{-1}$ rad。如果不缩放，内点法的数值条件会很差。所有变量都做仿射缩放：

```math
\hat x = (x - x_c) \oslash D_x, \quad D_x = (10\,\text{km},\ 10\,\text{km},\ 3\,\text{km},\ 100\,\text{m/s},\ 0.3,\ 1); \qquad
\hat u = u \oslash D_u, \quad D_u = (10^\circ,\ 0.5\,\text{rad},\ 50^\circ); \qquad \hat s = s/100\,\text{s}
```

其中 $x_c$ 取起点和 ALI 的水平中点。雅可比矩阵也相应变换：$\hat A = D_x^{-1} A D_x$，$\hat B = D_x^{-1} B D_u$。
信赖域半径 $r$ 和缺陷 $d$ 都在缩放后的坐标里定义，所以“$r=1$”的含义是“每个量最多变化一个特征尺度”。

### 3.6 迭代流程与收敛判据

```
输入：当前状态 x0、当前指令 u_prev、ALI 目标 xf、初值 (X̄, Ū, s̄)
r ← 1
for it = 1 … 12（热启动时 6）:
    沿 (X̄, Ū, s̄) 计算 A, B⁻, B⁺, S, z 和所有线性化约束系数   # 一次向量化调用
    解凸子问题（CLARABEL；失败时用 ECOS）
    if 不可行:  r ← r/2;  continue
    d_max ← 用非线性流映射计算的缺陷
    (X̄, Ū, s̄) ← 新解
    if d_max < 5e-3 且 ‖ν‖₁ < 1e-3 且（步长 < 2e-3 或代价相对变化 < 1%）:
        收敛，退出
    按 d_max 调整 r
```

子问题用 cvxpy 搭建，并采用 **DPP（Disciplined Parametrized Programming）** 形式：所有随迭代变化的量
（$A$、$B$、$z$、边界、线性化系数、信赖域中心和半径）都声明为 `cp.Parameter`。
这样每个节点数 $N$ 只需要编译一次，之后每次迭代只是更新参数，省掉了反复做规范化的开销。
节点数按飞行时间选取，约每 7.5 s 一个节点：$N = \text{clip}\big(5\,\text{round}(t_f/37.5),\ 20,\ 45\big)$。

**上面那个实例的迭代日志**（RWY15，$N=35$，本机原生 Python）：

| 迭代 | 步长 $\lVert\Delta\rVert_\infty$ | $\lVert\nu\rVert_1$ | 缺陷 $d_\text{max}$ | $t_f$ (s) | 代价 |
|---|---|---|---|---|---|
| 1 | 1.501 | 0.0000 | **0.3665** | 246.9 | 15.37 |
| 2 | 0.452 | 0.0000 | 0.0142 | 252.4 | 5.59 |
| 3 | 0.070 | 0.0000 | 0.0009 | 253.6 | 4.58 |
| 4 | 0.049 | 0.0000 | 0.0005 | 254.5 | 4.40 |
| 5 | 0.035 | 0.0000 | 0.0004 | 255.3 | 4.30 |
| 6 | 0.027 | 0.0000 | 0.0004 | 256.2 | 4.25 |
| 7 | 0.024 | 0.0000 | 0.0003 | 257.0 | 4.20 |

结果：7 次迭代收敛，用时 340 ms，终端误差在显示精度下为零。可以看到：

- 第 1 步迈得很大（1.5 个特征尺度），线性化严重失准，缺陷达到 0.37。第 2 步立刻把缺陷压到 0.014，之后进入二次收敛区。
- $t_f$ 从 247 s 增加到 257 s。初值里的能量调度并不最优，SCP 选择多飞 10 秒，以换取更平滑的控制。
- 最后几步代价的相对变化小于 1%，触发了收敛判据。

### 3.7 接受与恢复

SCP 的结果要满足以下条件才算“好计划”：缺陷 < 0.02，终端误差满足 $|\Delta E|,|\Delta N| < 300$ m、$|\Delta h| < 150$ m、
$|\Delta V| < 8$ m/s、$|\Delta\gamma| < 3^\circ$、$|\Delta\psi| < 8^\circ$。

如果没有到达 ALI，就看终端误差往哪个方向偏，然后做出相应调整（`planner.plan`）。用到达 ALI 时的**能量高度超量**来判断：

```math
e_\text{exc} = \Delta h + \frac{V_\text{ALI}\,\Delta V}{g}
```

- $e_\text{exc} > 0$，即太高或太快：把能量带目标 $b^\star$ 下调 $e_\text{exc}/(\Delta E_\text{max}-\Delta E_\text{min})$，然后选一条更长的路线（更大的 HAC、多一圈、S 形转弯或盘旋）；
- 太低或太慢：换成缩短的进近；
- 结果完全不可用：重新做几何选择；
- 同一条路线不会重复尝试。

还有一个反方向的规则：几何估算认为不可达，而 SCP 实际到达了 ALI，那么**以 SCP 为准**。几何估算只是启发式。

### 3.8 什么时候重规划

只有 TAEM 段会重规划，并且距 ALI 要超过 35 s。触发条件：

- 定时：每 20 s 一次，从当前基准热启动；
- 误差过大：水平偏差 > 1500 m，或高度偏差 > 600 m，或能量偏差 > 900 m，或 MPC 连续失败 3 次；
- 风变化：10 m 高度的风向量变化超过 3 m/s。

在 HAC 截获之前（并且距 ALI > 60 s），重规划还可以**重新选择跑道端和 HAC 侧**。这是热启动 SCP 无法做到的离散决策。
为了防止来回切换，设置了滞回：每次飞行最多切换 2 次，两次之间至少间隔 40 s。

实时服务里规划在独立进程中运行，浏览器版在独立的 Web Worker 中运行，求解期间仿真循环不会卡住。

---

## 第 4 课 模型预测控制（MPC）

> 代码：`sim/gnc/mpc.py`

### 4.1 为什么有了最优轨迹还需要 MPC

SCP 给出的是**开环**最优轨迹。如果直接按时间回放它的控制序列 $\bar u(t)$，飞机很快就会偏离。原因至少有四个：

1. **模型误差**：规划用的是 3-DOF 配平表，真实对象是 6-DOF 分片气动加飞控，内环有动态响应；
2. **风和湍流**：规划只知道平均风廓线，实际还有阵风、Dryden 湍流，以及雷暴中的上升和下沉气流；
3. **离散化**：规划节点间隔约 7.5 s；
4. **计算延迟**：一次规划要几百毫秒到几秒，期间飞机一直在飞。

所以需要一个**闭环**的跟踪器。MPC 的做法是：每 0.4 s，以当前实际状态为起点，在基准轨迹附近求解一个短时域的线性二次问题，
只执行第一段控制，下次再从头求解。这叫**滚动时域**（receding horizon）。

### 4.2 空间重索引：不要和时间表赛跑

跟踪时间参数化的轨迹有一个陷阱。假设飞机因为逆风比计划晚了 10 s 到达某处，如果仍然按时钟跟踪 $\bar x(t)$，
MPC 会认为飞机“落后了 1.5 km”，于是压低机头加速去追。对滑翔机来说这是在白白浪费能量。

解决办法是**按空间位置**找参考点。在上一次索引附近的窗口内（往回 30 个点，往前 400 个点，保证单调前进），
找加权距离最近的基准点，并在线段上细化到小数索引：

```math
i^\star = \arg\min_i \big\lVert W\,(p_i - p)\big\rVert^2, \qquad W = \text{diag}(1,\ 1,\ w_h)
```

TAEM 段取 $w_h = 3$，高度偏差计入投影，使参考点同时反映能量状态。
A/L 段取 $w_h = 0.15$，几乎只按水平位置投影，因为 A/L 的纵向由“按距离给高度”的着陆律负责（第 5 课）。

找到 $i^\star$ 对应的基准时间 $\tau_0$ 后，预测时域取 $t_k = \tau_0 + k\Delta t$，$k = 0..K$，其中 $K=28$，$\Delta t = 1$ s。

### 4.3 沿基准的线性时变模型

把基准插值到这些时刻，得到 $(x^r_k, u^r_k)$。仍然用 RK4 流映射加中心差分做线性化，不过这里改为零阶保持（ZOH），每段 2 个 RK4 子步。
定义偏差 $\delta x = x - x^r$，$\delta u = u - u^r$：

```math
\delta x_{k+1} = A_k\,\delta x_k + B_k\,\delta u_k + c_k, \qquad c_k = \Phi(x^r_k, u^r_k, \Delta t) - x^r_{k+1}
```

$c_k$ 是基准轨迹自身的“缺陷”。基准是 SCP 节点经过插值得到的，又拼接了 A/L 剖面，并不严格满足动力学，所以把它显式写进模型。
航向分量要把差值包裹到 $(-\pi, \pi]$。

### 4.4 凝聚（condensing）

把 $K$ 步的预测写成只关于控制序列的形式。令 $\delta U = [\delta u_0; \dots; \delta u_{K-1}]$（84 维），则

```math
\delta X = \Phi_x\,\delta x_0 + \Gamma\,\delta U + C
```

其中 $\Phi_x$、$\Gamma$、$C$ 递推构造：

```math
\Phi_{k+1} = A_k\Phi_k,\qquad \Gamma_{k+1} = A_k\Gamma_k + [\,0\ \cdots\ B_k\ \cdots\ 0\,],\qquad C_{k+1} = A_k C_k + c_k
```

凝聚后，决策变量只剩 84 个控制量，没有状态变量，也没有等式约束。这样得到的是一个小而稠密的 QP。

### 4.5 代价函数：用 Bryson 法则选权重

权重矩阵的设计原则是 **Bryson 法则**：每一项除以它的“可接受偏差”的平方，$Q_{ii} = 1/\sigma_i^2$。这样调参就变成了回答一个直观的问题：
多大的偏差可以接受？

水平位置偏差不应该在 E、N 方向上同等对待。**侧偏**需要严格控制，**沿航迹偏差**则可以宽松，原因见 4.2。
所以先旋转到航迹坐标系，再加权：

```math
Q^{(EN)}_k = c\,c^\top/\sigma_\perp^2 + a\,a^\top/\sigma_\parallel^2, \qquad a = (\sin\psi^r_k,\ \cos\psi^r_k),\ \ c = (\cos\psi^r_k,\ -\sin\psi^r_k)
```

此外还有一个**能量高度项**。$\delta E_h \approx \delta h + (V^r/g)\,\delta V$ 是状态偏差的线性组合，于是

```math
Q_k = \text{blkdiag}\Big(Q^{(EN)}_k,\ \tfrac{1}{\sigma_h^2},\ \tfrac{1}{\sigma_V^2},\ \tfrac{1}{\sigma_\gamma^2},\ \tfrac{1}{\sigma_\psi^2}\Big) + \frac{e_k e_k^\top}{\sigma_E^2},\qquad e_k = (0,\ 0,\ 1,\ V^r_k/g,\ 0,\ 0)
```

终端步 $Q_K$ 再乘以一个系数（TAEM 为 3，A/L 为 2）。完整的代价是

```math
J = \sum_{k=0}^{K}\delta x_k^\top Q_k\,\delta x_k \;+\; \sum_{k=0}^{K-1}\delta u_k^\top R_u\,\delta u_k \;+\; \sum_{k=0}^{K-1}(\delta u_k - \delta u_{k-1})^\top R_\Delta\,(\delta u_k - \delta u_{k-1}),
\qquad \delta u_{-1} := u_\text{prev} - u^r_0
```

最后一项里的 $\delta u_{-1}$ 让第一步的变化量相对于**上一次实际下发的指令**计算，这样 MPC 每次更新时指令不会跳变。

| $\sigma$ | 侧偏 | 沿航迹 | 高度 | 能量 | $\gamma$ | $\psi$ | $V$ | $\alpha$ | $\phi$ | $\delta_{sb}$ | $\Delta\alpha$ | $\Delta\phi$ | $\Delta\delta_{sb}$ | 终端 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| TAEM | 250 m | 4000 m | 150 m | 250 m | 3° | 10° | 25 m/s | 4° | 35° | 40° | 1° | 6° | 6° | ×3 |
| A/L | 40 m | 6000 m | 25 m | 60 m | 1.5° | 4° | 8 m/s | 3° | 20° | 30° | 1° | 5° | 6° | ×2 |

可以看到，从 TAEM 到 A/L，侧偏容忍从 250 m 收紧到 40 m，沿航迹容忍反而放宽了。

### 4.6 约束与 QP

```math
\min_{\delta U}\ \tfrac12\,\delta U^\top H\,\delta U + g^\top\delta U \qquad \text{s.t.}\qquad l \le \begin{bmatrix} I \\ D\end{bmatrix}\delta U \le u
```

```math
H = \Gamma^\top\bar Q\,\Gamma + \bar R_u + D^\top\bar R_\Delta D, \qquad g = \Gamma^\top\bar Q\,(\Phi_x\delta x_0 + C) - D^\top\bar R_\Delta\,d_0
```

$D$ 是差分矩阵，第一行相对于 $u_\text{prev}$ 取差分，$d_0$ 把 $u_\text{prev}-u^r_0$ 带进代价。约束分两类：

- **幅值**：$\alpha \in [1^\circ, \alpha_\text{max}(M)]$，$|\phi| \le \phi_\text{lim}$，$0 \le \delta_{sb} \le 98.6^\circ$。
  $\phi_\text{lim}$ 随阶段变化：TAEM 为 60°（超声速 70°），A/L 在 300 m 以上为 30°，以下为 12°。
- **速率**：$|u_k - u_{k-1}| \le \dot u_\text{max}\Delta t$，约束的是**完整**控制量的增量，所以基准自身的增量 $\Delta u^r$ 也要算进边界。

如果基准本身就超出了限制（上下界交叉），就把边界放宽到包含它，保证 QP 可行。

**求解**：原生版用 OSQP。$H$ 的上三角稀疏结构固定不变，每次只更新数值（`update(Px=…)`）并热启动，一次约 10–13 ms。
浏览器版用 Clarabel 解同一个 QP。两者都失败时，退回到“无约束最小二乘解 + 截断到边界”，并记录状态 `fallback-lsq`。

### 4.7 执行

MPC 每 0.4 s 求解一次，制导以 10 Hz 运行。两次求解之间，从 MPC 的控制序列中按时间线性插值取指令。
这样指令是连续的，同时也利用了 MPC 预测的前几步，而不只是第一步。

---

## 第 5 课 进近与着陆：Orbiter 式剖面和拉平律

> 代码：`sim/gnc/approach.py`。依据 NASA JSC-23266《Approach, Landing and Rollout Flight Procedures Handbook》§4.3–4.7、§5.3.3、附录 B.3/B.4 [5]。

A/L 段的剖面是**预先计算的几何曲线**，不在线优化。原因是：这一段时间短（约 60 s），对精度要求极高，
而且真实轨道器的程序本身就是固定剖面。在线优化反而会带来不必要的不确定性。

### 5.1 垂直剖面：从 −20° 到接地

沿跑道中线（$x$ 为距入口的距离，入口前为负），重心离跑道的高度 $H(x)$ 由四段组成。
下表中的关键点是用 `vertical_profile(ApproachConfig())` 实际算出来的。

**① 外下滑道 OGS**：一条 −20° 的直线，瞄准入口前 1500 m：

```math
H_\text{OGS}(x) = (x_\text{aim,OGS} - x)\tan 20^\circ
```

**② 预拉平圆弧**：一段半径 $R = 8000$ m 的圆弧，与 OGS 相切，并与“IGS 上方 $d_1 = 17.5$ ft”这条线相切。
两条直线的交点为

```math
x_\text{int} = \frac{x_\text{aim,OGS}\,t_o - x_\text{aim,IGS}\,t_i - d_1}{t_o - t_i},\qquad t_o = \tan 20^\circ,\ t_i = \tan 1.5^\circ
```

切点到交点的距离为 $R\tan(\Delta\gamma/2)$，$\Delta\gamma = 18.5^\circ$。圆心在 OGS 切点沿法向 $R$ 处。
沿这段圆弧飞行需要的法向过载增量约为 $V^2/(gR) \approx 150^2/(9.81 \times 8000) \approx 0.29\,g$，和手册中的 0.33 g 一致。

**③ 指数过渡**：如果一直沿圆弧飞到与 IGS 相切，曲率会在切点突变。所以在圆弧上高出 IGS $d_2 = 35$ ft 的地方离开圆弧，
用指数曲线渐近地并入 IGS：

```math
H(x) = H_\text{IGS}(x) + d_2\,e^{-(x-x_e)/\lambda}, \qquad \lambda = \frac{d_2}{-H'_\text{arc}(x_e) - \tan 1.5^\circ}
```

$\lambda$ 的取值保证在 $x_e$ 处斜率连续。

**④ 内下滑道 IGS**：−1.5°，瞄准入口后 520 m。

**⑤ 最终拉平**：见 5.4。

| 关键点 | $x$（相对入口） | 重心高度 |
|---|---|---|
| ALI | −9742 m | 3000 m |
| 拉起开始（OGS 切点） | −2897 m | 508 m（≈1670 ft） |
| 离开圆弧，进入指数段 | −662 m | 41.6 m |
| 指数段时间常数 | $\lambda = 291$ m | |
| 进入最终拉平（基准） | −114 m | 18.2 m |
| 基准接地点 | +722 m | 4.4 m（接地姿态时重心高度） |

**为什么把 OGS 瞄准点从真实的 7500 ft 改成 1500 m？** 在本仿真中，如果用 2.3 km 的瞄准点，拉起圆弧会在入口前约 1.5 km 就结束，
之后飞机要贴着地面（60 m 以下）平飞一公里多才到跑道。把瞄准点拉近后，入口前 1.5 km 处主轮离地约 134 m（原来 58 m）。
代价是 IGS 上的稳定段比手册要求的 5 s 短。这类权衡在仿真工程中很常见：模型与真实飞机不完全一样，照搬真实参数未必得到真实的行为。

### 5.2 速度剖面

速度剖面沿着几何剖面积分得到（`_integrate`）：

- **OGS 段保持等效空速**。先由 EAS 剖面求出所需的 $dV/dt$，再反解需要的阻力：
  ```math
  D_\text{req} = -m\frac{dV}{dt} - mg\sin\gamma \quad\Rightarrow\quad C_{D,\text{req}} = \frac{D_\text{req}}{\bar qS} \quad\Rightarrow\quad \delta_{sb} = C_D^{-1}(C_{D,\text{req}};\ \alpha, M)
  ```
  其中迎角由升力平衡给出：$L = mg\cos\gamma + mV\dot\gamma$，$\dot\gamma$ 来自剖面曲率。
- **预拉平之后，减速板保持一个常值**。这个值用二分法求，目标是在接地时 TAS = 98 m/s（`_sb_preflare`）。
  如果 0° 到 98.6° 的范围内找不到解，就把 OGS 的 EAS 以 10 kt 为步长调整，范围为 280–320 KEAS，强逆风且能量不足时最高可到 340 KEAS（`_adapt_eas`）。
- **风切变项**：顺风通常越接近地面越弱。下降时飞机靠惯性保持地速，所以空速会**增加**：
  ```math
  \dot V = -\frac{D}{m} - g\sin\gamma - \frac{\partial w_\parallel}{\partial x}\,V_g\cos\gamma
  ```
- **侧风**：给出偏流角 $\psi_\text{air} = \psi_\text{rwy} - \arcsin(w_\perp/V)$。

另外，IGS 瞄准点和接地点会随顺逆风移动：逆风时每 m/s 后移 25 m，最多 400 m；顺风时每 m/s 前移 10 m，最多 250 m（`wind_aim`）。

### 5.3 进近段纵向律：按距离给高度

A/L 段的横向（倾侧角）和减速板仍由 MPC 给出，**纵向改由一个经典的着陆律控制**（`al_vertical_accel`）。
早期版本让 MPC 也负责纵向，但 MPC 的时间步长是 1 s，跟不住 8 km 半径的拉起圆弧，曾经冲到基准下方 5 m。

这个律的思路是：根据当前的跑道坐标 $x$ 查出“这个距离上应有的高度” $H(x)$，然后计算需要的垂直加速度：

```math
a_z = \underbrace{H''(x)\,v_x^2}_{\text{曲率前馈}} + k_h\,\underbrace{(H(x) - h)}_{e_h} + k_d\,\big(\underbrace{H'(x)\,v_x}_{\dot h_\text{ref}} - \dot h\big) + a_\text{int},
\qquad k_h = 0.2\ \text{s}^{-2},\ k_d = 0.8\ \text{s}^{-1}
```

- 第一项是关键。沿曲线以水平速度 $v_x$ 飞行时，垂直加速度必须等于 $H''v_x^2$，否则一定会偏离曲线。
- 积分项 $\dot a_\text{int} = k_i e_h$，$k_i = 0.03$，限幅 ±1 m/s²，并带抗饱和。它用来修正配平表与 6-DOF 之间的升力差。
- 结果限幅在 $[-0.6g,\ 1.2g]$。
- **下沉率保护**：下沉不能超过拉平表（延伸到 80 ft 以上）在当前高度允许的值，即 $a_z \ge k_d(\dot h_\text{min} - \dot h) - k\dot h$。
  如果飞机在短五边偏高，宁可着陆点靠后，也不能为了回到剖面而俯冲出拉平救不回来的下沉率。

**从加速度到迎角。** 在垂直方向上做力平衡，阻力用当前的升阻比表示：

```math
L\Big(\cos\gamma\cos\phi - \frac{\sin\gamma}{L/D}\Big) = m(g + a_z)
\quad\Rightarrow\quad
C_{L,\text{req}} = \frac{L}{\bar q S\cdot G(h)}, \qquad \alpha = C_L^{-1}(C_{L,\text{req}};\ M, \delta_{sb}, \text{gear})
```

注意 $\gamma < 0$ 时，$-\sin\gamma/(L/D) > 0$，也就是下降时阻力有一个向上的分量。
$G(h)$ 是**地面效应**修正：

```math
G(h) = 1 + 0.131\,e^{-h_\text{cg}/11\,\text{m}}
```

配平表是自由流数据，而 6-DOF 气动里有地面效应。不配平时，接地高度上的升力约增加 27%（手册说 L/D 约增加 25%）；
但地面效应同时带来低头力矩，需要升降副翼上偏来配平，最后几秒体襟翼又来不及动，所以配平后实际只剩约 9%。
上面的拟合式就是这个配平后的净增量。

**从 MPC 切换到着陆律**：在 ALI 处用 3 s 线性混合，避免指令跳变。

### 5.4 最终拉平：下沉率–高度表

真实轨道器的最终拉平跟踪一张**下沉率–高度表**（JSC-23266 §5.3.3）：52 ft 时 13 fps，2 ft 时 3 fps，中间线性，2 ft 以下保持 3 fps。写成公式：

```math
\dot h_\text{ref}(h) = -k\,(h + h_\text{off}), \qquad k = \frac{(13-3)\,\text{fps}}{(52-2)\,\text{ft}} = 0.2\ \text{s}^{-1},\quad h_\text{off} = 13\ \text{ft} = 3.96\ \text{m}
```

如果飞机完美跟踪这张表，高度满足一阶线性常微分方程，解是指数曲线：

```math
h(t) + h_\text{off} = (h_0 + h_\text{off})\,e^{-kt}
```

时间常数为 $1/k = 5$ s，对应地面距离 $L_f = v_\text{ff}/k \approx 112/0.2 = 560$ m。

> **请注意**：这条曲线的渐近线是 $h = -h_\text{off} < 0$，而不是 $h = 0$。
> 也就是说，参考下沉率**永远不会为零**，飞机会一直下降到跑道上，不会平飘，也不会反弹。
> 这正是手册要求的“no float, no balloon”。拉平律的设计意图就体现在 $h_\text{off}$ 这一个常数里。

**进入高度**：在表上的下沉率等于当前下沉率的高度进入拉平，并限制在 30–80 ft 之间：

```math
h_\text{FF} = \text{clip}\Big(\frac{-\dot h}{k} - h_\text{off},\ 30\,\text{ft},\ 80\,\text{ft}\Big)
```

下沉越快，进入越早。

**拉平律**（`flare_alpha`）：

```math
a_z = \text{clip}\Big(\underbrace{-k\,\dot h}_{\text{前馈：}\ddot h_\text{ref}} + k_d\big(\dot h_\text{ref}(h) - \dot h\big),\ -0.15g,\ 0.6g\Big)
```

前馈项是参考下沉率对时间的导数：$\ddot h_\text{ref} = -k\dot h$。低头方向的权限很小（−0.15 g），原因是轨道器的俯仰响应是**非最小相位**的：
升降副翼上偏抬头时，首先会损失一部分升力。手册要求这一阶段“操纵量尽量小”。

**防擦尾**：俯仰姿态限制为 $\theta_\text{max} = 9.7^\circ + 5.3^\circ\cdot\text{clip}((h-10)/50,\,0,\,1)$，换算成迎角上限 $\alpha_\text{max} = \theta_\text{max} - \gamma$。

**飘过接地点**：如果飞过了基准接地点还没接地（速度过高、顺风或风切变造成的上飘），就加大目标下沉率（按飘过的距离线性增加，飘过 300 m 时加满 1 m/s），
并逐步打开减速板（飘过 250 m 时开满），防止冲出跑道。

**横向**：拉平阶段用一个简单的中线保持律，坡度限制在 ±5°（$y$ 为侧偏，单位 m；$\phi$ 单位 rad）：

```math
\phi = \text{clip}\big(-0.012\,y - 0.08\,\dot y,\ \pm 5^\circ\big)
```

### 5.5 接地与滑跑

- **主轮接地**：立即放减速伞（40 ft 锥形带条伞：引导伞 → 收口 → 全开），地速低于约 60 kt 时抛伞。
- **降前轮**：飞控切换到主轮着地模式，以 −1.5°/s 的俯仰角速率放下前轮，计入起落架力矩。
- **滑跑**：前轮转向 $\delta = \text{clip}(-0.08\,y - 0.8\,\psi_\text{err},\ \pm8^\circ)$（$\psi_\text{err}$ 和 $\delta$ 以度计）；地速 85 m/s 以下刹车 55%，15 m/s 以下刹车 100%（带防滑）；
  空速 80 m/s 以下减速板全开。升降副翼下偏卸载，防止减速伞的抬头力矩把前轮抬起来。

### 5.6 与真实轨道器的对比

| 阶段 | 真实轨道器（JSC-23266） | 本仿真 |
|---|---|---|
| 外下滑道 OGS | −20°（重载 −18°），瞄准入口前 7500 ft，300 KEAS，减速板保持空速 | −20°，入口前 1500 m，290 KEAS（随风 280–340） |
| 预拉平 | 2000 ft 开始；约 0.33 g 的等过载圆弧，在 1700 ft 与 OGS 相切 | 8 km 半径（≈0.29 g），约 1670 ft 切入 |
| 过渡到 IGS | 圆弧上高出 IGS 35 ft 处开始指数衰减 | 相同 |
| 内下滑道 IGS | −1.5°，瞄准入口后 1000 ft，至少稳定 5 s | −1.5°，瞄准入口后 520 m，稳定段较短 |
| 起落架 | 300 ± 100 ft 放下 | 90 m（≈300 ft） |
| 最终拉平 | 下沉率–高度表，30–80 ft 进入 | 同一张表 |
| 接地 | 入口后 2500 ft，195–205 KEAS，下沉率 2–3 fps | 见“测试与结果” |

---

## 第 6 课 当规划失败时：降级模式

> 代码：`sim/gnc/guidance_scpmpc.py`

在真实系统中，优化器失败不是“异常”，而是需要事先设计好应对方式的工况。本项目的做法分两层：

**第一层：保留旧计划。** 重规划失败时，如果正在飞的计划是好的，并且仍在被跟踪（误差 < 4 km），就继续飞它，5 s 后再试，最多连续 3 次。

**第二层：降级模式。** 没有可用计划时，根据规划器判断的能量方向选择模式，并且每 5 s 冷启动重规划一次，直到有可用计划：

| 模式 | 能量 | 做法 |
|---|---|---|
| **GEOREF** | 正常 | MPC 跟踪所选路线的几何基准，即第 2 课的初值轨迹 |
| **MAXRANGE** | 不足或转弯受限 | 以最佳升阻比飞向还够得着的最远进近点（WP2、ALI 或延长中线，计入对准转弯的余量），截获中线后沿中线飞；飞过入口时仍高或转离时，重新选点 |
| **HOLD** | 过剩 | 绕 HAC 中心盘旋（约 35° 坡度，半径 $R = \max(V^2/(g\tan35^\circ),\ 4\,\text{km})$），减速板 80° |

MAXRANGE 和 HOLD 不经过 SCP，所以也没有 SCP 的地形约束。它们各自带一个**地形前视**：
按平均滑翔斜率预测前方 8–60 s 的高度，只考虑比机场高 100 m 以上的地形；发生冲突时，选最近的安全航向，并收起减速板以延长滑翔。
低空时还会限制坡度。如果在场外 50 m 以下仍未到达跑道，则改平并拉起，以减小冲击。

故障注入测试：`python tools/gnc_selftest.py --fault-scp 1500`，全程让 SCP 失败，只靠降级模式和 A/L 基准飞回，10 次中成功 9 次。

---

## 第 7 课 内环：电传飞控

> 代码：`sim/fcs.py`（50 Hz）

制导只输出 $(\alpha, \phi, \delta_{sb})$。把它们变成舵面偏角，是飞控的工作。这里采用**时间尺度分离的非线性动态逆**，分三层：

**(1) 姿态外环 → 体轴角速率指令。** 迎角先经过一个参考模型（一阶，时间常数 0.7 s，速率限制 4°/s），再加 PI 修正，得到 $\dot\alpha$；
航迹滚转角 $\mu$ 用 P 控制得到 $\dot\mu$；侧滑角用 PI 控制。然后用**精确的航迹运动学**换算成稳定轴角速率，其中包括升力、侧力、重力前馈和转弯协调：

```math
\begin{aligned}
p_s &= \cos\beta\Big[\dot\mu - \frac{L}{mV}(\tan\gamma\sin\mu + \tan\beta) + \frac{g}{V}\cos\gamma\cos\mu\tan\beta\Big] \\
r_s &= \frac{Y + mg\cos\gamma\sin\mu}{mV} - \dot\beta \\
q &= \dot\alpha + \tan\beta\,p_s + \frac{L - mg\cos\gamma\cos\mu}{mV\cos\beta}
\end{aligned}
\qquad
\begin{aligned}
p &= p_s\cos\alpha - r_s\sin\alpha \\
r &= p_s\sin\alpha + r_s\cos\alpha
\end{aligned}
```

注意最后一步：滚转是**绕速度矢量**进行的。迎角为 15° 时，绕机体 x 轴滚转会把迎角转成侧滑，所以必须换算到稳定轴。

**(2) 角速率内环 → 所需力矩。** 期望角加速度 $\dot\omega_d = K_\omega(\omega_\text{cmd} - \omega)$，$K_\omega = (2.6, 2.4, 2.2)$，并限制在可用权限的 40% 以内。所需力矩为

```math
M_\text{req} = J\dot\omega_d + \omega \times J\omega - M_\text{gear}, \qquad \Delta M = M_\text{req} - M_0(u_\text{act})
```

$J$ 是完整的惯量张量，包括 $I_{xz}$。$M_0$ 是在当前舵面位置上计算的气动力矩。

**(3) 控制分配。** 控制效能矩阵 $B = \partial M/\partial u$（3×5：4 片升降副翼 + 方向舵）**在线**获得：
每个飞控周期用完整的分片气动模型做 6 次批量求值，按有限差分计算。然后求加权、正则化的伪逆，并向偏好分布（同侧内外升降副翼相等，方向舵居中）靠拢：

```math
\Delta u = d + W B^\top\big(B W B^\top + \lambda I\big)^{-1}(\Delta M - B d), \qquad d = u_\text{pref} - u_\text{act}
```

计算前按 $\bar q S\cdot(b, \bar c, b)$ 对力矩做归一化。饱和的作动器从求解中移除，剩余的需求重新分配，最多迭代 3 次。
**俯仰优先**：如果滚转和偏航的需求会导致饱和，就按 1 → 0.6 → 0.3 → 0 的比例缩小这两个轴的需求，保证俯仰力矩不受影响。
角速率限制也随**剩余可达力矩**自适应：沿伪逆方向，按剩余行程估算各轴、各方向的最大角加速度，从而得到速率上限。

**(4) 其他。** 体襟翼自动配平：缓慢地把对称升降副翼卸载到目标值（巡航 −1°，着陆构型 +3°）。
作动器模型为一阶滞后 + 速率限制 + 位置限制：升降副翼 −33°/+18°、20°/s；方向舵 ±27.1°；减速板 0–98.6°、6.1°/s。
迎角包线保护：跨声速和低超声速时，为副翼和方向舵保留余量。

---

## 课后思考题

1. 在 3.4 中，如果去掉虚拟控制 $\nu$，从一个能量明显过剩的出生点做冷启动，你认为第一轮子问题会出现什么情况？试着修改 `w_nu` 做实验。
2. 3.6 的日志中，$t_f$ 随迭代单调增加。如果把 $w_{\Delta u}$ 调小 10 倍，$t_f$ 会怎样变化？为什么？
3. MPC 的能量项 $e_k = (0,0,1,V^r/g,0,0)$ 是 $E_h$ 的一阶近似。在什么情况下这个近似会明显失准？
4. 4.2 说按时间跟踪会让滑翔机“追时间表”。请构造一个持续逆风的场景，比较空间重索引与时间索引两种做法的能量损失。
5. 5.4 中，如果把 $h_\text{off}$ 改成 0，拉平曲线会变成什么样？接地时间和接地点会怎样变化？
6. 为什么 A/L 的纵向用经典着陆律而不用 MPC？试把 MPC 的步长改成 0.25 s、时域保持 28 s（`Mpc(model, K=112, dt=0.25)`），观察对拉起圆弧的跟踪效果和 QP 的求解时间。
7. 第 2 课的能量估计用了 $\sqrt{\cos\phi}$ 而不是 $\cos\phi$。写一个小脚本，在一组出生状态上分别用两种写法调用 `select_plan`，再用 SCP 是否到达 ALI 作为“真值”，比较两者的可达性误判率。

---

# 仿真的逼真程度

> 这一部分说明各个子系统“做到了什么程度”，以及哪些地方做了简化。

## 飞行动力学（`sim/dynamics.py`、`sim/simulator.py`）

- **6-DOF 刚体**：四元数姿态，完整惯量张量（$I_{xx} = 1.30\times10^6$，$I_{yy} = 9.38\times10^6$，$I_{zz} = 9.65\times10^6$，$I_{xz} = 2.25\times10^5$ kg·m²），质量 99.8 t。
- **积分**：RK4，物理步长 0.01 s（100 Hz）；靠近地面或轮子着地时，细分为 4 个子步（400 Hz），用于计算起落架的刚性接触。
- **时序**：飞控 50 Hz，制导 10 Hz，MPC 2.5 Hz，作动器在每个物理子步更新。
- **地球模型**：平面地球。地球曲率只在画面中体现。重力随高度按平方反比变化。

## 空气动力学（`sim/aero.py`）

不是查一张整机气动表，而是**部件叠加**：每一步都根据每个单元的**当地来流**计算力，来流包括机体相对气流、$\omega\times r$ 和阵风。

- **升力面条带**：左右机翼各 32 条（含翼套和机身穿越段）、体襟翼、垂尾。
  - 简单后掠分解：只用垂直于条带展向的速度分量计算力；
  - 亚声速时取 3/4 弦长处的当地来流，超声速时过渡到 1/2 弦长；力作用在当地气动中心，从 0.25c 移到 0.5c；
  - 升力线斜率：亚声速用 DATCOM，超声速用三角翼线性理论（区分亚声速和超声速前缘，含椭圆积分），跨声速平滑过渡；
  - 法向力/轴向力写法在任意迎角下都有效：位势项加部分前缘吸力，Polhamus 涡升力，涡破裂和失速衰减，接近 90° 时变为平板特性。
- **舵面**：4 片升降副翼、体襟翼、方向舵。亚声速用薄翼理论的襟翼效率 $\tau(c_f)$ 和 $c_m$ 增量，超声速用 Ackeret 理论的舵面载荷；
  大偏角时效率下降，并计算偏转阻力和铰链力矩。方向舵/减速板做成两块向外张开的分裂式面板。
- **型阻**：湍流摩擦（当地雷诺数，计入压缩性和防热瓦粗糙度）乘以形状因子，加上跨声速阻力发散和超声速波阻。
- **机身 + OMS 吊舱**：28 段细长体，Munk 附加质量力（垂直和侧向分开计算），Allen–Perkins 粘性横流，前体波阻，随 Mach 变化的底部阻力。
- **地面效应**：按每个条带的离地高度计算，包括 Wieselsberger 诱导阻力减小、升力线斜率增加和气动中心后移。
- **其他**：大迎角和高 Mach 时垂尾的侧洗与机身遮蔽；起落架阻力随放下程度变化；减速伞。
- `evaluate` 支持一次传入 $K$ 组舵面偏角，飞控用它在一次调用里完成有限差分。

**标定结果**（重心 $x=0$，起落架收起，减速板关闭）：

| Mach | $C_{L\alpha}$ /° | 静稳定裕度 | $(L/D)_\text{max}$ | $C_{m\delta_e}$ /° |
|---|---|---|---|---|
| 0.3 | 0.050 | +0.8 % | 4.87 | −0.0055 |
| 0.6 | 0.052 | +1.6 % | 4.98 | −0.0058 |
| 1.5 | 0.038 | +12.8 % | 2.26 | −0.0031 |
| 2.5 | 0.025 | ≈+1 % | 2.21 | −0.0015 |

增量：减速板全开 $\Delta C_D \approx +0.079$，起落架放下 $\Delta C_D \approx +0.017$。完整表格用 `python tools/aero_report.py` 打印。

**简化**：没有气动弹性，没有非定常气动（除附加质量项外），不考虑高超声速段（出生点 Mach ≤ 2.6）。系数是按公开资料“调到像 Orbiter”，不是风洞数据。

## 起落架（`sim/gear.py`）

每个轮子都是沿机体 z 轴的油气减震支柱：渐进弹簧，非对称阻尼（压缩和回弹不同），带触底。轮胎法向力、滚动阻力、带简单防滑的刹车
（摩擦力限制在峰值）、tanh 饱和的侧偏力、摩擦圆、前轮转向。尾部、机腹、翼尖、喷管是结构接触点，用于检测擦尾和坠毁。
静态载荷分配：前轮约 20%，两个主轮各约 40%。按当前几何，擦尾角约 10.9°（真实 Orbiter 约 15°）。

## 大气与风（`sim/atmosphere.py`、`sim/wind.py`）

- US1976 标准大气。
- 平均风：10 m 参考风速，500 m 以下 1/7 次幂律。
- 湍流：MIL-F-8785C 的 Dryden 形式，低空（< 1000 ft）和中高空（≥ 2000 ft）两套参数，中间线性混合，在机体轴生成。
- 自动天气：风速、风向和湍流按 Ornstein–Uhlenbeck 过程缓慢漂移。前端根据云场算出飞机所在位置的风暴强度并发给后端：
  积雨云内强湍流和上升气流，云下雨幡中下沉气流和阵风锋。垂直气流在 150 m 以下逐渐减弱。

## 航电与座舱显示（`web/js/ui/meds.js`、`hud.js`、`web/js/world/cockpit.js`）

显示器仿照轨道器 MEDS 玻璃座舱（2000–2011 年的 OV-103/104/105）。目标是**符号、颜色、刷新节奏和交互方式**都尽量接近真实，而不只是看起来像。

- **MDU 硬件**：喷漆面板，带 Dzus 紧固件的边框，6 个边缘键，拨动开关（ADI ATTITUDE INRTL/LVLH/REF、ERROR、RATE、地图方向、电源），
  亮度和地图量程旋钮（5–160 NM 六挡），带按键音；开机先跑自检（背光、LCD 初始化、BITE）。窗口可以拖动、缩放、折叠，位置保存在 localStorage。
- **边缘键菜单**：MAIN MENU → FLT INST / SUBSYS STATUS / DPS / MEDS MAINT，每台 MDU 都能切换到任意页面。
- **A/E PFD**：ADI 八球（逐像素着色的球体，带速率刻度，品红色误差针来自制导指令）；α、M/VR（低于 Mach 0.9 改为 KEAS）、H（5000 ft 以下改用雷达高度）、Ḣ 四条带式表；
  Nz 加速度表；HSI；下滑道指示；TAEM/A-L 模式通告。
- **HSIT**（SPEC 50 HORIZ SIT 与 MEDS HSI 的符号）：
  - 20/40/60 s 等坡度预测圈，按 FSSR 的转弯率 $g\tan\phi/V_g$ 计算；
  - 品红色 HAC 圆，带转弯方向箭头；跑道按比例绘制；SCP 规划显示为品红色虚线；HAC 切换后旧 HAC 保留 60 s；
  - H/C/R 方位指针，顶部标尺和右侧标尺随阶段切换（到 HAC 的时间、HAC 径向误差、跑道侧偏、高度误差）；
  - 读数：GS、TAS、KEAS、M、风、顶风/侧风分量、经 HAC 到接地点的距离、ΔAZ/HTA/ΔY、TTG、ΔE（能量走廊 +8000/−4000 ft）、减速板实际值和指令值；DPS 状态字符 ↑ ↓ M；
  - 颜色按 NASA 座舱升级颜色表和 FAA AC 25-11B；信息行按优先级排序，新信息先反色闪烁 5 s。
- **显示处理器的时序模型**：模拟 1980 年代 GPC 的节奏。
  - 每 480 ms 采样一次导航状态，160 ms 后才上屏，所以画面一格一格地跳，并且有延迟；
  - 慢变量（目标、方位、风、能量、所有数字读数）按 960 ms 周期计算；
  - 改变地图量程后，地图区先空白，等下一个数据周期和坐标变换完成；
  - 3 s 没有数据时，整屏画大 X，并显示 POLL FAIL。
- **其他页面**：SPI 舵面位置指示；VERT SIT、HORIZ SIT、GNC SUMM 三个 DPS 风格的 51×26 字符页面，1 Hz 刷新，底行显示故障信息。
- **字体**：所有页面的文字都不用系统字体，而是逐字矢量描画的笔画字体（8×12 网格，圆头笔划），只有大写字母，与符号发生器用同一支“笔”画出。
- **告警**：主告警灯只在真正的故障时闪烁。
  - 状态类（能量不足、降级模式）：条件消失后自动熄灭；
  - 锁存类（坠毁、擦尾、重着陆、冲出跑道）：保持到人工确认。
  - C&W 告警矩阵 40 盏灯中只有 GPC 一盏会亮，其余子系统不模拟故障。
- **HUD**：按 SCOM 进近着陆格式绘制，只显示在组合镜范围内，并且是准直的。内容包括速度矢量、制导菱形、KEAS 和高度带、俯仰刻度、跑道轮廓、拉平三角、Nz 和减速板指令。
  带式表用临界阻尼低通平滑，数字窗按 5 Hz 采样保持。

**简化**：只模拟 GNC 相关的显示内容。其他子系统（APU、液压、电源等）的页面和开关是静态的。DPS 页面的内容是按真实格式排版的仿真数据，不是真实的飞行软件。

## 3D 座舱（`web/js/world/cockpit.js`）

- **布局**：完整封闭的前飞行甲板。视点在指令长眼位：距 MDU 面板约 30 in，面板后倾 15°。
  有六块前窗 W1–W6 和两块顶窗，遮光罩，F2–F8、C2/C3、L1/L2、R1/R2、O1–O9 各面板，九台 MDU 显示实时画面，还有座椅、驾驶杆、脚蹬。
- **渲染方式**：座舱在以机体为原点的独立场景中渲染，避免 float32 精度不足导致舱内阴影闪烁。
  阳光只从舷窗射入；天空光按形状因子和体素遮挡烘焙。
- **舷窗雨水**：逐滴做物理模拟。
  - 雨滴谱为 Marshall–Palmer 分布，撞击时按 $K = We^{1/2}Re^{1/4}$ 判断是否溅射；
  - 水滴受重力、表面气流拖曳和接触线钉扎力作用；
  - 渲染时视线按斯涅尔定律和菲涅尔公式穿过玻璃 → 水 → 空气。
- **头部运动**：用眼位比力（质心过载加上 $\dot\omega\times r + \omega\times(\omega\times r)$）驱动三个轴的阻尼弹簧，另外叠加抖振、湍流和跑道颠簸。

## 渲染

- **大气**：Hillaire 2020 方案的四张 LUT，逐像素计算空气透视。
- **体积云**：Perlin–Worley 噪声，Beer–Powder 散射和多次散射近似，时域重投影。
- **地形**：14 级几何裁剪图，CDLOD 形变消除接缝。
- **机场**：FAA 标线，进近灯，PAPI（包括 20° 外下滑道 PAPI）。
- **雷暴**：雨幡、闪电（含回击）、按声速延迟的雷声、地面积水的屏幕空间反射。
- **机体**：材质为程序化的防热瓦，操纵面绕真实铰链轴转动。

---

# 测试与结果

```bash
python tools/sim_selftest.py                  # 6-DOF / 飞控自检
python tools/gnc_selftest.py                  # GNC 回归：多进程并行，同步规划器
python tools/gnc_selftest.py --wind --turb    # 有风和湍流
python tools/gnc_selftest.py --fault-scp 90   # 前 90 s 所有规划都失败，检验降级模式
```

**成功标准**：在跑道入口之后接地，下沉率 < 3 m/s，偏离中线 < 20 m，并在跑道上停住。

最近一次全面测试结果如下。注意：这组测试是在 OGS 瞄准点为 2.3 km 时测的，改到 1.5 km 后还没有完整重测。

| 工况 | 成功率 | 接地下沉率（均值 / 最大） | 接地点（入口后） | 偏离中线 |
|---|---|---|---|---|
| 无风 ×40（RWY33 19 次，RWY15 21 次，含 Mach 2.4 / 24 km 出生） | 40/40 | 0.85 / 0.90 m/s，207 KGS | 575–581 m | 0.0 m |
| 10 m/s 风 + 中度湍流 ×12 | 12/12 | 1.26 / 1.88 m/s，182–223 KGS | 177–946 m | ≤ 4 m |
| 近场高能量出生（8/15/22 km × 15/20 km 高 × 3 个方位） | 18/18 | | | |
| 高速背向、直进过高等极端出生点 | 8/8 | | | |
| `--fault-scp 90` | 12/12 | | | |
| `--fault-scp 1500`（全程没有 SCP） | 9/10 | | | |

- **SCP**：平均 3.2–3.6 次迭代，约 180 ms（冷启动约 0.5 s），收敛率 98–100%。
- **MPC**：每次约 13 ms。
- **飞行包线**：全程最大动压 16.8 kPa，过载 0.4–2.1 g。

**典型着陆时间线**（无风，种子 1000）：

| 相对接地时间 | 事件 |
|---|---|
| −32 s | 约 1500 ft 开始拉起，下沉率 53 m/s |
| −16 s | 约 150 ft 进入指数段 |
| −11 s | 在 IGS 上稳定，下沉率约 3 m/s |
| −6 s | 39 ft、225 KEAS 进入最终拉平 |
| 入口 | 主轮高 31 ft |
| 0 | 入口后 575 m 接地，下沉率 0.91 m/s（3.0 fps），202 KGS |

拉平阶段下沉率按 3.0 → 2.5 → 1.8 → 1.2 → 1.0 m/s 持续减小，**全程没有改平，也没有反弹**。

**WebSocket 消息**：`state.gnc`（阶段、能量误差、剩余航程、SCP/MPC 状态、基准指令）；`plan`（SCP 节点、HAC、ALI、进近折线）；`mpc`（预测轨迹）。
想自己画图分析的同学可以直接订阅这些消息。

---

# 代码结构

```
server.py              HTTP 静态服务 + WebSocket；实时仿真循环（时间倍率 0.25–8×）
sim/                   飞行仿真（Python，numpy 向量化）
  dynamics.py          6-DOF 刚体（四元数、完整惯量、RK4）
  aero.py              部件叠加气动（条带、舵面、细长体机身、地面效应、起落架、减速伞）
  fcs.py               电传飞控：α/β/μ 外环 + 非线性动态逆 + 在线控制效能 + 俯仰优先分配
  gear.py              起落架、轮胎、刹车防滑、前轮转向、结构接触
  trim.py              配平求解与配平表（α × Mach × 减速板 × 起落架）
  atmosphere.py wind.py  US1976；风廓线 + Dryden 湍流
  guidance.py          制导接口（注册表）+ 占位制导 hold_glide
  gnc/                 ★ 讲义的主角
    model.py           第 1 课：3-DOF 规划模型、配平表插值、流映射与雅可比
    geometry.py        第 2 课：跑道/HAC、Dubins、能量预算、方案选择、初值
    scp.py             第 3 课：自由终端时间 PTR 序列凸优化
    planner.py         第 3 课：规划前端（热启动、恢复、重选跑道、进程/Worker）
    mpc.py             第 4 课：LTV-MPC（凝聚 QP，OSQP / Clarabel）
    reference.py       第 4 课：稠密基准轨迹与空间投影
    approach.py        第 5 课：A/L 剖面、速度积分、着陆律、拉平律
    guidance_scpmpc.py 第 5–6 课：阶段状态机、降级模式
  spawn.py terrain.py  随机出生点、地形高度查询
  browser.py           浏览器版入口（Pyodide Worker）
worldgen/              程序化世界生成：地形侵蚀、海岸、河湖、地表覆盖、天气图
shared/                Python 与 JS 共用：orbiter_geometry.json（机体几何）、world.json
web/                   three.js r170 前端（已随仓库附带）
  js/render/           大气、体积云、地形、后处理
  js/world/            航天飞机模型、机场、座舱、天气、雨水
  js/ui/               相机、HUD、MEDS 显示器、头部运动
  js/sim/              浏览器版：Pyodide 启动、仿真 Worker、规划 Worker
tools/                 自检、GNC 回归测试、气动报告、浏览器版构建与发布
deploy/                nginx 配置示例
```

---

# 已知限制

- 本机上带 GNC 的仿真约为 2.5–3 倍实时，`hold_glide` 约 4 倍。
- `--spawn any` 的出生点有相当一部分能量不够飞回机场：GNC 会告警（LOW ENERGY），以 MAXRANGE 模式尽量接近机场，但多半会在场外坠地。
- Mach 2.4 以上、离场较远又背向机场的出生点仍可能失败。这个速度下升力只够约 1 g，转弯半径达几十公里，超声速倾侧速率只有 2–2.5°/s。
- 无头测试使用同步规划器，求解期间仿真时间不推进。实时服务中求解是异步的，冷启动的几秒延迟在测试中没有覆盖。
- 规划模型用配平后的 3-DOF 气动表，不含地面效应；A/L 段的俯仰由着陆律控制，不经过 MPC。
- 动力学按平面地球计算，不考虑高超声速段。

---

# 参考文献

1. D. Malyuta, T. P. Reynolds, M. Szmuk, T. Lew, R. Bonalli, M. Pavone, B. Açıkmeşe. *Convex Optimization for Trajectory Generation: A Tutorial on Generating Dynamically Feasible Trajectories Reliably and Efficiently.* IEEE Control Systems Magazine, 42(5), 2022. ——SCP / PTR 的最佳入门材料。
2. M. Szmuk, B. Açıkmeşe. *Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time.* AIAA GNC Conference, 2018. ——时间伸缩与自由终端时间。
3. J. B. Rawlings, D. Q. Mayne, M. M. Diehl. *Model Predictive Control: Theory, Computation, and Design*, 2nd ed. Nob Hill, 2017.
4. B. Stellato, G. Banjac, P. Goulart, A. Bemporad, S. Boyd. *OSQP: An Operator Splitting Solver for Quadratic Programs.* Mathematical Programming Computation, 2020.
5. NASA JSC-23266. *Space Shuttle Approach, Landing, and Rollout Flight Procedures Handbook.*
6. T. E. Moore. *Space Shuttle Entry Terminal Area Energy Management.* NASA TM-104744, 1991.
7. S. Diamond, S. Boyd. *CVXPY: A Python-Embedded Modeling Language for Convex Optimization.* JMLR, 17(83), 2016.
8. P. J. Goulart, Y. Chen. *Clarabel: An Interior-Point Solver for Conic Programs with Quadratic Objectives.* arXiv, 2024.
9. S. Sieberling, Q. P. Chu, J. A. Mulder. *Robust Flight Control Using Incremental Nonlinear Dynamic Inversion and Angular Acceleration Prediction.* Journal of Guidance, Control, and Dynamics, 2010.
10. MIL-F-8785C. *Flying Qualities of Piloted Airplanes*, 1980. ——Dryden 湍流模型。
11. S. Hillaire. *A Scalable and Production Ready Sky and Atmosphere Rendering Technique.* EGSR, 2020.

---

# 许可证

本项目以 [Apache License 2.0](LICENSE) 发布，见 [`LICENSE`](LICENSE) 和 [`NOTICE`](NOTICE)。

第三方组件：

- `web/vendor/three/`：[three.js](https://threejs.org) r170，MIT 许可，原许可证见 `web/vendor/three/LICENSE`。
- 运行时依赖（不随仓库分发，通过 pip 或 CDN 获取）：NumPy、SciPy（BSD）；cvxpy、Clarabel、OSQP（Apache-2.0）；ECOS（GPL-3.0，仅作 SCP 的备用求解器，可不安装）；websockets（BSD）；Pyodide（MPL-2.0）。
  浏览器版构建（`tools/build_web.py`）会把 Pyodide 及其 wheel 复制到 `dist/` 中自托管，发布时请一并遵守它们的许可证。

欢迎提 issue 和 PR。如果这份讲义帮你理解了 SCP 或 MPC，也欢迎告诉我们哪里还可以讲得更清楚。
