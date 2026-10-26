# Delta 机械臂运动学建模（不含硬件约束）

> **范围**：本文只处理几何与运动学——正解、纯几何可达范围、逆解与可达性判定。
> **不考虑**关节限位、球铰锥角、杆件干涉、奇异回避等工程约束；这些属于后置裁剪，见 §9。
>
> **参数**：$R = 139\,\mathrm{mm}$，$r = 39\,\mathrm{mm}$，$L = 140\,\mathrm{mm}$，$l = 200\,\mathrm{mm}$。

---

## 1. 结构约定与符号

### 1.1 符号表

| 符号 | 含义 |
|---|---|
| $R$ | 静平台半径（固定铰点 $\mathbf{C}_i$ 所在圆） |
| $r$ | 动平台半径（动铰点 $\mathbf{A}_i$ 所在圆） |
| $L$ | 主动臂长度，$\mathbf{C}_i \to \mathbf{B}_i$ |
| $l$ | 从动臂长度，$\mathbf{B}_i \to \mathbf{A}_i$ |
| $\theta_i$ | 第 $i$ 条腿主动臂转角（从水平径向量起，向上为正） |
| $\varphi_i$ | 第 $i$ 条腿方位角，$\varphi_1=0^\circ,\ \varphi_2=120^\circ,\ \varphi_3=240^\circ$ |
| $\mathbf{u}_i$ | 径向单位向量 |
| $\mathbf{t}_i$ | 切向单位向量 |
| $\hat{\mathbf{z}}$ | 竖直单位向量 |
| $\mathbf{P}$ | 动平台中心位置 |

三条腿的方位角、径向与切向单位向量：

$$
\varphi_i = \frac{2\pi\,(i-1)}{3},\qquad
\mathbf{u}_i = \begin{bmatrix}\cos\varphi_i \\ \sin\varphi_i \\ 0\end{bmatrix},\qquad
\mathbf{t}_i = \begin{bmatrix}-\sin\varphi_i \\ \cos\varphi_i \\ 0\end{bmatrix},\qquad
\hat{\mathbf{z}} = \begin{bmatrix}0\\0\\1\end{bmatrix}
$$

### 1.2 坐标系

基座坐标系 $O\text{-}XYZ$：原点在静平台几何中心，$Z$ 轴竖直向上，静平台上表面为 $z=0$ 平面。静平台铰点：

$$
\mathbf{C}_i = R\,\mathbf{u}_i = \begin{bmatrix} R\cos\varphi_i \\ R\sin\varphi_i \\ 0\end{bmatrix}
$$

### 1.3 关键前提：动平台姿态被锁死

平行四边形从动臂（每腿两根平行杆、两端等距叉口）保证动平台叉口方向始终与主动臂叉口方向一致。主动臂只绕切向轴转动，其叉口方向在整个运动过程中不变，三条腿合起来把动平台的三个转动自由度全部锁死。

因此动平台只有三个平移自由度，动铰点相对中心的偏移是常量：

$$
\mathbf{A}_i = \mathbf{P} + r\,\mathbf{u}_i
\tag{1.1}
$$

> 该式是后续全部推导的基础。若采用单杆 + 球铰（无平行四边形），此式不成立，必须引入动平台姿态角 $\alpha$ 重新推导。

### 1.4 局部坐标：把三维问题降到二维

对第 $i$ 条腿建立「径向 / 切向 / 竖直」局部坐标：

$$
a = \mathbf{P}\cdot\mathbf{u}_i,\qquad
b = \mathbf{P}\cdot\mathbf{t}_i,\qquad
z = P_z
\tag{1.2}
$$

在该局部系下，静平台铰点、肘点、动平台铰点分别为

$$
\mathbf{C}_i \to \begin{bmatrix} R \\ 0 \\ 0\end{bmatrix},\qquad
\mathbf{B}_i \to \begin{bmatrix} R + L\cos\theta_i \\ 0 \\ L\sin\theta_i\end{bmatrix},\qquad
\mathbf{A}_i \to \begin{bmatrix} a + r \\ b \\ z\end{bmatrix}
\tag{1.3}
$$

---

## 2. 正运动学：$\theta_1,\theta_2,\theta_3 \;\to\; \mathbf{P}$

### 2.1 由关节角求肘点

主动臂只能在其腿的竖直径向平面内摆动，转角 $\theta_i$ 从水平径向量起、向上为正：

$$
\mathbf{B}_i = \mathbf{C}_i + L\left(\cos\theta_i\,\mathbf{u}_i + \sin\theta_i\,\hat{\mathbf{z}}\right)
= \left(R + L\cos\theta_i\right)\mathbf{u}_i + L\sin\theta_i\,\hat{\mathbf{z}}
\tag{2.1}
$$

此时三条腿彼此独立，尚未耦合。

### 2.2 等效球心

连杆长度恒定给出三条约束：

$$
\left\|\mathbf{A}_i - \mathbf{B}_i\right\| = l,\qquad i = 1,2,3
\tag{2.2}
$$

代入式 (1.1)：

$$
\left\|\mathbf{P} + r\,\mathbf{u}_i - \mathbf{B}_i\right\| = l
\;\Longleftrightarrow\;
\left\|\mathbf{P} - \left(\mathbf{B}_i - r\,\mathbf{u}_i\right)\right\| = l
$$

定义**等效球心**：

$$
\mathbf{E}_i := \mathbf{B}_i - r\,\mathbf{u}_i
= \left[\left(R - r\right) + L\cos\theta_i\right]\mathbf{u}_i + L\sin\theta_i\,\hat{\mathbf{z}}
\tag{2.3}
$$

约束化为三个标准球面方程：

$$
\left\|\mathbf{P} - \mathbf{E}_i\right\| = l,\qquad i = 1,2,3
\tag{2.4}
$$

几何含义：动平台半径 $r$ 被「挪」到球心上，$\mathbf{A}_i$ 与 $\mathbf{P}$ 的刚性偏移被吸收，于是只剩一个未知量 $\mathbf{P}$。后续会用到：

$$
\left\|\mathbf{E}_i\right\|^2 = \left(R-r\right)^2 + 2\left(R-r\right)L\cos\theta_i + L^2
\tag{2.5}
$$

### 2.3 两两相减：三球面 → 两平面

展开式 (2.4)：

$$
\left\|\mathbf{P}\right\|^2 - 2\,\mathbf{E}_i\cdot\mathbf{P} + \left\|\mathbf{E}_i\right\|^2 = l^2
$$

用 $i=1$ 分别减 $i=2$、$i=3$，二次项 $\left\|\mathbf{P}\right\|^2$ 与 $l^2$ 同时消去：

$$
2\left(\mathbf{E}_j - \mathbf{E}_i\right)\cdot\mathbf{P} = \left\|\mathbf{E}_j\right\|^2 - \left\|\mathbf{E}_i\right\|^2
$$

记

$$
\mathbf{v}_1 = \mathbf{E}_2 - \mathbf{E}_1,\qquad \mathbf{v}_2 = \mathbf{E}_3 - \mathbf{E}_1
\tag{2.6}
$$

则 $\mathbf{P}$ 必须同时落在两个平面上：

$$
\mathbf{v}_1\cdot\mathbf{P} = \frac{\left\|\mathbf{E}_2\right\|^2 - \left\|\mathbf{E}_1\right\|^2}{2} =: \beta_1,
\qquad
\mathbf{v}_2\cdot\mathbf{P} = \frac{\left\|\mathbf{E}_3\right\|^2 - \left\|\mathbf{E}_1\right\|^2}{2} =: \beta_2
\tag{2.7}
$$

**几何意义**：$\mathbf{v}_1\cdot\mathbf{P}=\text{常数}$ 表示「$\mathbf{P}$ 在 $\mathbf{v}_1$ 方向的投影固定」，是一个以 $\mathbf{v}_1$ 为法向的平面。因 $\mathbf{P}$ 到 $\mathbf{E}_1$、$\mathbf{E}_2$ 的距离都等于 $l$，该平面正是 $\mathbf{E}_1\mathbf{E}_2$ 的垂直平分面；第二个平面同理。两平面相交得一条直线，$\mathbf{P}$ 在该直线上。

### 2.4 沿交线参数化，解出 $\alpha,\beta$

交线方向：

$$
\mathbf{n} = \frac{\mathbf{v}_1 \times \mathbf{v}_2}{\left\|\mathbf{v}_1 \times \mathbf{v}_2\right\|}
\tag{2.8}
$$

（$\mathbf{n}$ 同时垂直于 $\mathbf{v}_1,\mathbf{v}_2$，故落在两平面内，正是交线方向。）把 $\mathbf{P}$ 在基 $\{\mathbf{v}_1,\mathbf{v}_2,\mathbf{n}\}$ 下展开：

$$
\mathbf{P} = \mathbf{E}_1 + \alpha\,\mathbf{v}_1 + \beta\,\mathbf{v}_2 + \gamma\,\mathbf{n}
\tag{2.9}
$$

代回式 (2.7)，记

$$
a_{11} = \mathbf{v}_1\cdot\mathbf{v}_1,\quad
a_{12} = \mathbf{v}_1\cdot\mathbf{v}_2,\quad
a_{22} = \mathbf{v}_2\cdot\mathbf{v}_2
$$

$$
b_1 = \beta_1 - \mathbf{E}_1\cdot\mathbf{v}_1,\qquad
b_2 = \beta_2 - \mathbf{E}_1\cdot\mathbf{v}_2
$$

得到二元一次方程组：

$$
\begin{bmatrix}
a_{11} & a_{12}\\[2pt]
a_{12} & a_{22}
\end{bmatrix}
\begin{bmatrix}\alpha\\[2pt] \beta\end{bmatrix}
=
\begin{bmatrix}b_1\\[2pt] b_2\end{bmatrix}
\tag{2.10}
$$

克莱姆法则：

$$
\Delta = a_{11}a_{22} - a_{12}^{2},\qquad
\alpha = \frac{b_1 a_{22} - b_2 a_{12}}{\Delta},\qquad
\beta = \frac{a_{11} b_2 - a_{12} b_1}{\Delta}
\tag{2.11}
$$

> 注意 $b_1,b_2$ 中的 $\mathbf{E}_1\cdot\mathbf{v}$ 项不可省。它来自式 (2.9) 中以 $\mathbf{E}_1$ 为原点的平移，省略会使平面过原点，导致 XY 方向整体偏移。

### 2.5 解 $\gamma$：直线与球面求交

令

$$
\mathbf{W} = \alpha\,\mathbf{v}_1 + \beta\,\mathbf{v}_2,\qquad
\mathbf{m} = \mathbf{E}_1 + \mathbf{W}
\tag{2.12}
$$

因 $\mathbf{n}\perp\mathbf{v}_1,\ \mathbf{n}\perp\mathbf{v}_2$，故 $\mathbf{n}\perp\mathbf{W}$，即 $\mathbf{W}\cdot\mathbf{n}=0$。把式 (2.9) 代入第一个球面方程：

$$
\left\|\mathbf{W} + \gamma\,\mathbf{n}\right\|^2 = l^2
\;\Longrightarrow\;
\left\|\mathbf{W}\right\|^2 + 2\gamma\left(\mathbf{W}\cdot\mathbf{n}\right) + \gamma^2\left\|\mathbf{n}\right\|^2 = l^2
$$

交叉项为零，且 $\left\|\mathbf{n}\right\|=1$，于是

$$
\left\|\mathbf{W}\right\|^2 = \alpha^2 a_{11} + \beta^2 a_{22} + 2\alpha\beta\, a_{12}
\tag{2.13}
$$

$$
\gamma^2 = l^2 - \left\|\mathbf{W}\right\|^2
\tag{2.14}
$$

$$
\gamma = \pm\sqrt{\,l^2 - \left\|\mathbf{W}\right\|^2\,}
\tag{2.15}
$$

最终两组解：

$$
\boxed{\ \mathbf{P} = \mathbf{m} \pm \gamma\,\mathbf{n}\ }
\tag{2.16}
$$

### 2.6 解的选取（装配模式）

两个 $\gamma$ 对应同一台机构的两种装配形态（类似四连杆的「交叉 / 开口」构型），装配完成后只能稳定在一种，不会自动跳变。三种用法：

1. **连续性**（轨迹跟踪必用）：与上一控制周期的 $\mathbf{P}$ 比较，取距离更近的一支。
2. **动平台在下**（静态分析）：取 $z$ 较小的那支；本例几何下等价于 $\gamma<0$。
3. **装配标定**：首次上电人工认定一次，之后按连续性跟踪。

> **不要把「取 $+\gamma$」或「取 $-\gamma$」写死为常量。** 随位姿变化，正确解会在两支之间切换（穿过肘部伸直/折叠的折叠点时两支合并），硬编码必然在部分位姿选错。

### 2.7 无解与退化判据

| 判据 | 条件 | 含义 |
|---|---|---|
| 三球心共线 | $\left|\Delta\right| \approx 0$ | $\mathbf{v}_1 \parallel \mathbf{v}_2$，两平面平行，无唯一交线 |
| 三球不相交 | $l^2 - \left\|\mathbf{W}\right\|^2 < 0$ | 交线整条在球外，臂长不足 |
| 球心重合 | $\left\|\mathbf{v}_1\right\|\approx 0$ 或 $\left\|\mathbf{v}_2\right\|\approx 0$ | $\mathbf{n}$ 方向不确定 |

实现时必须显式判定前两条，否则会产出 NaN 或伪解。

---

## 3. 动平台中心活动范围（纯几何）

### 3.1 单腿可达性 = 一个环面

把式 (1.3) 代入式 (2.2)，并记动铰点相对静铰点的径向偏移

$$
a' = a + r - R
\tag{3.1}
$$

得到

$$
\left(a' - L\cos\theta\right)^2 + \left(z - L\sin\theta\right)^2 + b^2 = l^2
\tag{3.2}
$$

固定 $\mathbf{P}$（即固定 $a',b,z$），只有 $\theta$ 可变。令

$$
\rho = \sqrt{a'^2 + z^2}
\tag{3.3}
$$

则式 (3.2) 前两项是「$(a',z)$ 平面内点 $(a',z)$ 到以原点为心、半径 $L$ 的圆上一点的距离平方」，其最小值与最大值分别为 $\left(\rho-L\right)^2$、$\left(\rho+L\right)^2$。于是「存在 $\theta$ 使式 (3.2) 成立」的充要条件为

$$
\left(\rho - L\right)^2 + b^2 \ \le\ l^2\ \le\ \left(\rho + L\right)^2 + b^2
\tag{3.4}
$$

**几何解释**：$\theta$ 扫动时，$\mathbf{E}_i$ 在局部系中描出一个圆

$$
\Gamma_i:\ \text{圆心}\ \begin{bmatrix}R-r\\0\\0\end{bmatrix},\ \text{半径}\ L,\ \text{位于}\ b=0\ \text{平面内}
$$

单腿可达等价于「以 $\mathbf{P}$ 为心、半径 $l$ 的球与 $\Gamma_i$ 相交」，在空间中正是**环面**：母圆半径 $L$、管半径 $l$。

### 3.2 工作空间 = 三个环面之交

$$
\mathcal{W} = \mathcal{T}_1 \cap \mathcal{T}_2 \cap \mathcal{T}_3,\qquad
\mathcal{T}_i = \left\{\mathbf{P} : \operatorname{dist}\left(\mathbf{P},\Gamma_i\right) \le l \le \operatorname{dist}_{\max}\left(\mathbf{P},\Gamma_i\right)\right\}
\tag{3.5}
$$

这是**不考虑任何硬件约束时的纯几何工作空间**，只与 $R,r,L,l$ 有关。

### 3.3 中心轴线上的解析解

在中心轴上 $a=0,\ b=0$，故 $a' = -(R-r)$、$\rho = \sqrt{(R-r)^2 + z^2}$，式 (3.4) 化为

$$
\left(\rho - L\right)^2 \le l^2 \le \left(\rho + L\right)^2
$$

右半：$\rho + L \ge l \Rightarrow \rho \ge l-L = 60$，而本例 $\rho \ge R-r = 100$，恒成立。
左半：$\left|\rho - L\right| \le l \Rightarrow \rho \le L+l$。于是

$$
z^2 \le \left(L + l\right)^2 - \left(R - r\right)^2
\tag{3.6}
$$

代入数值：

$$
\left|z\right| \le \sqrt{340^2 - 100^2} = \sqrt{105600} = 324.96\ \mathrm{mm}
$$

即中心轴纯几何可达范围为 $z \in \left[-324.96,\ +324.96\right]$，总长 $649.9\,\mathrm{mm}$。

**物理解释**：边界处 $\rho = L + l$，等效球心与 $\mathbf{P}$ 的距离恰好等于两臂长之和，即主动臂与从动臂**完全共线伸直**——这同时是该腿的奇异位形（§7）。

### 3.4 数值结果（$\mathbf{P}$ 空间扫描）

方法：在 $z$ 与半径 $\rho$ 上取网格，每个 $(\rho,z)$ 采样 24 个方位角，全部方位满足式 (3.4) 才计为可用半径。

| 项目 | 数值 |
|---|---|
| 纯几何 $z$ 范围 | $-322.5 \sim +322.5\ \mathrm{mm}$（网格步长 2.5 mm） |
| $z$ 行程 | $645.0\ \mathrm{mm}$ |
| 最大可达半径 | $198.0\ \mathrm{mm}$（$z \in [-128,\ +120]$ 区间均可达） |

半径–高度包络（每 20 mm 一行）：

| $z$ (mm) | 最大半径 (mm) | $z$ (mm) | 最大半径 (mm) |
|---|---|---|---|
| $-320$ | 14.0 | $0$ | 176.0 |
| $-300$ | 60.0 | $+20$ | 184.0 |
| $-280$ | 92.0 | $+40$ | 190.0 |
| $-260$ | 118.0 | $+60$ | 196.0 |
| $-240$ | 140.0 | $+80$ | 198.0 |
| $-220$ | 158.0 | $+100$ | 198.0 |
| $-200$ | 172.0 | $+120$ | 198.0 |
| $-180$ | 182.0 | $+140$ | 194.0 |
| $-160$ | 190.0 | $+160$ | 190.0 |
| $-140$ | 194.0 | $+180$ | 182.0 |
| $-120$ | 198.0 | $+200$ | 172.0 |
| $-100$ | 198.0 | $+220$ | 158.0 |
| $-80$ | 198.0 | $+240$ | 140.0 |
| $-60$ | 196.0 | $+280$ | 92.0 |
| $-40$ | 190.0 | $+300$ | 60.0 |
| $-20$ | 184.0 | $+320$ | 14.0 |

包络形状：**上下为尖锥、中间最宽，且关于 $z=0$ 上下对称**（可达条件只依赖 $z^2$）。

两点说明：

1. 该范围是**几何上限**。叠加主动臂关节限位（本例 $-90^\circ \sim +60^\circ$）后，上端会被砍掉一大截，见 §9。
2. 数值上只到 $\pm 322.5$ 而非 $\pm 324.96$，是网格步长与「24 方位全可达」判据共同造成的。

### 3.5 三种求工作空间的方法

| 方法 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| 关节空间正扫 | 遍历 $\theta_1,\theta_2,\theta_3$，逐个正解 | 不依赖逆解分支；直接给出关节角 | 点数 $n^3$ 增长快；边界分辨率受网格限制 |
| 笛卡尔空间测试 | 遍历 $\mathbf{P}$，逐点做逆解可达判定 | 直接得到 $\mathbf{P}$ 空间边界 | 需要 §4.3 的可达判据 |
| 解析包络 | 用式 (3.4) 直接求每个 $z$ 的最大半径 | 无采样误差、最快 | 一般只在中心轴等对称位置有闭式解 |

本文 §3.4 用第二种（配合式 (3.4) 判定），§3.3 用第三种做对照，二者吻合。

> **扫描实现的一个陷阱**：关节空间正扫时若只保留单一装配分支（例如恒取 $z$ 较小的一支），点云只会覆盖可达集的一半——实测 $n=81$ 时 $z$ 上界从 $+324$ 掉到 $+181$。必须两个分支都收集，才能与式 (3.4) 的结果一致。

---

## 4. 逆运动学：$\mathbf{P}\;\to\;$ 是否可达 + 三个电机角

### 4.1 投影到腿平面

给定 $\mathbf{P}=(x,y,z)$，第 $i$ 条腿的局部坐标为

$$
a = x\cos\varphi_i + y\sin\varphi_i,\qquad
b = -x\sin\varphi_i + y\cos\varphi_i,\qquad
a' = a + r - R
\tag{4.1}
$$

每条腿的方程只依赖 $(a',b,z)$，三条腿形式相同，故可逐腿独立求解。

### 4.2 圆–圆求交

由式 (3.2)：

$$
\left(a' - L\cos\theta\right)^2 + \left(z - L\sin\theta\right)^2 + b^2 = l^2
$$

展开并利用 $\cos^2\theta + \sin^2\theta = 1$：

$$
a'^2 + b^2 + z^2 + L^2 - l^2 = 2L\left(a'\cos\theta + z\sin\theta\right)
$$

两边除以 $2L$，记

$$
d = \frac{a'^2 + b^2 + z^2 + L^2 - l^2}{2L}
\tag{4.2}
$$

则

$$
a'\cos\theta + z\sin\theta = d
\tag{4.3}
$$

令

$$
\rho = \sqrt{a'^2 + z^2},\qquad \psi = \operatorname{atan2}\left(z,\ a'\right)
\tag{4.4}
$$

即 $a' = \rho\cos\psi,\ z = \rho\sin\psi$，式 (4.3) 左边合并为一个余弦：

$$
\rho\cos\left(\theta - \psi\right) = d
\;\Longleftrightarrow\;
\cos\left(\theta - \psi\right) = \frac{d}{\rho}
\tag{4.5}
$$

### 4.3 可达判据

式 (4.5) 有实数解，当且仅当

$$
\left|\frac{d}{\rho}\right| \le 1
\tag{4.6}
$$

展开成分量形式：

$$
\left|\,a'^2 + b^2 + z^2 + L^2 - l^2\,\right| \ \le\ 2L\sqrt{a'^2 + z^2}
\tag{4.7}
$$

三条腿全部满足，该 $\mathbf{P}$ 才可达。

**与环面判据等价**：令 $A = \rho^2 + L^2 + b^2 - l^2$，则

$$
\text{式 (4.7)}^2:\quad A^2 - 4L^2\rho^2 \le 0
$$

而

$$
\left[\left(\rho-L\right)^2 + b^2 - l^2\right]\left[\left(\rho+L\right)^2 + b^2 - l^2\right] = A^2 - 4L^2\rho^2
$$

故两式完全等价——「逆解可达」与「环面判定」是同一件事的两种写法。

### 4.4 两个解与选解

$$
\theta_i = \psi \pm \arccos\!\left(\frac{d}{\rho}\right)
\tag{4.8}
$$

两解关于 $\psi$ 对称，角距

$$
\Delta\theta = 2\arccos\!\left(\frac{d}{\rho}\right)
\tag{4.9}
$$

- $\Delta\theta \to 0$：主动臂与从动臂伸直或折叠（奇异，§7）
- $\Delta\theta \to \pi$：肘部张开最大

选解规则与 §2.6 同构：

1. **轨迹跟踪**：取与上一周期 $\theta_i$ 最接近的一支。
2. **首次装配**：按机械装配构型固定一支，之后按连续性跟踪。
3. **校验**：两支都算，取与正解一致者（§5）。

三条腿的选支**互相独立**——非对称位姿下，三条腿正确解所在的分支号可以不同。

### 4.5 退化情形

当 $\rho = 0$（即 $a'=0$ 且 $z=0$）时，式 (4.5) 左边恒为 $0$：

$$
\rho = 0:\quad \text{有解} \iff d = 0 \iff b^2 = l^2 - L^2
$$

其余情况该腿无解。实现时必须先判 $\rho > \varepsilon$，否则除零。

### 4.6 电机指令换算

逆解输出的是主动臂转角 $\theta_i$，驱动器侧还需换算：

$$
N_i = \frac{\theta_i \cdot G}{2\pi}\cdot m\cdot 200,\qquad
C_i = \frac{\theta_i}{2\pi}\cdot N_{\text{enc}}\cdot 4
\tag{4.10}
$$

其中 $G$ 为减速比，$m$ 为步进细分，$200$ 为整步数（$1.8^\circ/\text{步}$），$N_{\text{enc}}$ 为编码器线数。

> 这一步属于执行层，但符号约定必须与 $\theta$（从水平径向量起、向上为正）严格对齐，否则整体反向。

---

## 5. 正逆解一致性

正解与逆解描述的是同一个几何约束 $\left\|\mathbf{A}_i-\mathbf{B}_i\right\|=l$，因此必然互逆：

$$
\theta \xrightarrow{\ \text{式 (2.1)(2.3)(2.16)}\ } \mathbf{P}
\xrightarrow{\ \text{式 (4.1)–(4.8)}\ } \theta
$$

数值校验（取 49,517 个有效位姿随机抽 2,000 个）：

$$
\max_i\left|\operatorname{wrap}\left(\theta_i^{\mathrm{IK}} - \theta_i^{\mathrm{FK}}\right)\right| = 1.02\times10^{-13}\ ^\circ
$$

即机器精度级别一致。

> 校验必须**逐腿**取最近分支。若强制三条腿共用同一分支号，非对称位姿会得到上百度伪残差——那是校验写法错，不是算法错。

---

## 6. 数值算例

参数 $R=139,\ r=39,\ L=140,\ l=200$（mm）。

| $\mathbf{P}$ (mm) | 可达 | 每条腿的两个 $\theta$ 解（度） |
|---|---|---|
| $(0,0,-313.2)$ | 是 | $-125.42\ /\ \mathbf{-90.00}$ |
| $(0,0,-300)$ | 是 | $-134.41\ /\ \mathbf{-82.46}$ |
| $(0,0,-250)$ | 是 | $-158.10\ /\ \mathbf{-65.50}$ |
| $(0,0,-200)$ | 是 | $-178.35\ /\ \mathbf{-54.78}$ |
| $(0,0,0)$ | 是 | $\mathbf{68.20}\ /\ 291.80$ |
| $(0,0,325)$ | 否 | —（超出 $\rho \le L+l$） |
| $(40,0,-250)$ | 是 | 腿1：$-154.09/-52.90$；腿2、3：$-157.64/-73.64$ |

读法：

- $z=-313.2$ 时解之一为 $-90^\circ$，正是「主动臂竖直下垂」的装配姿态；另一支 $-125.4^\circ$ 为「肘内」形态。
- $z=0$ 时两解相差约 $223.6^\circ$，说明肘部极度张开。
- 中心轴上三腿角度相同（对称位姿）；偏离中心后三腿角度各异，这正是逆解必须逐腿独立处理的原因。

---

## 7. 奇异位形

由式 (4.6)，奇异发生在

$$
\left|\frac{d}{\rho}\right| = 1
\tag{7.1}
$$

| 条件 | 形态 | 后果 |
|---|---|---|
| $d/\rho = +1$ | 主动臂与从动臂**完全伸直**（共线向外） | 两解合并，该方向刚度为零，微扰导致关节速度爆炸 |
| $d/\rho = -1$ | 完全**折叠**（共线向内） | 同上 |

工程含义：

- 工作空间边界即奇异面，控制上必须留裕度
- 越靠近边界，$\arccos\left(d/\rho\right)$ 对 $\mathbf{P}$ 的导数越大，逆解灵敏度急剧上升
- 速度层面：$\dot{\theta} = J^{-1}\dot{\mathbf{P}}$，$J$ 奇异时 $\dot{\theta}\to\infty$

因此实际作业区间应取在 $\left|d/\rho\right| \le 0.9$ 之类的安全阀内，而不是贴着几何边界。

---

## 8. 代码对照

| 数学步骤 | 函数 |
|---|---|
| 式 (2.1)(2.3) 肘点与等效球心 | `elbow_points()` |
| 式 (2.6)–(2.16) 三球求交 | `forward_kinematics()` |
| 式 (4.1)–(4.8) 逆解与可达判定 | `inverse_kinematics()` |
| 式 (3.4) 环面判据 | `leg_solvable()` |
| §3.4 工作空间扫描 | `workspace()` / `iter_joint_grid()` |
| §5 闭环校验 | `verify_roundtrip()` |

完整代码见附录 A。

---

## 9. 硬件约束如何叠加

本文推导在叠加硬件约束后**无需修改**，只需在工作空间判定上追加掩码：

| 约束 | 数学表达 | 影响边界 |
|---|---|---|
| 主动臂行程 | $\theta_{\min} \le \theta_i \le \theta_{\max}$ | 上下两端锥尖 |
| 臂端球铰锥角 | $\arccos\left(\hat{\mathbf{b}}_i\cdot\hat{\mathbf{s}}_i(\theta_i)\right) \le \gamma_{\text{arm}}$ | 高 $z$ 区收窄 |
| 平台端球铰锥角 | $\arccos\left(-\hat{\mathbf{b}}_i\cdot\hat{\mathbf{h}}_i\right) \le \gamma_{\text{plat}}$ | 大半径区收窄 |
| 杆间干涉 | $\operatorname{dist}_{\min}\left(\text{rod}_i,\text{rod}_j\right) > d_{\text{rod}}$ | 内圈与相邻杆交叉区 |

其中 $\hat{\mathbf{b}}_i$ 为从动臂单位方向，$\hat{\mathbf{s}}_i$ 为随主动臂转动的臂端球窝轴，$\hat{\mathbf{h}}_i$ 为固定在动平台上的平台端球窝轴。

实测这些掩码的通过率远低于几何可达率——**硬件约束才是真实工作空间的决定因素**，纯几何范围只是其上界。

---

## 附录 A　完整算法代码

```python
"""
Delta 机械臂纯运动学内核（不含任何硬件约束）。

约定
  - 基座坐标系：原点在静平台中心，Z 轴向上，静平台上表面 z = 0
  - 主动臂转角 theta 从水平径向量起，向上为正
  - 动平台姿态被平行四边形锁死，A_i = P + r * u_i
  - 长度单位 mm，角度单位 rad
"""

import numpy as np

# ---------------- 参数 ----------------
R = 139.0          # 静平台半径
r = 39.0           # 动平台半径
L = 140.0          # 主动臂长度
l = 200.0          # 从动臂长度

PHI = np.array([0.0, 2.0 * np.pi / 3.0, 4.0 * np.pi / 3.0])
U_DIR = np.stack([np.cos(PHI), np.sin(PHI), np.zeros(3)], axis=-1)   # (3 腿, 3 分量)
T_DIR = np.stack([-np.sin(PHI), np.cos(PHI), np.zeros(3)], axis=-1)
Z_AXIS = np.array([0.0, 0.0, 1.0])


def wrap(x):
    """把角度折叠到 (-pi, pi]"""
    return np.remainder(x + np.pi, 2.0 * np.pi) - np.pi


# ---------------- 正解 ----------------
def elbow_points(thetas):
    """式 (2.1)(2.3)：返回肘点 B_i 与等效球心 E_i，形状 (..., 3 腿, 3 分量)"""
    c, s = np.cos(thetas), np.sin(thetas)
    B = (R + L * c)[..., None] * U_DIR + (L * s)[..., None] * Z_AXIS
    E = (R - r + L * c)[..., None] * U_DIR + (L * s)[..., None] * Z_AXIS
    return B, E


def forward_kinematics(thetas, lower=True):
    """
    式 (2.6)-(2.16)：三球求交

    thetas : (..., 3)
    lower  : True 取 z 较小的一支（动平台在下方）；轨迹跟踪应改用连续性

    返回 P (..., 3), B (..., 3, 3), ok (...,)
    """
    thetas = np.asarray(thetas, dtype=float)
    B, E = elbow_points(thetas)
    E1, E2, E3 = E[..., 0, :], E[..., 1, :], E[..., 2, :]

    # 两个平面：v1·P = beta1, v2·P = beta2
    v1, v2 = E2 - E1, E3 - E1

    # 交线方向
    n_raw = np.cross(v1, v2)
    nn = np.linalg.norm(n_raw, axis=-1)
    ok = nn > 1e-9
    n = n_raw / np.where(ok, nn, 1.0)[..., None]

    a11 = np.sum(v1 * v1, axis=-1)
    a12 = np.sum(v1 * v2, axis=-1)
    a22 = np.sum(v2 * v2, axis=-1)
    beta1 = 0.5 * (np.sum(E2 * E2, -1) - np.sum(E1 * E1, -1))
    beta2 = 0.5 * (np.sum(E3 * E3, -1) - np.sum(E1 * E1, -1))
    b1 = beta1 - np.sum(E1 * v1, axis=-1)          # 不可省略 E1·v1
    b2 = beta2 - np.sum(E1 * v2, axis=-1)

    det = a11 * a22 - a12 * a12
    ok = ok & (np.abs(det) > 1e-9)
    ds = np.where(np.abs(det) > 1e-9, det, 1.0)
    alpha = (b1 * a22 - b2 * a12) / ds
    beta = (a11 * b2 - a12 * b1) / ds

    # 解 gamma
    W2 = alpha * alpha * a11 + beta * beta * a22 + 2.0 * alpha * beta * a12
    g2 = l * l - W2
    ok = ok & (g2 >= 0.0)
    g = np.sqrt(np.maximum(g2, 0.0))

    mid = E1 + alpha[..., None] * v1 + beta[..., None] * v2
    p_lo = mid - g[..., None] * n
    p_hi = mid + g[..., None] * n
    if lower:
        P = np.where((p_lo[..., 2] <= p_hi[..., 2])[..., None], p_lo, p_hi)
    else:
        P = np.where((p_lo[..., 2] <= p_hi[..., 2])[..., None], p_hi, p_lo)
    return P, B, ok


# ---------------- 逆解 ----------------
def inverse_kinematics(P, prev=None, branch="plus"):
    """
    式 (4.1)-(4.8)：逐腿圆-圆求交

    P      : (..., 3) 动平台中心
    prev   : 上一周期的 theta (..., 3)，用于连续性选支；给定时 branch 被忽略
    branch : "plus" 取 theta = psi + arccos；"minus" 取 theta = psi - arccos

    返回 theta (..., 3), ok (...,)
    """
    P = np.asarray(P, dtype=float)
    a = np.einsum("...k,ik->...i", P, U_DIR)       # 径向分量，每条腿一个
    b = np.einsum("...k,ik->...i", P, T_DIR)       # 切向分量
    z = P[..., 2][..., None]                       # 竖直分量，广播到三条腿

    ap = a + r - R                                 # 式 (4.1)
    rho = np.hypot(ap, z)                          # 式 (4.4)
    safe = rho > 1e-9

    d = (ap * ap + b * b + z * z + L * L - l * l) / (2.0 * L)   # 式 (4.2)
    ratio = np.where(safe, d / np.where(safe, rho, 1.0), np.inf)

    ok = np.all(safe, axis=-1) & np.all(np.abs(ratio) <= 1.0, axis=-1)   # 式 (4.6)

    psi = np.arctan2(z, ap)
    delta = np.arccos(np.clip(ratio, -1.0, 1.0))
    th_p = psi + delta
    th_m = psi - delta

    if prev is None:
        theta = th_p if branch == "plus" else th_m
    else:
        prev = np.asarray(prev, dtype=float)
        use_p = np.abs(wrap(th_p - prev)) <= np.abs(wrap(th_m - prev))
        theta = np.where(use_p, th_p, th_m)

    return theta, ok


def leg_solvable(P):
    """式 (3.4)：单腿环面判据，返回 (..., 3) 布尔"""
    P = np.asarray(P, dtype=float)
    a = np.einsum("...k,ik->...i", P, U_DIR)
    b = np.einsum("...k,ik->...i", P, T_DIR)
    z = P[..., 2][..., None]
    ap = a + r - R
    rho = np.hypot(ap, z)
    return ((rho - L) ** 2 + b * b <= l * l) & (l * l <= (rho + L) ** 2 + b * b)


def reachable(P):
    """三条腿同时可解才算 P 可达"""
    return np.all(leg_solvable(P), axis=-1)


# ---------------- 工作空间 ----------------
def iter_joint_grid(n, lo=-np.pi, hi=np.pi, chunk=200_000):
    """按块生成 n^3 个关节组合（无硬件约束，theta 取全周）"""
    g = np.linspace(lo, hi, n)
    total = n ** 3
    for s in range(0, total, chunk):
        idx = np.arange(s, min(s + chunk, total))
        k = idx % n
        j = (idx // n) % n
        i = idx // (n * n)
        yield np.stack([g[i], g[j], g[k]], axis=-1)


def workspace(n=121, chunk=200_000):
    """关节空间正扫 -> 可达点云 (N, 3)

    两个装配分支都要收集：只取 lower 支会丢掉只以「上支」出现的位姿，
    点云的 z 上界会被截掉一半。
    """
    pts = []
    for th in iter_joint_grid(n, chunk=chunk):
        P_lo, _, ok = forward_kinematics(th, lower=True)
        P_hi, _, _ = forward_kinematics(th, lower=False)
        if np.any(ok):
            pts.append(P_lo[ok])
            pts.append(P_hi[ok])
    return np.concatenate(pts) if pts else np.empty((0, 3))


# ---------------- 自检 ----------------
def verify_roundtrip(n_sample=5000, seed=0):
    """随机 theta -> FK -> IK，两支都算、逐腿取最近，返回最大角度残差（度）"""
    rng = np.random.default_rng(seed)
    th = rng.uniform(-np.pi, np.pi, size=(n_sample, 3))
    P, _, ok = forward_kinematics(th)
    th, P = th[ok], P[ok]

    th_p, ok_p = inverse_kinematics(P, branch="plus")
    th_m, _ = inverse_kinematics(P, branch="minus")

    err = np.minimum(np.abs(wrap(th_p - th)), np.abs(wrap(th_m - th)))
    return np.degrees(np.max(err)), bool(np.all(ok_p))


if __name__ == "__main__":
    err_deg, all_ok = verify_roundtrip()
    print("正解-逆解闭环最大残差 = %.2e 度（可达判定全部通过=%s）" % (err_deg, all_ok))

    P = np.array([0.0, 0.0, -313.2])
    th, ok = inverse_kinematics(P)
    print("P=%s 可达=%s theta(度)=%s" % (P, ok, np.degrees(th)))

    pts = workspace(n=81)
    print("工作空间点云：%d 点，Z 范围 [%.1f, %.1f]"
          % (len(pts), pts[:, 2].min(), pts[:, 2].max()))
```
