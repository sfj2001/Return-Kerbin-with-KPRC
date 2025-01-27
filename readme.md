# 0️⃣写在前面

非常高兴能和各位坎巴拉的玩家在这里见面！本程序能够实现**赤道平面近坎轨道**返回**坎巴拉太空中心**，我非常期待能和大家在这里交流，最终能够使得这个程序变得更好！

## 使用说明

### 特别注意

🔔 请根据你的飞船质量和迎风面积对下列文件中的代码进行修改

- chose_exchange_point.py
- second_landing_perdict.py


```
C_D, C_L, m, A = 2.0, 0.0, 4132, np.pi*1.3*1.3  # 阻力系数、升力系数、质量、面积
```

🔔 除此之外，你需要保证：
- 你的飞船应该装备有一个能够正常出力的引擎
- 当你的飞船进入坎星大气时，是一个**低升阻比的航天器**（即就是一个带着防热大底的返回舱）而不是其他的什么东西



### 环境要求

- 安装**坎巴拉太空计划**并安装了🔗[KPRC](https://www.curseforge.com/kerbal/ksp-mods/krpc-control-the-game-using-c-c-java-lua-python/files)
- Python 3.8 及以上
- 安装 `krpc` 库：`pip install krpc`

### KPRC 相关推荐资料

- 🔗[官方文档](https://krpc.github.io/krpc/)
- 🔗[基础使用指南](https://www.bilibili.com/opus/400594077896210025)


### 运行程序

在终端中运行以下命令启动回收程序：
```
python main.py
```

# 1️⃣进行航天器轨迹仿真的数学模型

本模型描述**旋转天体大气层内物体运动**，考虑以下物理效应：
1. 天体万有引力
2. 大气阻力与升力
3. 离心力与科里奥利力
4. 极坐标系下的运动学耦合


## 🔔 假设和简化

- 不计大气运动，**假定**大气的升力系数和阻力系数为常量
- 假定在大气上界之上，航天器没有空气阻力的影响





## 变量定义
| 符号    | 物理意义           | 代码对应 |
|---------|--------------------|----------|
| $r$     | 径向距离           | `r`      |
| $\theta$| 极角               | `theta`  |
| $v_r$   | 径向速度           | `vr`     |
| $v_\theta$ | 横向速度        | `vtheta` |

### 天体参数
| 符号  | 物理意义               | 数值/表达式              |
|-------|------------------------|--------------------------|
| $\mu$ | 引力参数               | 天体质量×引力常数        |
| $\omega$ | 自转角速度         | $\frac{2\pi}{21600}\ \text{rad/s}$ |
| $\rho(r)$ | 大气密度函数     | 高度相关的经验公式       |

特别地，仅当飞船对地高度低于大气上界高度时，考虑大气的作用

### 控制参数
| 参数  | 物理意义           | 代码变量 |
|-------|--------------------|----------|
| $C_D$ | 阻力系数           | `C_D`    |
| $C_L$ | 升力系数           | `C_L`    |
| $A$   | 参考面积           | `A`      |
| $m$   | 物体质量           | `m`      |

## 核心微分方程组
### 受力分析图

☝️ 引擎推力**显然在无动力再入阶段**`T=0`


<p align="center">
    <img src="src\pic\pic_F_aly.png" alt="受力分析图">
</p>
<p align="center">
    受力分析图，展示了航天器在再入段过程中所受的主要力，包括升力(L)、阻力(D)、引擎推力(T)和重力(Mg)。
</p>

### 运动学方程
$$
\begin{cases}
\displaystyle \frac{dr}{dt} = v_r \\[10pt]
\displaystyle \frac{d\theta}{dt} = \frac{v_\theta}{r}
\end{cases}
$$

### 动力学方程
**径向加速度**：
$$
\frac{dv_r}{dt} = \underbrace{-\frac{\mu}{r^2}}_{\text{引力}} + \underbrace{\frac{v_\theta^2}{r}}_{\text{向心项}} + \underbrace{\omega^2 r}_{\text{离心力}} + \underbrace{\frac{\rho v}{2m}(C_L v_r - C_D v_\theta)}_{\text{气动效应}}
$$

**横向加速度**：
$$
\frac{dv_\theta}{dt} = \underbrace{-\frac{v_r v_\theta}{r}}_{\text{运动耦合}} \underbrace{-2\omega v_r}_{\text{科里奥利}} + \underbrace{\frac{\rho v}{2m}(-C_L v_\theta - C_D v_r)}_{\text{气动效应}}
$$

## 关键参数计算
### 速度合成
$$
v = \sqrt{v_r^2 + v_\theta^2}
$$

### 飞行路径角
$$
\gamma = \arctan\left(\frac{v_r}{v_\theta}\right)
$$

### 气动力分量
| 分量             | 表达式                          | 方向       |
|------------------|---------------------------------|----------------|
| 阻力加速度径向   | $a_{D,r} = -\frac{D}{m}\cos\gamma$ | 抵抗运动方向   |
| 阻力加速度横向   | $a_{D,\theta} = -\frac{D}{m}\sin\gamma$ |                |
| 升力加速度径向   | $a_{L,r} = \frac{L}{m}\sin\gamma$   | 垂直运动方向   |
| 升力加速度横向   | $a_{L,\theta} = -\frac{L}{m}\cos\gamma$ |                |

其中：
$$
\begin{aligned}
D &= \frac{1}{2}\rho v^2 C_D A \\
L &= \frac{1}{2}\rho v^2 C_L A
\end{aligned}
$$

下图为采用本文策略所得到的返回曲线

<p align="center">
    <img src="src\pic\实际返回曲线.png" alt="实际返回曲线">
</p>



## 🐛当前程序仍存在的问题

### ❗落点精度不高


造成这一问题的主要原因可能与飞船的减速过程有关

原则上，应该在第一次降轨操作后，再进行一次轨道修正


### ❗飞船再入段攻角较大

程序在再入段对攻角的控制采用PID的合理性有待近一步讨论



# 2️⃣其他

## 🔎参考文献

📕 星际航行概论 钱学森著 科学出版社


### 联系方式

如有任何问题，请通过以下方式联系我：

- 	📫 邮箱: 1405988307@qq.com
- GitHub: [sjf2001](https://github.com/sfj2001)

感谢使用本程序！

