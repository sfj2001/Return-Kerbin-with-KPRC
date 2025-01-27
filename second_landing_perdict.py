import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib import rcParams

# 中文显示
rcParams['font.sans-serif'] = ['SimHei']
rcParams['axes.unicode_minus'] = False
# Kerbin constants
R_k = 600_000  # m (星球半径)
mu_k = 3.5316e12  # m^3/s^2 (引力常数)
atm_altitude = 70_000  # m (大气层高度)
rho_0 = 1.223  # kg/m^3 (大气层表面密度)
H = 6_000  # m (大气层标高)
C_D, C_L, m, A = 2.0, 0.3, 4132, np.pi*1.3*1.3  # 阻力系数、升力系数、质量、面积

# Helper functions
def density(r):
    """计算大气密度"""
    h = r - R_k
    return rho_0 * np.exp(-h / H) if 0 <= h <= atm_altitude else 0

def equations(t, y):
    """轨迹仿真的运动方程"""
    r, theta, vr, vtheta = y
    v = np.sqrt(vr**2 + vtheta**2)
    gamma = np.arctan2(vr, vtheta)
    rho = density(r)

    # 重力加速度
    a_grav_r = -mu_k / r**2

    # 阻力和升力加速度，仅在大气层内生效
    D = 0.5 * rho * v**2 * C_D * A / m if rho > 0 else 0
    L = 0.5 * rho * v**2 * C_L * A / m if rho > 0 else 0
    a_drag_r = -D * np.cos(gamma)
    a_drag_theta = -D * np.sin(gamma)
    a_lift_r = L * np.sin(gamma)
    a_lift_theta = -L * np.cos(gamma)

    # 自转角速度
    omega = 2 * np.pi / 21600  # rad/s
    a_coriolis_theta = -2 * omega * vr  # 科里奥利加速度的角向分量
    a_centrifugal_r = omega**2 * r  # 离心加速度的径向分量


    # 径向加速度
    drdt = vr
    dvrdt = a_grav_r + a_drag_r + a_lift_r + a_centrifugal_r + vtheta**2 / r

    # 角速度变化率
    dthetadt = vtheta / r
    dvthetadt = a_drag_theta + a_lift_theta + a_coriolis_theta - vr * vtheta / r

    return [drdt, dthetadt, dvrdt, dvthetadt]

def hit_surface(t, y):
    """检测是否到达星球表面"""
    r, theta, vr, vtheta = y
    return r - R_k
hit_surface.terminal = True
hit_surface.direction = -1

############################################################################################################
# 第二次落点预测


def cartesian_to_polar(x, y, vx, vy):
    """
    将直角坐标转换为极坐标。
    输入：
    - x, y: 航天器的位置
    - vx, vy: 航天器的速度
    输出：
    - r, theta: 极坐标位置
    - vr, vtheta: 极坐标速度
    """
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)
    vr = (x * vx + y * vy) / r  # 径向速度
    vtheta = (x * vy - y * vx) / r  # 角向速度
    return r, theta, vr, vtheta

## 第二次落点预报程序
def simulate_reentry_trajectory_cartesian(position, velocity, target_theta):
    """
    根据直角坐标下的初始条件仿真再入轨迹，并展示结果。
    :param position: 航天器的初始位置 [x, y] (m)
    :param velocity: 航天器的初始速度 [vx, vy] (m/s)
    :param target_theta: 目标着陆点的极角 (rad)
    """
    # 转换为极坐标
    r, theta, vr, vtheta = cartesian_to_polar(position[0], position[1], velocity[0], velocity[1])
    initial_conditions = [r, theta, vr, vtheta]

    # 仿真轨迹
    t_span = (0, 50000)
    solution = solve_ivp(equations, t_span, initial_conditions, method="RK45", max_step=1, events=hit_surface)

    # 提取仿真结果
    r_plot = solution.y[0]
    theta_plot = solution.y[1]
    x_plot = r_plot * np.cos(theta_plot)
    y_plot = r_plot * np.sin(theta_plot)

    # 理论落点
    final_theta = solution.y[1][-1]
    actual_x = R_k * np.cos(final_theta)
    actual_y = R_k * np.sin(final_theta)

    # 绘制轨迹
    plt.figure(figsize=(10, 6))

    # 绘制初始轨道
    r_orbit = r  # 初始轨道半径
    theta_orbit = np.linspace(0, 2 * np.pi, 500)
    x_orbit = r_orbit * np.cos(theta_orbit)
    y_orbit = r_orbit * np.sin(theta_orbit)
    plt.plot(x_orbit, y_orbit, label="初始轨道", linestyle="--", color="gray")

    # 绘制仿真轨迹
    plt.plot(x_plot, y_plot, label="再入轨迹", color="blue")

    # 标注目标点和实际着陆点
    target_x = R_k * np.cos(target_theta)
    target_y = R_k * np.sin(target_theta)
    plt.scatter(target_x, target_y, color='green', label="目标点", zorder=5)
    plt.scatter(actual_x, actual_y, color='orange', label="实际着陆点", zorder=5)

    # 绘制星球表面
    circle = plt.Circle((0, 0), R_k, color='gray', fill=False, linestyle='--', label="星球表面")
    plt.gca().add_artist(circle)

    # 图例和标题
    plt.xlabel("X (m) - 水平距离")
    plt.ylabel("Y (m) - 垂直距离")
    plt.title("再入轨迹仿真")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

    return final_theta  # 返回实际落点的极角

###################################################################################
# 落点位置计算
def simulate_final_theta(position, velocity):
    """
    根据直角坐标下的初始条件仿真再入轨迹，并展示结果。
    :param position: 航天器的初始位置 [x, y] (m)
    :param velocity: 航天器的初始速度 [vx, vy] (m/s)
    :param target_theta: 目标着陆点的极角 (rad)
    """
    # 转换为极坐标
    r, theta, vr, vtheta = cartesian_to_polar(position[0], position[1], velocity[0], velocity[1])
    initial_conditions = [r, theta, vr, vtheta]

    # 仿真轨迹
    t_span = (0, 50000)
    solution = solve_ivp(equations, t_span, initial_conditions, method="RK45", max_step=1, events=hit_surface)

    # 理论落点
    final_theta = solution.y[1][-1]


    r_plot = solution.y[0]
    theta_plot = solution.y[1]
    x_plot = r_plot * np.cos(theta_plot)
    y_plot = r_plot * np.sin(theta_plot)

    return final_theta,x_plot,y_plot # 返回实际落点的极角



###################################################################################
# 落点修正程序
from scipy.optimize import minimize

# 目标函数，返回落点误差
def landing_error(delta_v, r0, theta0, vr0, vtheta0, t0, t_burn):
    """
    计算给定速度增量下的落点误差。
    """
    delta_vr, delta_vtheta = delta_v  # 分解速度增量

    # 初始条件
    r = r0
    theta = theta0
    vr = vr0 + delta_vr  # 更新径向速度
    vtheta = vtheta0 + delta_vtheta  # 更新角向速度
    y0 = [r, theta, vr, vtheta]

    # 时间积分
    t_span = (t_burn, 5000)  # 从点火后开始积分
    solution = solve_ivp(equations, t_span, y0, method="RK45", events=hit_surface)

    # 目标点极角
    target_theta = np.radians(0)  # 目标点的极角

    if solution.t_events[0].size > 0:  # 如果轨迹落地
        final_theta = solution.y[1][-1]  # 落点角度
        return abs(final_theta - target_theta)  # 返回误差
    else:
        return float("inf")  # 未落地，返回极大误差

# 优化函数
def optimize_velocity_increment(position, velocity):
    """
    优化速度增量以最小化落点误差。
    """

    # 将直角坐标转化为极坐标
    r0, theta0, vr0, vtheta0 = cartesian_to_polar(position[0], position[1], velocity[0], velocity[1])


    t0 = 0
    t_burn = t0 + 45

    initial_guess = [0, 0]  # 初始增量猜测
    bounds = [(-500, 500), (-500, 500)]  # 增量限制范围（可以调整）
    
    # 使用scipy.optimize.minimize进行优化
    result = minimize(
        landing_error,
        initial_guess,
        args=(r0, theta0, vr0, vtheta0, t0, t_burn),
        bounds=bounds,
        method="L-BFGS-B"
    )

    if result.success:
        return result.x  # 返回优化得到的速度增量
    else:
        raise ValueError("优化失败，未找到合适的速度增量。")
    

    # 绘制轨迹并标注落点
# def plot_trajectory_with_adjustment(position, velocity, delta_v):
#     """
#     绘制调整后的轨迹，并标记理论落点和目标点。
#     """
#     # 将直角坐标转化为极坐标
#     r0, theta0, vr0, vtheta0 = cartesian_to_polar(position[0], position[1], velocity[0], velocity[1])

#     t_burn = 45

#     delta_vr, delta_vtheta = delta_v

#     # 初始条件
#     r = r0
#     theta = theta0
#     vr = vr0 + delta_vr  # 更新径向速度
#     vtheta = vtheta0 + delta_vtheta  # 更新角向速度
#     y0 = [r, theta, vr, vtheta]

#     # 积分轨迹
#     t_span = (t_burn, 5000)
#     # solution = solve_ivp(equations, t_span, y0, method="RK45", events=hit_surface)

#     solution = solve_ivp(equations, t_span, y0, method="RK45", max_step=1, events=hit_surface)

#     # 提取轨迹数据
#     r_traj = solution.y[0]
#     theta_traj = solution.y[1]
#     x_traj = r_traj * np.cos(theta_traj)
#     y_traj = r_traj * np.sin(theta_traj)

#     # 提取落点位置
#     if solution.t_events[0].size > 0:
#         final_r = solution.y[0][-1]
#         final_theta = solution.y[1][-1]
#         x_landing = final_r * np.cos(final_theta)
#         y_landing = final_r * np.sin(final_theta)
#     else:
#         x_landing, y_landing = None, None

#     # 目标点位置
#     target_theta = np.radians(0)  # 目标点极角
#     x_target = R_k * np.cos(target_theta)
#     y_target = R_k * np.sin(target_theta)

#     # 绘图
#     plt.figure(figsize=(10, 6))
#     plt.plot(x_traj, y_traj, label="调整后轨迹", color="blue")
#     plt.scatter(x_target, y_target, color="green", label="目标点", zorder=5, s=100, marker="o")
#     if x_landing is not None and y_landing is not None:
#         plt.scatter(x_landing, y_landing, color="red", label="理论落点", zorder=5, s=100, marker="x")
#     plt.scatter(0, 0, color="gray", label="星球中心", s=50)

#     # 绘制星球表面
#     circle1 = plt.Circle((0, 0), R_k, color='gray', fill=False, linestyle='--', label="星球表面")
#     plt.gca().add_artist(circle1)

#     circle2 = plt.Circle((0, 0), R_k+70_000, color='gray', fill=False, linestyle='--', label="大气上界")
#     plt.gca().add_artist(circle2)

#     plt.xlabel("X (m)")
#     plt.ylabel("Y (m)")
#     plt.title("调整后轨迹与落点")
#     plt.legend()
#     plt.axis('equal')
#     plt.grid()
#     plt.show()

def plot_trajectory_with_adjustment(position, velocity, delta_v):
    """
    绘制调整前后的轨迹，并标记理论落点和目标点。
    """
    # 将直角坐标转化为极坐标
    r0, theta0, vr0, vtheta0 = cartesian_to_polar(position[0], position[1], velocity[0], velocity[1])

    t_burn = 45  # 点火时间

    # 提取速度增量
    delta_vr, delta_vtheta = delta_v

    # ====================
    # 调整后的轨迹
    # ====================
    r_adjusted = r0
    theta_adjusted = theta0
    vr_adjusted = vr0 + delta_vr  # 更新径向速度
    vtheta_adjusted = vtheta0 + delta_vtheta  # 更新角向速度
    y0_adjusted = [r_adjusted, theta_adjusted, vr_adjusted, vtheta_adjusted]

    # 积分调整后的轨迹
    t_span = (t_burn, 5000)
    solution_adjusted = solve_ivp(equations, t_span, y0_adjusted, method="RK45", max_step=1, events=hit_surface)

    # 提取调整后的轨迹数据
    r_traj_adjusted = solution_adjusted.y[0]
    theta_traj_adjusted = solution_adjusted.y[1]
    x_traj_adjusted = r_traj_adjusted * np.cos(theta_traj_adjusted)
    y_traj_adjusted = r_traj_adjusted * np.sin(theta_traj_adjusted)

    # 提取调整后的落点位置
    if solution_adjusted.t_events[0].size > 0:
        final_r_adjusted = solution_adjusted.y[0][-1]
        final_theta_adjusted = solution_adjusted.y[1][-1]
        x_landing_adjusted = final_r_adjusted * np.cos(final_theta_adjusted)
        y_landing_adjusted = final_r_adjusted * np.sin(final_theta_adjusted)
    else:
        x_landing_adjusted, y_landing_adjusted = None, None

    # ====================
    # 调整前的轨迹
    # ====================
    r_original = r0
    theta_original = theta0
    vr_original = vr0  # 未调整径向速度
    vtheta_original = vtheta0  # 未调整角向速度
    y0_original = [r_original, theta_original, vr_original, vtheta_original]

    # 积分调整前的轨迹
    solution_original = solve_ivp(equations, t_span, y0_original, method="RK45", max_step=1, events=hit_surface)

    # 提取调整前的轨迹数据
    r_traj_original = solution_original.y[0]
    theta_traj_original = solution_original.y[1]
    x_traj_original = r_traj_original * np.cos(theta_traj_original)
    y_traj_original = r_traj_original * np.sin(theta_traj_original)

    # 提取调整前的落点位置
    if solution_original.t_events[0].size > 0:
        final_r_original = solution_original.y[0][-1]
        final_theta_original = solution_original.y[1][-1]
        x_landing_original = final_r_original * np.cos(final_theta_original)
        y_landing_original = final_r_original * np.sin(final_theta_original)
    else:
        x_landing_original, y_landing_original = None, None

    # ====================
    # 绘图
    # ====================
    plt.figure(figsize=(12, 8))

    # 绘制调整后的轨迹
    plt.plot(x_traj_adjusted, y_traj_adjusted, label="调整后轨迹", color="blue", linewidth=2)

    # 绘制调整前的轨迹
    plt.plot(x_traj_original, y_traj_original, label="调整前轨迹", color="orange", linestyle="--", linewidth=2)

    # 标注目标点
    target_theta = np.radians(0)  # 目标点极角
    x_target = R_k * np.cos(target_theta)
    y_target = R_k * np.sin(target_theta)
    plt.scatter(x_target, y_target, color="green", label="目标点", zorder=5, s=120, marker="o")

    # 标注调整后的落点
    if x_landing_adjusted is not None and y_landing_adjusted is not None:
        plt.scatter(x_landing_adjusted, y_landing_adjusted, color="red", label="调整后落点", zorder=5, s=120, marker="x")

    # 标注调整前的落点
    if x_landing_original is not None and y_landing_original is not None:
        plt.scatter(x_landing_original, y_landing_original, color="purple", label="调整前落点", zorder=5, s=120, marker="x")

    # 绘制星球表面
    circle1 = plt.Circle((0, 0), R_k, color="gray", linestyle="-", fill=False, label="星球表面")
    plt.gca().add_artist(circle1)

    # 绘制大气上界
    circle2 = plt.Circle((0, 0), R_k + 70_000, color="gray", linestyle="--", fill=False, label="大气上界")
    plt.gca().add_artist(circle2)

    # 图例与标题
    plt.xlabel("X (m)", fontsize=14)
    plt.ylabel("Y (m)", fontsize=14)
    plt.title("调整前后轨迹与落点", fontsize=16)
    plt.legend(fontsize=12, loc="upper right")
    plt.axis("equal")
    plt.grid(True)

    # 显示绘图
    plt.tight_layout()
    plt.show()
