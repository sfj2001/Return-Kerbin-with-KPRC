import numpy as np
from scipy.integrate import solve_ivp

# Kerbin constants
R_k = 600_000  # m (星球半径)
mu_k = 3.5316e12  # m^3/s^2 (引力常数)
atm_altitude = 70_000  # m (大气层高度)
rho_0 = 1.223  # kg/m^3 (大气层表面密度)
H = 6_000  # m (大气层标高)
C_D, C_L, m, A = 2.0, 0.0, 4132, np.pi*1.3*1.3  # 阻力系数、升力系数、质量、面积

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

def trajectory_optimization(initial_orbit_altitude, target_theta, tolerance=1e-3, max_iter=150):
    """
    轨迹优化函数：
    输入：
    - initial_orbit_altitude: 初始轨道高度（单位：m）
    - target_theta: 目标着陆点的相角（单位：弧度）
    输出：
    - maneuver_theta: 轨道机动点的极角（单位：弧度）
    - delta_v: 在轨道机动节点需要减少的径向速度（单位：m/s）
    - final_theta: 实际着陆点的相角（单位：弧度）
    """
    current_error = float("inf")
    maneuver_theta = np.radians(-90)  # 初始轨道机动点角度
    delta_v = -120  # 初始速度变化
    step_size_theta = np.radians(5)  # 初始轨道机动点角度步长
    step_size_v = 10  # 初始速度变化步长
    iterations = 0

    reduction_factor = 0.9  # 步长衰减因子
    min_step_theta = np.radians(0.1)  # 最小角度步长
    min_step_v = 0.5  # 最小速度步长

    # 添加备份机制
    last_success_theta = maneuver_theta
    last_success_v = delta_v

    while abs(current_error) > tolerance and iterations < max_iter:
        iterations += 1

        # 初始圆轨道的半径和速度
        r_orbit = R_k + initial_orbit_altitude
        v_orbit = np.sqrt(mu_k / r_orbit)

        # 设置变轨点初始条件
        vr = 0  # 径向速度初始为0
        vtheta = v_orbit + delta_v  # 切向速度
        y0 = [r_orbit, maneuver_theta, vr, vtheta]  # 初始条件
        t_span = (0, 50000)

        # 仿真轨迹
        solution = solve_ivp(equations, t_span, y0, method="RK45", max_step=1, events=hit_surface)

        if solution.t_events[0].size > 0:  # 仿真中检测到着陆
            # 提取实际着陆点
            final_theta = solution.y[1][-1]
            current_error = final_theta - target_theta

            # 更新备份
            last_success_theta = maneuver_theta
            last_success_v = delta_v

            # 动态调整步长
            if abs(current_error) < tolerance:
                break  # 满足误差要求，退出循环
            step_size_theta = max(step_size_theta * reduction_factor, min_step_theta)
            step_size_v = max(step_size_v * reduction_factor, min_step_v)

            # 更新优化变量
            maneuver_theta -= step_size_theta * np.sign(current_error)
            delta_v -= step_size_v * np.sign(current_error)
        else:  # 仿真未检测到着陆
            maneuver_theta = last_success_theta
            delta_v = last_success_v

            # 缩小步长
            step_size_theta = max(step_size_theta * reduction_factor, min_step_theta)
            step_size_v = max(step_size_v * reduction_factor, min_step_v)

    # 结果输出
    final_theta = last_success_theta if iterations >= max_iter else solution.y[1][-1]
    return maneuver_theta, delta_v, final_theta
############################################################################################################
# # 第二次落点预测
# import matplotlib.pyplot as plt
# from matplotlib import rcParams

# # 中文显示
# rcParams['font.sans-serif'] = ['SimHei']
# rcParams['axes.unicode_minus'] = False

# def cartesian_to_polar(x, y, vx, vy):
#     """
#     将直角坐标转换为极坐标。
#     输入：
#     - x, y: 航天器的位置
#     - vx, vy: 航天器的速度
#     输出：
#     - r, theta: 极坐标位置
#     - vr, vtheta: 极坐标速度
#     """
#     r = np.sqrt(x**2 + y**2)
#     theta = np.arctan2(y, x)
#     vr = (x * vx + y * vy) / r  # 径向速度
#     vtheta = (x * vy - y * vx) / r  # 角向速度
#     return r, theta, vr, vtheta

# def simulate_reentry_trajectory_cartesian(position, velocity, target_theta):
#     """
#     根据直角坐标下的初始条件仿真再入轨迹，并展示结果。
#     :param position: 航天器的初始位置 [x, y] (m)
#     :param velocity: 航天器的初始速度 [vx, vy] (m/s)
#     :param target_theta: 目标着陆点的极角 (rad)
#     """
#     # 转换为极坐标
#     r, theta, vr, vtheta = cartesian_to_polar(position[0], position[1], velocity[0], velocity[1])
#     initial_conditions = [r, theta, vr, vtheta]

#     # 仿真轨迹
#     t_span = (0, 50000)
#     solution = solve_ivp(equations, t_span, initial_conditions, method="RK45", max_step=1, events=hit_surface)

#     # 提取仿真结果
#     r_plot = solution.y[0]
#     theta_plot = solution.y[1]
#     x_plot = r_plot * np.cos(theta_plot)
#     y_plot = r_plot * np.sin(theta_plot)

#     # 理论落点
#     final_theta = solution.y[1][-1]
#     actual_x = R_k * np.cos(final_theta)
#     actual_y = R_k * np.sin(final_theta)

#     # 绘制轨迹
#     plt.figure(figsize=(10, 6))

#     # 绘制初始轨道
#     r_orbit = r  # 初始轨道半径
#     theta_orbit = np.linspace(0, 2 * np.pi, 500)
#     x_orbit = r_orbit * np.cos(theta_orbit)
#     y_orbit = r_orbit * np.sin(theta_orbit)
#     plt.plot(x_orbit, y_orbit, label="初始轨道", linestyle="--", color="gray")

#     # 绘制仿真轨迹
#     plt.plot(x_plot, y_plot, label="再入轨迹", color="blue")

#     # 标注目标点和实际着陆点
#     target_x = R_k * np.cos(target_theta)
#     target_y = R_k * np.sin(target_theta)
#     plt.scatter(target_x, target_y, color='green', label="目标点", zorder=5)
#     plt.scatter(actual_x, actual_y, color='orange', label="实际着陆点", zorder=5)

#     # 绘制星球表面
#     circle = plt.Circle((0, 0), R_k, color='gray', fill=False, linestyle='--', label="星球表面")
#     plt.gca().add_artist(circle)

#     # 图例和标题
#     plt.xlabel("X (m) - 水平距离")
#     plt.ylabel("Y (m) - 垂直距离")
#     plt.title("再入轨迹仿真")
#     plt.legend()
#     plt.axis('equal')
#     plt.grid(True)
#     plt.show()

#     return final_theta  # 返回实际落点的极角