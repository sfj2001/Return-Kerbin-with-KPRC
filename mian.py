import krpc
import numpy as np
import sys
import os
import time
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation


from choose_exchange_point import trajectory_optimization

from second_landing_perdict import simulate_final_theta
from change_axis import change_xyz
from in_air_control import predictor_corrector_control,calculate_distance

########################################################################################

def calculate_mean_motion(vessel):
    """
    计算航天器在近圆轨道上的平均角速度（弧度/秒）。
    """
    orbit = vessel.orbit
    mu = orbit.body.gravitational_parameter  # 引力常数
    a = orbit.semi_major_axis  # 半长轴，近圆轨道时为轨道高度 + 星球半径
    mean_motion = np.sqrt(mu / a**3)  # 平均角速度
    return mean_motion

def xyz_2_rt(point):
    """
    根据 XY 坐标计算航天器在极坐标中的极角和半径。
    """
    x = point[0]
    y = point[1]
    theta_rad = np.arctan2(y, x)  # 极角 (弧度)
    r_in_rt   = np.sqrt(x*x + y*y)  # 极角 (度数)
    return theta_rad, r_in_rt



########################################################################################

def main():
    os.system('cls')

    # 连接 KRPC
    conn = krpc.connect(name="Custom Coordinate System (Centered at Planet Core)")
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body

    print('与航天器成功建立通信链路')

    #############################################################################
    # 第一次落点预报+轨道机动点、需要减小的径向速度的确定

    # 获取航天器的轨道高度数据
    position = vessel.position(body.reference_frame)
    radial_distance = np.linalg.norm(position)
    altitude = radial_distance - body.equatorial_radius

    # 计算轨道机动点和需要的径向速度
    initial_orbit_altitude = altitude  # 初始圆轨道高度 (m)
    target_theta = np.radians(0)  # 目标着陆点的相角 (0°)
    maneuver_theta, delta_v, theoretical_trajectory = trajectory_optimization(initial_orbit_altitude, target_theta)

    print("#################第一次落点预报#################")
    print(f"轨道机动点的极角: {np.degrees(maneuver_theta):.2f}°")
    print(f"需要减少的径向速度: {delta_v:.2f} m/s")

    # 设置轨道机动节点
    mean_motion = calculate_mean_motion(vessel)
    spacecraft_position = np.array(vessel.position(body.reference_frame))
    spacecraft_position_new = change_xyz(spacecraft_position)
    vessel_angle = xyz_2_rt(spacecraft_position_new)[0]
    omega = 2 * np.pi / 21600  # 星球自转角速度
    delt_omiga = mean_motion - omega

    if vessel_angle > maneuver_theta and vessel_angle < np.pi:
        delt_angle = vessel_angle - maneuver_theta
        wait_time = (2 * np.pi - delt_angle) / delt_omiga
    else:
        delt_angle = maneuver_theta - vessel_angle
        wait_time = delt_angle / delt_omiga

    ut = conn.space_center.ut  # 当前时间
    maneuver_time = ut + wait_time
    node = vessel.control.add_node(ut=maneuver_time, prograde=delta_v, normal=0, radial=0)
    print('轨道机动计划设置完成')

    ## 调整航天器朝向
    print("调整航天器朝向...")

    # 读取飞船的默认参考系
    current_reference_frame = vessel.auto_pilot.reference_frame
    # 切换飞船的参考系
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)
    vessel.auto_pilot.engage()
    time.sleep(5)  # 稳定调整方向

    # 开启SAS并开启时间加速
    vessel.auto_pilot.disengage()
    vessel.control.sas = True
    conn.space_center.warp_to(maneuver_time - 8)  # 时间加速至机动节点

    # 再次调整朝向
    print("再次调整航天器朝向...")
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_direction = (0, 1, 0)
    vessel.auto_pilot.wait()

    time.sleep(1)

    # 执行机动并开启引擎
    print("执行机动...")

    # 开始点火
    vessel.control.throttle = 1.0  # 初始油门设置为最大
    while True:
        remaining_delta_v = node.remaining_delta_v

        vessel.auto_pilot.engage()
        vessel.auto_pilot.target_direction = (0, 1, 0)


        if remaining_delta_v < 0.5:  # 如果剩余 Δv 小于 0.5 m/s，则停止点火
            print("机动任务完成，停止引擎")
            vessel.control.throttle = 0.0  # 停止引擎
            break

    node.remove()  # 删除当前节点
    vessel.auto_pilot.disengage()
    # 恢复飞船的默认参考系
    vessel.auto_pilot.reference_frame = current_reference_frame
    #############################################################################
    


    print("#################等待进入大气#################")

    vessel.auto_pilot.engage()

    while True:
        position = vessel.position(body.reference_frame)

        vessel.auto_pilot.target_pitch_and_heading(0, 0)

        if np.linalg.norm(position) < 75_000 + 600_000:
            print("准备大气刹车")
            break
        time.sleep(1)
    print("#################推返分离#################")
    vessel.control.activate_next_stage()  # 执行一级分离
    vessel.control.activate_next_stage()  # 执行一级分离
    time.sleep(2)
    # 设定初始攻角
    inertial_gong_angle = 28
    vessel.auto_pilot.target_pitch_and_heading(180-inertial_gong_angle, 90)
    #############################################################################
    ## 大气内攻角控制开始
    while True:
        position = vessel.position(body.reference_frame)

        if np.linalg.norm(position) < 65_000 + 600_000:
            print("准备大气刹车")
            break
        time.sleep(1)

    print("#################大气内攻角控制开始#################")

    vessel.auto_pilot.engage()  # 启动自动驾驶
    target_location = (-0.0972, -74.5577)  # KSC 纬度, 经度
    trajectory_data = predictor_corrector_control(conn, vessel, body, target_location)

    # 自由落体段
    vessel.control.roll = 0
    vessel.control.sas = True
    vessel.control.rcs = False

    print("#################大气内攻角控制结束#################")
    #############################################################################
    # 第三次落点预报
    time_time = 0
    while True:
        position = vessel.position(body.reference_frame)
        
        # 如果高度超过限制，则跳出程序
        if np.linalg.norm(position) < 5_000 + 600_000 and time_time == 0:
            print("开启降落伞")
            print("着陆制导结束")

            vessel.control.activate_next_stage()  # 执行一级分离
            vessel.control.activate_next_stage()  # 执行一级分离
            time_time = time_time + 1


        if vessel.flight().surface_altitude < 100:
            lat = vessel.flight().latitude
            lon = vessel.flight().longitude
        
            S_remaining = calculate_distance(body, lat, lon, 0.0972, -74.5577)

            print(f"落点经度：{lat}°  落点纬度：{lon}° |距离目标点还有 {S_remaining/1000:.1f}km")
            break


   

if __name__ == "__main__":
    main()
