import time
import numpy as np
import math
import krpc


import threading
import csv
import os

# 定义 CSV 文件名
CSV_FILENAME = "./src/data_log.csv"

def save_data_async(pos):
    """异步存储数据到 CSV 文件"""
    def write_to_csv():
        with open(CSV_FILENAME, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), pos[0], pos[1],pos[2]])

    thread = threading.Thread(target=write_to_csv)
    thread.daemon = True  # 让线程在主程序退出时自动结束
    thread.start()

def predict_landing_point(vessel, body, dt=1.0, max_time=3000):
    """
    使用数值积分方法预测飞船的理论落点。
    """
    lat = vessel.flight().latitude
    lon = vessel.flight().longitude
    alt = vessel.flight().surface_altitude
    V = vessel.flight().true_air_speed

    for _ in range(int(max_time / dt)):
        # 计算航迹角（Flight Path Angle）
        vertical_speed = vessel.flight().vertical_speed
        gamma = math.asin(vertical_speed / V) if abs(V) > 0 else 0  # 计算航迹角

        # 计算新的位置
        dS = V * math.cos(gamma) * dt  # 计算沿航线的位移
        lat += (dS / body.equatorial_radius) * (180 / math.pi)  # 更新纬度
        alt -= V * math.sin(gamma) * dt  # 更新高度
        
        # 速度衰减（简化假设，不考虑大气阻力）
        V *= 0.995

        # 低速或低高度时停止预测
        if alt < 10_000 or V < 100:
            break

    return lat, lon  # 返回预测的着陆点


# 计算两点之间的距离
def calculate_distance(body, lat1, lon1, target_lat, target_lon):
    """计算剩余航程（使用欧几里得距离公式）"""
    target = body.surface_position(target_lat, target_lon, body.reference_frame)
    current = body.surface_position(lat1, lon1, body.reference_frame)
    
    # 计算欧几里得距离
    distance = math.sqrt((target[0] - current[0])**2 + 
                         (target[1] - current[1])**2 + 
                         (target[2] - current[2])**2)


    return distance

# 得到落点误差
def calculate_landing_error(vessel, body, target_lat, target_lon):
    """
    计算飞船的理论落点与目标落点之间的偏差。
    """
    predicted_lat, predicted_lon = predict_landing_point(vessel, body)
    return calculate_distance(body, predicted_lat, predicted_lon, target_lat, target_lon)


def calculate_crossrange(conn, vessel, body, lat, lon, target_lat, target_lon):
    """计算横向偏差(带方向)"""
    # bearing = vessel.flight().bearing
    heading = vessel.flight().heading  # 修正错误，使用heading代替bearing
    target_bearing = conn.space_center.transform_direction(
        (target_lon - lon, target_lat - lat, 0),
        body.reference_frame,
        vessel.surface_reference_frame
    )
    return math.radians(target_bearing[1]) * body.equatorial_radius

######################### pid 控制器 #########################
    
def set_roll_target(vessel, target_roll, kp=0.05, ki=0.01, kd=0.00, dt=0.1):
    """非阻塞 PID Pitch 控制"""
    current_roll = vessel.flight().roll
    error = target_roll - current_roll
    derivative = (error - set_roll_target.previous_error) / dt
    set_roll_target.integral += error * dt
    set_roll_target.previous_error = error
    
    control_signal = kp * error + ki * set_roll_target.integral + kd * derivative
    vessel.control.roll = max(min(control_signal, 1), -1)


# 初始化 PID 变量


set_roll_target.previous_error = 0
set_roll_target.integral = 0

################################# 角度计算类 
def compute_angles(alpha, sigma):
    """
    计算向量 A 与 X、Y、Z 轴的夹角。
    :param alpha: A 与 Y 轴的夹角（弧度制）
    :param sigma: B 与 Z 轴的夹角（弧度制）
    :return: (theta_X, theta_Y, theta_Z) 夹角（弧度制）
    """
    cos_theta_X = np.sin(alpha) * np.sin(sigma)
    cos_theta_Y = np.cos(alpha)
    cos_theta_Z = np.sin(alpha) * np.cos(sigma)
    
    theta_X = np.arccos(cos_theta_X)
    theta_Y = np.arccos(cos_theta_Y)
    theta_Z = np.arccos(cos_theta_Z)
    
    return theta_X, theta_Y, theta_Z


def calculate_angles(alpha_deg, sigma_deg):
    # 将角度转换为弧度
    alpha = math.radians(alpha_deg)
    sigma = math.radians(sigma_deg)
    
    # 计算夹角
    theta_xoy = math.atan(math.tan(alpha) * math.sin(sigma))
    theta_yoz = math.atan(math.tan(alpha) * math.cos(sigma))
    
    # 转换回角度
    theta_xoy_deg = math.degrees(theta_xoy)
    theta_yoz_deg = math.degrees(theta_yoz)
    
    return theta_xoy_deg, theta_yoz_deg



def predictor_corrector_control(conn, vessel, body, target_location, time_step=0.1, max_iterations=1000):
    """
    预测校正法计算飞船再入时的侧倾角变化，使其返回坎巴拉太空中心
    """
    flight_info = vessel.flight()
    trajectory = []
    last_time = time.time()
    Z_prev = 0
    sigma = 0  # 初始侧倾角
    target_lat, target_lon = target_location
    
    Z_last = 1

    while vessel.flight().surface_altitude > 10000:
        current_g = vessel.flight().g_force
        lat = vessel.flight().latitude
        lon = vessel.flight().longitude
        V = vessel.flight().true_air_speed
        V_e = vessel.orbit.speed

        ## 飞船升力 阻力 
        LL = vessel.flight().lift
        DD = vessel.flight().drag

        
        
        S_remaining = calculate_distance(body, lat, lon, target_lat, target_lon)
        # S_remaining = calculate_landing_error(vessel, body, target_lat, target_lon)

        Z = calculate_crossrange(conn, vessel, body, lat, lon, target_lat, target_lon)
        
        dt = time.time() - last_time
        Z_dot = (Z - Z_prev) / dt if dt > 0 else 0
        Z_prev = Z
        last_time = time.time()
        
        Z_boundary = 700 + 400 * (V / V_e)
        K5 = 0.25 + 0.15 * (V / V_e)
        
        if abs(Z + K5 * Z_dot) > Z_boundary:
            sigma_sign = -1 if (Z + K5 * Z_dot) > 0 else 1
        else:
            sigma_sign = 1 if sigma >= 0 else -1


        # 变号换方向
        if Z * Z_last < 0:
            sigma_sign = -sigma_sign

        Z_last = Z
        
        sigma_size = 25 * (1-math.exp(- S_remaining / 40_000))
        sigma = sigma_size * sigma_sign
        
        if current_g > 5.0:
            sigma *= 0.7


        # 限制侧倾角的范围
        sigma = max(min(sigma, 20), -20)

        if S_remaining > 200_000:
            sigma = 0

        # target_pitch = 30
        # target_roll  = 0
        # target_yaw   = 270

        # set_roll_target(vessel, target_roll)  # 设定 roll


        if S_remaining > 50_000:
            target_pitch = 27
        else:
            target_pitch = 23


        op_sigma,op_pitch = calculate_angles(target_pitch,sigma)


        # vessel.auto_pilot.target_pitch = target_pitch  # 设定目标偏航角
        # vessel.auto_pilot.target_heading = 270 - 0.5*sigma  # 设定目标偏航角


        vessel.auto_pilot.target_pitch = op_pitch  # 设定目标偏航角
        vessel.auto_pilot.target_heading = 270 - op_sigma  # 设定目标偏航角
        
        

        # vessel.control.sas = True
        vessel.control.rcs = True
        
        trajectory.append((lat, lon, sigma))
        print(f"[IN_AIR] P: {target_pitch:.1f}° | R: {sigma:.1f}° | S_Dist: {S_remaining/1000:.1f}km | Z_Dist: {Z/1000:.1f}km |G: {current_g:.1f}g")
        
        time.sleep(1 / 10)

        ##### 数据存储
        position = vessel.position(body.reference_frame)
        save_data_async(position)

        if S_remaining < 10_000 or vessel.flight().surface_altitude < 20_000:
            vessel.auto_pilot.disengage()  # 关闭自动驾驶
            break
    
    return trajectory


def main():
    # 连接 KRPC
    conn = krpc.connect(name="Custom Coordinate System (Centered at Planet Core)")
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body
    # 设定目标地点 (坎巴拉太空中心)

    vessel.auto_pilot.engage()  # 启动自动驾驶
    target_location = (-0.0972, -74.5577)  # KSC 纬度, 经度
    trajectory_data = predictor_corrector_control(conn, vessel, body, target_location)

    print("制导完成，进入降落伞阶段")

    while True:
        if vessel.flight().surface_altitude < 10_000:
            vessel.control.activate_next_stage()
        time.sleep(1)  # 等待降落伞部署
        if vessel.flight().surface_altitude < 100:
            lat = vessel.flight().latitude
            lon = vessel.flight().longitude
            
            S_remaining = calculate_distance(body, lat, lon, 0.0972, -74.5577)

            print(f"落点经度：{lat}°  落点纬度：{lon}° |距离目标点还有 {S_remaining/1000:.1f}km")
            break


if __name__ == "__main__":

    main()