## 坐标变换函数库

import numpy as np

def change_xyz(point):
    # 已知旋转角度（以弧度为单位）
    theta = np.radians(-74.5753)  # 45度顺时针旋转
    """绕Y轴顺时针旋转点"""
    rotation_matrix = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

    rotated_position = np.dot(rotation_matrix, point)

    # 交换 Y 和 Z 轴
    transformed_position = np.array([rotated_position[0], rotated_position[2], rotated_position[1]])
    return transformed_position

def in_change_xyz(point):
    """
    逆变换过程：
    1. 交换 Y 和 Z 轴。
    2. 绕新的 Y 轴逆时针旋转固定角度 74.5753°。
    """
    # 交换 Y 和 Z 轴
    swapped_position = np.array([point[0], point[2], point[1]])

    # 已知旋转角度（以弧度为单位）
    theta = np.radians(74.5753)  # 74.5753° 逆时针旋转

    # 绕 Y 轴旋转矩阵
    rotation_matrix = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

    # 应用旋转
    transformed_position = np.dot(rotation_matrix, swapped_position)
    
    return transformed_position