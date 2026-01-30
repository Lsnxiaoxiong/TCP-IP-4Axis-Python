import cv2
import numpy as np
import json
import glob
import os

# ================= 配置 =================
# 随便选一个你常用的 Aruco 字典，要跟你的图片一致
# 根据你之前的代码，你是 4x4_50 或者 AprilTag 36h11，这里用通用的检测逻辑
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
PARAMETERS = cv2.aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, PARAMETERS)

# 图像中心点 (根据你的内参 cx, cy 填入，或者读图获取)
# 如果你是 1280x720 的分辨率：
IMG_CX = 640
IMG_CY = 360


def get_marker_center(image_path):
    """读取图片并返回第一个Tag的中心像素坐标 (u, v)"""
    img = cv2.imread(image_path)
    if img is None: return None

    corners, ids, rejected = DETECTOR.detectMarkers(img)

    if ids is not None and len(ids) > 0:
        # 取第一个检测到的 Tag
        c = corners[0][0]  # shape: (4, 2)
        # 计算中心点
        center_x = np.mean(c[:, 0])
        center_y = np.mean(c[:, 1])
        return np.array([center_x, center_y])
    return None


def calibrate_2d_linear():
    # 1. 读取数据
    with open("poses.json", 'r') as f:
        poses_map = json.load(f)

    img_names = sorted(poses_map.keys())

    # 存储有效的配对数据
    robot_xy_list = []  # 机械臂物理坐标 [[x1, y1], [x2, y2], ...]
    pixel_uv_list = []  # 图片像素坐标 [[u1, v1], [u2, v2], ...]

    print(f"开始读取 {len(img_names)} 张图片...")

    for name in img_names:
        full_path = os.path.join("images", name)
        if not os.path.exists(full_path): continue

        # 获取像素坐标
        uv = get_marker_center(full_path)
        if uv is None:
            print(f"  [跳过] {name} 未检测到 Tag")
            continue

        # 获取机械臂坐标
        p = poses_map[name]
        # 注意：这里我们只关心 x, y (单位 mm)
        robot_xy = np.array([p['x'], p['y']])

        robot_xy_list.append(robot_xy)
        pixel_uv_list.append(uv)

    n = len(robot_xy_list)
    print(f"有效数据点: {n} 个")

    if n < 3:
        print("数据太少，无法计算。")
        return

    robot_xy_arr = np.array(robot_xy_list)
    pixel_uv_arr = np.array(pixel_uv_list)

    # 2. 计算“变化量” (Difference)
    # 因为我们不知道 Tag 在世界坐标系的绝对位置，
    # 但我们知道：机械臂动了多少 -> 像素动了多少。
    # 我们用“第 i 点”减去“第 0 点”来构建相对运动方程。

    d_robot = robot_xy_arr[1:] - robot_xy_arr[0]  # shape (N-1, 2)
    d_pixel = pixel_uv_arr[1:] - pixel_uv_arr[0]  # shape (N-1, 2)

    # 3. 求解线性方程 (最小二乘法)
    # 目标：d_robot = d_pixel @ Matrix
    # Matrix shape 应该是 (2, 2)
    # A * M = B  =>  M = lstsq(A, B)

    # lstsq 返回的是 (solution, residuals, rank, s)
    M, residuals, rank, s = np.linalg.lstsq(d_pixel, d_robot, rcond=None)

    # M 的形状是 (2, 2)。
    # M[0, 0] 是 u 对 x 的影响
    # M[1, 0] 是 v 对 x 的影响 ...

    print("\n====== 标定结果 ======")
    print("变换矩阵 M (2x2):")
    print(M)

    # 4. 验证与解释
    # M[0,0] 和 M[1,1] 近似代表 mm/pixel 的比例
    scale_x = np.sqrt(M[0, 0] ** 2 + M[1, 0] ** 2)
    scale_y = np.sqrt(M[0, 1] ** 2 + M[1, 1] ** 2)
    print(f"\n估计比例:")
    print(f"X方向: 1 像素 ≈ {scale_x:.4f} mm")
    print(f"Y方向: 1 像素 ≈ {scale_y:.4f} mm")

    # 5. 保存结果
    result = {
        "k11": M[0, 0], "k12": M[0, 1],  # 对应 X = k11*u + k12*v
        "k21": M[1, 0], "k22": M[1, 1],  # 对应 Y = k21*u + k22*v
        "description": "d_robot = d_pixel * M. usage: target_robot = current_robot + (target_pixel - center_pixel) @ M"
    }

    with open("calibration_2d.json", 'w') as f:
        json.dump(result, f, indent=4)

    print("\n参数已保存至 calibration_2d.json")

    # 6. 回测验证
    print("\n[回测误差验证]")
    total_err = 0
    for i in range(1, n):
        real_d = d_robot[i - 1]
        pred_d = np.dot(d_pixel[i - 1], M)
        err = np.linalg.norm(real_d - pred_d)
        total_err += err

    print(f"平均定位预测误差: {total_err / (n - 1):.3f} mm")


if __name__ == '__main__':
    calibrate_2d_linear()