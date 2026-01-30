import cv2
import numpy as np
import json
import math
import glob
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares

# ==========================================
# 1. 修正配置
# ==========================================
# 务必重新核对这两个数值！建议用游标卡尺量一下 5个格子的总长然后除以5
REAL_MARKER_LENGTH = 0.0285
REAL_MARKER_SEP = 0.0115

# 【关键步骤】请用尺子量一下：
# 相机镜头玻璃表面，相对于机械臂法兰平面的垂直距离是多少？
# 如果相机比法兰低（更靠近地面），通常是正数。
# 比如：相机大概在法兰下方 4cm 处 -> 填 0.04
FIXED_Z_OFFSET = 0.01  # <--- 请修改这里 (单位: 米)

# ==========================================
# 2. 初始化
# ==========================================
# 你的相机内参
K = np.array([[916.4883, 0, 659.0355], [0, 916.7917, 380.5868], [0, 0, 1]], dtype=np.float64)
dist = np.zeros(5)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
# 5x5
board = cv2.aruco.GridBoard((5, 5), REAL_MARKER_LENGTH, REAL_MARKER_SEP, aruco_dict)

# ==========================================
# 3. 读取数据
# ==========================================
with open("poses.json", "r") as f:
    pose_data = json.load(f)

img_paths = glob.glob("images/*.jpg")
raw_data = []

print(f"正在读取 {len(img_paths)} 张图片...")
for path in img_paths:
    filename = path.split("\\")[-1]
    if filename not in pose_data: continue

    img = cv2.imread(path)
    corners, ids, _ = detector.detectMarkers(img)
    if ids is None or len(ids) < 4: continue

    objPoints, imgPoints = board.matchImagePoints(corners, ids)
    # 增加鲁棒性：只用检测点数比较多的图
    if objPoints is None or len(objPoints) < 8: continue

    ret, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, K, dist)
    if ret:
        p = pose_data[filename]
        raw_data.append({
            "name": filename,
            "rx": p["x"] / 1000.0, "ry": p["y"] / 1000.0, "rz": p["z"] / 1000.0, "rr": p["r"],
            "cam_rvec": rvec, "cam_tvec": tvec
        })

print(f"有效用于计算的数据: {len(raw_data)} 组")

# ==========================================
# 4. 降维优化器 (只算 X, Y, Yaw，固定 Z)
# ==========================================
def solve_opt_2d(init_yaw_deg):
    def error_func(params):
        x, y, yaw = params # <--- 只有3个参数了，Z用固定的
        z = FIXED_Z_OFFSET

        # 构建外参 T_flange_cam
        T_fc = np.eye(4)
        T_fc[:3, 3] = [x, y, z]

        base_rot = R.from_euler('z', yaw, degrees=True).as_matrix()
        flip_x_180 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        T_fc[:3, :3] = base_rot @ flip_x_180

        world_positions = []
        for d in raw_data:
            T_base_flange = np.eye(4)
            T_base_flange[:3, 3] = [d["rx"], d["ry"], d["rz"]]
            T_base_flange[:3, :3] = R.from_euler('z', math.radians(d["rr"])).as_matrix()

            T_cam_board = np.eye(4)
            T_cam_board[:3, :3] = cv2.Rodrigues(d["cam_rvec"])[0]
            T_cam_board[:3, 3] = d["cam_tvec"].flatten()

            T_base_board = T_base_flange @ T_fc @ T_cam_board
            world_positions.append(T_base_board[:3, 3])

        world_positions = np.array(world_positions)
        # 计算所有点到中心的距离作为误差
        centroid = np.mean(world_positions, axis=0)
        return (world_positions - centroid).flatten()

    # 初始猜测: X=0, Y=0, Yaw=传入值
    res = least_squares(error_func, [0, 0, init_yaw_deg])

    final_errors = error_func(res.x).reshape(-1, 3)
    std_dev = np.std(final_errors, axis=0) * 1000
    score = np.mean(std_dev)

    return res.x, std_dev, score

# ==========================================
# 5. 运行
# ==========================================
print("\n=== 使用固定 Z 高度进行优化 ===")
print(f"固定 Z Offset: {FIXED_Z_OFFSET * 1000:.1f} mm")

# 我们已经知道角度大概是 -93度，直接在这个附近搜，提高精度
best_res, best_std, best_score = solve_opt_2d(-93.35)

print("\n" + "=" * 30)
print(f"🏆 优化结果 (误差 {best_score:.2f} mm)")
print(f"   标准差 (X, Y, Z): {best_std}")

opt_x, opt_y, opt_yaw = best_res

print(f"\n[最终参数]")
print(f"X  : {opt_x * 1000:.2f} mm")
print(f"Y  : {opt_y * 1000:.2f} mm")
print(f"Z  : {FIXED_Z_OFFSET * 1000:.2f} mm (手动固定)")
print(f"Yaw: {opt_yaw:.2f} 度")

# 保存
T_final = np.eye(4)
T_final[:3, 3] = [opt_x, opt_y, FIXED_Z_OFFSET]
flip_x_180 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
base_rot = R.from_euler('z', opt_yaw, degrees=True).as_matrix()
T_final[:3, :3] = base_rot @ flip_x_180

np.save("T_flange_cam.npy", T_final)
print("矩阵已保存。")