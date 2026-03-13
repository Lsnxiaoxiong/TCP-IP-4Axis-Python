import json
import cv2
import numpy as np

class Config:
    """
    相机校准配置加载器

    从 cali.json 文件加载相机和机械臂的校准参数，
    提供像素坐标到机械臂姿态的转换比例计算。
    """
    def __init__(self):
        """
        初始化配置加载器

        从 cali.json 文件加载相机校准参数，包括:
        - 机械臂工作区域的左上角和右下角坐标
        - 相机视图中对应的像素坐标
        - 按键按压深度和最大深度
        - 初始位置坐标

        Raises:
            RuntimeError: 当 cali.json 文件加载失败时抛出
        """
        with open("cali.json", "r") as f:
            config = json.load(f)
        try:
            self.pix_points= config["pix_points"]
            self.robot_points = config["robot_points"]
            self.H, mask = cv2.findHomography(np.array(self.pix_points, dtype=np.float32),np.array(self.robot_points, dtype=np.float32))
            self.left_top = (config["left_top"]["x"], config["left_top"]["y"])
            self.right_bottom = (config["right_bottom"]["x"], config["right_bottom"]["y"])
            self.pix_left_top = (config["pix_left_top"]["x"], config["pix_left_top"]["y"])
            self.pix_right_bottom = (config["pix_right_bottom"]["x"], config["pix_right_bottom"]["y"])
            self.press_deep = config["press_deep"]
            self.max_deep = config["max_deep"]
            self.init_pix = (config["init_pos"]["pix_x"], config["init_pos"]["pix_y"])
            self.init_pos = (config["init_pos"]["pos_x"], config["init_pos"]["pos_y"], config["init_pos"]["pos_z"])

            self.robot_on_keyboard = config["robot_on_keyboard"]
            if self.robot_on_keyboard* self.init_pos[2] > 0:
                self.robot_to_keyboard = abs(self.robot_on_keyboard-self.init_pos[2])
            else:
                self.robot_to_keyboard = abs(self.robot_on_keyboard) + abs(self.init_pos[2])
            self.cap_to_robot_end = config["cap_to_keyborad"] - self.robot_to_keyboard
        except Exception as e:
            print(e)
            raise RuntimeError("加载 cali.json 配置文件失败")

    def pixel_to_world(self, u, v):
        """
        像素坐标到机械臂坐标的转换

        使用单应矩阵（Homography）将像素坐标转换为机械臂工作空间的世界坐标。
        该变换基于相机校准期间获取的对应点对计算得出。

        Args:
            u: 像素 X 坐标（列索引）
            v: 像素 Y 坐标（行索引）

        Returns:
            tuple: (x, y) 转换后的机械臂坐标（单位：毫米）
                - x: 机械臂 X 坐标
                - y: 机械臂 Y 坐标
        """
        p = np.array([u, v, 1])

        p2 = self.H @ p

        p2 = p2 / p2[2]

        x = p2[0]
        y = p2[1]

        return float(x), float(y)

    @property
    def scale(self):
        """
        计算平均比例尺

        基于像素坐标和机械臂坐标的差值，计算 X 和 Y 轴的平均比例尺。
        注意：X/Y 轴在计算中是互换的（像素 X 对应机械臂 Y，反之亦然）。

        Returns:
            float: 平均比例尺（像素/毫米）
        """
        y_scale = (self.pix_right_bottom[0] - self.pix_left_top[0]) / (self.right_bottom[1] - self.left_top[1])
        x_scale = (self.pix_right_bottom[1] - self.pix_left_top[1]) / (self.right_bottom[0] - self.left_top[0])
        return (y_scale + x_scale) / 2

    @property
    def scale_x(self):
        """
        计算 X 轴比例尺

        像素 Y 坐标差值与机械臂 X 坐标差值的比率。
        用于将像素 Y 方向的位移转换为机械臂 X 方向的位移。

        Returns:
            float: X 轴比例尺（像素/毫米）
        """
        return (self.pix_right_bottom[1] - self.pix_left_top[1]) / (self.right_bottom[0] - self.left_top[0])

    @property
    def scale_y(self):
        """
        计算 Y 轴比例尺

        像素 X 坐标差值与机械臂 Y 坐标差值的比率。
        用于将像素 X 方向的位移转换为机械臂 Y 方向的位移。

        Returns:
            float: Y 轴比例尺（像素/毫米）
        """
        return (self.pix_right_bottom[0] - self.pix_left_top[0]) / (self.right_bottom[1] - self.left_top[1])








if __name__ == "__main__":
    import cv2
    import numpy as np

    # 像素坐标
    pixel_pts = np.array([
        [275, 97],
        [352, 447],
        [1074, 397],
        [415, 457],
        [425, 314],
        [859, 357],
        [772, 249],
    ], dtype=np.float32)

    # 平面坐标 (mm)
    world_pts = np.array([
        [214.86, -190.15],
        [393.92, -147.13],
        [358.30, 230.80],
        [397.77, -114.32],
        [324.08, -109.50],
        [340.08, 119.33],
        [283.56, 71.63],
    ], dtype=np.float32)

    # 计算单应矩阵
    H, mask = cv2.findHomography(pixel_pts, world_pts)

    print("Homography matrix:")
    print(H)
    # print(pixel_to_world(H, 744, 283))

    conf = Config()
    print(conf.pixel_to_world(744, 283))