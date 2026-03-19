import json
import cv2
import numpy as np

class Config:
    """
    配置类

    从 cali.json 加载相机校准参数和机械臂配置信息，
    提供像素坐标到世界坐标的转换功能。

    Attributes:
        pix_points: 相机像素坐标点列表
        robot_points: 机械臂世界坐标点列表 (mm)
        H: 单应性矩阵，用于像素到世界的坐标变换
        press_deep: 按压深度(mm)
        max_deep: 机械臂最大深度(mm)
        init_pos: 机械臂初始位置坐标 (x, y, z)
        robot_on_keyboard: 机械臂末端接触键盘时z坐标(mm)
        robot_to_keyboard: 机械臂末端到键盘的高度(mm)
        cap_to_robot_end: 相机到机械臂末端的高度(mm)
    """
    def __init__(self):
        """
        初始化配置类

        从 cali.json 文件加载校准参数和机械臂配置，
        计算单应性矩阵用于坐标变换。

        Raises:
            FileNotFoundError: cali.json 文件不存在
            json.JSONDecodeError: JSON 格式错误
            RuntimeError: 配置参数缺失或无效
        """
        with open("cali.json", "r") as f:
            config = json.load(f)
        try:
            self.pix_points= config["pix_points"]
            self.robot_points = config["robot_points"]
            self.H, mask = cv2.findHomography(np.array(self.pix_points, dtype=np.float32),np.array(self.robot_points, dtype=np.float32))

            self.press_deep = config["press_deep"]
            self.max_deep = config["max_deep"]
            self.init_pos = (config["init_pos"]["pos_x"], config["init_pos"]["pos_y"], config["init_pos"]["pos_z"])

            self.robot_on_keyboard = config["robot_on_keyboard"]
            if self.robot_on_keyboard*self.init_pos[2] > 0:
                self.robot_to_keyboard = abs(self.robot_on_keyboard-self.init_pos[2])
            else:
                self.robot_to_keyboard = abs(self.robot_on_keyboard) + abs(self.init_pos[2])
            self.cap_to_robot_end = config["cap_to_keyborad"] - self.robot_to_keyboard
        except Exception as e:
            print(e)
            raise RuntimeError("加载 cali.json 配置文件失败")

    def pixel_to_world(self, u, v):
        """
        将像素坐标转换为世界坐标

        使用单应性矩阵将图像像素坐标 (u, v) 转换为机械臂世界坐标 (x, y)。

        Args:
            u: 像素坐标 U
            v: 像素坐标 V

        Returns:
            tuple: (x, y) 世界坐标 (mm)
        """
        p = np.array([u, v, 1])
        p2 = self.H @ p
        p2 = p2 / p2[2]

        x = p2[0]
        y = p2[1]

        return float(x), float(y)



if __name__ == "__main__":

    conf = Config()
    print(conf.pixel_to_world(744, 283))