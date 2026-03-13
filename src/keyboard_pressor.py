import time
from enum import Enum

import cv2

from src.config import Config
from src.realsense435i import RealSense435i
from src.dobot_mg400 import DobotMG400
from src.yolov8_onnx import Yolov8Engine
import threading



class KeyboardPressor:
    """
    键盘按压控制器

    集成 RealSense 相机、YOLOv8 目标检测和 Dobot 机械臂，
    实现视觉引导的自动按键功能。

    支持两种按压模式：
    1. 基于目标检测的自动识别按键按压
    2. 基于鼠标点击的指定位置按压

    Attributes:
        config: 相机校准配置
        robot: Dobot MG400 机械臂控制器
        cap: RealSense 相机
        engine: YOLOv8 推理引擎
        robot_pix_x: 机器人当前像素 X 坐标
        robot_pix_y: 机器人当前像素 Y 坐标
        scale: 比例尺
        press_deep: 按压深度
    """

    def __init__(self, model_path: str):
        """
        初始化键盘按压控制器

        Args:
            model_path: YOLOv8 ONNX 模型文件路径
        """
        self.click_point = (640, 360)
        self.config = Config()
        self.robot = DobotMG400(init_pose=self.config.init_pos,
                                max_deep=self.config.max_deep)
        self.robot_pix_x, self.robot_pix_y = self.config.init_pix[0], self.config.init_pix[1]

        self.scale = self.config.scale
        self.press_deep = self.config.press_deep

        self.to_init_pose(self.config.init_pos[0], self.config.init_pos[1])
        # _inference_thread.start()

    def to_init_pose(self,x ,y ):
        """
        移动到初始姿态

        重置机器人到初始位置，并重置相机的目标点。
        """

        self.robot.run_point([x,y,self.config.init_pos[2],0])
        self.robot.to_init_pose()


    def press_pix_point(self, point: tuple):
        """
        按压指定像素位置

        根据像素坐标计算深度和位移，控制机械臂执行按压动作。

        Args:
            point: 目标像素坐标 (x, y)
        """
        pix_x, pix_y = point[0], point[1]
        depth = point[2]
        robot_z = self.config.init_pos[2] -(depth - self.config.cap_to_robot_end) - self.config.press_deep
        robot_x, robot_y = self.config.pixel_to_world(pix_x, pix_y)

        self.robot.run_point([robot_x, robot_y, self.config.init_pos[2],0])
        self.robot.run_point([robot_x, robot_y, robot_z,0])
        time.sleep(1)
        self.to_init_pose(robot_x, robot_y)


