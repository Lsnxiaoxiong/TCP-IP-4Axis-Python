
import time
from enum import Enum

import cv2

from src.config import Config
from src.realsense435i import RealSense435i
from src.dobot_mg400 import DobotMG400
from src.yolov8_onnx import Yolov8Engine
import threading


class Keyboard(Enum):
    """
    键盘按键枚举类

    定义可识别的键盘按键及其对应的索引值。

    Members:
        NUM_ZERO: 数字键 0
        NUM_ONE: 数字键 1
        NUM_TWO: 数字键 2
    """
    NUM_ZERO = 0
    NUM_ONE = 1
    NUM_TWO = 2


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
        self.cap = RealSense435i(init_point=(self.config.init_pix[0], self.config.init_pix[1]))
        self.engine: Yolov8Engine = Yolov8Engine(model_path=model_path)
        self.robot_pix_x, self.robot_pix_y = self.config.init_pix[0], self.config.init_pix[1]

        self.scale = self.config.scale
        self.press_deep = self.config.press_deep

        self.to_init_pose()

        _inference_thread = threading.Thread(target=self._inference_loop)
        _inference_thread.daemon = True
        _inference_thread.start()

    def to_init_pose(self):
        """
        移动到初始姿态

        重置机器人到初始位置，并重置相机的目标点。
        """
        self.robot_pix_x, self.robot_pix_y = self.config.init_pix[0], self.config.init_pix[1]
        self.robot.to_init_pose()
        self.cap.init_tar()

    def pix2pose(self, point: tuple):
        """
        像素坐标转换为机械臂位移

        将像素坐标差值转换为机械臂的 X/Y 方向位移。
        注意：像素 X 对应机械臂 Y，像素 Y 对应机械臂 X（轴互换）。

        Args:
            point: 目标像素坐标 (x, y)

        Returns:
            tuple: (delta_pose_x, delta_pose_y) 机械臂位移量（mm）
        """
        delta_pix_x = point[0] - self.robot_pix_x
        delta_pix_y = point[1] - self.robot_pix_y

        delta_pose_x = delta_pix_y / self.config.scale_x
        delta_pose_y = delta_pix_x / self.config.scale_y
        print('delta_pix_x', delta_pose_x, 'delta_pose_y', delta_pose_y)
        return delta_pose_x, delta_pose_y

    def press_pix_point(self, point: tuple):
        """
        按压指定像素位置

        根据像素坐标计算深度和位移，控制机械臂执行按压动作。

        Args:
            point: 目标像素坐标 (x, y)
        """
        pix_x, pix_y = point[0], point[1]
        depth = self.cap.get_point_depth((pix_x, pix_y)) * 1000
        delta_z = -(depth - self.config.cap_to_robot_end) - self.config.press_deep
        delta_x, delta_y = self.pix2pose((pix_x, pix_y))

        self.robot_pix_x, self.robot_pix_y = pix_x, pix_y
        print(delta_z)
        self.robot.to_delta_pose(delta_x, delta_y, delta_z)
        time.sleep(1)
        self.to_init_pose()

    def press_num(self, num: Keyboard):
        """
        按压指定数字键

        基于 YOLOv8 检测结果，找到对应数字键的位置并执行按压。

        Args:
            num: Keyboard 枚举值（NUM_ZERO, NUM_ONE, NUM_TWO）

        Returns:
            None: 如果未检测到目标按键则直接返回
        """
        index = num.value
        detect_res = self.engine.latest_res
        if index not in detect_res:
            print(f"未检测到{num.name}")
            return
        xc, yc = detect_res[num.value]["xc"], detect_res[num.value]["yc"]
        pix_x, pix_y = xc * self.engine.orig_img_size[1], yc * self.engine.orig_img_size[0]
        depth = self.cap.get_point_depth((pix_x, pix_y)) * 1000
        if depth == 0:
            print("检测不到深度！")
            return
        delta_z = -(depth - self.config.cap_to_robot_end) - self.config.press_deep
        delta_x, delta_y = self.pix2pose((pix_x, pix_y))

        self.robot_pix_x, self.robot_pix_y = pix_x, pix_y
        print(delta_z)
        self.robot.to_delta_pose(delta_x, delta_y, delta_z)
        time.sleep(1)
        self.to_init_pose()

    def mouse_callback(self, event, x, y, flags, param):
        """
        鼠标事件回调函数

        捕获鼠标点击位置用于手动指定按压点。

        Args:
            event: OpenCV 鼠标事件类型
            x: 点击位置 X 坐标
            y: 点击位置 Y 坐标
            flags: 鼠标事件标志位
            param: 附加参数
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_point = (x, y)
            print(f"鼠标点击位置 - 像素坐标：({x}, {y})")

    def _inference_loop(self):
        """
        YOLO 推理循环（后台线程运行）

        持续捕获相机帧，执行目标检测，
        并在图像上绘制检测结果和校准区域标记。
        """
        while True:
            color_frame = self.cap.get_latest()[0]
            self.engine.inference(color_frame=color_frame)
            img = cv2.cvtColor(color_frame, cv2.COLOR_RGB2BGR)
            for index in self.engine.latest_res.keys():
                cv2.circle(img, (int(self.engine.orig_img_size[1] * self.engine.cur_detect_res[index]["xc"]),
                                 int(self.engine.orig_img_size[0] * self.engine.cur_detect_res[index]["yc"])),
                           5,
                           (0, 0, 255),
                           )
                cv2.line(img, (340, 200), (940, 200), (0, 255, 0), thickness=1)
                cv2.line(img, (340, 200), (340, 540), (0, 255, 0), thickness=1)
                cv2.line(img, (340, 540), (940, 540), (0, 255, 0), thickness=1)
                cv2.line(img, (940, 200), (940, 540), (0, 255, 0), thickness=1)
            cv2.imshow("frame", img)
            cv2.setMouseCallback("frame", self.mouse_callback)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



