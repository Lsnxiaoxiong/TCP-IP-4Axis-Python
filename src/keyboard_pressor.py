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
    Keyboard key enumeration class.

    Defines recognizable keyboard keys and their corresponding index values
    for YOLOv8 object detection.

    Members:
        NUM_ZERO: Number key 0 (YOLO class index 0)
        NUM_ONE: Number key 1 (YOLO class index 1)
        NUM_TWO: Number key 2 (YOLO class index 2)
    """
    NUM_ZERO = 0
    NUM_ONE = 1
    NUM_TWO = 2


class KeyboardPressor:
    """
    Keyboard pressing controller for vision-guided robotic operations.

    Integrates RealSense camera, YOLOv8 object detection, and Dobot robotic arm
    to achieve automatic key pressing based on visual guidance.

    Supports two pressing modes:
    1. Automatic key detection and pressing based on YOLOv8 detection
    2. Manual position pressing based on mouse click coordinates

    Attributes:
        config: Camera calibration configuration
        robot: Dobot MG400 robotic arm controller
        cap: RealSense camera
        engine: YOLOv8 inference engine
        robot_pix_x: Current robot pixel X coordinate
        robot_pix_y: Current robot pixel Y coordinate
        scale: Scale ratio for pixel-to-pose conversion
        press_deep: Key press depth in millimeters
    """

    def __init__(self, model_path: str):
        """
        Initialize the keyboard pressing controller.

        Args:
            model_path: Path to the YOLOv8 ONNX model file
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
        Move to initial pose.

        Resets the robot to the initial position and resets the camera target point.
        """
        self.robot_pix_x, self.robot_pix_y = self.config.init_pix[0], self.config.init_pix[1]
        self.robot.to_init_pose()
        self.cap.init_tar()

    def pix2pose(self, point: tuple):
        """
        Convert pixel coordinates to robotic arm displacement.

        Transforms pixel coordinate differences to robotic arm X/Y direction displacement.
        Note: Pixel X corresponds to robot Y, and pixel Y corresponds to robot X (axes swapped).

        Args:
            point: Target pixel coordinates (x, y)

        Returns:
            tuple: (delta_pose_x, delta_pose_y) - Robot displacement in millimeters
        """
        delta_pix_x = point[0] - self.robot_pix_x
        delta_pix_y = point[1] - self.robot_pix_y

        delta_pose_x = delta_pix_y / self.config.scale_x
        delta_pose_y = delta_pix_x / self.config.scale_y
        print('delta_pix_x', delta_pose_x, 'delta_pose_y', delta_pose_y)
        return delta_pose_x, delta_pose_y

    def press_pix_point(self, point: tuple):
        """
        Press at specified pixel position.

        Calculates depth and displacement based on pixel coordinates,
        then controls the robotic arm to execute the pressing action.

        Args:
            point: Target pixel coordinates (x, y)
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

    def press_pix_point_with_depth(self, point: tuple):
        """
        Press at specified pixel position with known depth.

        Calculates displacement based on pixel coordinates and given depth,
        then controls the robotic arm to execute the pressing action.

        Args:
            point: Target pixel coordinates and depth (x, y, depth)
        """
        pix_x, pix_y, depth = point[0], point[1], point[2]
        delta_z = -(depth - self.config.cap_to_robot_end) - self.config.press_deep
        delta_x, delta_y = self.pix2pose((pix_x, pix_y))

        self.robot_pix_x, self.robot_pix_y = pix_x, pix_y
        print(delta_z)
        self.robot.to_delta_pose(delta_x, delta_y, delta_z)
        time.sleep(1)
        self.to_init_pose()

    def press_num(self, num: Keyboard):
        """
        Press specified number key.

        Based on YOLOv8 detection results, finds the position of the corresponding
        number key and executes the pressing action.

        Args:
            num: Keyboard enum value (NUM_ZERO, NUM_ONE, NUM_TWO)

        Returns:
            None: Returns early if the target key is not detected
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
        Mouse event callback function.

        Captures mouse click position for manually specifying press points.

        Args:
            event: OpenCV mouse event type
            x: Click position X coordinate
            y: Click position Y coordinate
            flags: Mouse event flags
            param: Additional parameters
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_point = (x, y)
            print(f"鼠标点击位置 - 像素坐标：({x}, {y})")

    def _inference_loop(self):
        """
        YOLO inference loop (runs in background thread).

        Continuously captures camera frames, executes object detection,
        and draws detection results and calibration region markers on the image.
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
