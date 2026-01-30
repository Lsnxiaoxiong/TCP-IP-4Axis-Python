import json
import threading
import time

from camera.realsense_435i import RealSense435i
from dobot_mg400 import DobotMG400


class Config:
    def __init__(self):
        with open("cali.json", "r") as f:
            config = json.load(f)
        try:
            self.left_top = (config["left_top"]["x"], config["left_top"]["y"])
            self.right_bottom = (config["right_bottom"]["x"], config["right_bottom"]["y"])
            self.pix_left_top = (config["pix_left_top"]["x"], config["pix_left_top"]["y"])
            self.pix_right_bottom = (config["pix_right_bottom"]["x"], config["pix_right_bottom"]["y"])
            self.press_deep = config["press_deep"]
        except Exception as e:
            print(e)
            raise RuntimeError("加载cali.json配置文件失败")

    @property
    def scale(self):
        y_scale = (self.pix_right_bottom[0] - self.pix_left_top[0]) / (self.right_bottom[1] - self.left_top[1])
        x_scale = (self.pix_right_bottom[1] - self.pix_left_top[1]) / (self.right_bottom[0] - self.left_top[0])
        return (y_scale + x_scale) / 2


class KeyboardPressor:
    def __init__(self):
        self.config = Config()
        self.robot = DobotMG400(init_pose=(self.config.left_top[0], self.config.left_top[1]))
        self.cap = RealSense435i(init_point=(self.config.pix_left_top[0], self.config.pix_left_top[1]))

        self.robot_pix_x, self.robot_pix_y = self.config.pix_left_top[0], self.config.pix_left_top[1]

        self.scale = self.config.scale
        self.press_deep = self.config.press_deep

        self.to_init_pose()
        _cap_thread = threading.Thread(target=self.cap.start_pix_trace)
        _cap_thread.daemon = True
        _cap_thread.start()

    def to_init_pose(self):
        self.robot_pix_x, self.robot_pix_y = self.config.pix_left_top[0], self.config.pix_left_top[1]
        self.robot.to_init_pose()
        self.cap.init_tar()

    def pix2pose(self):
        delta_pix_x = self.cap.tar_x - self.robot_pix_x
        delta_pix_y = self.cap.tar_y - self.robot_pix_y

        delta_pose_x = delta_pix_y / self.scale
        delta_pose_y = delta_pix_x / self.scale
        print('delta_pix_x', delta_pose_x, 'delta_pose_y', delta_pose_y)
        return delta_pose_x, delta_pose_y

    def to_tar_pose(self):
        delta_pose_x, delta_pose_y = self.pix2pose()
        self.robot_pix_x, self.robot_pix_y = self.cap.tar_x, self.cap.tar_y
        self.robot.to_delta_pose(delta_pose_x, delta_pose_y, 0)

    def press_keyboard(self):
        delta_pose_x, delta_pose_y = self.pix2pose()
        self.robot_pix_x, self.robot_pix_y = self.cap.tar_x, self.cap.tar_y
        self.robot.to_delta_pose(delta_pose_x, delta_pose_y, -80)

if __name__ == '__main__':
    controller = KeyboardPressor()
    while True:
        time.sleep(3)
        cmd = input("输入指令，press:按下目标点")
        if cmd == "press":
            controller.press_keyboard()
        time.sleep(2)
        controller.to_init_pose()

