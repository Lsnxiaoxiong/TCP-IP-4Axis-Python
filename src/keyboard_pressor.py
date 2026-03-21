import time
from src.config import Config
from src.dobot_mg400 import DobotMG400
import threading

from src.tcp_server import TCPServer


class KeyboardPressor:
    """
    键盘按压执行器

    结合 RealSense 相机和 Dobot MG400 机械臂，
    实现基于视觉的键盘按键按压功能。
    支持像素坐标到机器人世界坐标的转换，
    并执行精确的按键按压动作。

    Attributes:
        config: 配置类实例
        robot: Dobot MG400 机械臂实例
        press_deep: 按压深度 (mm)
    """


    def __init__(self):
        """
        初始化键盘按压执行器

        加载配置，初始化机械臂，并移动到初始位置。
        """
        self.config = Config()
        self.robot = DobotMG400(init_pose=self.config.init_pos)
        self.server = TCPServer('localhost', 45679)
        self._status_thread = threading.Thread(target=self.server.start)
        self._status_thread.start()
        self.press_deep = self.config.press_deep

        self.to_init_pose(self.config.init_pos[0], self.config.init_pos[1])


    def to_init_pose(self,x ,y ):
        """
        移动到初始姿态

        控制机械臂先移动到目标点的上方位置，然后返回到预设的初始位置。

        Args:
            x: 目标点 X 坐标（机器人坐标系）
            y: 目标点 Y 坐标（机器人坐标系）
        """

        self.robot.run_point([x,y,self.config.init_pos[2],0])
        self.robot.to_init_pose()


    def press_pix_point(self, point: tuple):
        """
        按压指定的像素点

        将像素坐标转换为机器人世界坐标，控制机械臂执行按压动作。
        按压流程：
        1. 移动到目标点上方
        2. 下降到按压深度
        3. 保持 1 秒
        4. 返回初始位置

        Args:
            point: 像素坐标和深度元组 (pix_x, pix_y, depth)
        """
        pix_x, pix_y = point[0], point[1]
        depth = point[2]
        # 参考1.2.1原理
        robot_z = self.config.init_pos[2] -(depth - self.config.cap_to_robot_end) - self.config.press_deep
        robot_x, robot_y = self.config.pixel_to_world(pix_x, pix_y)
        res = self.robot.run_point([robot_x, robot_y, self.config.init_pos[2],0])
        self.server.send_msg(res)
        res = self.robot.run_point([robot_x, robot_y, robot_z,0])
        self.server.send_msg(res)
        time.sleep(1)
        self.to_init_pose(robot_x, robot_y)

