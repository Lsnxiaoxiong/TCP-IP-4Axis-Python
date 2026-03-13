import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re

current_actual = None
algorithm_queue = None
enableStatus_robot = None
robotErrorState = False
globalLockValue = threading.Lock()


class DobotMG400:
    """
    Dobot MG400 机械臂控制封装类

    提供与 Dobot MG400 机械臂的 TCP/IP 通信接口，
    支持运动控制、状态反馈和错误处理。

    Attributes:
        dashboard: Dashboard API 实例（控制命令）
        move: Move API 实例（运动命令）
        feed: Feedback API 实例（状态反馈）
        init_pose: 初始姿态坐标 (x, y, z)
        max_deep: 最大按压深度
        x: 当前 X 坐标
        y: 当前 Y 坐标
        z: 当前 Z 坐标
    """
    def __init__(self,
                 ip: str = "192.168.1.6",
                 dashboard_port: int = 29999,
                 mover_port: int = 30003,
                 feed_port: int = 30004,
                 init_pose: tuple = (219.99, -144.64,40),
                 max_deep: float = 40
                 ):
        """
        初始化 Dobot MG400 机械臂控制器

        Args:
            ip: 控制器 IP 地址，默认 192.168.1.6
            dashboard_port: Dashboard 端口，默认 29999
            mover_port: Move 端口，默认 30003
            feed_port: Feedback 端口，默认 30004
            init_pose: 初始姿态坐标 (x, y, z)，默认 (219.99, -144.64, 40)
            max_deep: 最大按压深度（mm），默认 40
        """

        self.ip = ip
        self.dashboard_port = dashboard_port
        self.mover_port = mover_port
        self.feed_port = feed_port
        self.init_pose = init_pose
        self.max_deep = max_deep

        self.dashboard, self.move, self.feed = self.connect_robot()
        print("开始使能...")
        self.dashboard.EnableRobot()
        print("完成使能:)")
        feed_thread = threading.Thread(target=self.get_feed, args=(self.feed,))
        feed_thread.daemon = True
        feed_thread.start()
        feed_thread1 = threading.Thread(target=self.clear_robot_error, args=(self.dashboard,))
        feed_thread1.daemon = True
        feed_thread1.start()

        self.x, self.y, self.z = self.init_pose[0], self.init_pose[1], self.init_pose[2]


    def to_init_pose(self):
        """
        移动到初始姿态

        先移动到安全高度（Z 为 init_pose[2]），再移动到初始 XY 位置，
        最后下降到初始 Z 坐标。
        """

        # self.run_point([self.x, self.y, self.init_pose[2], 0])
        # self.x, self.y, self.z = self.init_pose[0], self.init_pose[1], self.init_pose[2]
        self.run_point([self.init_pose[0], self.init_pose[1], self.init_pose[2], 0])


    def connect_robot(self):
        """
        连接到机器人控制器

        建立与 Dashboard、Move 和 Feedback 三个端口的 TCP 连接。

        Returns:
            tuple: (dashboard, move, feed) 三个 API 实例

        Raises:
            Exception: 连接失败时抛出
        """
        try:
            print("正在建立连接...")
            dashboard = DobotApiDashboard(self.ip, self.dashboard_port)
            move = DobotApiMove(self.ip, self.mover_port)
            feed = DobotApi(self.ip, self.feed_port)
            print(">.<连接成功>!<")
            return dashboard, move, feed
        except Exception as e:
            print(":(连接失败:(")
            raise e

    def run_point(self, point_list: list):
        """
        执行点到点运动

        使用直线插补（MovL）移动到目标点，并等待到达。

        Args:
            point_list: 目标点坐标列表 [x, y, z, r]
        """
        self.move.MovL(point_list[0], point_list[1], point_list[2], point_list[3])
        self.wait_arrive(point_list)

    def get_feed(self, feed: DobotApi):
        """
        反馈数据接收线程函数

        持续从机器人接收 1440 字节的状态反馈数据，
        更新全局状态变量（当前位置、使能状态、错误状态等）。

        Args:
            feed: DobotApi 反馈实例
        """
        global current_actual
        global algorithm_queue
        global enableStatus_robot
        global robotErrorState
        hasRead = 0
        while True:
            data = bytes()
            while hasRead < 1440:
                temp = feed.socket_dobot.recv(1440 - hasRead)
                if len(temp) > 0:
                    hasRead += len(temp)
                    data += temp
            hasRead = 0
            feedInfo = np.frombuffer(data, dtype=MyType)
            if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
                globalLockValue.acquire()
                # Refresh Properties
                current_actual = feedInfo["tool_vector_actual"][0]
                algorithm_queue = feedInfo['isRunQueuedCmd'][0]
                enableStatus_robot = feedInfo['EnableStatus'][0]
                robotErrorState = feedInfo['ErrorStatus'][0]
                globalLockValue.release()
            sleep(0.001)

    def wait_arrive(self, point_list):
        """
        等待机械臂到达目标点

        阻塞等待直到所有坐标轴与目标位置的偏差小于 1mm。

        Args:
            point_list: 目标点坐标列表 [x, y, z, r]
        """
        while True:
            is_arrive = True
            globalLockValue.acquire()
            if current_actual is not None:
                for index in range(4):
                    if (abs(current_actual[index] - point_list[index]) > 1):
                        is_arrive = False
                if is_arrive:
                    globalLockValue.release()
                    return
            globalLockValue.release()
            sleep(0.001)

    def clear_robot_error(self, dashboard: DobotApiDashboard):
        """
        错误处理线程函数

        监控机器人错误状态，当检测到错误时：
        1. 解析错误代码
        2. 打印错误描述（控制器/伺服/碰撞）
        3. 等待用户确认后清除错误并继续运行

        Args:
            dashboard: DobotApiDashboard 实例
        """
        global robotErrorState
        dataController, dataServo = alarmAlarmJsonFile()  # 读取控制器和伺服告警码
        while True:
            globalLockValue.acquire()
            if robotErrorState:
                numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
                numbers = [int(num) for num in numbers]
                if (numbers[0] == 0):
                    if (len(numbers) > 1):
                        for i in numbers[1:]:
                            alarmState = False
                            if i == -2:
                                print("机器告警 机器碰撞 ", i)
                                alarmState = True
                            if alarmState:
                                continue
                            for item in dataController:
                                if i == item["id"]:
                                    print("机器告警 Controller errorid", i, item["zh_CN"]["description"])
                                    alarmState = True
                                    break
                            if alarmState:
                                continue
                            for item in dataServo:
                                if i == item["id"]:
                                    print("机器告警 Servo errorid", i, item["zh_CN"]["description"])
                                    break

                        choose = input("输入 1, 将清除错误，机器继续运行：")
                        if int(choose) == 1:
                            dashboard.ClearError()
                            sleep(0.01)
                            dashboard.Continue()

            else:
                if int(enableStatus_robot[0]) == 1 and int(algorithm_queue[0]) == 0:
                    dashboard.Continue()
            globalLockValue.release()
            sleep(5)


if __name__ == '__main__':
    mg400 = DobotMG400()
    # mg400.run_point([171.41,-289.32,40,0]) #初始位置

    mg400.run_point([218.75, -157.5, 40, 0])
