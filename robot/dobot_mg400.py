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
    def __init__(self,
                 ip: str = "192.168.1.6",
                 dashboard_port: int = 29999,
                 mover_port: int = 30003,
                 feed_port: int = 30004,
                 init_pose: tuple = (219.99, -144.64),
                 max_deep: float = 40
                 ):

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

        self.x, self.y, self.z = self.init_pose[0], self.init_pose[1], 40


    def to_init_pose(self):

        self.run_point([self.x, self.y, 40, 0])
        self.x, self.y, self.z = self.init_pose[0], self.init_pose[1], 40
        self.run_point([self.x, self.y, self.z, 0])

    def to_delta_pose(self, delta_x, delta_y,delta_z):
        self.x += delta_x
        self.y += delta_y
        self.z += delta_z
        if self.z < self.max_deep:
            self.z = self.max_deep

        self.run_point([self.x, self.y, 40, 0])
        self.run_point([self.x, self.y, self.z, 0])

    def connect_robot(self):
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
        self.move.MovL(point_list[0], point_list[1], point_list[2], point_list[3])
        self.wait_arrive(point_list)

    def get_feed(self, feed: DobotApi):
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

                        choose = input("输入1, 将清除错误, 机器继续运行: ")
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
