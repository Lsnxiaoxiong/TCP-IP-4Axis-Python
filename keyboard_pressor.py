import json
import queue
import time
from enum import Enum

import cv2

from camera.realsense_435i import RealSense435i
from robot.dobot_mg400 import DobotMG400
from yolo.yolov8_onnx import Yolov8Engine2


class Keyboard(Enum):
    NUM_ZERO = 0
    NUM_ONE = 1
    NUM_TWO = 2


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
            self.cap_to_end_dist = config["cap_to_keyborad"]-(40+abs(config["robot_on_keyboard"]))
            self.max_deep = config["max_deep"]
            self.init_pix = (config["init_pos"]["pix_x"], config["init_pos"]["pix_y"])
            self.init_pos = (config["init_pos"]["pos_x"], config["init_pos"]["pos_y"])
        except Exception as e:
            print(e)
            raise RuntimeError("加载cali.json配置文件失败")

    @property
    def scale(self):
        y_scale = (self.pix_right_bottom[0] - self.pix_left_top[0]) / (self.right_bottom[1] - self.left_top[1])
        x_scale = (self.pix_right_bottom[1] - self.pix_left_top[1]) / (self.right_bottom[0] - self.left_top[0])
        return (y_scale + x_scale) / 2

    @property
    def scale_x(self):
        """
        机械臂x轴
        """
        return (self.pix_right_bottom[1] - self.pix_left_top[1]) / (self.right_bottom[0] - self.left_top[0])

    @property
    def scale_y(self):
        """
        机械臂y轴
        """
        return (self.pix_right_bottom[0] - self.pix_left_top[0]) / (self.right_bottom[1] - self.left_top[1])


class KeyboardPressor:
    def __init__(self, model_path: str):
        self.click_point = (640, 360)
        self.config = Config()
        self.robot = DobotMG400(init_pose=(self.config.init_pos[0], self.config.init_pos[1]),
                                max_deep=self.config.max_deep)
        self.cap = RealSense435i(init_point=(self.config.init_pix[0], self.config.init_pix[1]))
        self.engine: Yolov8Engine2 = Yolov8Engine2(model_path=model_path)
        self.robot_pix_x, self.robot_pix_y = self.config.init_pix[0], self.config.init_pix[1]

        self.scale = self.config.scale
        self.press_deep = self.config.press_deep

        self.to_init_pose()

        _inference_thread = threading.Thread(target=self.inference)
        _inference_thread.daemon = True
        _inference_thread.start()

    def to_init_pose(self):
        self.robot_pix_x, self.robot_pix_y = self.config.init_pix[0], self.config.init_pix[1]
        self.robot.to_init_pose()
        self.cap.init_tar()

    def pix2pose(self, point: tuple):
        delta_pix_x = point[0] - self.robot_pix_x
        delta_pix_y = point[1] - self.robot_pix_y

        delta_pose_x = delta_pix_y / self.config.scale_x
        delta_pose_y = delta_pix_x / self.config.scale_y
        # delta_pose_x = delta_pix_y / self.scale
        # delta_pose_y = delta_pix_x / self.scale
        print('delta_pix_x', delta_pose_x, 'delta_pose_y', delta_pose_y)
        return delta_pose_x, delta_pose_y

    # def to_tar_pose(self):
    #     delta_pose_x, delta_pose_y = self.tar2pose()
    #     self.robot_pix_x, self.robot_pix_y = self.cap.tar_x, self.cap.tar_y
    #     self.robot.to_delta_pose(delta_pose_x, delta_pose_y, 0)
    #
    # def press_keyboard(self):
    #     delta_pose_x, delta_pose_y = self.tar2pose()
    #     self.robot_pix_x, self.robot_pix_y = self.cap.tar_x, self.cap.tar_y
    #     self.robot.to_delta_pose(delta_pose_x, delta_pose_y, -80)

    def press_pix_point(self, point: tuple):
        pix_x, pix_y = point[0], point[1]
        depth = self.cap.get_point_depth((pix_x, pix_y)) * 1000
        delta_z = -(depth - self.config.cap_to_end_dist) - self.config.press_deep
        delta_x, delta_y = self.pix2pose((pix_x, pix_y))

        self.robot_pix_x, self.robot_pix_y = pix_x, pix_y
        print(delta_z)
        self.robot.to_delta_pose(delta_x, delta_y, delta_z)
        time.sleep(1)
        self.to_init_pose()

    def press_num(self, num: Keyboard):
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
        delta_z = -(depth - self.config.cap_to_end_dist) - self.config.press_deep
        delta_x, delta_y = self.pix2pose((pix_x, pix_y))

        self.robot_pix_x, self.robot_pix_y = pix_x, pix_y
        print(delta_z)
        self.robot.to_delta_pose(delta_x, delta_y, delta_z)
        time.sleep(1)
        self.to_init_pose()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_point = (x, y)
            print(f"鼠标点击位置 - 像素坐标: ({x}, {y})")

    def inference(self):
        while True:
            color_frame = self.cap.get_latest()[0]
            self.engine.inference(color_frame=color_frame)
            # print(f"检测结果：{engine.cur_detect_res}")
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


import socket
import threading


class TCPServer:
    def __init__(self, host='localhost', port=45678):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 允许端口重用
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.msg_queue = queue.Queue()
        self._lock = threading.Lock()
        self.cur_socket = None

    def send_msg(self, msg: str):
        if self.cur_socket is None:
            return
        self.cur_socket.send(msg.encode('utf-8'))

    @property
    def msg(self):
        while self.msg_queue.empty():
            time.sleep(1)
        return self.msg_queue.get()

    def handle_client(self, client_socket, address):
        """处理客户端连接"""
        print(f"客户端 {address} 已连接")
        self.cur_socket = client_socket
        try:
            while True:
                # 接收数据
                data = client_socket.recv(1024)
                if not data:
                    break

                message = data.decode('utf-8')
                print(f"收到来自 {address} 的消息: {message}")
                data = json.loads(message)
                self.msg_queue.put(data)
                print(data)

                # response = f"服务器收到: {message}"
                # client_socket.send(response.encode('utf-8'))

        except Exception as e:
            print(f"处理客户端 {address} 时出错: {e}")
        finally:
            self.cur_socket = None
            client_socket.close()
            print(f"客户端 {address} 已断开连接")

    def start(self):
        """启动服务器"""
        try:
            # 绑定地址和端口
            self.socket.bind((self.host, self.port))
            # 开始监听
            self.socket.listen(5)
            print(f"TCP服务器启动，监听 {self.host}:{self.port}")

            while True:
                # 等待客户端连接
                client_socket, address = self.socket.accept()

                # 为每个客户端创建新线程
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, address)
                )
                client_thread.daemon = True
                client_thread.start()

        except KeyboardInterrupt:
            print("\n服务器正在关闭...")
        except Exception as e:
            print(f"服务器错误: {e}")
        finally:
            self.socket.close()


if __name__ == '__main__':

    # controller = KeyboardPressor(
    #     model_path=r"best.onnx")
    server = TCPServer('localhost', 45678)
    server_worker = threading.Thread(target=server.start)
    server_worker.daemon = True
    server_worker.start()

    while True:
        msg = server.msg
        try:
            if "cmd" in msg:
                """
                    {"cmd": 1}
                """
                cmd = msg["cmd"]
                # if cmd == 0:
                #     print("0")
                #     controller.press_num(Keyboard.NUM_ZERO)
                # elif cmd == 1:
                #     controller.press_num(Keyboard.NUM_ONE)
                # elif cmd == 2:
                #     controller.press_num(Keyboard.NUM_TWO)
                # elif cmd == "q":
                #     break
                print("cmd")
            if "point" in msg:
                """
                    {"point":[123,456]}
                """
                point = msg["point"]
                print("p")
                # controller.press_pix_point((point[0],point[1]))
            server.send_msg("1")
        except Exception as e:
            print(e)
