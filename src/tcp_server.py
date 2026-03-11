import json
import queue
import socket
import threading
import time


class TCPServer:
    """
    用于与labview程序建立通信
    """
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
        """处理客户端连接，TCPServer调用子函数"""
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