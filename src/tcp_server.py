import json
import queue
import socket
import threading
import time


class TCPServer:
    """
    TCP 服务器类

    用于与 LabVIEW 程序建立通信，提供基于 TCP 的消息收发功能。
    支持多客户端连接（每个客户端一个线程），使用消息队列存储接收的数据。

    Attributes:
        host: 服务器主机地址
        port: 服务器端口号
        socket: 服务器 socket 对象
        msg_queue: 消息队列，存储接收到的 JSON 消息
        cur_socket: 当前连接的客户端 socket
    """
    def __init__(self, host='localhost', port=45678):
        """
        初始化 TCP 服务器

        Args:
            host: 服务器主机地址，默认 'localhost'
            port: 服务器端口号，默认 45678
        """
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 允许端口重用
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.msg_queue = queue.Queue()
        self._lock = threading.Lock()
        self.cur_socket = None

    def send_msg(self, msg: str):
        """
        发送消息到当前连接的客户端

        Args:
            msg: 要发送的字符串消息

        Returns:
            None: 如果没有客户端连接则直接返回
        """
        if self.cur_socket is None:
            return
        self.cur_socket.send(msg.encode('utf-8'))

    @property
    def msg(self):
        """
        获取接收到的消息

        阻塞等待直到有可用消息。

        Returns:
            dict: 从客户端接收并解析的 JSON 数据
        """
        while self.msg_queue.empty():
            time.sleep(1)
        return self.msg_queue.get()

    def handle_client(self, client_socket, address):
        """
        处理客户端连接（每个客户端一个线程）

        持续接收客户端消息，解析 JSON 格式并存入消息队列。

        Args:
            client_socket: 客户端 socket 对象
            address: 客户端地址
        """
        print(f"客户端 {address} 已连接")
        self.cur_socket = client_socket
        try:
            while True:
                # 接收数据
                data = client_socket.recv(1024)
                if not data:
                    break

                message = data.decode('utf-8')
                print(f"收到来自 {address} 的消息：{message}")
                data = json.loads(message)
                self.msg_queue.put(data)
                print(data)

                # response = f"服务器收到：{message}"
                # client_socket.send(response.encode('utf-8'))

        except Exception as e:
            print(f"处理客户端 {address} 时出错：{e}")
        finally:
            self.cur_socket = None
            client_socket.close()
            print(f"客户端 {address} 已断开连接")

    def start(self):
        """
        启动 TCP 服务器

        绑定地址和端口，开始监听客户端连接。
        为每个连接的客户端创建独立线程进行处理。
        """
        try:
            # 绑定地址和端口
            self.socket.bind((self.host, self.port))
            # 开始监听
            self.socket.listen(5)
            print(f"TCP 服务器启动，监听 {self.host}:{self.port}")

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
            print(f"服务器错误：{e}")
        finally:
            self.socket.close()
