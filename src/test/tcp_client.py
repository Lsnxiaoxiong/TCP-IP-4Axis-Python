import socket


class TCPClient:
    """
    TCP 客户端类

    用于连接到 TCP 服务器并发送/接收消息。

    Attributes:
        host: 服务器主机地址
        port: 服务器端口号
        socket: 客户端 socket 对象
    """
    def __init__(self, host='localhost', port=45678):
        """
        初始化 TCP 客户端

        Args:
            host: 服务器主机地址，默认 'localhost'
            port: 服务器端口号，默认 45678
        """
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        """
        连接到服务器

        Returns:
            bool: 连接成功返回 True，失败返回 False
        """
        try:
            self.socket.connect((self.host, self.port))
            print(f"已连接到服务器 {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"连接失败：{e}")
            return False

    def send_message(self, message):
        """
        发送消息到服务器并接收响应

        Args:
            message: 要发送的字符串消息

        Returns:
            str: 服务器返回的响应，失败时返回 None
        """
        try:
            self.socket.send(message.encode('utf-8'))
            response = self.socket.recv(1024)
            return response.decode('utf-8')
        except Exception as e:
            print(f"发送消息失败：{e}")
            return None

    def close(self):
        """
        关闭连接

        关闭客户端 socket。
        """
        self.socket.close()
        print("连接已关闭")


# 使用 TCP 客户端
if __name__ == "__main__":
    client = TCPClient()

    if client.connect():
        try:
            while True:
                message = input("请输入消息 (输入'quit'退出): ")
                if message.lower() == 'quit':
                    break

                response = client.send_message(message)
                if response:
                    print(f"服务器回复：{response}")
        finally:
            client.close()
