import socket


class TCPClient:
    def __init__(self, host='localhost', port=45678):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        """连接到服务器"""
        try:
            self.socket.connect((self.host, self.port))
            print(f"已连接到服务器 {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def send_message(self, message):
        """发送消息"""
        try:
            self.socket.send(message.encode('utf-8'))
            response = self.socket.recv(1024)
            return response.decode('utf-8')
        except Exception as e:
            print(f"发送消息失败: {e}")
            return None

    def close(self):
        """关闭连接"""
        self.socket.close()
        print("连接已关闭")


# 使用TCP客户端
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
                    print(f"服务器回复: {response}")
        finally:
            client.close()