import threading

from src.dobot_mg400 import DobotMG400
from src.tcp_server import TCPServer



if __name__ == '__main__':
    robot = DobotMG400()
    server = TCPServer('localhost', 45678)
    server_worker = threading.Thread(target=server.start)
    server_worker.daemon = True
    server_worker.start()

    while True:
        msg = server.msg
        try:
            if "point" in msg:
                """ 
                    {"point":[x,y,depth]}
                    {"point":[123,456,20]}
                """
                point = msg["point"]
                robot.run_point(point)
                print("p")
            server.send_msg("1")
        except Exception as e:
            print(e)
