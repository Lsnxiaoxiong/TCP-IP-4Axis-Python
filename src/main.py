import threading

from src.keyboard_pressor import KeyboardPressor
from src.tcp_server import TCPServer



if __name__ == '__main__':

    server = TCPServer('localhost', 45678)
    server_worker = threading.Thread(target=server.start)
    server_worker.daemon = True
    server_worker.start()
    controller = KeyboardPressor(
        model_path=r"best.onnx")

    while True:
        msg = server.msg
        try:
            if "point" in msg:
                """ 
                    {"point":[x,y,depth]}
                    {"point":[123,456,20]}
                """
                point = msg["point"]
                print("p")
                controller.press_pix_point(point)
            server.send_msg("1")
        except Exception as e:
            print(e)
