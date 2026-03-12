import threading

from src.keyboard_pressor import KeyboardPressor, Keyboard
from src.tcp_server import TCPServer



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
            if "points" in msg:
                """
                    {"points":[123,456]}
                """
                point = msg["points"]
                print("p")
                # controller.press_pix_point((point[0],point[1]))
            server.send_msg("1")
        except Exception as e:
            print(e)
