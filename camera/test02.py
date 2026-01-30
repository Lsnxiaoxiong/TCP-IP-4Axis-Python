import threading
import time

import cv2

from camera.realsense_435i import RealSense435i


def show_rgb(cap, name):
    while True:
        time.sleep(0.5)
        cv2.imshow(f"depth{name}", cap.get_latest()[0])
        cv2.waitKey(1)


def show_depth(cap: RealSense435i):
    cv2.imshow("depth", cap.get_latest()[1])
    cv2.waitKey(1)


if __name__ == '__main__':
    cap: RealSense435i = RealSense435i()
    threading.Thread(target=show_rgb, args=(cap, 1)).start()
    threading.Thread(target=show_rgb, args=(cap, 2)).start()
    # threading.Thread(target=show_depth,args=(cap,)).start()

    time.sleep(10)
