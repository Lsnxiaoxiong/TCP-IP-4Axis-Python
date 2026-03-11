import cv2

from src.realsense_435i import RealSense435i

if __name__ == '__main__':
    cap = RealSense435i()
    for color_frame,depth_frame in cap.get_frames():
        cv2.imshow("color",color_frame)
        cv2.imshow("depth",depth_frame)
        cv2.waitKey(1)