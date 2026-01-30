import json
import time

from camera.realsense_435i import RealSense435i
from dobot_mg400 import DobotMG400



if __name__ == '__main__':
    poses = {}
    mg400 = DobotMG400()
    cap = RealSense435i()
    with open("pose.txt", 'r') as f:
        data = f.readlines()
    for pos in data:
        pos = pos.strip().split(',')
        print([pos[0], pos[1], pos[2], pos[3]])
        mg400.run_point([float(pos[0]), float(pos[1]), float(pos[2]), float(pos[3])])

        img_name = cap.save_color_img()
        poses[img_name] = {
            "x": float(pos[0]),
            "y": float(pos[1]),
            "z": float(pos[2]),
            "r": float(pos[3])
        }
        time.sleep(1.3)
    with open("poses.json", 'w') as f:
        json.dump(poses, f)
    # print(data)