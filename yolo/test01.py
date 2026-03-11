import cv2

from src.realsense_435i import RealSense435i
from yolo.yolov8_onnx import Yolov8Engine2

if __name__ == "__main__":
    engine:Yolov8Engine2 = Yolov8Engine2(model_path=r"C:\ProgramData\VIRobotics\Train\yolov8\runs\detect\train3\weights\best.onnx")

    cap: RealSense435i = RealSense435i()

    while True:
        color_frame = cap.get_latest()[0]
        engine.inference(color_frame=color_frame)
        # print(f"检测结果：{engine.cur_detect_res}")
        img = cv2.cvtColor(color_frame, cv2.COLOR_RGB2BGR)
        for index in engine.latest_res.keys():
            cv2.circle(img,(int(engine.orig_img_size[1]*engine.cur_detect_res[index]["xc"]),
                       int(engine.orig_img_size[0] * engine.cur_detect_res[index]["yc"])),
                       5,
                       (0,0,255),
                       )
            cv2.line(img, (340, 200), (940, 200), (0, 255, 0), thickness=1)
            cv2.line(img, (340, 200), (340, 540), (0, 255, 0), thickness=1)
            cv2.line(img, (340, 540), (940, 540), (0, 255, 0), thickness=1)
            cv2.line(img, (940, 200), (940, 540), (0, 255, 0), thickness=1)
        cv2.imshow("frame",img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # time.sleep(0.3)