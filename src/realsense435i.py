import threading
import time

import pyrealsense2 as rs
import cv2
import numpy as np


class RealSense435i:
    """
    Intel RealSense D435i 相机封装类

    提供彩色图像和深度图像的捕获、对齐和显示功能。
    支持鼠标交互选择目标点，并获取该点的深度信息。

    Attributes:
        latest_color: 最新彩色图像帧
        latest_depth: 最新深度图像帧（彩色映射）
        depth_frame: 原始深度帧
        tar_x: 目标点 X 坐标
        tar_y: 目标点 Y 坐标
    """

    def __init__(self, init_point:tuple = (340, 200)):
        """
        初始化 RealSense 相机

        Args:
            init_point: 初始目标点坐标 (x, y)，默认为 (340, 200)
        """
        self.lock = threading.Lock()

        self.latest_color = None
        self.latest_depth = None
        self.depth_frame = None
        self.pipeline = rs.pipeline()
        config = rs.config()
        # 显式配置流（可选，但推荐）
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
        self.pipeline.start(config)

        # 创建 align 对象：将深度图对齐到彩色图
        self.align_to_color = rs.align(rs.stream.color)
        self.colorizer = rs.colorizer()
        self.init_point = init_point
        self.tar_x, self.tar_y = self.init_point[0], self.init_point[1]
        cap_thread = threading.Thread(target=self._capture_loop)
        cap_thread.daemon = True
        cap_thread.start()
        self.has_frame = False

    def init_tar(self):
        """
        初始化目标点到初始位置

        将目标点坐标重置为构造函数中指定的初始点。
        """
        self.tar_x, self.tar_y = self.init_point[0], self.init_point[1]

    def mouse_callback(self, event, x, y, flags, param):
        """
        鼠标事件回调函数

        捕获鼠标左键点击事件，更新目标点坐标。

        Args:
            event: OpenCV 鼠标事件类型
            x: 鼠标点击位置的 X 坐标
            y: 鼠标点击位置的 Y 坐标
            flags: 鼠标事件标志位
            param: 附加参数
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.tar_x, self.tar_y = x, y
            print(f"鼠标点击位置 - 像素坐标：({x}, {y})")

    def _capture_loop(self):
        """
        相机捕获循环（后台线程运行）

        持续从 RealSense 相机获取帧数据，将对齐后的彩色和深度图像
        存储到实例变量中供其他方法访问。
        """
        while True:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align_to_color.process(frames)

            depth = aligned.get_depth_frame()
            color = aligned.get_color_frame()
            if not depth or not color:
                continue

            color_img = np.asanyarray(color.get_data())
            # color_bgr = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)

            depth_color = np.asanyarray(
                self.colorizer.colorize(depth).get_data()
            )

            with self.lock:
                self.latest_color = color_img
                self.latest_depth = depth_color
                self.depth_frame = depth
                self.has_frame = True

    def get_latest(self):
        """
        获取最新的彩色和深度图像

        等待直到有可用帧，然后返回锁保护的图像数据副本。

        Returns:
            tuple: (color_img, depth_color, depth_frame)
                - color_img: 彩色图像 (RGB)
                - depth_color: 深度图像（彩色映射）
                - depth_frame: 原始深度帧对象
        """
        while not self.has_frame:
            time.sleep(1)
        with self.lock:
            return self.latest_color, self.latest_depth, self.depth_frame

    def get_point_depth(self, point:tuple=(640,320)):
        """
        获取指定像素点的深度值

        Args:
            point: 像素坐标 (x, y)，默认为 (640, 320)

        Returns:
            float: 深度值（米）
        """
        depth_frame = self.get_latest()[2]
        dist = depth_frame.get_distance(int(point[0]),int(point[1]))
        return dist

    def get_frames(self):
        """
        获取连续的帧流（生成器）

        Yields:
            tuple: (color_image_bgr, depth_colormap)
                - color_image_bgr: BGR 格式的彩色图像
                - depth_colormap: 彩色映射的深度图像
        """
        colorizer = rs.colorizer()
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                # 对齐深度帧到彩色帧
                aligned_frames = self.align_to_color.process(frames)
                # 获取对齐后的帧
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # 使用 colorizer 将深度图转换为彩色
                depth_colormap_frame = colorizer.colorize(depth_frame)

                # 转换为 numpy 数组
                color_image = np.asanyarray(color_frame.get_data())
                depth_colormap = np.asanyarray(depth_colormap_frame.get_data())

                color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                yield color_image_bgr, depth_colormap

        finally:
            self.pipeline.stop()

    def display(self):
        """
        显示彩色和深度图像窗口

        打开两个 OpenCV 窗口分别显示彩色图像和深度图像，
        并在彩色图像窗口上绑定鼠标回调。
        """
        color_img,depth_img = self.get_latest()[0],self.get_latest()[1]
        cv2.imshow("Color", color_img)
        cv2.setMouseCallback("Color", self.mouse_callback)
        cv2.imshow("Depth", depth_img)

        # 可选：左右拼接显示，更直观对比
        # combined = np.hstack((color_image_bgr, depth_colormap))
        # cv2.imshow("Combined", combined)
        cv2.waitKey(1)

    def cali(self):
        """
        相机校准模式

        显示带校准标记的彩色图像窗口，用于相机参数标定。
        显示绿色矩形框和中心点，支持鼠标交互选择参考点。
        按'q'键退出校准模式。
        """
        # 创建 colorizer 对象
        colorizer = rs.colorizer()
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                # 对齐深度帧到彩色帧
                aligned_frames = self.align_to_color.process(frames)
                # 获取对齐后的帧
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # 使用 colorizer 将深度图转换为彩色
                depth_colormap_frame = colorizer.colorize(depth_frame)

                # 转换为 numpy 数组
                color_image = np.asanyarray(color_frame.get_data())
                depth_colormap = np.asanyarray(depth_colormap_frame.get_data())

                color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                # cv2.circle(color_image_bgr, (640, 360), 5, (0, 0, 255))
                # cv2.circle(color_image_bgr, (644, 473), 5, (0, 0, 255))
                # cv2.line(color_image_bgr, (340, 200), (940, 200), (0, 255, 0), thickness=1)
                # cv2.line(color_image_bgr, (340, 200), (340, 540), (0, 255, 0), thickness=1)
                # cv2.line(color_image_bgr, (340, 540), (940, 540), (0, 255, 0), thickness=1)
                # cv2.line(color_image_bgr, (940, 200), (940, 540), (0, 255, 0), thickness=1)
                cv2.imshow("Color", color_image_bgr)
                cv2.setMouseCallback("Color", self.mouse_callback)
                # cv2.imshow("Depth", depth_colormap)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()



if __name__ == '__main__':
    cap = RealSense435i()
    # cap.display()
    cap.cali()
