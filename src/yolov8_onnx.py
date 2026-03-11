import threading
import time

import cv2
import numpy as np


import onnxruntime as ort


class Yolov8Engine2:
    """
    YOLOv8 ONNX 推理引擎

    使用 ONNX Runtime 运行 YOLOv8 目标检测模型，
    支持 CUDA 加速和 CPU fallback。

    Attributes:
        model_path: ONNX 模型文件路径
        conf: 置信度阈值
        iou: NMS 的 IoU 阈值
        session: ONNX Runtime 推理会话
        input_size: 模型输入尺寸
        orig_img_size: 原始图像尺寸
        cur_detect_res: 当前检测结果
    """
    def __init__(self,
                 model_path: str,
                 conf: float = 0.45,
                 iou: float = 0.65
                 ):
        """
        初始化 YOLOv8 推理引擎

        Args:
            model_path: ONNX 模型文件路径
            conf: 置信度阈值，默认 0.45
            iou: NMS 的 IoU 阈值，默认 0.65
        """
        self.input_name = None
        self.input_size = None
        self.model_path = model_path
        self.conf = conf
        self.iou = iou
        self.session = None
        self.orig_img_size = ()

        self.cur_detect_res = {}
        self._lock = threading.Lock()

        self.load()

    def load(self):
        """
        加载 ONNX 模型并配置推理引擎

        自动检测可用的执行 provider（CUDA/CPU），
        优先使用 CUDA 加速，如果不可用则回退到 CPU。

        Raises:
            Exception: 模型加载失败时抛出
        """
        # 检查可用的 providers
        available_providers = ort.get_available_providers()
        print(f"可用的 providers: {available_providers}")

        # 设置 session options
        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        sess_options.intra_op_num_threads = 4  # 根据你的 CPU 核心数调整

        # 优先使用 CUDA，如果不可用则使用 CPU
        providers = []
        if 'CUDAExecutionProvider' in available_providers:
            providers.append(('CUDAExecutionProvider', {
                'device_id': 0,
                'arena_extend_strategy': 'kNextPowerOfTwo',
                'gpu_mem_limit': 2 * 1024 * 1024 * 1024,  # 2GB
                'cudnn_conv_algo_search': 'EXHAUSTIVE',
                'do_copy_in_default_stream': True,
            }))
            print("使用 CUDA 加速")

        providers.append('CPUExecutionProvider')

        try:
            self.session = ort.InferenceSession(
                self.model_path,
                sess_options=sess_options,
                providers=providers
            )

            # 打印实际使用的 provider
            print(f"实际使用的 provider: {self.session.get_providers()}")

        except Exception as e:
            raise Exception(f"模型加载失败：{str(e)}")

        self.input_size = self.session.get_inputs()[0].shape[2:]
        self.input_name = self.session.get_inputs()[0].name
        print(f"模型输入尺寸：{self.input_size}")

    def inference(self, color_frame):
        """
        执行目标检测推理

        对输入帧进行预处理、推理和后处理，
        将检测结果存储到 cur_detect_res 中。

        Args:
            color_frame: 输入的彩色图像帧（numpy 数组）
        """
        temp_res = {}

        # 添加计时
        start_time = time.time()

        input_image, resized_image = self.preprocess(color_frame)
        preprocess_time = time.time()

        outputs = self.session.run(None, {self.input_name: input_image})
        inference_time = time.time()

        boxes, confidences, class_ids = self.postprocess(outputs)
        postprocess_time = time.time()

        # 打印各阶段耗时（调试用）
        # print(f"预处理：{(preprocess_time-start_time)*1000:.2f}ms, "
        #       f"推理：{(inference_time-preprocess_time)*1000:.2f}ms, "
        #       f"后处理：{(postprocess_time-inference_time)*1000:.2f}ms")

        for box, confidence, class_id in zip(boxes, confidences, class_ids):
            xc = box[0] / self.input_size[0]
            yc = box[1] / self.input_size[1]
            w = box[2] / self.input_size[0]
            h = box[3] / self.input_size[1]
            temp_res[int(class_id)] = {
                "xc": xc,
                "yc": yc,
                "w": w,
                "h": h,
                "confidence": confidence,
            }

        with self._lock:
            self.cur_detect_res = temp_res

    @property
    def latest_res(self):
        """
        获取最新的检测结果

        返回检测结果的副本以避免并发问题。
        检测结果包含每个类别的边界框、置信度等信息。

        Returns:
            dict: 检测结果字典，格式为 {class_id: {"xc": x_center, "yc": y_center, "w": width, "h": height, "confidence": conf}}
        """
        with self._lock:
            return self.cur_detect_res.copy()  # 返回副本避免并发问题

    def preprocess(self, color_frame):
        """
        图像预处理

        将输入图像转换为模型所需的格式：
        1. BGR 转 RGB（如果需要）
        2. 缩放到模型输入尺寸
        3. 归一化到 [0, 1]
        4. 转换为 CHW 格式并添加 batch 维度

        Args:
            color_frame: 输入的彩色图像帧

        Returns:
            tuple: (input_image, img_resized)
                - input_image: 预处理后的图像（NCHW 格式）
                - img_resized: 缩放后的图像
        """
        # 如果输入已经是 RGB，跳过转换
        if color_frame.shape[2] == 3:
            img = color_frame if color_frame.dtype == np.uint8 else color_frame.astype(np.uint8)
        else:
            img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)

        self.orig_img_size = img.shape[:2]

        # 使用 cv2.resize 的 INTER_LINEAR（默认）比 INTER_CUBIC 快
        img_resized = cv2.resize(img, self.input_size, interpolation=cv2.INTER_LINEAR)

        # 合并操作减少中间数组
        img_normalized = (img_resized.astype(np.float32) / 255.0).transpose(2, 0, 1)
        img_expanded = np.expand_dims(img_normalized, axis=0)

        return img_expanded, img_resized

    def postprocess(self, outputs):
        """
        推理后处理

        解析模型输出，应用置信度阈值过滤和 NMS：
        1. 过滤低置信度检测
        2. 提取类别和置信度
        3. 应用 NMS 去除重叠框

        Args:
            outputs: ONNX 模型输出

        Returns:
            tuple: (boxes, confidences, class_ids)
                - boxes: 检测框列表 [x1, y1, x2, y2]
                - confidences: 置信度列表
                - class_ids: 类别 ID 列表
        """
        predictions = outputs[0][0]

        # 向量化操作
        scores = predictions[4:].max(axis=0)
        mask = scores > self.conf

        if not mask.any():
            return [], [], []

        filtered_predictions = predictions[:, mask]
        class_ids = filtered_predictions[4:].argmax(axis=0)
        confidences = filtered_predictions[4:].max(axis=0)

        # 直接使用切片，避免 vstack
        boxes = filtered_predictions[:4].T.tolist()
        class_ids = class_ids.tolist()
        confidences = confidences.tolist()

        # NMS
        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf, self.iou)
            final_boxes = [boxes[i] for i in indices]
            final_confidences = [confidences[i] for i in indices]
            final_class_ids = [class_ids[i] for i in indices]
            return final_boxes, final_confidences, final_class_ids

        return [], [], []

if __name__ == "__main__":
    import onnxruntime as ort

    print(ort.get_available_providers())
