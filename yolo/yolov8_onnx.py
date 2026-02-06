import threading
import time

import cv2
import numpy as np


import onnxruntime as ort


class Yolov8Engine:
    def __init__(self,
                 model_path: str,
                 conf: float = 0.45,
                 iou: float = 0.65
                 ):


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
        try:
            self.session = ort.InferenceSession(self.model_path, providers=['CUDAExecutionProvider'])
        except Exception as e:
            raise f"模型加载失败：{str(e)}"
        self.input_size = self.session.get_inputs()[0].shape[2:]
        self.input_name = self.session.get_inputs()[0].name

    def inference(self, color_frame) :

        temp_res = {}
        input_image, resized_image = self.preprocess(color_frame)
        outputs = self.session.run(None, {self.input_name: input_image})
        boxes, confidences, class_ids = self.postprocess(outputs)
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

        # with self._lock:
        self.cur_detect_res = temp_res

    @property
    def latest_res(self):
        # with self._lock:
        return self.cur_detect_res

    def preprocess(self, color_frame):
        """
        预处理输入图片：调整大小、归一化、变换到 CHW 格式并增加批次维度。
        """

        img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)  # 转为 RGB
        self.orig_img_size = img.shape[:2]
        img_resized = cv2.resize(img, self.input_size)  # 调整大小
        img_normalized = img_resized / 255.0  # 归一化
        img_transposed = np.transpose(img_normalized, (2, 0, 1))  # HWC -> CHW
        img_expanded = np.expand_dims(img_transposed, axis=0).astype(np.float32)  # 增加 batch 维度
        return img_expanded, img_resized

    def postprocess(self, outputs):
        """
        后处理模型输出：筛选出有效的边界框并进行非极大值抑制。
        """
        predictions = outputs[0][0]
        mask = predictions[4:].max(axis=0) > self.conf
        filtered_predictions = predictions[:, mask]

        # argmax获取最大值的索引
        class_ids = filtered_predictions[4:].argmax(axis=0)
        confidences = filtered_predictions[4:].max(axis=0)
        results = np.vstack((filtered_predictions[:4], class_ids))
        results = results.transpose((1, 0))

        boxes = results[:, :4].tolist()
        class_ids = results[:, 4].tolist()

        # 非极大值抑制
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf, self.iou)
        final_boxes = [boxes[i] for i in indices]
        final_confidences = [confidences[i] for i in indices]
        final_class_ids = [class_ids[i] for i in indices]
        return final_boxes, final_confidences, final_class_ids




class Yolov8Engine2:
    def __init__(self,
                 model_path: str,
                 conf: float = 0.45,
                 iou: float = 0.65
                 ):
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
        print(f"模型输入尺寸: {self.input_size}")

    def inference(self, color_frame):
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
        # print(f"预处理: {(preprocess_time-start_time)*1000:.2f}ms, "
        #       f"推理: {(inference_time-preprocess_time)*1000:.2f}ms, "
        #       f"后处理: {(postprocess_time-inference_time)*1000:.2f}ms")

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
                {"xc": xc,
                "yc": yc,
                "w": w,
                "h": h,
                "confidence": confidence,}
        """
        with self._lock:
            return self.cur_detect_res.copy()  # 返回副本避免并发问题

    def preprocess(self, color_frame):
        """优化的预处理"""
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
        """优化的后处理"""
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
