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

        self.load()

    def load(self):
        try:
            self.session = ort.InferenceSession(self.model_path, providers=['CUDAExecutionProvider'])
        except Exception as e:
            raise f"模型加载失败：{str(e)}"
        self.input_size = self.session.get_inputs()[0].shape[2:]
        self.input_name = self.session.get_inputs()[0].name

    def inference(self, color_frame) :
        self.cur_detect_res = {}
        input_image, resized_image = self.preprocess(color_frame)
        outputs = self.session.run(None, {self.input_name: input_image})
        boxes, confidences, class_ids = self.postprocess(outputs)
        for box, confidence, class_id in zip(boxes, confidences, class_ids):
            xc = box[0] / self.input_size[0]
            yc = box[1] / self.input_size[1]
            w = box[2] / self.input_size[0]
            h = box[3] / self.input_size[1]

            self.cur_detect_res[int(class_id)] = {
                "xc": xc,
                "yc": yc,
                "w": w,
                "h": h,
                "confidence": confidence,
            }



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
