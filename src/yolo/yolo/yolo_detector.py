#!/usr/bin/env python3

import cv2
import numpy as np
import onnxruntime as ort

class YOLODetector:
    def __init__(self, model_path, conf_thres=0.5, iou_thres=0.4):
        self.conf_threshold = conf_thres
        self.iou_threshold = iou_thres
        
        #Force CPU for the PC test
        providers = ['CPUExecutionProvider']
        self.session = ort.InferenceSession(model_path, providers=providers)
        
        # Get model input details (usually 640x640)
        model_inputs = self.session.get_inputs()
        self.input_name = model_inputs[0].name
        self.input_shape = model_inputs[0].shape  # [1, 3, 640, 640]
        self.input_width = self.input_shape[3]
        self.input_height = self.input_shape[2]

    def preprocess(self, img):
        # Resize to model size and change color space BGR -> RGB
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_res = cv2.resize(img_rgb, (self.input_width, self.input_height))
        
        # Normalize 0-255 to 0.0-1.0
        img_data = img_res.astype(np.float32) / 255.0
        # HWC to CHW: (640, 640, 3) -> (3, 640, 640)
        img_data = img_data.transpose(2, 0, 1)
        # Add batch dimension: (1, 3, 640, 640)
        return np.expand_dims(img_data, axis=0)

    def detect(self, frame):
        img_h, img_w = frame.shape[:2]
        blob = self.preprocess(frame)
        
        # Run inference
        outputs = self.session.run(None, {self.input_name: blob})
        
        # YOLOv8/v11 Output shape is [1, 84, 8400]
        # 84 = [x, y, w, h] + 80 class probabilities
        output = np.squeeze(outputs[0]).T  # Transpose to [8400, 84]
        
        # 1. Filter by confidence
        scores = np.max(output[:, 4:], axis=1)
        mask = scores > self.conf_threshold
        output = output[mask]
        scores = scores[mask]
        class_ids = np.argmax(output[:, 4:], axis=1)
        
        # 2. Convert boxes from [cx, cy, w, h] to [x1, y1, x2, y2]
        boxes = output[:, :4]
        boxes[:, 0] = (boxes[:, 0] - (boxes[:, 2] / 2)) * (img_w / self.input_width)
        boxes[:, 1] = (boxes[:, 1] - (boxes[:, 3] / 2)) * (img_h / self.input_height)
        boxes[:, 2] = (boxes[:, 2] * (img_w / self.input_width)) + boxes[:, 0]
        boxes[:, 3] = (boxes[:, 3] * (img_h / self.input_height)) + boxes[:, 1]
        
        # 3. Apply Non-Maximum Suppression (NMS)
        indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), 
                                   self.conf_threshold, self.iou_threshold)
        
        results = []
        if len(indices) > 0:
            for i in indices.flatten():
                results.append({
                    "box": boxes[i].astype(int),
                    "conf": scores[i],
                    "class_id": class_ids[i]
                })
        return results
