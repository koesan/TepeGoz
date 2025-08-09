# modules/fire_detector.py
# ONNX tabanlı yangın/duman tespit modülü.

import onnxruntime as ort
import numpy as np
import cv2
from typing import Optional, List, Tuple
from config import FOREST_FIRE_DETECTION_MODEL

class FireDetector:
    # Modelin giriş boyutu, güven eşikleri ve sınıf isimleri ile başlatılır
    def __init__(
        self,
        input_size: Tuple[int, int] = (640, 640),
        conf_threshold: float = 0.5,
        iou_threshold: float = 0.45,
        class_names: Optional[List[str]] = None,
        providers: Optional[List[str]] = None
    ):

        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.class_names = class_names if class_names else ["smoke", "fire"]

        if providers is None:
            providers = ["CPUExecutionProvider"]

        # ONNX modelini yükle
        self.session = ort.InferenceSession(FOREST_FIRE_DETECTION_MODEL, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [o.name for o in self.session.get_outputs()]

    # Görüntüyü modele uygun boyuta getirir ve normalize eder
    def _preprocess(self, frame: np.ndarray):
        h0, w0 = frame.shape[:2]
        w_in, h_in = self.input_size

        # Oranı hesapla, görüntüyü modele göre ölçeklendir
        r = min(w_in / w0, h_in / h0)
        new_unpad = (int(round(w0 * r)), int(round(h0 * r)))

        # Padding (kenar boşlukları) ekle
        dw, dh = w_in - new_unpad[0], h_in - new_unpad[1]
        dw /= 2
        dh /= 2

        # Yeniden boyutlandır ve padding uygula
        img = cv2.resize(frame, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh)), int(round(dh))
        left, right = int(round(dw)), int(round(dw))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

        # BGR->RGB dönüştür, float32'ye çevir ve normalize et (0-1 arası)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

        # Kanal sırasını (HWC->CHW) değiştir ve batch dimension ekle
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, 0)

        return img, r, (left, top)

    # Merkez koordinatlı kutuları (x,y,w,h) sol-üst ve sağ-alt köşelere (x1,y1,x2,y2) dönüştürür
    @staticmethod
    def _xywh2xyxy(x):
        y = x.copy()
        y[..., 0] = x[..., 0] - x[..., 2] / 2
        y[..., 1] = x[..., 1] - x[..., 3] / 2
        y[..., 2] = x[..., 0] + x[..., 2] / 2
        y[..., 3] = x[..., 1] + x[..., 3] / 2
        return y

    # Non-Maximum Suppression uygular; benzer kutuları elemek için
    @staticmethod
    def _nms(boxes, scores, iou_thres):
        if boxes.shape[0] == 0:
            return np.array([], dtype=np.int32)
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(iou <= iou_thres)[0]
            order = order[inds + 1]
        return np.array(keep, dtype=np.int32)

    # Model çıktısını filtreler, kutuları normalize eder ve NMS uygular
    def _postprocess(self, preds, ratio, pad):
        if preds.ndim == 3 and preds.shape[0] == 1:
            preds = preds[0]

        xywh = preds[:, :4]         # Kutular (x,y,w,h)
        obj_conf = preds[:, 4]      # Nesne güveni
        class_probs = preds[:, 5:]  # Sınıf olasılıkları

        class_ids = np.argmax(class_probs, axis=1)
        class_scores = class_probs[np.arange(len(class_probs)), class_ids]
        final_conf = obj_conf * class_scores  # Toplam güven skoru

        mask = final_conf >= self.conf_threshold  # Güven eşiği filtresi
        if not np.any(mask):
            return []

        xyxy = self._xywh2xyxy(xywh[mask])  # (x,y,w,h) -> (x1,y1,x2,y2)
        scores = final_conf[mask]
        classes = class_ids[mask]
        left, top = pad

        # Eğer kutular normalize ise tekrar genişlik ve yükseklik ile çarp
        if xyxy.max() <= 1.0:
            w_in, h_in = self.input_size
            xyxy[:, [0, 2]] *= w_in
            xyxy[:, [1, 3]] *= h_in

        # Padding'i çıkar
        xyxy[:, [0, 2]] -= left
        xyxy[:, [1, 3]] -= top

        # Oranı kullanarak orijinal görüntü boyutuna geri dön
        xyxy /= ratio

        detections = []

        # Aynı sınıf için NMS uygula ve sonucu topla
        for cls in np.unique(classes):
            idxs = np.where(classes == cls)[0]
            keep = self._nms(xyxy[idxs], scores[idxs], self.iou_threshold)
            for k in keep:
                detections.append((int(classes[idxs[k]]), float(scores[idxs[k]])))

        return detections

    # Frame'den yangın veya duman tespiti yapar
    def predict_from_frame(self, frame: np.ndarray) -> Optional[str]:
        img, ratio, pad = self._preprocess(frame)
        ort_inputs = {self.input_name: img.astype(np.float32)}
        preds = self.session.run(self.output_names, ort_inputs)[0]
        dets = self._postprocess(preds, ratio, pad)

        if not dets:
            return None

        found_fire = any(self.class_names[cls] == "fire" for cls, _ in dets)
        found_smoke = any(self.class_names[cls] == "smoke" for cls, _ in dets)

        if found_fire:
            return "fire"
        if found_smoke:
            return "smoke"
        return None
