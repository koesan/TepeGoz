import numpy as np
import cv2
from typing import Optional, List, Tuple

# Ultralytics kütüphanesini içe aktarıyoruz
from ultralytics import YOLO

# config.py dosyasından model yolunu alıyoruz
from config import FOREST_FIRE_DETECTION_MODEL

class FireDetector:
    """YOLOv8 tabanlı ultralytics modeli kullanarak yangın ve duman tespiti yapar."""

    def __init__(
        self,
        conf_threshold: float = 0.1,
        iou_threshold: float = 0.45,
    ):
        """
        Modeli yükler ve yapılandırma parametrelerini ayarlar.
        ultralytics kütüphanesi kendi içinde ön-işleme ve son-işleme yaptığı için 
        bu fonksiyon daha basittir.
        """
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold

        try:
            # Model dosyasını ultralytics.YOLO ile yüklüyoruz
            self.model = YOLO(FOREST_FIRE_DETECTION_MODEL)
            # Modelin sınıf isimlerini alıyoruz
            self.class_names = self.model.names
            print(f"[YOLO] Yangın tespit modeli {FOREST_FIRE_DETECTION_MODEL} başarıyla yüklendi.")
            print(f"[YOLO] Model sınıfları: {self.class_names}")

        except Exception as e:
            print(f"[YOLO] Yangın tespit modeli yüklenirken hata: {e}")
            self.model = None

    def detect(self, frame: np.ndarray) -> list:
        """Bir frame üzerinde yangın/duman tespiti yapar."""
        if self.model is None:
            # Model yüklenememişse boş liste döndür
            return []

        try:
            # Ultralytics modelinin predict metodunu çağırarak doğrudan tespit yapıyoruz.
            # Ön-işleme ve son-işleme adımlarını kendisi halleder.
            results = self.model.predict(
                frame,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False # Terminal çıktısını azaltmak için
            )

            # İlk sonucun (tek bir görüntü olduğu için) verilerini alıyoruz
            # detection_results'ı, eski yapıyla uyumlu bir liste formatına dönüştürüyoruz
            detection_results = []
            if results:
                r = results[0]  # İlk sonuç nesnesi
                for box in r.boxes:
                    # Kutu koordinatlarını, skoru ve sınıfı alıyoruz
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    score = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                    class_name = self.class_names[class_id]

                    detection_results.append({
                        "box": np.array([x1, y1, x2, y2]),
                        "score": score,
                        "class_id": class_id,
                        "class_name": class_name
                    })

            return detection_results
            
        except Exception as e:
            print(f"[YOLO] 'detect' metodu sırasında hata: {e}")
            return []

"""

import onnxruntime as ort
import numpy as np
import cv2
from typing import Optional, List, Tuple

from config import FOREST_FIRE_DETECTION_MODEL

class FireDetector:

    def __init__(
        self,
        input_size: Tuple[int, int] = (640, 640),
        conf_threshold: float = 0.1,
        iou_threshold: float = 0.45,
        providers: Optional[List[str]] = None
    ):

        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.class_names = ["smoke", "fire"]

        if providers is None:
            providers = ["CPUExecutionProvider"]
        
        try:
            self.session = ort.InferenceSession(FOREST_FIRE_DETECTION_MODEL, providers=providers)
            self.input_name = self.session.get_inputs()[0].name
            self.output_names = [o.name for o in self.session.get_outputs()]
            print(f"[YOLO] Yangın tespit modeli {FOREST_FIRE_DETECTION_MODEL} başarıyla yüklendi.")
        except Exception as e:
            print(f"[YOLO] Yangın tespit modeli yüklenirken hata: {e}")
            self.session = None

    def _preprocess(self, frame: np.ndarray):
        h0, w0 = frame.shape[:2]
        w_in, h_in = self.input_size

        r = min(w_in / w0, h_in / h0)
        new_unpad = (int(round(w0 * r)), int(round(h0 * r)))

        dw, dh = w_in - new_unpad[0], h_in - new_unpad[1]
        dw /= 2
        dh /= 2

        img = cv2.resize(frame, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh)), int(round(dh))
        left, right = int(round(dw)), int(round(dw))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, 0)

        return img, r, (left, top)

    @staticmethod
    def _xywh2xyxy(x):
        y = x.copy()
        y[..., 0] = x[..., 0] - x[..., 2] / 2
        y[..., 1] = x[..., 1] - x[..., 3] / 2
        y[..., 2] = x[..., 0] + x[..., 2] / 2
        y[..., 3] = x[..., 1] + x[..., 3] / 2
        return y

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

    def _postprocess(self, preds, ratio, pad):
        print(f"[YOLO DEBUG] Post-işleme başlatıldı. Tahminlerin boyutu: {preds.shape}")
        
        # Genellikle (1, 84, N) veya (1, N, 84) formatındadır.
        if preds.shape[1] < preds.shape[2]:
            preds = np.transpose(preds, (0, 2, 1))
            
        preds = preds[0]  # batch'ten çıkart

        xywh = preds[:, :4]       # kutular (x,y,w,h)
        obj_conf = preds[:, 4]    # nesne güveni
        class_probs = preds[:, 5:5+len(self.class_names)] # sınıf olasılıkları

        class_ids = np.argmax(class_probs, axis=1)
        class_scores = class_probs[np.arange(len(class_probs)), class_ids]
        final_conf = obj_conf * class_scores

        print(f"[YOLO DEBUG] Tespit edilen toplam kutu sayısı (ön-filtreleme): {len(final_conf)}")
        print(f"[YOLO DEBUG] Maksimum güvenilirlik skoru: {np.max(final_conf) if len(final_conf) > 0 else 0}")
        print(f"[YOLO DEBUG] Ayarlanan conf_threshold: {self.conf_threshold}")

        mask = final_conf >= self.conf_threshold
        if not np.any(mask):
            print("[YOLO DEBUG] Eşik değerini geçen bir tespit bulunamadı.")
            return np.array([]), np.array([]), np.array([])

        xyxy = self._xywh2xyxy(xywh[mask])
        scores = final_conf[mask]
        classes = class_ids[mask]
        left, top = pad

        if xyxy.max() <= 1.0:
            w_in, h_in = self.input_size
            xyxy[:, [0, 2]] *= w_in
            xyxy[:, [1, 3]] *= h_in

        xyxy[:, [0, 2]] -= left
        xyxy[:, [1, 3]] -= top
        xyxy /= ratio
        
        final_boxes, final_scores, final_class_ids = [], [], []
        for cls in np.unique(classes):
            idxs = np.where(classes == cls)[0]
            keep = self._nms(xyxy[idxs], scores[idxs], self.iou_threshold)
            for k in keep:
                final_boxes.append(xyxy[idxs[k]])
                final_scores.append(scores[idxs[k]])
                final_class_ids.append(classes[idxs[k]])
        
        print(f"[YOLO DEBUG] NMS sonrası tespit edilen final kutu sayısı: {len(final_boxes)}")
        return np.array(final_boxes), np.array(final_scores), np.array(final_class_ids)

    def detect(self, frame: np.ndarray) -> list:
        print("[YOLO DEBUG] 'detect' metodu çağrıldı.")
        if self.session is None:
            print("[YOLO DEBUG] ONNX oturumu başlatılamadığı için tespit yapılamıyor.")
            return []
        try:
            img, ratio, pad = self._preprocess(frame)
            print(f"[YOLO DEBUG] Ön-işlenmiş görüntü boyutu: {img.shape}")
            ort_inputs = {self.input_name: img.astype(np.float32)}
            preds = self.session.run(self.output_names, ort_inputs)[0]
            
            boxes, scores, class_ids = self._postprocess(preds, ratio, pad)
            
            results = []
            for i in range(len(boxes)):
                results.append({
                    "box": boxes[i],
                    "score": scores[i],
                    "class_id": class_ids[i],
                    "class_name": self.class_names[class_ids[i]]
                })
            
            print(f"[YOLO DEBUG] 'detect' metodundan dönen sonuçlar: {results}")
            return results
        except Exception as e:
            print(f"[YOLO] 'detect' metodu sırasında hata: {e}")
            return []
"""