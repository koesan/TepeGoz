"""
============================================================
TepeGöz - Gözlem Drone Yönetim Sistemi - Drone Switcher Modülü
============================================================

Yapan / Author : Barış Enes Kümet
Tarih / Date   : 18.08.2025
Modül Adı      : drone_switcher.py

Açıklama (Türkçe):
------------------
Bu modül, birden fazla drone ile görev yürütürken aktif dronu 
takip eder ve gerekli durumlarda (örneğin: batarya kritik 
seviyeye düştüğünde veya görev süresi 5 dakikayı aştığında) 
drone değişimi yapar.

Özet Fonksiyonellik:
--------------------
1. Görev başladığında süre sayacı başlatılır.
2. Aktif dronun bataryası kritik seviyeye düşerse veya görev 
   süresi dolarsa, dron otomatik olarak RTL (Return to Launch) 
   moduna geçirilir.
3. Görev otomatik olarak bir sonraki drona devredilir.
4. Görev tipi ve kaldığı nokta yeni drone’a aktarılır (varsa).

---

Description (English):
----------------------
This module manages multiple drones during a mission by 
monitoring the active drone and switching to another one when 
necessary (e.g., when the battery level drops below a critical 
threshold or the mission duration exceeds 5 minutes).

Summary of Functionality:
-------------------------
1. Starts a mission timer when the mission begins.
2. If the active drone’s battery reaches a critical level or 
   the mission time limit is exceeded, the drone is switched 
   to RTL (Return to Launch) mode.
3. The mission is automatically handed over to the next drone.
4. The mission type and the last mission point are transferred 
   to the new drone (if available).
"""

import numpy as np
import cv2
from typing import Optional, List, Tuple
from ultralytics import YOLO
from config import DETECTOR_TYPES

#YOLOv11 tabanlı ultralytics modeli kullanarak yangın ve duman tespiti yapar
class FireDetector:
    def __init__(self):        

        detector_config = DETECTOR_TYPES.get('fire', {})
        self.model_path = detector_config.get('model_path')
        self.conf_threshold = detector_config.get('conf_threshold', 0.1)
        self.iou_threshold = detector_config.get('iou_threshold', 0.45)
        self.model = None
        self.class_names = {}

        if self.model_path:
            try:
                self.model = YOLO(self.model_path)
                self.class_names = self.model.names
                print(f"[YOLO] Yangın tespit modeli {self.model_path} başarıyla yüklendi.")
                print(f"[YOLO] Model sınıfları: {self.class_names}")

            except Exception as e:
                print(f"[YOLO] Yangın tespit modeli yüklenirken hata: {e}")
        else:
            print("[YOLO] Yangın modeli için dosya yolu belirtilmemiş.")

    # Bir görüntü karesi üzerinde yangın/duman tespiti yapar ve sonuçları döndürür
    def detect(self, frame: np.ndarray) -> list:
        
        if self.model is None:
            return []

        try:
            results = self.model.predict(
                frame,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False
            )

            detection_results = []
            if results:
                r = results[0]
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    score = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                    class_name = self.class_names.get(class_id, "unknown")

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