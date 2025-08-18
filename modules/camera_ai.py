"""
============================================================
TepeGöz - Gözlem Drone Yönetim Sistemi - Drone Switcher Modülü
============================================================

Yapan / Author : Barış Enes Kümet
Tarih / Date   : 18.08.2025
Modül Adı      : camera_ai.py

Açıklama (Türkçe):
------------------
Bu modül, drone'lardan gelen kamera yayınlarını işlemek ve üzerinde yapay zeka tabanlı nesne algılama yapmak için tasarlanmıştır. 
ROS (Robot Operating System) üzerinden gelen görüntü mesajlarını alır, dinamik olarak yüklenen YOLO modellerini kullanarak yangın, 
insan gibi nesneleri tespit eder ve işlenmiş görüntüleri canlı MJPEG akışı olarak sunar.

Özet Fonksiyonellik:
--------------------
1. ROS üzerinden kamera yayınlarına abone olur.
2. Görev tipine göre farklı nesne algılama modellerini (YOLO) dinamik olarak yükler ve kullanır.
3. Görüntü üzerinde tespit edilen nesnelerin etrafına kutular ve etiketler çizer.
4. İşlenmiş canlı kamera yayınını bir MJPEG akışı olarak yayınlar.

---

Description (English):
----------------------
This module is designed to process camera feeds from drones and perform AI-based object detection on them. 
It receives image messages via ROS (Robot Operating System), uses dynamically loaded YOLO models to detect objects such as fire or humans, 
and provides the processed images as a live MJPEG stream.

Summary of Functionality:
-------------------------
1. Subscribes to camera feeds via ROS.
2. Dynamically loads and uses different object detection models (YOLO) based on the mission type.
3. Draws bounding boxes and labels around detected objects on the image.
4. Publishes the processed live camera feed as an MJPEG stream.
"""

import cv2
import numpy as np
import rospy
import threading
import time
import importlib 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from config import CAMERA_TOPICS, LOG_THROTTLE_SEC, JPEG_QUALITY, PLACEHOLDER_IMAGE_SIZE, FONT_SCALE, FONT_THICKNESS, DETECTOR_TYPES
# FireDetector ve HumanDetector artık doğrudan import edilmeyecek.

class CameraAIHandler:

    def __init__(self):
        self.bridge = CvBridge()
        self.camera_feeds = {}
        self.lock = threading.Lock()
        self.is_ros_node_initialized = False
        self.subscribed_ports = set()
        self.last_log_time = {}
        self.latest_detection_results = []
        self.detection_lock = threading.Lock()
        self.active_detector = None
        self.detectors = {}

        # Modelleri döngü ile dinamik olarak 1 kereden yükle
        try:
            for mission_type, detector_info in DETECTOR_TYPES.items():
                module_name = detector_info['module']
                class_name = detector_info['class_name']
                
                module = importlib.import_module(module_name)
                detector_class = getattr(module, class_name)
                self.detectors[mission_type] = detector_class()

            print("[YOLO] Tüm algılama modelleri başarıyla yüklendi.")
        except Exception as e:
            print(f"[YOLO] Algılama modeli yüklenirken hata: {e}")
            self.detectors = {}

    # En son tespit sonuçlarını döndürür.        
    def get_latest_detection_results(self):
      
        with self.detection_lock:
            return self.latest_detection_results

    # Görev tipine göre aktif detektörü ayarlar.
    def set_mission_type(self, mission_type):
        
        self.active_detector = self.detectors.get(mission_type)
        if self.active_detector:
            print(f"[YOLO] Aktif detektör: {mission_type}")
            return True
        else:
            print(f"[YOLO] '{mission_type}' için detektör bulunamadı!")
            return False

    # ROS düğümünü başlatır
    def init_ros_node(self):
        
        if not self.is_ros_node_initialized:
            try:
                rospy.init_node('web_camera_listener', anonymous=True, disable_signals=True)
                self.is_ros_node_initialized = True
            except rospy.exceptions.ROSException as e:
                print(f"[ROS Kamera] init hatası / zaten başlatılmış olabilir: {e}")
                self.is_ros_node_initialized = True

    # Port için ROS topic'ine abone olur
    def subscribe_to_camera_topic_for_port(self, port):
        
        if port is None:
            return False
        if port in self.subscribed_ports:
            return True
        topic = CAMERA_TOPICS.get(port)
        if not topic:
            print(f"[ROS Kamera] CAMERA_TOPICS içinde port yok: {port}")
            return False

        self.init_ros_node()

        try:
            rospy.Subscriber(topic, Image, self._camera_callback_wrapper, callback_args=port, queue_size=1)
            self.subscribed_ports.add(port)
            print(f"[ROS Kamera] Abone olundu: {topic} (port: {port})")
            return True
        except Exception as e:
            print(f"[ROS Kamera] Subscriber oluşturulurken hata: {e}")
            return False

    # Kamera mesajını alır ve JPEG olarak saklar
    def _camera_callback_wrapper(self, msg, port):
        
        now = time.time()
        last = self.last_log_time.get(port, 0)
        if now - last > LOG_THROTTLE_SEC:
            self.last_log_time[port] = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"[ROS Kamera] CvBridge hata: {e}")
            return
        except Exception as e:
            print(f"[ROS Kamera] Image->CV çevirme hatası: {e}")
            return

        if self.active_detector:
            try:
                detection_results = self.active_detector.detect(cv_image)

                with self.detection_lock:
                    self.latest_detection_results = detection_results
                
                if detection_results:
                    for result in detection_results:
                        box = result['box']
                        score = result['score']
                        class_name = result['class_name']
                        x1, y1, x2, y2 = box.astype(int)
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"{class_name}: {score:.2f}"
                        cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            except Exception as e:
                print(f"[YOLO] Algılama işlemi sırasında hata: {e}")

        try:
            ret, buf = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ret:
                print(f"[ROS Kamera] JPEG encode başarısız (port: {port})")
                return

            jpeg_bytes = buf.tobytes()
            with self.lock:
                self.camera_feeds[port] = jpeg_bytes

            if now - self.last_log_time.get(f"enc_{port}", 0) > LOG_THROTTLE_SEC:
                self.last_log_time[f"enc_{port}"] = now
        except Exception as e:
            print(f"[ROS Kamera] Encode/saklama hatası: {e}")

    # Son JPEG frame'i döner
    def get_latest_frame(self, port):
        
        with self.lock:
            return self.camera_feeds.get(port)

    # Belirtilen portun frame bilgisini siler
    def clear_frame(self, port):
        
        with self.lock:
            if port in self.camera_feeds:
                del self.camera_feeds[port]

    # MJPEG olarak frame üretir
    def generate_frames(self, active_drone_port):
        
        if active_drone_port is not None:
            subscribed = self.subscribe_to_camera_topic_for_port(active_drone_port)
            if not subscribed:
                print(f"[CameraAIHandler] {active_drone_port} için abonelik başarısız.")

        boundary = b'--frame\r\n'
        content_type = b'Content-Type: image/jpeg\r\n\r\n'

        while True:
            try:
                frame = self.get_latest_frame(active_drone_port)
                if frame:
                    yield boundary + content_type + frame + b'\r\n'
                else:
                    placeholder = self._create_placeholder_image(f"Kamera Beslemesi Yok (port: {active_drone_port})")
                    if placeholder:
                        yield boundary + content_type + placeholder + b'\r\n'
                time.sleep(0.05)
            except GeneratorExit:
                break
            except Exception as e:
                print(f"[CameraAIHandler] generate_frames hatası: {e}")
                time.sleep(0.2)

    # Yazılı placeholder resim oluşturur
    def _create_placeholder_image(self, text):
        
        try:
            w, h = PLACEHOLDER_IMAGE_SIZE
            img = np.zeros((h, w, 3), dtype=np.uint8)
            font = cv2.FONT_HERSHEY_SIMPLEX
            (tw, th), _ = cv2.getTextSize(text, font, FONT_SCALE, FONT_THICKNESS)
            x = max(10, (w - tw) // 2)
            y = max(20, (h + th) // 2)
            cv2.putText(img, text, (x, y), font, FONT_SCALE, (200, 200, 200), FONT_THICKNESS, cv2.LINE_AA)
            ret, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ret:
                return None
            return buf.tobytes()
        except Exception as e:
            print(f"[CameraAIHandler] Placeholder oluşturma hatası: {e}")
            return None