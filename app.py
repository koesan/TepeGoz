"""
============================================================
TepeGöz - Gözlem Drone Yönetim Sistemi - Ana Uygulama Modülü
============================================================

Yapan / Author : Barış Enes Kümet
Tarih / Date   : 18.08.2025
Modül Adı      : app.py

Açıklama (Türkçe):
------------------
Bu modül, "TepeGöz" sisteminin ana sunucu uygulamasını 
(Flask tabanlı) çalıştırır. Kullanıcı arayüzü ile dronlar ve 
görev kontrol modülleri arasında bir köprü görevi görür. 
RESTful API endpoint'leri aracılığıyla dron bağlantısını, 
görev planlamasını ve canlı durum takibini yönetir. Ayrıca 
ROS (Robot Operating System) ile entegre olarak kamera 
verilerini işler ve bir video akışı sunar.

Özet Fonksiyonellik:
--------------------
1. Drona bağlanma, dron seçme ve bağlantı durumlarını yönetme.
2. Gözlem alanı (grid) belirleme ve görevleri başlatıp durdurma.
3. Dronun, görevin ve kamera tespitlerinin (yangın, duman vb.) 
   anlık durumunu sorgulayan bir API endpoint'i sunma.
4. Gözlem dronunun kamerasından canlı video akışını sağlama.
5. Arka planda ROS düğümünü başlatarak ROS tabanlı modüllerle 
   iletişim kurma.

---

Description (English):
----------------------
This module runs the main server application (Flask-based) for 
the "TepeGöz" system. It acts as a bridge between the user 
interface and the drone/mission control modules. It manages 
drone connections, mission planning, and live status monitoring 
via RESTful API endpoints. It also integrates with ROS (Robot 
Operating System) to process camera data and provide a video 
stream.

Summary of Functionality:
-------------------------
1. Manages drone connection, selection, and connection status.
2. Defines the observation area (grid) and starts/stops missions.
3. Provides an API endpoint to query the real-time status of the 
   drone, mission, and camera detections (fire, smoke, etc.).
4. Provides a live video feed from the observation drone's camera.
5. Starts a background ROS node to communicate with ROS-based modules.
"""

from flask import Flask, render_template, request, jsonify, Response
import threading
import os
import rospy
from modules.drone_manager import DroneManager
from modules.mission_controller import MissionController
import time
from config import APP_HOST, APP_PORT, DETECTION_RETENTION_TIME

app = Flask(__name__, template_folder='templates')

# Modül sınıflarını başlatıyoruz
drone_manager = DroneManager()
mission_controller = MissionController(drone_manager)
active_drone_port = None
last_detection_results = []
last_detection_time = 0

@app.route('/')
def index():
    """Ana sayfayı render eder."""
    return render_template('index.html')

@app.route('/connect_drone', methods=['POST'])
def connect_drone():
    """Drona bağlanma isteğini işler."""
    data = request.json
    connection_string = data.get('connection_string')
    return jsonify(drone_manager.start_connection(connection_string))

@app.route('/select_drone', methods=['POST'])
def select_drone():
    """Kontrol edilecek dronu seçer."""
    port = request.json.get('port')
    return jsonify(drone_manager.select_drone(port))

@app.route('/set_area', methods=['POST'])
def set_area():
    """Görev alanını belirler."""
    data = request.json
    coordinates = data.get('coordinates')
    return jsonify(mission_controller.set_area(coordinates))

@app.route('/start_mission', methods=['POST'])
def start_mission():
    """Görevi başlatır."""
    data = request.json
    mission_type = data.get('mission_type')
    return jsonify(mission_controller.start_mission(mission_type))

@app.route('/stop_mission', methods=['POST'])
def stop_mission():
    """Görevi durdurur."""
    return jsonify(mission_controller.stop_mission())

@app.route('/status')
def status():
    """Dron ve görev durumunu, tespit sonuçlarıyla birlikte döndürür."""
    global last_detection_time, last_detection_results

    drone_status = drone_manager.get_status()
    mission_status = mission_controller.get_status()

    detections = drone_manager.camera_handler.get_latest_detection_results()
    if detections:
        last_detection_time = time.time()
        last_detection_results = detections
    elif time.time() - last_detection_time <= DETECTION_RETENTION_TIME:
        detections = last_detection_results
    else:
        detections = []

    # NumPy array'lerini JSON'a uygun listeye çevir
    safe_detections = []
    for det in detections:
        safe_detections.append({
            "box": det["box"].tolist() if hasattr(det["box"], "tolist") else det["box"],
            "score": det["score"],
            "class_id": det["class_id"],
            "class_name": det["class_name"]
        })

    # Dron ve görev durumunu birleştir
    combined_status = {**drone_status, **mission_status}
    combined_status["mission_status_message"] = drone_manager.mission_status_message
    combined_status["detection_results"] = safe_detections or []

    return jsonify(combined_status)

@app.route('/camera_feed')
def camera_feed():
    """Dron kamerasından canlı video akışı sağlar."""
    port = drone_manager.active_drone_port
    if port is None:
        return "Kamera beslemesi yok", 404
    return Response(drone_manager.camera_handler.generate_frames(port),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # ROS'u ayrı bir thread'de başlat
        ros_thread = threading.Thread(target=rospy.spin)
        ros_thread.daemon = True
        
        # ROS düğümünü ve thread'i başlat
        drone_manager.camera_handler.init_ros_node()
        ros_thread.start()

        app.run(host=APP_HOST, port=APP_PORT, threaded=True, debug=True, use_reloader=False)

    except rospy.ROSInterruptException:
        print("ROS düğümü durduruldu.")
    except Exception as e:
        print(f"Uygulama başlatılırken bir hata oluştu: {e}")