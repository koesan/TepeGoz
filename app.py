# app.py

from flask import Flask, render_template, request, jsonify, Response
import threading
import os
import rospy
from modules.drone_manager import DroneManager
from modules.mission_controller import MissionController
from modules.camera_ai import CameraAIHandler

app = Flask(__name__, template_folder='templates')

# Modül sınıflarını başlatıyoruz
drone_manager = DroneManager()
mission_controller = MissionController(drone_manager)
camera_handler = CameraAIHandler()
active_drone_port = None

@app.route('/')
def index():
    # Ana sayfa: Dron arayüzünü yükler.
    return render_template('index.html')

@app.route('/connect_drone', methods=['POST'])
def connect_drone():
    # Web arayüzünden gelen bağlantı isteğini işler.
    data = request.json
    connection_string = data.get('connection_string')
    return jsonify(drone_manager.start_connection(connection_string))

@app.route('/select_drone', methods=['POST'])
def select_drone():
    # Web arayüzünden gelen dron seçimi isteğini işler.
    port = request.json.get('port')
    return jsonify(drone_manager.select_drone(port))

@app.route('/set_area', methods=['POST'])
def set_area():
    # Web arayüzünden gelen koordinatları alır.
    data = request.json
    coordinates = data.get('coordinates')
    return jsonify(mission_controller.set_area(coordinates))

@app.route('/start_mission', methods=['POST'])
def start_mission():
    # Görevi başlatma isteğini işler.
    return jsonify(mission_controller.start_mission())

@app.route('/stop_mission', methods=['POST'])
def stop_mission():
    # Görevi durdurma isteğini işler.
    return jsonify(mission_controller.stop_mission())

@app.route('/status')
def status():
    # Web arayüzü için anlık dron ve görev durumunu döndürür.
    drone_status = drone_manager.get_status()
    mission_status = mission_controller.get_status()
    
    # Grid_status sözlüğündeki tuple anahtarları string'e dönüştürme
    if 'grid_status' in mission_status:
        json_ready_grid_status = {}
        for key, value in mission_status['grid_status'].items():
            # Tuple anahtarı, "lat,lon" gibi bir string'e çevrilir
            if isinstance(key, tuple):
                json_ready_grid_status[f"{key[0]},{key[1]}"] = value
            else:
                json_ready_grid_status[key] = value
        mission_status['grid_status'] = json_ready_grid_status
    
    # İki durum bilgisini birleştirme
    combined_status = {**drone_status, **mission_status}
    combined_status["mission_status_message"] = drone_manager.mission_status_message
    
    return jsonify(combined_status)

@app.route('/camera_feed')
def camera_feed():
    port = drone_manager.active_drone_port
    if port is None:
        return "Kamera beslemesi yok", 404
    return Response(camera_handler.generate_frames(port),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


        
    return Response(camera_handler.generate_frames(active_drone_port), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # ROS düğümünü ve listener'ı ayrı bir thread'de başlat
        ros_thread = threading.Thread(target=rospy.spin)
        ros_thread.daemon = True
        
        camera_handler.init_ros_node()
        ros_thread.start()

        # Flask uygulamasını başlat
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=True)

    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except Exception as e:
        print(f"Uygulama başlatılırken bir hata oluştu: {e}")