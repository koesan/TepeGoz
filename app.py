from flask import Flask, render_template, request, jsonify, Response
import threading
import os
import rospy
from modules.drone_manager import DroneManager
from modules.mission_controller import MissionController
import time

app = Flask(__name__, template_folder='templates')

# Modül sınıflarını başlatıyoruz
drone_manager = DroneManager()
mission_controller = MissionController(drone_manager)
active_drone_port = None
last_detection_results = []
last_detection_time = 0

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/connect_drone', methods=['POST'])
def connect_drone():
    data = request.json
    connection_string = data.get('connection_string')
    return jsonify(drone_manager.start_connection(connection_string))

@app.route('/select_drone', methods=['POST'])
def select_drone():
    port = request.json.get('port')
    return jsonify(drone_manager.select_drone(port))

@app.route('/set_area', methods=['POST'])
def set_area():
    data = request.json
    coordinates = data.get('coordinates')
    return jsonify(mission_controller.set_area(coordinates))

@app.route('/start_mission', methods=['POST'])
def start_mission():
    data = request.json
    mission_type = data.get('mission_type')
    return jsonify(mission_controller.start_mission(mission_type))

@app.route('/stop_mission', methods=['POST'])
def stop_mission():
    return jsonify(mission_controller.stop_mission())

@app.route('/status')
def status():
    global last_detection_time, last_detection_results

    drone_status = drone_manager.get_status()
    mission_status = mission_controller.get_status()

    detections = drone_manager.camera_handler.get_latest_detection_results()
    if detections:
        last_detection_time = time.time()
        last_detection_results = detections
    elif time.time() - last_detection_time <= 60:
        detections = last_detection_results
    else:
        detections = []

    # NumPy array'leri listeye çevir
    safe_detections = []
    for det in detections:
        safe_detections.append({
            "box": det["box"].tolist() if hasattr(det["box"], "tolist") else det["box"],
            "score": det["score"],
            "class_id": det["class_id"],
            "class_name": det["class_name"]
        })

    if 'grid_status' in mission_status:
        mission_status['grid_status'] = {
            f"{k[0]},{k[1]}" if isinstance(k, tuple) else k: v
            for k, v in mission_status['grid_status'].items()
        }

    combined_status = {**drone_status, **mission_status}
    combined_status["mission_status_message"] = drone_manager.mission_status_message
    combined_status["detection_results"] = safe_detections or []

    return jsonify(combined_status)

@app.route('/camera_feed')
def camera_feed():
    port = drone_manager.active_drone_port
    if port is None:
        return "Kamera beslemesi yok", 404
    return Response(drone_manager.camera_handler.generate_frames(port),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        ros_thread = threading.Thread(target=rospy.spin)
        ros_thread.daemon = True
        
        drone_manager.camera_handler.init_ros_node()
        ros_thread.start()

        app.run(host='0.0.0.0', port=5000, threaded=True, debug=True, use_reloader=False)

    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except Exception as e:
        print(f"Uygulama başlatılırken bir hata oluştu: {e}")
