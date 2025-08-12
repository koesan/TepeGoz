# config.py

# Dron Sabitleri ve Ayarlar
TAKEOFF_ALTITUDE = 10
CRITICAL_BATTERY_LEVEL = 30
MAX_TOUR_COUNT = 1
SQUARE_SIZE = 20
DRONE_SPEED = 90

FOREST_FIRE_DETECTION_MODEL = "./models/fire_m.pt"
# Kamera ve ROS Ayarları
CAMERA_TOPICS = {
    5763: "/webcam/image_raw",
    5753: "/webcam_2/image_raw_2"
}