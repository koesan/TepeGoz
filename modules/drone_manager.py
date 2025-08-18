"""
============================================================
TepeGöz - Gözlem Drone Yönetim Sistemi - Drone Switcher Modülü
============================================================

Yapan / Author : Barış Enes Kümet
Tarih / Date   : 18.08.2025
Modül Adı      : drone_manager.py

Açıklama (Türkçe):
------------------
Bu modül, birden fazla drone'un bağlantısını, durum takibini ve yönetimini üstlenen ana birimdir. DroneKit kütüphanesini kullanarak dronlar ile iletişim kurar, aktif dronu belirler ve uçuş verilerini (konum, hız, batarya seviyesi vb.) anlık olarak sağlar. Ayrıca, kamera ve yapay zeka modülü ile entegre çalışır.

Özet Fonksiyonellik:
--------------------
1. Drone'lara TCP/IP üzerinden eş zamansız (asenkron) bağlantı kurar ve bağlantı durumunu yönetir.
2. Birden fazla bağlı drone arasından aktif olarak kontrol edilecek olanı seçer.
3. Aktif drone'un anlık konum, batarya seviyesi, uçuş modu gibi telemetri verilerini sağlar.
4. Kamera ve yapay zeka modülü ile entegrasyonu yönetir.

---

Description (English):
----------------------
This module is the core unit responsible for managing the connections, status, and control of multiple drones. It uses the DroneKit library to communicate with the drones, selects the active drone, and provides real-time flight data (location, speed, battery level, etc.). It also works in an integrated manner with the camera and AI module.

Summary of Functionality:
-------------------------
1. Establishes and manages asynchronous TCP/IP connections to drones.
2. Selects the active drone to be controlled from among multiple connected drones.
3. Provides real-time telemetry data for the active drone, such as current location, battery level, and flight mode.
4. Manages the integration with the camera and AI module.
"""

import threading
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import rospy
from .camera_ai import CameraAIHandler
from config import (
    CONNECTION_TIMEOUT,
    DRONE_CONNECTION_RETRY_COUNT,
    CONNECTION_STATUS_CONNECTED,
    CONNECTION_STATUS_NOT_CONNECTED,
    MISSION_STATUS_MESSAGES
)

# Dron bağlantılarını ve durum takibini yönetir
class DroneManager:
    
    def __init__(self):
        self.drones = {} # Bağlı dronlar: port -> vehicle objesi veya "connecting" durumu
        self.active_drone = None # Şu an kontrol edilen dron
        self.active_drone_port = None # Aktif dronun port numarası
        self.connection_status = CONNECTION_STATUS_NOT_CONNECTED # Genel bağlantı durumu
        self.mission_status_message = MISSION_STATUS_MESSAGES["NOT_CONNECTED"] # Arayüze gösterilecek durum mesajı
        self.camera_handler = CameraAIHandler() # Kamera ve yapay zeka işlemlerini yöneten nesne

    # Verilen port üzerinden dron bağlantısını arka planda kurar.
    def connect_drone_async(self, connection_string, port):
        
        corrected_connection_string = f"tcp:127.0.0.1:{port}"
        vehicle = None

        for i in range(DRONE_CONNECTION_RETRY_COUNT):
            try:
                # Dinamik bağlantı mesajını göster
                self.mission_status_message = MISSION_STATUS_MESSAGES["RETRYING"].format(current=i + 1, total=DRONE_CONNECTION_RETRY_COUNT)
                print(self.mission_status_message)
                
                vehicle = connect(corrected_connection_string, wait_ready=True, timeout=CONNECTION_TIMEOUT)
                
                # Bağlantı başarılıysa döngüden çık
                break
            except Exception as e:
                print(f"Bağlantı hatası (Deneme {i + 1}): {e}")
                
                if i < DRONE_CONNECTION_RETRY_COUNT - 1:
                    time.sleep(5)
                else:
                    print(MISSION_STATUS_MESSAGES["ALL_RETRIES_FAILED"])

        # Döngüden çıktıktan sonra bağlantının başarılı olup olmadığını kontrol et
        if vehicle:
            self.drones[port] = vehicle
            if not self.active_drone:
                self.active_drone = vehicle
                self.active_drone_port = port
            print(f"Başarılı: Dron {port} bağlandı.")
            self.connection_status = CONNECTION_STATUS_CONNECTED
            self.mission_status_message = MISSION_STATUS_MESSAGES["CONNECTED"]
            self.camera_handler.subscribe_to_camera_topic_for_port(port)
        else:
            self.drones.pop(port, None)
            self.connection_status = CONNECTION_STATUS_NOT_CONNECTED
            self.mission_status_message = MISSION_STATUS_MESSAGES["CONNECTION_ERROR"]
            if not self.drones:
                self.connection_status = CONNECTION_STATUS_NOT_CONNECTED
                self.mission_status_message = MISSION_STATUS_MESSAGES["NOT_CONNECTED"]
    
    # Dışarıdan gelen bağlantı isteğini ayrı bir thread'de başlatır.
    def start_connection(self, connection_string):
        
        try:
            extracted_port = int(connection_string.split(':')[-1])
            if extracted_port in self.drones and self.drones[extracted_port] != "connecting":
                return {"status": "error", "message": f"Bu porta bağlı bir dron zaten var: {extracted_port}"}

            self.drones[extracted_port] = "connecting"
            threading.Thread(target=self.connect_drone_async, args=(connection_string, extracted_port), daemon=True).start()
            
            return {"status": "ok", "message": MISSION_STATUS_MESSAGES["CONNECTING"]}
        except (IndexError, ValueError):
            return {"status": "error", "message": "Geçersiz bağlantı adresi formatı."}


    # Aktif olarak kontrol edilecek dronu seçer
    def select_drone(self, port):
        
        try:
            port = int(port)
            if port in self.drones and self.drones[port] != "connecting":
                self.active_drone = self.drones[port]
                self.active_drone_port = port
                print(f"Dron {port} aktif dron olarak seçildi.")
                print("bağlı dornlar")
                print(self.drones)
                return {"status": "ok", "message": f"Dron {port} seçildi."}
            return {"status": "error", "message": "Geçersiz dron seçimi veya dron hala bağlanıyor."}
        except (ValueError, TypeError):
            return {"status": "error", "message": "Geçersiz port numarası."}

    # Aktif dron ve tüm bağlı dronlar hakkında detaylı durum bilgisi döndürür.
    def get_status(self):

        connected_drones_list = []
        for port, vehicle_obj in self.drones.items():
            connected_drones_list.append({
                "port": port,
                "is_active": (self.active_drone_port == port) if vehicle_obj != "connecting" else False
            })

        if not self.active_drone or self.active_drone == "connecting":
            return {
                "status": CONNECTION_STATUS_NOT_CONNECTED,
                "connected_drones": connected_drones_list,
                "is_mission_active": False,
                "status_message": self.mission_status_message
            }

        current_location = {"lat": 0, "lon": 0, "alt": 0}
        ground_speed = 0
        if hasattr(self.active_drone, 'location') and self.active_drone.location.global_relative_frame:
            loc = self.active_drone.location.global_relative_frame
            current_location = {"lat": loc.lat, "lon": loc.lon, "alt": loc.alt}
            ground_speed = self.active_drone.ground_speed if hasattr(self.active_drone, 'ground_speed') else 0

        return {
            "status": CONNECTION_STATUS_CONNECTED,
            "current_location": current_location,
            "ground_speed": ground_speed,
            "battery_level": self.active_drone.battery.level if hasattr(self.active_drone, 'battery') and self.active_drone.battery else 0,
            "mode": self.active_drone.mode.name if hasattr(self.active_drone, 'mode') and self.active_drone.mode else "UNKNOWN",
            "is_armed": self.active_drone.armed if hasattr(self.active_drone, 'armed') else False,
            "connected_drones": connected_drones_list,
            "is_mission_active": False,
            "status_message": self.mission_status_message
        }