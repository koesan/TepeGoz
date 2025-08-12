# modules/drone_manager.py

import threading
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import rospy
from .camera_ai import CameraAIHandler 

# Dron bağlantılarını ve durum takibini yapar.

class DroneManager:
    
    def __init__(self):
        self.drones = {}                                        # Bağlı dronlar: port -> vehicle objesi
        self.active_drone = None                                # Şu an kontrol edilen dron
        self.active_drone_port = None                           # Aktif dronun port numarası
        self.connection_status = "not_connected"  
        self.mission_status_message = "Bağlanmayı bekliyor..."
        self.camera_handler = CameraAIHandler()                 # Kamera verilerini yöneten nesne

    # Verilen port üzerinden dron bağlantısını arka planda kurar.
    def connect_drone_async(self, connection_string, port):
        
        try:
            corrected_connection_string = f"tcp:127.0.0.1:{port}"
            print(f"Drona {corrected_connection_string} üzerinden bağlanılıyor...")
            vehicle = connect(corrected_connection_string, wait_ready=True, timeout=60)  # Bağlan ve hazır olana kadar bekle
            self.drones[port] = vehicle

            # Eğer aktif dron yoksa ilk bağlananı seç
            if not self.active_drone:
                self.active_drone = vehicle
                self.active_drone_port = port

            print(f"Başarılı: Dron {port} bağlandı.")
            self.connection_status = "connected"
            self.mission_status_message = "Dron bağlandı. Görev için hazır."

            # Kameradan görüntü almak için abone ol
            self.camera_handler.subscribe_to_camera_topic_for_port(port)

        except Exception as e:
            print(f"Bağlantı hatası: {e}")
            self.drones.pop(port, None)
            self.connection_status = "not_connected"
            self.mission_status_message = "Bağlantı hatası. Tekrar deneyin."
        finally:
            if not self.drones:
                self.connection_status = "not_connected"
                self.mission_status_message = "Bağlanmayı bekliyor..."

    # Dışardan gelen bağlantı isteğini başlatır.
    def start_connection(self, connection_string):
        
        try:
            extracted_port = int(connection_string.split(':')[-1])
            # Aynı portta zaten bağlı ya da bağlanıyor mu kontrol et
            if extracted_port in self.drones and self.drones[extracted_port] != "connecting":
                return {"status": "error", "message": f"Bu porta bağlı bir dron zaten var: {extracted_port}"}

            self.drones[extracted_port] = "connecting"
            # Bağlantıyı ayrı bir thread'de başlat
            threading.Thread(target=self.connect_drone_async, args=(connection_string, extracted_port), daemon=True).start()

            return {"status": "ok", "message": f"Drona {connection_string} üzerinden bağlanılıyor..."}
        except (IndexError, ValueError):
            return {"status": "error", "message": "Geçersiz bağlantı adresi formatı."}

    # Aktif olarak kontrol edilecek dronu seçer.
    def select_drone(self, port):
        
        try:
            port = int(port)
            if port in self.drones and self.drones[port] != "connecting":
                self.active_drone = self.drones[port]
                self.active_drone_port = port
                print(f"Dron {port} aktif dron olarak seçildi.")
                return {"status": "ok", "message": f"Dron {port} seçildi."}
            return {"status": "error", "message": "Geçersiz dron seçimi veya dron hala bağlanıyor."}
        except (ValueError, TypeError):
            return {"status": "error", "message": "Geçersiz port numarası."}

    # Aktif dron ve bağlantı durumu hakkında bilgi verir.
    def get_status(self):
        
        connected_drones_list = []
        for port, vehicle_obj in self.drones.items():
            if vehicle_obj != "connecting":
                connected_drones_list.append({
                    "port": port,
                    "is_active": (self.active_drone is not None and self.active_drone == vehicle_obj)
                })
            else:
                connected_drones_list.append({"port": port, "is_active": False})

        # Eğer aktif dron yoksa veya hala bağlanıyorsa temel durum bilgisi döner
        if not self.active_drone or self.active_drone == "connecting":
            return {
                "status": "not_connected",
                "connected_drones": connected_drones_list,
                "is_mission_active": False,
                "status_message": self.mission_status_message
            }

        # Aktif dronun konumu ve durumu
        current_location = {"lat": 0, "lon": 0, "alt": 0}
        ground_speed = 0
        if hasattr(self.active_drone, 'location') and self.active_drone.location.global_relative_frame:
            loc = self.active_drone.location.global_relative_frame
            current_location = {"lat": loc.lat, "lon": loc.lon, "alt": loc.alt}
            ground_speed = self.active_drone.ground_speed if hasattr(self.active_drone, 'ground_speed') else 0

        return {
            "status": "connected",                                                                # Burada dronun bağlı olduğunu belirtir.  
            "current_location": current_location,                                                 # Dronun şu anki konumu (lat,lon).
            "ground_speed": ground_speed,                                                         # Dronun yere göre hız bilgisi.
            "battery_level": self.active_drone.battery.level if self.active_drone.battery else 0, # Dronun batarya şarj durumu (yüzde olarak).
            "mode": self.active_drone.mode.name if self.active_drone.mode else "UNKNOWN",         # Dronun uçuş modu (örneğin: GUIDED, AUTO, MANUAL).
            "is_armed": self.active_drone.armed,                                                  # Dronun motorlarının aktif olup olmadığı.
            "connected_drones": connected_drones_list,                                            # Sisteme bağlı tüm dronların listesi ve her birinin aktif olup olmadığı bilgisi.
            "is_mission_active": False,                                                           # Görev aktifliği başka yerden güncellenecek  
            "status_message": self.mission_status_message                                         # Dronun veya sistemin genel durum mesajı (örneğin "Dron bağlandı").
        }
