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

import threading
import time
from dronekit import VehicleMode
from config import (
    CRITICAL_BATTERY_LEVEL, RTL_LANDING_TIMEOUT_S,
    MONITOR_INTERVAL_S, MISSION_STATUS_MESSAGES
)

class DroneSwitcher:
    """
    DroneSwitcher sınıfı, aktif dronu sürekli izler ve gerektiğinde
    diğer bağlı dronlardan birine geçiş yapar.
    """

    def __init__(self, drone_manager, mission_controller):
        # Drone yöneticisi ve görev kontrolcüsü referansları
        self.drone_manager = drone_manager
        self.mission_controller = mission_controller

        # İlk anda bağlı dronların portlarını al (liste)
        self.drone_ports = list(self.drone_manager.drones.keys())

        # Şu anki drone index’i (self.drone_ports listesine göre)
        self.current_drone_index = 0

        # İzleme thread durumu
        self.is_monitoring_active = False
        self._monitor_thread = None

        # İç state: görev süresi ve son görev noktası
        self.last_mission_point = None
        self.mission_start_time = None   # ⏱ süre sayacı

        print(f"Drone Switcher başlatıldı. Toplam {len(self.drone_ports)} dron mevcut.")

    # Görev başladığında çağrılır → sayaç başlatılır
    def notify_mission_started(self):
        self.mission_start_time = time.time()
        if not self.is_monitoring_active:
            self.start_monitoring()
        print("[DroneSwitcher] Görev başlangıç zamanı kaydedildi.")

    # Görev durduğunda çağrılır → sayaç sıfırlanır
    def notify_mission_stopped(self):
        self.stop_monitoring()
        self.mission_start_time = None
        print("[DroneSwitcher] Görev durdu, zaman sayacı sıfırlandı.")

    # İzleme thread’ini başlat
    def start_monitoring(self):
        if not self.is_monitoring_active:
            self.is_monitoring_active = True
            self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self._monitor_thread.start()
            print(MISSION_STATUS_MESSAGES["MONITOR_STARTED"])

    # İzleme thread’ini durdur
    def stop_monitoring(self):
        self.is_monitoring_active = False
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join()
        print(MISSION_STATUS_MESSAGES["MONITOR_STOPPED"])

    # Ana izleme döngüsü → batarya ve süre kontrolü
    def _monitor_loop(self):
        while self.is_monitoring_active:
            if self.drone_manager.active_drone and self.mission_controller.is_mission_active:
                vehicle = self.drone_manager.active_drone
                try:
                    # Batarya seviyesini oku
                    batt = getattr(vehicle, 'battery', None)
                    batt_level = batt.level if batt is not None and hasattr(batt, 'level') else None

                    # ⏱ Görev süresi kontrolü (5 dk)
                    time_exceeded = False
                    if self.mission_start_time:
                        elapsed = time.time() - self.mission_start_time
                        if elapsed >= 300:  # 5 dakika
                            time_exceeded = True
                            print(f"[DroneSwitcher] {self.drone_manager.active_drone_port} için 5 dk doldu ({elapsed:.1f}s). Drone değiştirilecek.")

                    # 🔋 Kritik batarya veya süre dolmuşsa → Drone değiştir
                    if (batt_level is not None and batt_level < CRITICAL_BATTERY_LEVEL) or time_exceeded:
                        # Kritik batarya mesajı
                        if batt_level is not None and batt_level < CRITICAL_BATTERY_LEVEL:
                            message = MISSION_STATUS_MESSAGES["CRITICAL_BATTERY"].format(
                                port=self.drone_manager.active_drone_port,
                                batt_level=batt_level
                            )
                            self.drone_manager.mission_status_message = message
                            print(message)

                        # Görevi durdur ve mevcut noktayı kaydet
                        self.mission_controller.is_mission_active = False
                        self.last_mission_point = self.mission_controller.current_mission_point

                        # RTL başlat
                        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["RTL_STARTED"]
                        vehicle.mode = VehicleMode('RTL')
                        time.sleep(RTL_LANDING_TIMEOUT_S)

                        # Yeni drona geç
                        self.switch_to_next_drone()

                except Exception as e:
                    # İzleme sırasında oluşan hata
                    message = MISSION_STATUS_MESSAGES["MONITOR_ERROR"].format(error=e)
                    self.drone_manager.mission_status_message = message
                    print(message)

            time.sleep(MONITOR_INTERVAL_S)

    # Bir sonraki drone’a geçiş yap
    def switch_to_next_drone(self):
        # Güncel bağlı dron listesini al
        drone_ports = list(self.drone_manager.drones.keys())
        if not drone_ports:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NO_DRONES_FOUND"]
            print(MISSION_STATUS_MESSAGES["NO_DRONES_FOUND"])
            return

        # Bir sonraki drone index’i hesapla
        self.current_drone_index = (self.current_drone_index + 1) % len(drone_ports)
        next_port = drone_ports[self.current_drone_index]

        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["SWITCH_TO_NEXT"]
        print(MISSION_STATUS_MESSAGES["SWITCH_TO_NEXT"])

        # Yeni drone nesnesini al
        next_drone = self.drone_manager.drones.get(next_port)
        if next_drone:
            # Yeni dronu aktif olarak ayarla
            self.drone_manager.active_drone = next_drone
            self.drone_manager.active_drone_port = next_port

            message = MISSION_STATUS_MESSAGES["NEXT_DRONE_SELECTED"].format(port=next_port)
            self.drone_manager.mission_status_message = message
            print(message)

            # Görev tipini devret
            handover_mission_type = self.mission_controller.current_mission_type
            if handover_mission_type:
                self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["HANDOVER_MISSION_TYPE"].format(
                    mission_type=handover_mission_type
                )
                print(self.drone_manager.mission_status_message)

            # Eğer kaldığı nokta varsa oradan devam et
            if self.last_mission_point:
                print(MISSION_STATUS_MESSAGES["RESUMING_FROM_POINT"].format(point=self.last_mission_point))
                self.mission_controller.start_mission(
                    mission_type=handover_mission_type,
                    resume_point=self.last_mission_point
                )
            else:
                # Aksi halde görev baştan başlatılır
                self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NO_LAST_POINT"]
                print(MISSION_STATUS_MESSAGES["NO_LAST_POINT"])
                self.mission_controller.start_mission(mission_type=handover_mission_type)

            # ⏱ Yeni drona geçildiğinde süre sıfırlanır
            self.mission_start_time = time.time()
        else:
            # Yeni drone nesnesi bulunamazsa hata mesajı
            message = MISSION_STATUS_MESSAGES["NO_DRONE_OBJECT"].format(port=next_port)
            self.drone_manager.mission_status_message = message
            print(message)
