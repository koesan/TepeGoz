"""
============================================================
TepeGöz - Gözlem Drone Yönetim Sistemi - Görev Kontrol Modülü
============================================================

Yapan / Author : Barış Enes Kümet
Tarih / Date   : 18.08.2025
Modül Adı      : mission_controller.py

Açıklama (Türkçe):
------------------
Bu modül, bir veya daha fazla drone için gözlem görevlerini 
planlar, yönetir ve dronların otonom hareketini kontrol eder. 
Önceden belirlenen bir alandaki hücreleri sistematik bir 
şekilde (boustrophedon deseni) tarayarak gözlem yapar.

Özet Fonksiyonellik:
--------------------
1. PID kontrolü kullanarak dronu hedeflenen GPS noktalarına yönlendirir.
2. Haversine formülü ile GPS noktaları arasındaki mesafeyi hesaplar.
3. Önceden belirlenen bir alanı bir ızgara (grid) yapısına böler.
4. Dronu bu ızgara üzerinde boustrophedon (yılan) deseni ile hareket ettirir.
5. Görev sırasındaki dron durumu (batarya, konum, hız) ve ızgara ilerleyişini izler.
6. Takılma (stuck) ve zaman aşımı (timeout) gibi durumlarda otomatik kurtarma manevraları dener.
7. Görev başlatma, durdurma ve durum sorgulama gibi API işlevlerini sunar.
8. `DroneSwitcher` modülü ile entegre çalışarak kritik durumlarda drone değişimini yönetir.

---

Description (English):
----------------------
This module plans and manages observation missions for one or more 
drones and controls their autonomous movement. It systematically 
scans a predefined area by traversing a grid in a boustrophedon 
(snake) pattern.

Summary of Functionality:
-------------------------
1. Guides the drone to target GPS points using a PID controller.
2. Calculates the distance between GPS points using the Haversine formula.
3. Divides a predefined area into a grid structure.
4. Moves the drone along this grid using a boustrophedon (snake) pattern.
5. Monitors the drone's status (battery, location, speed) and mission progress.
6. Attempts automatic recovery maneuvers in case of stuck or timeout situations.
7. Provides API functions for starting, stopping, and querying mission status.
8. Works in integration with the `DroneSwitcher` module to manage drone switching in critical situations.
"""

import threading
import time
import math
from collections import deque
from statistics import median
from dronekit import VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from modules.drone_switcher import DroneSwitcher
from config import (
    TAKEOFF_ALTITUDE, CRITICAL_BATTERY_LEVEL, SQUARE_SIZE, DRONE_SPEED,
    MAX_SPEED_M_S, CONTROL_INTERVAL_S, CELL_REACHED_THRESHOLD_M,
    FINE_APPROACH_THRESHOLD_M, FINE_APPROACH_SCALE, CENTER_CONFIRM_TOLERANCE_M,
    CENTER_CONFIRM_HOLD_S, FINE_APPROACH_HOLD_S, PER_CELL_TIMEOUT_S,
    RETRY_LIMIT, GPS_MEDIAN_WINDOW, STUCK_MOVED_THRESHOLD_M, STUCK_TIMEOUT_S,
    PID_KP, PID_KI, PID_KD,
    MISSION_STATUS_MESSAGES
)

# PID kontrolcüsü, dronun hedefe doğru hareketini ayarlar
class PID:

    def __init__(self, kp=PID_KP, ki=PID_KI, kd=PID_KD, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = integral_limit

    # PID kontrolcüsünü sıfırlar.
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    # Gelen hataya göre yeni kontrol çıktısı hesaplar.
    def step(self, error, dt):

        if dt <= 0:
            return 0.0
        self.integral += error * dt
        # anti-windup
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        deriv = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * deriv

# Gözlem görevlerini planlar, yönetir ve dronu kontrol eder.
class MissionController:
    
    def __init__(self, drone_manager):
        self.drone_manager = drone_manager
        self.current_mission_point = None
        self.current_mission_type = None
        self.mission_grid = None
        self.mission_grid_indices = None
        self.mission_coordinates = None
        self.centers_2d = []
        self.grid_status = {}
        self.num_rows = 0
        self.num_cols = 0
        self.is_mission_active = False
        self.mission_path_points = []
        self._mission_thread = None
        self.camera_handler = self.drone_manager.camera_handler
        self.drone_switcher = DroneSwitcher(drone_manager, self)

    # --- yardımcı fonksiyonlar ---

    # İki GPS noktası arasındaki mesafeyi Haversine formülüyle hesaplar.
    def _haversine_distance_m(self, lat1, lon1, lat2, lon2):

        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    # İki GPS noktası arasındaki kuzey-doğu farkını metre cinsinden döndürür.
    def _latlon_to_north_east_m(self, lat1, lon1, lat2, lon2):
    
        d_north = (lat2 - lat1) * 111320.0
        mean_lat = math.radians((lat1 + lat2) / 2.0)
        d_east = (lon2 - lon1) * 111320.0 * math.cos(mean_lat)
        return d_north, d_east

    # Dronu North, East, Down eksenlerinde hız komutuyla hareket ettirir.
    def send_ned_velocity(self, vehicle, vx, vy, vz=0):

        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
        try:
            vehicle.send_mavlink(msg)
            vehicle.flush()
        except Exception as e:
            print('send_ned_velocity hata:', e)

    # Verilen koordinatlara göre gözlem gridini oluşturur.
    def create_grid(self, cell_centers):
    
        self.mission_coordinates = cell_centers
        self.grid_status = {}
        self.centers_2d = []
        if not cell_centers:
            self.num_rows = self.num_cols = 0
            return
        total = len(cell_centers)
        num_cols = int(math.sqrt(total)) or 1
        num_rows = math.ceil(total / num_cols)
        self.num_rows, self.num_cols = num_rows, num_cols
        self.centers_2d = [[None for _ in range(num_cols)] for _ in range(num_rows)]
        idx = 0
        for r in range(num_rows):
            for c in range(num_cols):
                if idx < total:
                    lat, lon = cell_centers[idx]
                    self.centers_2d[r][c] = (lat, lon)
                    self.grid_status[(r, c)] = 'unvisited'
                    idx += 1
                else:
                    self.centers_2d[r][c] = None
        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["AREA_SET"].format(rows=num_rows, cols=num_cols)
        print(f"Grid oluşturuldu: {num_rows}x{num_cols} (row x col). Alt-sol'dan başlayacak.")

    # Boustrophedon (yılan deseni) taramasına göre bir sonraki hücreyi bulur.
    def _next_indices(self, row, col, horiz_dir, vert_dir):
        """
        789
        456  --> 1-2-3-6-5-4-7-8-9-6-5-4-1-2-3 şeklinde sürekli gidiş-dönüş.
        123
        """
        next_col = col + horiz_dir

        # Satır içinde ilerleyebiliyorsak
        if 0 <= next_col < self.num_cols and self.centers_2d[row][next_col] is not None:
            return row, next_col, horiz_dir, vert_dir

        # Satır bitti → bir alt/üst satıra geç
        next_row = row + vert_dir
        if 0 <= next_row < self.num_rows:
            horiz_dir *= -1  # Satır yönünü ters çevir
            new_col = 0 if horiz_dir == 1 else self.num_cols - 1
            return next_row, new_col, horiz_dir, vert_dir

        # Grid sınırına ulaşıldı → vert_dir yönünü ters çevir (gidiş/dönüş dönüşümü)
        vert_dir *= -1
        horiz_dir *= -1  # Yatay yön de tersine döner
        new_row = row + vert_dir
        new_col = 0 if horiz_dir == 1 else self.num_cols - 1

        return new_row, new_col, horiz_dir, vert_dir

    # GPS verilerini medyan filtre ile yumuşatır.
    def _get_median_location(self, vehicle, samples_deque):
    
        if not samples_deque:
            loc = vehicle.location.global_relative_frame
            return (loc.lat, loc.lon) if loc else (None, None)
        lats = [s[0] for s in samples_deque]
        lons = [s[1] for s in samples_deque]
        return median(lats), median(lons)

    # Dronun hedef merkezde stabil kalıp kalmadığını kontrol eder.
    def confirm_center_stability(self, vehicle, target_lat, target_lon, hold_s=None, tol_m=None):

        hold_s = CENTER_CONFIRM_HOLD_S if hold_s is None else hold_s
        tol_m = CENTER_CONFIRM_TOLERANCE_M if tol_m is None else tol_m
        start = time.time()
        while time.time() - start < hold_s:
            if not self.is_mission_active:
                return False
            loc = vehicle.location.global_relative_frame
            if not loc:
                return False
            d = self._haversine_distance_m(loc.lat, loc.lon, target_lat, target_lon)
            # sıfır hız göndermeyi sürdür
            self.send_ned_velocity(vehicle, 0, 0, 0)
            if d > tol_m:
                return False
            time.sleep(0.05)
        return True

    # Dronu bir hedefe PID kontrolü ile yaklaştırır.
    def _approach_target(self, vehicle, target_lat, target_lon, from_row, from_col, to_row, to_col):
        # retry loop
        for attempt in range(RETRY_LIMIT + 1):
            pid_n = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)
            pid_e = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)
            pid_n.reset(); pid_e.reset()

            samples = deque(maxlen=GPS_MEDIAN_WINDOW)
            start_ts = time.time()
            last_move_ts = time.time()
            last_pos = None

            while self.is_mission_active:
                # Batarya kontrolü artık drone_switcher tarafından yapılacak.

                loc = vehicle.location.global_relative_frame
                if loc and loc.lat is not None:
                    samples.append((loc.lat, loc.lon))

                if not samples:
                    time.sleep(CONTROL_INTERVAL_S)
                    continue

                # durum mesajını güncelle
                filt_lat, filt_lon = self._get_median_location(vehicle, samples)
                d_north, d_east = self._latlon_to_north_east_m(filt_lat, filt_lon, target_lat, target_lon)
                dist = math.hypot(d_north, d_east)
                speed = math.hypot(vehicle.velocity[0], vehicle.velocity[1])

                # durum mesajını güncelle
                self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["APPROACHING"].format(from_row=from_row, from_col=from_col, to_col=to_col, to_row=to_row, distance=dist, speed=speed)

                # reached check
                if dist <= CELL_REACHED_THRESHOLD_M:
                    # stop and confirm with slightly longer hold
                    self.send_ned_velocity(vehicle, 0, 0, 0)
                    # extra fine hold
                    hold_ok = self.confirm_center_stability(
                        vehicle, target_lat, target_lon, hold_s=FINE_APPROACH_HOLD_S, tol_m=CELL_REACHED_THRESHOLD_M)
                    if hold_ok:
                        return True
                    else:
                        # başarısızsa PID reset ve tekrar dene
                        pid_n.reset(); pid_e.reset()
                        time.sleep(0.12)
                        continue

                # PID compute using filtered north/east errors (target - current)
                vx = pid_n.step(d_north, CONTROL_INTERVAL_S)
                vy = pid_e.step(d_east, CONTROL_INTERVAL_S)

                # yakınlaşırken yavaşlama
                if dist < FINE_APPROACH_THRESHOLD_M:
                    vx *= FINE_APPROACH_SCALE
                    vy *= FINE_APPROACH_SCALE

                speed = math.hypot(vx, vy)
                if speed > MAX_SPEED_M_S:
                    scale = MAX_SPEED_M_S / speed
                    vx *= scale; vy *= scale

                try:
                    self.send_ned_velocity(vehicle, vx, vy, 0)
                except Exception as e:
                    print('Velocity gönderme hatası:', e)

                if last_pos is None and loc:
                    last_pos = (loc.lat, loc.lon)
                    last_move_ts = time.time()
                elif loc:
                    moved = self._haversine_distance_m(last_pos[0], last_pos[1], loc.lat, loc.lon)
                    if moved > STUCK_MOVED_THRESHOLD_M:
                        last_pos = (loc.lat, loc.lon); last_move_ts = time.time()
                    else:
                        if time.time() - last_move_ts > STUCK_TIMEOUT_S:
                            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["STUCK"]
                            print(MISSION_STATUS_MESSAGES["STUCK"])
                            try:
                                self.send_ned_velocity(vehicle, -vx * 0.4, -vy * 0.4, 0)
                                time.sleep(0.4)
                                self.send_ned_velocity(vehicle, vx * 1.1, vy * 1.1, 0)
                            except Exception as e:
                                print('Manevra hatası:', e)
                            last_move_ts = time.time()

                elapsed = time.time() - start_ts
                timeout = PER_CELL_TIMEOUT_S * (1 + attempt * 0.5)
                if elapsed > timeout:
                    print(MISSION_STATUS_MESSAGES["TIMEOUT"])
                    self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["TIMEOUT"]
                    break

                time.sleep(CONTROL_INTERVAL_S)

        return False

    def _run_mission_loop(self, mission_type, start_point=None):
        vehicle = self.drone_manager.active_drone
        if not vehicle or not self.centers_2d:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NO_AREA"]
            return

        try:
            self.mission_path_points = []
            self.mission_start_time = time.time()
            
            # Kalkış işlemi
            self.arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)
            self.is_mission_active = True
            
            # Görev başlangıç mesajı
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["BEGIN_MISSION_FLIGHT"]
            print(self.drone_manager.mission_status_message)
            
            # Grid tarama için başlangıç pozisyonları
            if start_point:
                cur_row, cur_col, horiz_dir, vert_dir = self._find_start_indices(start_point)
            else:
                cur_row = self.num_rows - 1
                cur_col = 0
                horiz_dir = 1
                vert_dir = -1
            
            prev_row, prev_col = None, None

            while self.is_mission_active:
                # Görevin o anki noktasını kaydet
                self.current_mission_point = (cur_row, cur_col, horiz_dir, vert_dir)

                # Hedef hücrenin mevcut koordinatlarını al
                target = self.centers_2d[cur_row][cur_col]
                
                # Eğer hücre boşsa, bir sonrakine geç
                if target is None:
                    # Önceki konumu güncelle
                    prev_row, prev_col = cur_row, cur_col
                    # Bir sonraki geçerli hücreyi bul
                    cur_row, cur_col, horiz_dir, vert_dir = self._next_indices(cur_row, cur_col, horiz_dir, vert_dir)
                    
                    # Eğer döngüden çıkış sinyali geldiyse (örn: -1, -1), döngüyü kır
                    if cur_row == -1 and cur_col == -1:
                        print('Tüm hücreler ziyaret edildi veya atlandı. Görev sonlandırılıyor.')
                        self.is_mission_active = False
                    continue

                lat, lon = target
                self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NAVIGATING_TO_CELL"].format(row=cur_row, col=cur_col)
                print(self.drone_manager.mission_status_message)

                # _approach_target metodunu doğru parametrelerle çağır
                ok = self._approach_target(vehicle, lat, lon, prev_row, prev_col, cur_row, cur_col)
                
                if ok:
                    self.grid_status[(cur_row, cur_col)] = 'visited'
                    self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["CELL_CONFIRMED"].format(row=cur_row, col=cur_col)
                    print(self.drone_manager.mission_status_message)
                    
                    # Kesin merkez doğrulama
                    stable = self.confirm_center_stability(vehicle, lat, lon, hold_s=CENTER_CONFIRM_HOLD_S, tol_m=CENTER_CONFIRM_TOLERANCE_M)
                    if not stable:
                        print('Center confirmation failed after visit — retrying approach')
                        continue
                else:
                    # Hücreye ulaşılamadıysa atlandı
                    self.grid_status[(cur_row, cur_col)] = 'old'
                    self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["CELL_SKIPPED"].format(row=cur_row, col=cur_col)
                    print(self.drone_manager.mission_status_message)

                # Bir sonraki hücreye geçmeden önce mevcut konumu "önceki konum" olarak ayarla
                prev_row, prev_col = cur_row, cur_col
                
                # Boustrophedon desenine göre bir sonraki hücreyi bul
                cur_row, cur_col, horiz_dir, vert_dir = self._next_indices(cur_row, cur_col, horiz_dir, vert_dir)
                
                # Eğer tüm hücreler ziyaret edildiyse veya atlandıysa döngüyü sonlandır
                if cur_row == -1 and cur_col == -1:
                    self.is_mission_active = False
                
                # Eğer satır değişecekse kısa bir duraklama yap
                if cur_row != prev_row:
                    target_next = self.centers_2d[cur_row][cur_col]
                    if target_next is not None:
                        time.sleep(0.12)

                time.sleep(0.08)

        except Exception as e:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["MISSION_ERROR"].format(error=str(e))
            print(self.drone_manager.mission_status_message)
            try:
                vehicle.mode = VehicleMode('RTL')
            except Exception:
                pass

        finally:
            if vehicle and self.is_mission_active:
                try:
                    self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["MISSION_COMPLETE"]
                    vehicle.mode = VehicleMode('RTL')
                except Exception:
                    pass
            self.is_mission_active = False
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["STOPPED"]

    # Kaldığı yerden devam etmek için başlangıç row, col ve yön bilgilerini bulur.
    def _find_start_indices(self, start_point):
        
        start_row, start_col, start_horiz_dir, start_vert_dir = start_point
        return start_row, start_col, start_horiz_dir, start_vert_dir

    # Dronu arm eder ve belirlenen yüksekliğe kalkış yaptırır.
    def arm_and_takeoff(self, vehicle, aTargetAltitude):
    
        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["ARMING"]
        print(MISSION_STATUS_MESSAGES["ARMING"])
        while not vehicle.is_armable:
            print('Waiting for armable...')
            time.sleep(1)
        vehicle.mode = VehicleMode('GUIDED')
        vehicle.armed = True
        while not vehicle.armed:
            print('Arming...')
            time.sleep(0.5)
        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["TAKING_OFF"].format(altitude=aTargetAltitude)
        print(self.drone_manager.mission_status_message)
        vehicle.simple_takeoff(aTargetAltitude)
        while True:
            alt = vehicle.location.global_relative_frame.alt
            print(f' Yükseklik: {alt:.2f}')
            if alt >= aTargetAltitude * 0.95:
                self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["TAKEOFF_SUCCESS"].format(altitude=aTargetAltitude)
                print(self.drone_manager.mission_status_message)
                break
            time.sleep(0.5)

        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["BEGIN_MISSION_FLIGHT"]
        print(self.drone_manager.mission_status_message)
        time.sleep(1)

    # ---start/stop/get_status/set_area---

    # Görev thread'ini başlatır
    def start_mission(self, mission_type, resume_point=None):

        # Aktif dron kontrolü
        if not self.drone_manager.active_drone:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NO_ACTIVE"]
            return {'status': 'error', 'message': MISSION_STATUS_MESSAGES["NO_ACTIVE"]}

        # Görev tipini kaydet
        self.current_mission_type = mission_type

        # Kamera işleyicisinde görev tipini ayarla
        if not self.camera_handler.set_mission_type(mission_type):
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["INVALID_TYPE"].format(mission_type=mission_type)
            return {"status": "error", "message": MISSION_STATUS_MESSAGES["INVALID_TYPE"].format(mission_type=mission_type)}

        # Görevin zaten aktif olup olmadığını kontrol et
        if self.is_mission_active:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["ALREADY_ACTIVE"]
            return {"status": "error", "message": MISSION_STATUS_MESSAGES["ALREADY_ACTIVE"]}

        # Görev koordinatlarının ayarlanıp ayarlanmadığını kontrol et
        if not self.mission_coordinates:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NO_AREA"]
            return {'status': 'error', 'message': MISSION_STATUS_MESSAGES["NO_AREA"]}

        # Görev thread’ini oluştur
        if resume_point:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["RESUMING"]
            self._mission_thread = threading.Thread(
                target=self._run_mission_loop,
                args=(mission_type, resume_point),
                daemon=True
            )
        else:
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["STARTING"]
            self._mission_thread = threading.Thread(
                target=self._run_mission_loop,
                args=(mission_type,),
                daemon=True
            )

        self._mission_thread.start()

        # ✅ DroneSwitcher’a haber ver
        if hasattr(self, "drone_switcher") and self.drone_switcher:
            self.drone_switcher.notify_mission_started()

        return {'status': 'ok', 'message': self.drone_manager.mission_status_message}

    # Aktif görevi durdurur ve dronu RTL moduna geçirir
    def stop_mission(self):
        if self.drone_manager.active_drone and self.is_mission_active:
            try:
                self.drone_manager.active_drone.mode = VehicleMode('RTL')
            except Exception:
                pass

            self.is_mission_active = False
            self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["STOPPED"]

            # ✅ DroneSwitcher’a haber ver
            if hasattr(self, "drone_switcher") and self.drone_switcher:
                self.drone_switcher.notify_mission_stopped()

            return {'status': 'ok', 'message': MISSION_STATUS_MESSAGES["STOPPED"]}

        self.drone_manager.mission_status_message = MISSION_STATUS_MESSAGES["NO_ACTIVE"]
        return {'status': 'error', 'message': MISSION_STATUS_MESSAGES["NO_ACTIVE"]}

    # Görevle ilgili güncel durum bilgilerini döndürür
    def get_status(self):

        elapsed = 0
        if self.is_mission_active and hasattr(self, 'mission_start_time'):
            try:
                elapsed = int(time.time() - self.mission_start_time)
            except Exception:
                elapsed = 0

        current_location = None
        battery_level = None
        status_message = getattr(self.drone_manager, "mission_status_message", None)

        # Aktif dron varsa konum ve batarya oku
        try:
            vehicle = getattr(self.drone_manager, "active_drone", None)
            if vehicle:
                loc = getattr(vehicle, "location", None)
                if loc:
                    g = getattr(loc, "global_relative_frame", None)
                    if g and g.lat is not None:
                        current_location = {"lat": float(g.lat), "lon": float(g.lon), "alt": float(getattr(g, "alt", 0.0))}
                batt = getattr(vehicle, "battery", None)
                if batt is not None and hasattr(batt, "level"):
                    battery_level = batt.level
        except Exception as e:
            print("get_status: konum/batarya okunurken hata:", e)

        # JSON uyumlu bir grid durumu oluştur.
        # Sözlük anahtarları tuple olamaz, bu yüzden stringe dönüştürülüyor.
        json_friendly_grid_status = {}
        for (row, col), status in self.grid_status.items():
            json_friendly_grid_status[f"{row},{col}"] = status

        return {
            "is_mission_active": self.is_mission_active,
            "elapsed_time": elapsed,
            "grid_status": json_friendly_grid_status,
            "mission_path_points": self.mission_path_points,
            "current_location": current_location,
            "battery_level": battery_level,
            "status_message": status_message
        }

    # Gözlem alanını ayarlar ve gridi oluşturur
    def set_area(self, coordinates):
    
        self.create_grid(coordinates)
        return {'status': 'ok', 'message': MISSION_STATUS_MESSAGES["AREA_SET"].format(rows=self.num_rows, cols=self.num_cols)}