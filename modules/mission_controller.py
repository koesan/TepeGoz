# modules/mission_controller.py

import threading
import time
import math
from collections import deque
from statistics import median
from dronekit import VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from config import TAKEOFF_ALTITUDE, CRITICAL_BATTERY_LEVEL, SQUARE_SIZE, DRONE_SPEED

class PID:
    def __init__(self, kp=0.8, ki=0.01, kd=0.08, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = integral_limit

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error, dt):
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        # anti-windup
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        deriv = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * deriv


class MissionController:
    # --- tuning parametreleri ---
    MAX_SPEED_M_S = 4.0
    CONTROL_INTERVAL_S = 0.15

    # yakınlaşma/merkez parametreleri
    CELL_REACHED_THRESHOLD_M = 1.2       # merkeze kabul eşiği
    FINE_APPROACH_THRESHOLD_M = 7.0      # bu mesafeye gelince yavaşlama uygulanır
    FINE_APPROACH_SCALE = 0.35           # yakınlaşma faktörü
    CENTER_CONFIRM_TOLERANCE_M = 1.5     # satır/kolon değişmeden önce kabul edilecek sapma
    CENTER_CONFIRM_HOLD_S = 0.2          # merkez doğrulama için bekleme süresi
    FINE_APPROACH_HOLD_S = 0.5           # merkezde onay için kısa bekleme
    PER_CELL_TIMEOUT_S = 60.0            # hücreye ulaşma için başlangıç timeout
    RETRY_LIMIT = 2                      # hücre atlamadan önce tekrar sayısı
    GPS_MEDIAN_WINDOW = 7                # GPS median filter (kaç örnekle medyan al)
    STUCK_MOVED_THRESHOLD_M = 0.12       # stuck detection
    STUCK_TIMEOUT_S = 8.0

    def __init__(self, drone_manager):
        self.drone_manager = drone_manager
        self.mission_coordinates = None
        self.centers_2d = []
        self.grid_status = {}
        self.num_rows = 0
        self.num_cols = 0
        self.is_mission_active = False
        self.mission_path_points = []
        self._mission_thread = None
        self.camera_handler = self.drone_manager.camera_handler  # CameraAIHandler'a erişim sağla

    # --- yardımcı fonksiyonlar ---
    def _haversine_distance_m(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def _latlon_to_north_east_m(self, lat1, lon1, lat2, lon2):
        d_north = (lat2 - lat1) * 111320.0
        mean_lat = math.radians((lat1 + lat2) / 2.0)
        d_east = (lon2 - lon1) * 111320.0 * math.cos(mean_lat)
        return d_north, d_east

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
        self.drone_manager.mission_status_message = f"Grid hazır: {num_cols}x{num_rows}, alt-sol'dan başlıyor."
        print(f"Grid oluşturuldu: {num_rows}x{num_cols} (row x col). Alt-sol'dan başlayacak.")

    # boustrophedon (tarla sürme / yılan deseni) tarama
    def _next_indices(self, row, col, horiz_dir, vert_dir):
        next_col = col + horiz_dir

        # Satır içinde ilerleyebiliyorsak
        if 0 <= next_col < self.num_cols and self.centers_2d[row][next_col] is not None:
            return row, next_col, horiz_dir, vert_dir

        # Satır bitti → bir üst/alt satıra geç
        next_row = row + vert_dir
        if 0 <= next_row < self.num_rows:
            horiz_dir *= -1  # yön değiştir
            return next_row, col, horiz_dir, vert_dir

        # Gridin en üstüne veya en altına ulaştıysak → yönü ters çevirip devam et
        vert_dir *= -1
        next_row = row + vert_dir
        if 0 <= next_row < self.num_rows:
            horiz_dir *= -1
            return next_row, col, horiz_dir, vert_dir

        return row, col, horiz_dir, vert_dir


    def _get_median_location(self, vehicle, samples_deque):
        if not samples_deque:
            loc = vehicle.location.global_relative_frame
            return (loc.lat, loc.lon) if loc else (None, None)
        lats = [s[0] for s in samples_deque]
        lons = [s[1] for s in samples_deque]
        return median(lats), median(lons)

    def confirm_center_stability(self, vehicle, target_lat, target_lon, hold_s=None, tol_m=None):
        hold_s = self.CENTER_CONFIRM_HOLD_S if hold_s is None else hold_s
        tol_m = self.CENTER_CONFIRM_TOLERANCE_M if tol_m is None else tol_m
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

    def _approach_target(self, vehicle, target_lat, target_lon):

        # retry loop
        for attempt in range(self.RETRY_LIMIT + 1):
            pid_n = PID(kp=0.9, ki=0.02, kd=0.12)
            pid_e = PID(kp=0.9, ki=0.02, kd=0.12)
            pid_n.reset(); pid_e.reset()

            samples = deque(maxlen=self.GPS_MEDIAN_WINDOW)
            start_ts = time.time()
            last_move_ts = time.time()
            last_pos = None

            while self.is_mission_active:
                """
                # batarya kontrolü
                try:
                    batt = getattr(vehicle, 'battery', None)
                    batt_level = batt.level if batt is not None and hasattr(batt, 'level') else None
                    if batt_level is not None and batt_level < CRITICAL_BATTERY_LEVEL:
                        print('Kritik batarya: RTL yapılıyor')
                        vehicle.mode = VehicleMode('RTL')
                        self.is_mission_active = False
                        return False
                except Exception:
                    pass
                """
                loc = vehicle.location.global_relative_frame
                if loc and loc.lat is not None:
                    samples.append((loc.lat, loc.lon))

                if not samples:
                    time.sleep(self.CONTROL_INTERVAL_S)
                    continue

                # filtered position
                filt_lat, filt_lon = self._get_median_location(vehicle, samples)
                d_north, d_east = self._latlon_to_north_east_m(filt_lat, filt_lon, target_lat, target_lon)
                dist = math.hypot(d_north, d_east)

                # reached check
                if dist <= self.CELL_REACHED_THRESHOLD_M:
                    # stop and confirm with slightly longer hold
                    self.send_ned_velocity(vehicle, 0, 0, 0)
                    # extra fine hold
                    hold_ok = self.confirm_center_stability(vehicle, target_lat, target_lon, hold_s=self.FINE_APPROACH_HOLD_S, tol_m=self.CELL_REACHED_THRESHOLD_M)
                    if hold_ok:
                        return True
                    else:
                        # başarısızsa PID reset ve tekrar dene
                        pid_n.reset(); pid_e.reset()
                        time.sleep(0.12)
                        continue

                # PID compute using filtered north/east errors (target - current)
                vx = pid_n.step(d_north, self.CONTROL_INTERVAL_S)
                vy = pid_e.step(d_east, self.CONTROL_INTERVAL_S)

                # yakınlaşırken yavaşlama
                if dist < self.FINE_APPROACH_THRESHOLD_M:
                    vx *= self.FINE_APPROACH_SCALE
                    vy *= self.FINE_APPROACH_SCALE

                speed = math.hypot(vx, vy)
                if speed > self.MAX_SPEED_M_S:
                    scale = self.MAX_SPEED_M_S / speed
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
                    if moved > self.STUCK_MOVED_THRESHOLD_M:
                        last_pos = (loc.lat, loc.lon); last_move_ts = time.time()
                    else:
                        if time.time() - last_move_ts > self.STUCK_TIMEOUT_S:
                            # small escape maneuver
                            print('Takılma tespit edildi — kaçış manevrası yapılıyor')
                            try:
                                self.send_ned_velocity(vehicle, -vx * 0.4, -vy * 0.4, 0)
                                time.sleep(0.4)
                                self.send_ned_velocity(vehicle, vx * 1.1, vy * 1.1, 0)
                            except Exception as e:
                                print('Manevra hatası:', e)
                            last_move_ts = time.time()

                elapsed = time.time() - start_ts
                timeout = self.PER_CELL_TIMEOUT_S * (1 + attempt * 0.5)
                if elapsed > timeout:
                    print(f'Per-cell timeout (attempt {attempt}): hedefe ulaşılamadı (elapsed={elapsed:.1f}s)')
                    break

                time.sleep(self.CONTROL_INTERVAL_S)

            print('Retrying approach with adjusted parameters...')
            self.MAX_SPEED_M_S = max(1.2, self.MAX_SPEED_M_S * 0.75)

        return False

    def run_mission(self):
        vehicle = self.drone_manager.active_drone
        if not vehicle or not self.centers_2d:
            print('Aktif dron veya grid yok')
            return

        try:
            self.mission_path_points = []
            self.mission_start_time = time.time()
            self.arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)
            self.is_mission_active = True

            cur_row = self.num_rows - 1
            cur_col = 0
            horiz_dir = 1
            vert_dir = -1

            while self.is_mission_active:
                target = self.centers_2d[cur_row][cur_col]
                if target is None:
                    cur_row, cur_col, horiz_dir, vert_dir = self._next_indices(cur_row, cur_col, horiz_dir, vert_dir)
                    continue

                lat, lon = target
                print(f'Approach cell -> row={cur_row}, col={cur_col}, target=({lat:.6f},{lon:.6f})')

                ok = self._approach_target(vehicle, lat, lon)
                if ok:
                    self.grid_status[(cur_row, cur_col)] = 'visited'
                    print(f'Visited confirmed: ({cur_row},{cur_col})')
                    # kesin merkez doğrulama: satır değişmeden önce 0.2s tut ve sapma <= tolerance
                    stable = self.confirm_center_stability(vehicle, lat, lon, hold_s=self.CENTER_CONFIRM_HOLD_S, tol_m=self.CENTER_CONFIRM_TOLERANCE_M)
                    if not stable:
                        print('Center confirmation failed after visit — retrying approach')
                        continue
                else:
                    # atlandı
                    self.grid_status[(cur_row, cur_col)] = 'old'
                    print(f'Cell skipped after retries: ({cur_row},{cur_col})')

                # ilerle
                prev_row, prev_col = cur_row, cur_col
                cur_row, cur_col, horiz_dir, vert_dir = self._next_indices(cur_row, cur_col, horiz_dir, vert_dir)

                # Eğer vertical move olacaksa, bir üst/alt hücreye geçmeden önce (önlem) kısa hover
                if cur_row != prev_row:
                    # kısa bekle ve hedefin merkezinde olduğunu doğrula
                    target_next = self.centers_2d[cur_row][cur_col]
                    if target_next is not None:
                        # küçük zaman ver, fakat _approach_target hedefi zaten doğrulayacak
                        time.sleep(0.12)

                time.sleep(0.08)

        except Exception as e:
            print('Run mission hata:', e)
            try:
                vehicle.mode = VehicleMode('RTL')
            except Exception:
                pass

        finally:
            print('Görev sona eriyor — RTL')
            try:
                vehicle.mode = VehicleMode('RTL')
            except Exception:
                pass
            self.is_mission_active = False
            self.drone_manager.mission_status_message = 'Görev tamamlandı veya durduruldu.'

    # Basit arm/takeoff fonksiyonu
    def arm_and_takeoff(self, vehicle, aTargetAltitude):
        self.drone_manager.mission_status_message = 'Kalkış için hazırlanıyor...'
        print('Kalkış hazırlanıyor...')
        while not vehicle.is_armable:
            print('Waiting for armable...')
            time.sleep(1)
        vehicle.mode = VehicleMode('GUIDED')
        vehicle.armed = True
        while not vehicle.armed:
            print('Arming...')
            time.sleep(0.5)
        vehicle.simple_takeoff(aTargetAltitude)
        while True:
            alt = vehicle.location.global_relative_frame.alt
            print(f' Yükseklik: {alt:.2f}')
            if alt >= aTargetAltitude * 0.95:
                print('Hedef yüksekliğe ulaşıldı')
                break
            time.sleep(0.5)

    # start/stop/get_status/set_area
    def start_mission(self, mission_type):

        if not self.drone_manager.active_drone:
            self.drone_manager.mission_status_message = "Aktif dron bulunamadı."
            return {'status': 'error', 'message': 'Lütfen önce bir dron seçin.'}

        if not self.camera_handler.set_mission_type(mission_type):
            self.drone_manager.mission_status_message = f"Geçersiz görev tipi: {mission_type}"
            return {"status": "error", "message": f"Geçersiz görev tipi: {mission_type}"}

        if self.is_mission_active:
            self.drone_manager.mission_status_message = "Görev zaten aktif."
            return {"status": "error", "message": "Görev zaten aktif."}

        if self.is_mission_active:
            self.drone_manager.mission_status_message = "Görev zaten aktif."
            return {'status': 'error', 'message': 'Görev zaten aktif.'}

        if not self.mission_coordinates:
            self.drone_manager.mission_status_message = "Lütfen önce bir gözlem alanı belirleyin."
            return {'status': 'error', 'message': 'Lütfen önce bir gözlem alanı belirleyin.'}

        self._mission_thread = threading.Thread(target=self.run_mission, daemon=True)
        self._mission_thread.start()
        return {'status': 'ok', 'message': 'Görev başlatıldı.'}

    def stop_mission(self):
        if self.drone_manager.active_drone and self.is_mission_active:
            try:
                self.drone_manager.active_drone.mode = VehicleMode('RTL')
            except Exception:
                pass
            self.is_mission_active = False
            return {'status': 'ok', 'message': 'Görev durduruldu.'}
        return {'status': 'error', 'message': 'Aktif bir görev yok.'}

    def get_status(self):

        elapsed = 0
        if self.is_mission_active and self.mission_start_time:
            try:
                elapsed = int(time.time() - self.mission_start_time)
            except Exception:
                elapsed = 0

        current_location = None
        battery_level = None
        status_message = getattr(self.drone_manager, "mission_status_message", None)

        # aktif dron varsa konum ve batarya oku 
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

        return {
            "is_mission_active": self.is_mission_active,
            "elapsed_time": elapsed,
            "grid_status": self.grid_status,
            "mission_path_points": self.mission_path_points,
            "current_location": current_location,
            "battery_level": battery_level,
            "status_message": status_message
        }


    def set_area(self, coordinates):
        self.create_grid(coordinates)
        return {'status': 'ok', 'message': 'Alan kaydedildi.'}
