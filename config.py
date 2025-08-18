"""
============================================================
TepeGöz - Gözlem Drone Yönetim Sistemi - Yapılandırma Modülü
============================================================

Yapan / Author : Barış Enes Kümet
Tarih / Date   : 18.08.2025
Modül Adı      : config.py

Açıklama (Türkçe):
------------------
Bu modül, TepeGöz sistemi için gerekli tüm sabitleri, eşikleri 
ve yapılandırma parametrelerini içerir. Dronların fiziksel 
hareket ayarlarından (kalkış yüksekliği, hız vb.) görev 
kontrol algoritmalarının hassasiyetine (PID katsayıları, 
zaman aşımları), kamera ve yapay zeka entegrasyonuna (YOLO 
modelleri, ROS topicleri) ve uygulama sunucusunun ayarları 
gibi tüm konfigürasyonları tek bir merkezi noktada toplar. 

Özet Fonksiyonellik:
--------------------
1. Drone'un kalkış, hız ve güvenlik (kritik batarya) ayarları.
2. Otonom görev kontrolü için PID, zaman aşımı ve hedef 
   yaklaşım eşikleri.
3. Çoklu drone yönetimi ve otomatik geçiş mekanizması 
   parametreleri.
4. Görüntü işleme ve yapay zeka (YOLO) modeli dosyaları ve 
   güvenilirlik eşikleri.
5. Flask sunucu adresi ve portu gibi genel uygulama ayarları.
6. Arayüzde gösterilen tüm durum ve hata mesajları için metin 
   tanımlamaları.

---

Description (English):
----------------------
This module contains all the necessary constants, thresholds, 
and configuration parameters for the TepeGöz system. It centralizes 
all configurations, from drone physical movement settings (takeoff 
altitude, speed, etc.) to mission control algorithm sensitivities 
(PID coefficients, timeouts), camera and AI integration (YOLO 
models, ROS topics), and application server settings.

Summary of Functionality:
-------------------------
1. Drone takeoff, speed, and safety (critical battery) settings.
2. PID, timeout, and target approach thresholds for autonomous 
   mission control.
3. Parameters for multi-drone management and automatic switching.
4. Image processing and AI (YOLO) model file paths and confidence 
   thresholds.
5. General application settings like the Flask server address and port.
6. Text definitions for all status and error messages displayed 
   on the user interface.
"""

# ==============================================================================
# DRON AYARLARI
# ==============================================================================
TAKEOFF_ALTITUDE = 10               # Kalkış yüksekliği (metre)
CRITICAL_BATTERY_LEVEL = 100         # Batarya seviyesi bu değerin altına düştüğünde dron geri döner (%)
MAX_TOUR_COUNT = 1                  # Bir görevin kaç kez tekrarlanacağı
SQUARE_SIZE = 20                    # Dronun gözlem yapacağı alanın boyutu (metre)
DRONE_SPEED = 90                    # Dronun hızı (cm/s)
CONNECTION_TIMEOUT = 20             # Dron bağlantısı için maksimum bekleme süresi (saniye)
DRONE_CONNECTION_RETRY_COUNT = 3    # Bağlantı deneme sayısı

# ==============================================================================
# GÖREV KONTROL PARAMETRELERİ
# ==============================================================================
MAX_SPEED_M_S = 4.0                 # Maksimum hız (m/s)
CONTROL_INTERVAL_S = 0.15           # PID kontrol döngüsü aralığı (saniye)
CELL_REACHED_THRESHOLD_M = 1.2      # Hedef merkeze kabul eşiği (metre)
FINE_APPROACH_THRESHOLD_M = 7.0     # Yavaşlama için mesafe eşiği (metre)
FINE_APPROACH_SCALE = 0.35          # Yakınlaşma hızı ölçeği
CENTER_CONFIRM_TOLERANCE_M = 1.5    # Merkez doğrulama için sapma toleransı (metre)
CENTER_CONFIRM_HOLD_S = 0.2         # Merkez doğrulama için bekleme süresi (saniye)
FINE_APPROACH_HOLD_S = 0.5          # Merkezde onay için kısa bekleme süresi (saniye)
PER_CELL_TIMEOUT_S = 60.0           # Bir hücreye ulaşma için maksimum süre (saniye)
RETRY_LIMIT = 2                     # Hücre atlamadan önce deneme sayısı
GPS_MEDIAN_WINDOW = 7               # GPS medyan filtreleme pencere boyutu
STUCK_MOVED_THRESHOLD_M = 0.12      # Takılma tespiti için hareket eşiği (metre)
STUCK_TIMEOUT_S = 8.0               # Takılma tespiti için zaman aşımı (saniye)
PID_KP = 0.9
PID_KI = 0.02
PID_KD = 0.12

# ==============================================================================
# ÇOKLU DRON VE SWİTCHER AYARLARI
# ==============================================================================
RTL_LANDING_TIMEOUT_S = 30.0        # Kritik bataryadan sonra RTL iniş bekleme süresi (saniye)
MONITOR_INTERVAL_S = 1.0            # Batarya monitörü kontrol döngüsü aralığı (saniye)

# ==============================================================================
# KAMERA VE ROS AYARLARI
# ==============================================================================
# Her port için ROS topic'ini belirten sözlük.
CAMERA_TOPICS = {
    5763: "/webcam/image_raw",
    5753: "/webcam_2/image_raw_2",
    5773: "/webcam/image_raw",
    5773: "/webcam_2/image_raw_2",
    5783: "/webcam/image_raw",
    5783: "/webcam_2/image_raw_2"
}

LOG_THROTTLE_SEC = 0.5              # Logların ne sıklıkta yazılacağını belirler (saniye)
JPEG_QUALITY = 80                   # JPEG görüntü sıkıştırma kalitesi (0-100 arası)

# ==============================================================================
# GÖRÜNTÜ İŞLEME VE YER TUTUCU AYARLARI
# ==============================================================================
PLACEHOLDER_IMAGE_SIZE = (640, 480) # Kamera akışı olmadığında gösterilen yer tutucu resmin boyutu
FONT_SCALE = 0.6                    # Yer tutucu resimdeki yazı fontunun boyutu
FONT_THICKNESS = 1                  # Yer tutucu resimdeki yazı kalınlığı

# ==============================================================================
# YAPAY ZKA (YOLO) AYARLARI
# ==============================================================================
DETECTOR_TYPES = {
    'fire': {
        'module': 'modules.fire_detector',
        'class_name': 'FireDetector',
        'model_path': "./models/fire_m.pt",
        'conf_threshold': 0.5,
        'iou_threshold': 0.45
    }
}
DEFAULT_DETECTION_THRESHOLD = 0.5   # Algılamanın geçerli sayılması için gereken minimum güvenilirlik puanı

# ==============================================================================
# UYGULAMA SUNUCU VE DURUM MESAJLARI
# ==============================================================================
APP_HOST = '0.0.0.0'                # Flask sunucusunun çalışacağı IP adresi.
APP_PORT = 5000                     # Flask sunucusunun çalışacağı port numarası.
DETECTION_RETENTION_TIME = 60       # Tespit sonuçlarının ekranda kalma süresi (saniye).

# Durum Mesajları
CONNECTION_STATUS_CONNECTED = "connected"
CONNECTION_STATUS_NOT_CONNECTED = "not_connected"

MISSION_STATUS_MESSAGES = {
    "NOT_CONNECTED": "Bağlanmayı bekliyor...",
    "CONNECTING": "Drona bağlanılıyor...",
    "CONNECTION_ERROR": "Bağlantı hatası. Tekrar deneyin.",
    "RETRYING": "Bağlantı başarısız. Yeniden deneniyor... (Deneme {current}/{total})",
    "ALL_RETRIES_FAILED": "Tüm bağlantı denemeleri başarısız oldu.",
    "CONNECTED": "Dron bağlandı. Görev için hazır.",
    "AREA_SET": "Gözlem alanı başarıyla ayarlandı. ({rows}x{cols} grid)",
    "STARTING": "Görev başlatılıyor...",
    "INVALID_TYPE": "Geçersiz görev tipi: {mission_type}",
    "ALREADY_ACTIVE": "Görev zaten aktif.",
    "NO_AREA": "Lütfen önce bir gözlem alanı belirleyin.",
    "STOPPED": "Görev durduruldu. Dron ana konuma dönüyor.",
    "NO_ACTIVE": "Aktif bir görev yok.",
    "TAKING_OFF": "Kalkış yapılıyor. Hedef yükseklik: {altitude} metre.",
    "ARMING": "Motorlar hazırlanıyor...",
    "TAKEOFF_SUCCESS": "Hedef yüksekliğe ulaşıldı: {altitude} metre.",
    "BEGIN_MISSION_FLIGHT": "Hedef yüksekliğe ulaşıldı, gözlem alanına doğru ilerleniyor.",
    "NAVIGATING_TO_CELL": "Gözlem noktasına gidiyor: Hücre [{row},{col}].",
    "APPROACHING": "Hücre ({from_row},{from_col}) -> Hücre ({to_row},{to_col}) noktasına yaklaşıyor. Mesafe: {distance:.1f} m, Hız: {speed:.1f} m/s.",
    "VISITING": "Hücre [{row},{col}] gözlemleniyor. Merkezde stabil kalınıyor.",
    "CELL_CONFIRMED": "Hücre gözlemi tamamlandı: [{row},{col}].",
    "CELL_SKIPPED": "Hücre gözlemi atlandı: [{row},{col}].",
    "STUCK": "Dron takıldı. Kaçış manevrası yapılıyor.",
    "TIMEOUT": "Hücreye ulaşma zaman aşımı. Sonraki hücreye geçiliyor.",
    "CRITICAL_BATTERY": "Kritik batarya seviyesi! RTL (ana konuma dönüş) başlatıldı.",
    "MISSION_COMPLETE": "Görev tamamlandı. Dron ana konuma dönüyor.",
    "MISSION_ERROR": "Görev sırasında beklenmedik bir hata oluştu: {error}",
    "CRITICAL_BATTERY": "Kritik batarya seviyesi! RTL (ana konuma dönüş) başlatıldı.",
    "MISSION_COMPLETE": "Görev tamamlandı. Dron ana konuma dönüyor.",
    "MISSION_ERROR": "Görev sırasında beklenmedik bir hata oluştu: {error}",
    "SWITCHING_DRONE": "Batarya bitmek üzere. Görev diğer drona devrediliyor...",
    "DRONE_IS_BACK": "Dron ana konuma döndü ve şarj oluyor.",
    "RESUMING": "Görev kaldığı yerden devam ettiriliyor.",
    "HANDOVER_MISSION_TYPE": "Görevin tipi devrediliyor: {mission_type}",
    "SWITCH_TO_NEXT": "Bir sonraki drona geçiş yapılıyor...",
    "NEXT_DRONE_SELECTED": "Yeni dron ({port}) seçildi. Görev devam ettiriliyor.",
    "MONITOR_STARTED": "[DroneSwitcher] İzleme başlatıldı.",
    "MONITOR_STOPPED": "[DroneSwitcher] İzleme durduruldu.",
     "RTL_STARTED": "[DroneSwitcher] Aktif dron RTL moduna geçirildi.",
    "MONITOR_ERROR": "[DroneSwitcher] İzleme sırasında hata oluştu: {error}",
    "RESUMING_FROM_POINT": "Kaldığı noktadan devam ediliyor: {point}",
    "NO_LAST_POINT": "Son görev noktası bulunamadı, baştan başlanıyor."
}