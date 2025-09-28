<div align="center">

# TepeGöz: Multi-Drone Autonomous Surveillance System

**⭐ If you find this project useful, give it a star! / Bu projeyi yararlı buluyorsanız yıldızlayın! ⭐**

[![GitHub stars](https://img.shields.io/github/stars/username/TepeGoz?style=social)](https://github.com/username/TepeGoz)
[![GitHub forks](https://img.shields.io/github/forks/username/TepeGoz?style=social)](https://github.com/username/TepeGoz/fork)

[![Docker](https://img.shields.io/badge/docker-supported-blue.svg)](https://www.docker.com/)
[![VS Code](https://img.shields.io/badge/vscode-devcontainer-green.svg)](https://code.visualstudio.com/)
[![ROS](https://img.shields.io/badge/ros-noetic-34a853.svg)](http://wiki.ros.org/noetic)
[![Gazebo](https://img.shields.io/badge/gazebo-11-orange.svg)](http://gazebosim.org/)
[![ArduPilot](https://img.shields.io/badge/ardupilot-SITL-yellow.svg)](https://ardupilot.org/)
[![MAVLink](https://img.shields.io/badge/mavlink-supported-red.svg)](https://mavlink.io/en/)
[![MAVROS](https://img.shields.io/badge/mavros-integration-blueviolet.svg)](http://wiki.ros.org/mavros)
[![Python](https://img.shields.io/badge/Python-3.8+-3776AB?style=flat&logo=python&logoColor=white)](https://www.python.org/)
[![Flask](https://img.shields.io/badge/Flask-2.3+-000000?style=flat&logo=flask&logoColor=white)](https://flask.palletsprojects.com/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=flat&logo=opencv&logoColor=white)](https://opencv.org/)
[![DroneKit](https://img.shields.io/badge/DroneKit-MAVLink-FF6F00?style=flat&logo=drone&logoColor=white)](https://dronekit-python.readthedocs.io/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-00FFFF?style=flat&logo=yolo&logoColor=black)](https://ultralytics.com/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420?style=flat&logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-red.svg?style=flat&logo=gnu&logoColor=white)](https://www.gnu.org/licenses/agpl-3.0)

<img src="images/logo.png" alt="TepeGöz Logo" width="350" />

---

| Ground Station and Web Interface | Solar Charging Station |
| :---: |:---:|
| <img src="images/web_site.png" alt="Web Interface" width="600" height="400"/> | <img src="images/dron_istansoy.png" alt="Solar Charging Station" width="600" height="400"/> |

---

🇬🇧[English](#english) | 🇹🇷[Türkçe](#türkçe)

</div>

---

## English

### 🇬🇧 Overview

**“Tepe Göz”** is a multi-drone surveillance system designed to provide **24/7 uninterrupted monitoring** of a designated area. The system combines a ground station, a solar-powered charging unit, and **at least two drones — scalable to more if needed** to ensure continuous operation.

### ✨ Key Features

🚁 **24/7 Continuous Operation** - Seamless drone rotation ensures zero downtime  
🔋 **Solar-Powered Charging** - Sustainable, off-grid energy solution  
🤖 **AI-Powered Detection** - Real-time fire, human, and motion detection using YOLOv8  
🗺️ **Autonomous Navigation** - Self-managed takeoff, mission execution, and landing  
📱 **Web-Based Control** - User-friendly Flask interface with live monitoring  

### 🛠️ How It Works

Its working principle is simple: while at least one drone is always on duty in the air, the other drone(s) recharge at the ground station. When the active drone’s battery reaches a critical level, it autonomously returns to the station, and a fully charged drone immediately takes off to continue the mission **without any interruption**. This seamless handover guarantees that the monitored area is never left unattended.

With its **AI-powered detection capabilities**, the system can identify events such as fires, intrusions, or unusual movements and notify the ground station. The ground station then instantly relays this information to the relevant personnel.

Since the system is powered by solar energy, it operates **independently of any infrastructure**, making it ideal for remote or off-grid areas. It also offers **rapid deployment**, becoming fully operational in about **30 minutes**.

Moreover, thanks to its **modular design**, the communication and sensor setup can be easily adapted to different mission requirements. The drones can be equipped with various cameras — such as standard day cameras, night vision, or thermal imaging — to ensure effective surveillance under diverse conditions.

### 🏗️ System Architecture

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                           TepeGöz Surveillance System                          │
├────────────────────────────────────────────────────────────────────────────────┤
│                                                                                │
│  ┌───────────────┐    ┌──────────────────┐    ┌───────────────┐                │
│  │  Drone A      │    │   Ground Station │    │  Drone B      │                │
│  │ (Patrolling)  │◄──►│                  │◄──►│ (Charging)    │                │
│  │               │    │  ┌─────────────┐ │    │               │                │
│  │ • YOLOv8 AI   │    │  │Solar Panel  │ │    │ • Standby     │                │
│  │ • Live Stream │    │  │             │ │    │ • Ready       │                │
│  │ • GPS Nav     │    │  └─────────────┘ │    │               │                │
│  └───────┬───────┘    │  ┌─────────────┐ │    └───────┬───────┘                │
│          │            │  │Web Interface│ │            │                        │
│          │ Battery Low│  │             │ │            │ Battery Full           │
│          ▼            │  └─────────────┘ │            ▼                        │
│  ┌───────┴───────┐    │  ┌─────────────┐ │    ┌───────┴───────┐                │
│  │ Drone A       │    │  │ROS Core     │ │    │ Drone B       │                │
│  │ (Returning)   │◄──►│  │             │ │◄──►│ (Taking Off)  │                │
│  │ • Auto-Landing│    │  └─────────────┘ │    │ • Start Patrol│                │
│  └───────┬───────┘    └──────────────────┘    └───────┬───────┘                │
│          │                                            │                        │
│          │ Landed & Charging                          │ Now Patrolling         │
│          ▼                                            ▼                        │
│  ┌───────┴───────┐                            ┌───────┴───────┐                │
│  │ Drone A       │                            │ Drone B       │                │
│  │ (Charging)    │                            │ (Patrolling)  │                │
│  │ • Standby     │                            │ • YOLOv8 AI   │                │
│  │ • Ready       │                            │ • Live Stream │                │
│  └───────────────┘                            └───────────────┘                │
│                                                                                │
│                           HANDOVER COMPLETE                                    │
│                                                                                │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                         CYCLE REPEATS                                   │   │
│  │  When Drone B battery low → Returns to charge                           │   │
│  │  Drone A takes over patrol mission                                      │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                │
└────────────────────────────────────────────────────────────────────────────────┘
```

### 💻 Tech Stack

| Component | Technology | Purpose |
|-----------|------------|---------|
| **OS** | Ubuntu 20.04 | Stable Linux environment |
| **Framework** | ROS1 Noetic | Robot communication & control |
| **Language** | Python 3.8+ | Core development |
| **Drone Control** | DroneKit | MAVLink drone communication |
| **AI Vision** | YOLOv8 (Ultralytics) | Real-time object detection |
| **Web Server** | Flask | REST API & web interface |
| **Frontend** | HTML5, TailwindCSS, Leaflet.js | Interactive mapping UI |
| **Computer Vision** | OpenCV | Image processing |

### 📂 Project Structure

```
tepe_goz/
├── app.py                      # Main Flask application and API endpoints
├── config.py                   # Configuration file for drone and mission parameters
├── modules/
│   ├── camera_ai.py            # Handles camera feeds and AI detection logic
│   ├── drone_manager.py        # Manages drone connections and status
│   ├── fire_detector.py        # Specific module for fire detection using YOLOv8
│   ├── mission_controller.py   # Manages mission planning and execution
│   └── ...                     # Other detection modules (e.g., human_detector)
├── static/
│   ├── css/
│   │   └── style.css
│   ├── images/
│   │   ├── logo5.png           # Project logo
│   │   └── ...                 # Other images (dron_istansoy.png, dron_istayon2.png)
│   └── js/
│       └── script.js           # Frontend JavaScript for map and UI
├── templates/
│   └── index.html              # Main web interface
├── models/
│   └── fire_m.pt               # Pre-trained YOLOv8 model for fire detection
├── README.md                   # This file
└── ...
```

### 🚀 Installation

```bash
# Clone the repository
git clone https://github.com/username/TepeGoz.git
cd TepeGoz

# Install dependencies
pip install -r requirements.txt


# Launch TepeGöz
python app.py
```

#### Web Interface
Open your browser and navigate to `http://localhost:5000` to access the control panel.

### 🎯 Use Cases

- **Forest Fire Monitoring** - Early fire detection and alert systems
- **Border Security** - Autonomous perimeter surveillance  
- **Wildlife Conservation** - Non-intrusive animal monitoring
- **Infrastructure Monitoring** - Pipeline, power line inspection
- **Emergency Response** - Disaster area assessment and monitoring

### 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### 👨‍💻 Author

**Barış Enes Kümet**  
- 📧 Email: [barisenesk72@gmail.com](mailto:barisenesk72@gmail.com)
- 🐙 GitHub: [@koesa](https://github.com/koesa)

---

## Türkçe

### 🇹🇷 Genel Bakış  

**“Tepe Göz”**, belirli bir alanın 7/24 kesintisiz şekilde izlenmesini sağlamak için tasarlanmış çoklu dron gözetim sistemidir. Sistem; bir yer istasyonu, güneş enerjisiyle çalışan şarj ünitesi ve **en az iki, gerektiğinde daha fazla dronu** bir araya getirerek kesintisiz gözetim sağlar.

### ✨ Temel Özellikler

🚁 **7/24 Kesintisiz Çalışma** - Sorunsuz drone rotasyonu ile sıfır kesinti süresi  
🔋 **Güneş Enerjili Şarj** - Sürdürülebilir, şebekeden bağımsız enerji çözümü  
🤖 **Yapay Zeka Destekli Tespit** - YOLOv8 ile gerçek zamanlı yangın, insan ve hareket tespiti  
🗺️ **Otonom Navigasyon** - Kendi kendini yöneten kalkış, görev yürütme ve iniş  
📱 **Web Tabanlı Kontrol** - Canlı izleme ile kullanıcı dostu Flask arayüzü  

### 🛠️ Nasıl Çalışır

Çalışma prensibi basittir: En az bir dron sürekli havada görev yaparken, diğer dron(lar) yer istasyonunda bataryasını şarj eder. Görevdeki dron’un enerjisi azaldığında otomatik olarak istasyona döner ve şarj olmuş bir dron devreye girerek gözetim görevini **hiçbir kesinti olmadan** sürdürür. Böylece gözlemlenen alanın her an kontrol altında tutulması garanti edilir.

Sistem, **yapay zekâ destekli tespit yetenekleri** sayesinde yangın, izinsiz giriş veya olağan dışı hareketleri algılayabilir ve yer istasyonunu uyarır. Yer istasyonu da bu bilgileri anında ilgili kişilere iletir.

Güneş enerjisiyle kendi kendini şarj edebildiği için **herhangi bir altyapıya ihtiyaç duymadan** çalışır; bu sayede uzak ve altyapısız bölgelerde de kullanılabilir. Ayrıca yaklaşık **30 dakika içinde kurulup devreye alınabilir**, yani hızlıca faaliyete geçebilir.

Bunun yanında, **modüler yapısı** sayesinde iletişim ve sensör altyapısı kolayca değiştirilebilir. Dronlara gündüz kamerası, gece görüş kamerası veya termal kamera takılarak farklı görev senaryolarına uyum sağlanabilir.

### 🏗️ Sistem Mimarisi

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                           TepeGöz Gözetim Sistemi                              │
├────────────────────────────────────────────────────────────────────────────────┤
│                                                                                │
│  ┌───────────────┐    ┌──────────────────┐    ┌───────────────┐                │
│  │  Drone A      │    │   Yer İstasyonu  │    │  Drone B      │                │
│  │ (Devriyede)   │◄──►│                  │◄──►│ (Şarj Oluyor) │                │
│  │               │    │  ┌─────────────┐ │    │               │                │
│  │ • YOLOv8 AI   │    │  │Güneş Paneli │ │    │ • Beklemede   │                │
│  │ • Canlı Yayın │    │  │             │ │    │ • Hazır       │                │
│  │ • GPS Nav     │    │  └─────────────┘ │    │               │                │
│  └───────┬───────┘    │  ┌─────────────┐ │    └───────┬───────┘                │
│          │            │  │Web Arayüzü  │ │            │                        │
│          │ Pil Düşük  │  │             │ │            │ Pil Dolu               │
│          ▼            │  └─────────────┘ │            ▼                        │
│  ┌───────┴───────┐    │  ┌─────────────┐ │    ┌───────┴───────┐                │
│  │ Drone A       │    │  │ROS Merkezi  │ │    │ Drone B       │                │
│  │ (Dönüyor)     │◄──►│  │             │ │◄──►│ (Kalkıyor)    │                │
│  │ • Oto-İniş    │    │  └─────────────┘ │    │• Devriye Başla│                │
│  └───────┬───────┘    └──────────────────┘    └───────┬───────┘                │
│          │                                               │                     │
│          │ İndi & Şarj Oluyor                           │ Şimdi Devriyede      │
│          ▼                                               ▼                     │
│  ┌───────┴───────┐                                ┌───────┴───────┐            │
│  │ Drone A       │                                │ Drone B       │            │
│  │ (Şarj Oluyor) │                                │ (Devriyede)   │            │
│  │ • Beklemede   │                                │ • YOLOv8 AI   │            │
│  │ • Hazır       │                                │ • Canlı Yayın │            │
│  └───────────────┘                                └───────────────┘            │
│                                                                                │
│                           EL DEĞİŞTİRME TAMAMLANDI                             │
│                                                                                │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                         DÖNGÜ TEKRARLANIR                               │   │
│  │  Drone B pili düşük olduğunda → Şarj için döner                         │   │
│  │  Drone A devriye görevini devralır                                      │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                │
└────────────────────────────────────────────────────────────────────────────────┘
```

### 💻 Teknolojiler

| Bileşen | Teknoloji | Amaç |
|---------|-----------|------|
| **İşletim Sistemi** | Ubuntu 20.04 | Kararlı Linux ortamı |
| **Framework** | ROS1 Noetic | Robot iletişimi ve kontrol |
| **Dil** | Python 3.8+ | Ana geliştirme |
| **Drone Kontrolü** | DroneKit | MAVLink drone iletişimi |
| **Yapay Zeka** | YOLOv8 (Ultralytics) | Gerçek zamanlı nesne tespiti |
| **Web Sunucu** | Flask | REST API ve web arayüzü |
| **Ön Yüz** | HTML5, TailwindCSS, Leaflet.js | Etkileşimli harita arayüzü |
| **Görüntü İşleme** | OpenCV | Görüntü işleme |

### 📂 Proje Dosyaları

```
tepe_goz/
├── app.py                      # Ana Flask uygulaması ve API uç noktaları
├── config.py                   # Drone ve görev parametreleri için yapılandırma dosyası
├── modules/
│   ├── camera_ai.py            # Kamera beslemesini ve yapay zeka tespit mantığını yönetir
│   ├── drone_manager.py        # Drone bağlantılarını ve durumunu yönetir
│   ├── fire_detector.py        # YOLOv8 kullanarak yangın tespiti için özel modül
│   ├── mission_controller.py   # Görev planlamasını ve yürütülmesini yönetir
│   └── ...                     # Diğer tespit modülleri (örn. human_detector)
├── static/
│   ├── css/
│   │   └── style.css
│   ├── images/
│   │   ├── logo5.png           # Proje logosu
│   │   └── ...                 # Diğer görseller (dron_istansoy.png, dron_istayon2.png)
│   └── js/
│       └── script.js           # Harita ve kullanıcı arayüzü için ön yüz JavaScript'i
├── templates/
│   └── index.html              # Ana web arayüzü
├── models/
│   └── fire_m.pt               # Yangın tespiti için önceden eğitilmiş YOLOv8 modeli
├── README.md                   # Dökümantasyon
└── ...
```

### 🚀 Kurulum

```bash
# Depoyu klonlayın
git clone https://github.com/username/TepeGoz.git
cd TepeGoz

# Bağımlılıkları kurun
pip install -r requirements.txt

# TepeGöz'ü başlatın
python app.py
```

#### Web Arayüzü
Tarayıcınızı açın ve kontrol paneline erişmek için `http://localhost:5000` adresine gidin.

### 🎯 Kullanım Alanları

- **Orman Yangını İzleme** - Erken yangın tespiti ve alarm sistemleri
- **Sınır Güvenliği** - Otonom çevre güvenlik gözetimi  
- **Vahşi Yaşam Koruma** - Müdahalesiz hayvan izleme
- **Altyapı İzleme** - Boru hattı, elektrik hattı denetimi
- **Acil Durum Müdahalesi** - Felaket alanı değerlendirme ve izleme

### 📄 Lisans

Bu proje **GNU Affero General Public License v3.0 (AGPLv3)** altında lisanslanmıştır - detaylar için [LICENSE](LICENSE) dosyasına bakın.

### 👨‍💻 Geliştirici

**Barış Enes Kümet**  
- 📧 E-posta: [barisenesk72@gmail.com](mailto:barisenesk72@gmail.com)
- 🐙 GitHub: [@koesa](https://github.com/koesan)
