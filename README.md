# GES ROS2 + Docker Sistem Mimarisi ve Kullanım Kılavuzu

GES robot platformu; Jetson üzerinde gerçek donanım çalıştıran ROS2 ortamı ile PC üzerinde simülasyon ve RViz2 görselleştirmesini tek bir Docker tabanlı mimaride birleştiren bir sistemdir. Bu README, sistemi sıfırdan kurmaktan nodeları çalıştırmaya kadar tüm adımları içerir.

---

## 1. Sistem Mimarisi

GES iki donanım hedefine sahiptir:

- **PC / Simulation (x86_64)**  
  - RViz2, Gazebo, simülasyon ve geliştirme ortamı  
  - Docker + ROS2 Humble (x86)

- **Jetson Xavier NX / Robot (ARM64)**  
  - Gerçek robot sürücüleri, LIDAR, GPIO, UART  
  - Docker + ROS2 Humble (ARM64) + CUDA

Her iki taraf aynı `ros2_ws` workspace yapısını kullanır; ancak farklı Dockerfile'lar ile build edilir.

### Mimari Blok Şeması

```
                 ┌────────────────────────────────┐
                 │            PC (sim)            │
                 │  - keyboard_input              │
                 │  - RViz2                       │
                 │  - Gazebo (opsiyonel)          │
                 └───────────────┬────────────────┘
                                 │ DDS / LAN
                                 │
                 ┌───────────────▼────────────────┐
                 │       Jetson / Robot           │
                 │  - motor_control               │
                 │  - rplidar_ros (LIDAR)         │
                 │  - gerçek sürücüler/uart/gpio  │
                 └────────────────────────────────┘
```

---

## 2. Proje Dosya Yapısı

```
ROS2inDocker/
│
├── docker-compose.yml
├── .env
├── Dockerfile.pc
├── Dockerfile.jetson
│
└── ros2_ws/
    ├── src/
    │   ├── motor_control/
    │   ├── keyboard_input/
    │   ├── rplidar_ros/
    │   ├── led_control/
    │   └── ...
    │
    ├── build/
    ├── install/
    └── log/
```

---

## 3. Kullanılan ROS2 Nodeları

### `keyboard_input`
- PC’de çalışır.
- Klavye ok tuşlarını okuyup `/cmd_vel` publish eder.

### `motor_control`
- Jetson üzerinde çalışır.
- `/cmd_vel` mesajlarını işleyip motor sürücülere gönderir.

### `rplidar_ros`
- Jetson’a bağlı RPLIDAR S2E sensörünü çalıştırır.
- `/scan` LaserScan mesajı yayınlar.

### `led_control`
- Jetson GPIO doğrulama amaçlı test node.

---

## 4. Topic Yapısı

| Topic      | Mesaj Tipi               | Açıklama                                |
|------------|---------------------------|------------------------------------------|
| `/cmd_vel` | geometry_msgs/Twist       | PC sim → Jetson motor kontrol            |
| `/scan`    | sensor_msgs/LaserScan     | Jetson LIDAR → PC RViz2                  |
| `/diagnostics` | std_msgs/String       | Motor/Lidar durum bilgisi                |

### Topic Veri Akışı

```
PC keyboard_input → /cmd_vel → Jetson motor_control
Jetson rplidar_ros → /scan → PC RViz2
```

---

## 5. LIDAR Entegrasyonu (RPLIDAR S2E)

### Cihaz izinleri (Jetson)
```bash
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

### LIDAR node başlatma
```bash
docker exec -it ros2_robot bash
source /opt/ros/$ROS_DISTRO/install/setup.bash
source /workspace/ros2_ws/install/setup.bash

ros2 launch rplidar_ros rplidar_s2e_launch.py
```

### RViz2 üzerinde görüntüleme (PC)
```bash
rviz2
```
RViz → Add → LaserScan → `/scan`

---

## 6. Docker Compose Profilleri

### Simülasyon (PC – x86_64)
```bash
docker compose --profile sim build
docker compose --profile sim up
```

Container’a giriş:
```bash
docker exec -it ros2_sim bash
```

Workspace build:
```bash
cd /workspace/ros2_ws
colcon build --symlink-install
```

Keyboard teleop:
```bash
ros2 run keyboard_input teleop
```

---

### Robot (Jetson – ARM64)
```bash
docker compose --profile robot build
docker compose --profile robot up
```

Container’a giriş:
```bash
docker exec -it ros2_robot bash
```

Workspace build:
```bash
cd /workspace/ros2_ws
colcon build --symlink-install
```

Motor kontrol node:
```bash
ros2 run motor_control motor_control_node
```

LIDAR:
```bash
ros2 launch rplidar_ros rplidar_s2e_launch.py
```

---

## 7. Jetson ve PC Arasındaki Build Farkları

Aynı Dockerfile her iki platformda kullanılamaz. Jetson build’inde hata veren paketler:

- python3-colcon-common-extensions  
- python3-rosdep  
- python3-vcstool  

Bu nedenle yapı ayrıştırılmıştır:

- **PC → Dockerfile.pc**  
- **Jetson → Dockerfile.jetson**

---

## 8. ROS2 Test Komutları

Node list:
```bash
ros2 node list
```

Topic list:
```bash
ros2 topic list
```

Topic içerik:
```bash
ros2 topic echo /scan
```

Publish test:
```bash
ros2 topic pub /diagnostics std_msgs/String "data: 'test'"
```

---

## 9. Sistem Başlangıç Akışı

1. `.env` dosyası yüklenir  
2. Docker Compose hedef profile göre build eder  
3. ros_entrypoint.sh ROS ortamını source eder  
4. `ros2_ws` colcon ile build edilir  
5. Node’lar devreye alınır  
6. DDS discovery ile Jetson ↔ PC otomatik bağlanır  

---

## 10. Özet

Bu README, GES robot platformunun **ROS2 + Docker altyapısını**, node-topology yapısını, LIDAR entegrasyonunu, build–run süreçlerini ve Jetson/PC mimari farklarını uçtan uca açıklayan temel başvuru dokümanıdır. Bu adımlar izlenerek sistem sıfırdan kurulabilir ve eksiksiz şekilde çalıştırılabilir.
