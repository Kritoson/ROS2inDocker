# GES ROS2 + Docker Sistem Mimarisi ve Kullanım Kılavuzu

Bu doküman, GES projesinin Docker ve ROS2 altyapısı üzerinde nasıl çalıştığını, mimariyi, kullanılan nodeları, topic yapısını, dosya hiyerarşisini, build–run süreçlerini ve Jetson/PC tarafındaki ayrımları detaylı şekilde açıklar.

---

## 1. Sistem Mimarisi

GES sistemi iki farklı donanım hedefi için tasarlanmıştır:

* **PC / Simulation (x86_64)** → Gazebo, RViz2, simülasyon araçları, ROS2 Humble (x86) ortamı
* **Jetson Xavier / Robot (arm64)** → Gerçek robot donanımı, motor sürücüler, sensörler, UART, GPIO

Bu iki taraf **aynı ROS2 workspace yapısını** kullanır, ancak **farklı Docker imajları** üzerinden build edilir.

### 1.1. Ana Mimarinin Özeti

* Docker Compose ile **iki profil** kullanılır:

  * `sim` → PC tarafı (x86) için ROS2 simülasyon mimarisi
  * `robot` → Jetson ARM64 hedefli gerçek robot mimarisi

* Her iki profil, aynı `ros2_ws` klasörünü kullanır.

* Jetson tarafında GPU, UART, GPIO gibi donanımlar container içinde mount edilir.

* PC tarafında Gazebo + RViz2 GPU hızlandırması kullanılır.

---

## 2. Dosya Hiyerarşisi

```
ROS2inDocker/
│
├── docker-compose.yml
├── .env
├── Dockerfile.pc          # x86_64 build
├── Dockerfile.jetson      # arm64 build (Jetson)
│
└── ros2_ws/
    ├── src/
    │   ├── motor_control/         # Motor komutları ve sürücü kontrol nodeları
    │   ├── keyboard_input/        # PC klavye teleop node
    │   ├── led_control/           # Jetson GPIO LED kontrol node
    │   └── ... diğer paketler ...
    │
    ├── build/
    ├── install/
    └── log/
```

---

## 3. Kullanılan Başlıca ROS2 Nodeları

### 3.1. `keyboard_input` (PC – Simulation)

* Klavyeden ok tuşlarını okur
* `/cmd_vel` topic’ine Twist mesajı gönderir

### 3.2. `motor_control`

* Jetson üzerinde motor sürücülere hız komutu gönderir
* `/cmd_vel` mesajlarını işler
* Gerektiğinde UART ile motor driver iletişimi yapar

### 3.3. `led_control` (Jetson)

* Jetson'un GPIO pinlerini kontrol eder
* `/led_toggle` benzeri bir topic üzerinden çalışır

---

## 4. Topic Yapısı

| Topic          | Tip                   | Açıklama                            |
| -------------- | --------------------- | ----------------------------------- |
| `/cmd_vel`     | `geometry_msgs/Twist` | Keyboard → motor_control veri akışı |
| `/led_toggle`  | `std_msgs/Bool`       | Jetson LED kontrol                  |
| `/diagnostics` | `std_msgs/String`     | Sistem durumu                       |

---

## 5. Docker Compose Mimarisi

### 5.1. Sim (PC) Profili

* x86 ortam için build eder
* GPU hızlandırmalı Gazebo/RViz çalışma ortamı sağlar
* Klavye kontrol node’unu çalıştırır

### 5.2. Robot (Jetson) Profili

* ARM64 build
* Jetson cihaz erişimleri:

  * `/dev/ttyUSB0` (motor driver)
  * `/dev/gpiochip*` (GPIO)
  * `/dev/dri` (GPU)

---

## 6. Build ve Çalıştırma Komutları

### 6.1. PC Tarafı (Simulation)

İmajı build et:

```
docker compose --profile sim build
```

Simülasyonu başlat:

```
docker compose --profile sim up
```

Container içine gir:

```
docker exec -it ros2_sim bash
```

ROS workspace’i build et:

```
cd /workspace/ros2_ws\colcon build --symlink-install
```

Keyboard teleop başlat:

```
ros2 run keyboard_input teleop
```

---

### 6.2. Jetson Tarafı (Robot)

İmajı build et:

```
docker compose --profile robot build
```

Robot tarafını başlat:

```
docker compose --profile robot up
```

Container içine gir:

```
docker exec -it ros2_robot bash
```

ROS2 build:

```
cd /workspace/ros2_ws\colcon build --symlink-install
```

Motor kontrol node’u çalıştır:

```
ros2 run motor_control motor_control_node
```

---

## 7. Jetson ve PC Arasındaki Önemli Build Farkları

Aynı Dockerfile iki tarafta kullanılamadığı için şu ayrım vardır:

* PC’de kullanılan paketler Jetson’da build hatası oluşturur:

  * python3-colcon-common-extensions
  * python3-rosdep
  * python3-vcstool

Çözüm → iki ayrı Dockerfile kullanılır (`Dockerfile.pc` ve `Dockerfile.jetson`).
Jetson için minimal, PC için tam ROS geliştirme ortamı kuruludur.

---

## 8. ROS2 Test Komutları

Aktif nodları listele:

```
ros2 node list
```

Topicleri listele:

```
ros2 topic list
```

Topic echo:

```
ros2 topic echo /cmd_vel
```

Publisher test:

```
pub /diagnostics std_msgs/String "data: 'test'"
```

---

## 9. Sistem Nasıl Ayağa Kalkar?

### 1) `.env` dosyası yüklenir

Örn:

```
TARGET_ARCH=arm64
ROS_DOMAIN_ID=15
```

### 2) Docker Compose seçilen profile göre imajı build eder

### 3) ros_entrypoint.sh ROS ortamını source eder

### 4) Workspace build edilir (`colcon build`)

### 5) Sistem servisleri devreye girer

* motor_control
* keyboard_input (sim)
* led_control

### 6) ROS ağının DDS üzerinden iletişimi başlar

Jetson ve PC aynı ağda çalışıyorsa otomatik discovery gerçekleşir.

---

## 10. Özet

Bu doküman, GES projesi için Docker + ROS2 birleşik mimarisinin nasıl çalıştığını, build süreçlerini, node-topology yapısını ve Jetson/PC tarafındaki farkları açıklayan ana referans dokümandır. Tüm geliştirme süreçleri bu README.md üzerinden izlenerek yönetilebilir.

---
