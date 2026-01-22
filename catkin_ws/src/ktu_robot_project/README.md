 ktu_robot_project

Bu proje, **TurtleBot3** kullanılarak **kapalı bir ev ortamında** (turtlebot3_house) **haritalama (SLAM)** ve **zigzag tarama** davranışlarını gerçekleştiren bir ROS (Robot Operating System) uygulamasıdır.

Proje iki ana fazdan oluşur:

1. **Oda bazlı duvar takibi ile haritalama (kapılara girmeden)**
2. **Dikdörtgen alan için dikey zigzag tarama algoritması**

Amaç: Robotun ev içindeki **sadece odaları** baz alarak harita çıkarması ve ardından belirlenen alanı **düzenli zigzag deseniyle** taramasıdır.

---

 Paket Bilgileri

* **Paket Adı:** ktu_robot_project
* **ROS Dağıtımı:** ROS Noetic
* **Simülasyon:** Gazebo
* **Robot:** TurtleBot3
* **Haritalama:** gmapping
* **Dil:** Python (rospy)

---

 Genel Mimari

Proje aşağıdaki bileşenlerden oluşur:

* `wall_follower_script.py`
  → Oda içi **duvar takibi** yapar, **kapıları algılar ama içeri girmez**.

* `zigzag_mapper.py`
  → Alanı **dikey zigzag** şeklinde tarar, satır sayısını takip eder ve görev bitince robotu durdurur.

* **Launch dosyaları**
  → Gazebo ortamı, SLAM ve ilgili node’ları başlatır.

---

  1. Oda Bazlı Haritalama (Wall Follower)

  Amaç

* Robotun sadece **odaların içini** haritalaması
* **Kapıları algılayıp içeri girmemesi**
* Sürekli **sağ duvar takibi** yapması

  Kullanılan Topic’ler

* **Sub:** `/scan` (LaserScan)
* **Pub:** `/cmd_vel` (Twist)

  Temel Mantık

Laser verisi üç bölgeye ayrılır:

* **Ön** (engel kontrolü)
* **Sağ** (duvar mesafesi)

Davranışlar:

* Ön kapalıysa → **Sola dön**
* Sağ taraf çok boş & ön açık → **Kapı algılandı, düz devam et**
* Duvar çok yakın → **Sola açıl**
* Duvar kayboluyorsa → **Sağa kır**
* Aksi halde → **Normal duvar takibi**

---

  2. Dikey Zigzag Tarama Algoritması

 Amaç

* Dikdörtgen alanı **satır satır** taramak
* Her satırda **düz ilerleme + yan kayma** yapmak
* Belirlenen satır sayısında görevi bitirmek

 Kullanılan Topic’ler

* **Sub:** `/scan` (engel algılama)
* **Sub:** `/odom` (yön ve konum bilgisi)
* **Pub:** `/cmd_vel`

 State (Durum) Makinesi

Algoritma bir **state machine** yapısına sahiptir:

1. `GO_TO_FIRST_WALL` – İlk duvara kadar ilerleme
2. `TURN_TO_START` – Zigzag başlangıç yönüne dönme
3. `FORWARD` – Satır tarama
4. `TURN_1` – Yan şeride geçiş için dönüş
5. `SHIFT` – Yan kayma
6. `TURN_2` – Yeni satıra hizalanma

Her satır tamamlandığında:

* Yön 180° çevrilir
* Satır sayacı artırılır

Belirlenen satır sayısı (örn. **14**) tamamlandığında:

* Robot durur
* Node kendini kapatır

---

  Launch Dosyaları

  Haritalama Launch

* TurtleBot3 house ortamı başlatılır
* gmapping SLAM çalıştırılır
* Duvar takip node’u aktif edilir

  Zigzag Launch

* Gazebo house ortamı açılır
* Zigzag tarama node’u çalıştırılır

---

  Dosya Yapısı

```
ktu_robot_project/
│── launch/
│   ├── mapping.launch
│   └── zigzag.launch
│
│── scripts/
│   ├── wall_follower_script.py
│   └── zigzag_mapper.py
│
│── package.xml
│── README.md
```

---

  Projenin Sağladıkları

* Kapıdan içeri dalmayan akıllı haritalama 
* Düzenli ve deterministik zigzag tarama 
* SLAM ile uyumlu yapı 
* Modüler ve anlaşılır kod mimarisi

---

  Çalıştırma

```bash
catkin_make
source devel/setup.bash
roslaunch ktu_robot_project mapping.launch
 veya
roslaunch ktu_robot_project zigzag.launch
```

---

  Geliştirici

* **İsim:** Ibrahim Halil Altundag
* **Paket:** ktu_robot_project


