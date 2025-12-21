ROS2 Turtlesim Uygulamaları 🐢
Bu depo, ROS2 (Robot Operating System 2) öğrenme sürecimde gerçekleştirdiğim turtlesim tabanlı uygulama çalışmalarını içermektedir. Proje kapsamında ROS2'nin temel yapı taşları olan Nodes (Düğümler), Topics (Konular) ve Services (Servisler) kullanılarak çeşitli robotik senaryolar simüle edilmiştir.

🚀 Proje İçeriği
Bu eğitim serisinde aşağıdaki yetkinlikler üzerine çalışılmıştır:

Node Oluşturma: C++ ve Python (veya hangisini kullandıysan) dilleriyle ROS2 düğümleri geliştirme.

Topic Haberleşmesi: geometry_msgs/msg/Twist mesaj tipini kullanarak kaplumbağayı hareket ettirme (Publisher) ve konum verisini (turtlesim/msg/Pose) okuma (Subscriber).

Service & Client Yapısı: spawn, kill, set_pen gibi servisleri kullanarak simülasyonu dinamik olarak yönetme.

Algoritmik Hareket: Kaplumbağanın belirli bir hedefe gitmesi veya belirli şekiller çizmesi için kontrol algoritmaları.

🛠 Kurulum ve Çalıştırma
Gereksinimler
ROS2 (Humble/Foxy veya kullandığın sürüm)

turtlesim paketi

Adımlar
Workspace Oluşturun ve Depoyu Klonlayın:

Bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MustafaSelimBakkal/Ros2_Turtlesim_Uygulamalar.git
Bağımlılıkları Yükleyin ve Derleyin:

Bash

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
Uygulamayı Çalıştırın: (Not: Buradaki 'paket_adi' ve 'düğüm_adi' kısımlarını kendi koduna göre güncellemelisin)

Bash

ros2 run <paket_adi> <düğüm_adi>
📂 Dosya Yapısı
my_robot_controller/: Ana kodların bulunduğu ROS2 paketi.

src/: Kaynak kodlar (Python/C++).

launch/: (Varsa) Birden fazla düğümü aynı anda başlatan launch dosyaları.

📈 Öğrenim Çıktıları
Bu çalışma sonucunda:

ROS2 ekosisteminde paket yönetimi kavrandı.

Yayıncı (Publisher) ve Abone (Subscriber) arasındaki veri akışı simüle edildi.

Robotik sistemlerde koordinat sistemi ve hız vektörleri üzerine pratik yapıldı.

👤 Mustafa Selim Bakkal

LinkedIn: www.linkedin.com/in/mustafaselimbakkal

GitHub: @MustafaSelimBakkal
