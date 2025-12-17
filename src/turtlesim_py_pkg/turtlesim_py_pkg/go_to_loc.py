#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node

# Robotun hız komutları (lineer ve açısal) için gerekli mesaj tipi
from geometry_msgs.msg import Twist
# Kaplumbağanın mevcut konumunu okumak için gerekli mesaj tipi
from turtlesim.msg import Pose

# Bu projeye özel oluşturulmuş (muhtemelen custom interface) mesaj türleri
# Turtle: Tek bir kaplumbağa verisi, TurtleArray: Kaplumbağa listesi
from turtlesim_interfaces.msg import Turtle
from turtlesim_interfaces.msg import TurtleArray
# Kaplumbağayı yakalamak (yok etmek) için servis
from turtlesim_interfaces.srv import CatchTurtle

# Callback fonksiyonuna ekstra parametre göndermek için kullanılır
from functools import partial


class GoToLocationNode(Node): #1
    def __init__(self):
        # 1. Düğümü "go_to_loc_node" ismiyle başlatıyoruz
        super().__init__("go_to_loc_node") 
        
        # Hedeve vardık mı kontrolü için hassasiyet eşikleri
        self.pose_threshold_linear = 0.5   # Hedefe 0.5 birim yaklaşırsak varmış sayacağız
        self.pose_threshold_angular = 0.01 # Açı farkı 0.01 radyandan azsa dönmeyi bırakacağız

        self.coeff = 1.5 # P-Kontrolcü katsayısı (Hız çarpanı)

        # Robotun anlık konumu ve yakalanacak hedef kaplumbağa verisi için değişkenler
        self.pose_ = None
        self.new_turtle_to_catch_ = None

        # 2. İletişim Kurulumları (Pub/Sub)
        # Hız komutlarını yayınlayacak Publisher (Yayıncı)
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        # Kendi konumumuzu dinleyen Subscriber (Abone)
        self.subcriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_turtle_pose, 10)
        
        # Ortamdaki yeni kaplumbağaların listesini dinleyen Subscriber
        self.new_turtle_subscriber_ = self.create_subscription(TurtleArray, "/new_turtles", self.callback_new_turtles, 10)

        # 3. Kontrol Döngüsü
        # Her 1 saniyede bir 'turtle_controller' fonksiyonunu çalıştıran zamanlayıcı.
        # (Not: 1 saniye otonom sürüş için biraz yavaştır, hareket kesik kesik olabilir)
        self.timer_ = self.create_timer(1, self.turtle_controller)
        
        self.get_logger().info("Go To Location Node has been started")  


    # Konum verisi geldiğinde çalışan fonksiyon: Konumu günceller
    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    # Yeni kaplumbağa listesi geldiğinde çalışan fonksiyon
    def callback_new_turtles(self, msg):
        # Eğer listede en az bir kaplumbağa varsa, ilkini hedef olarak belirle
        if len(msg.turtles) > 0:
            self.new_turtle_to_catch_ = msg.turtles[0]

    # 4. Ana Kontrol Mantığı (Robotun beyni)
    def turtle_controller(self):
        # Eğer henüz konumumuz veya bir hedefimiz yoksa hiçbir şey yapma
        if self.pose_ == None or self.new_turtle_to_catch_ == None:
            return

        msg = Twist()
        
        # Hedef ile aramızdaki X ve Y farkını hesapla
        dist_x = self.new_turtle_to_catch_.x - self.pose_.x
        dist_y = self.new_turtle_to_catch_.y - self.pose_.y

        # Pisagor teoremi ile hedefe olan kuş bakışı uzaklığı (hipotenüs) hesapla
        distance = math.sqrt(dist_x**2 + dist_y**2)
        
        # Hedefin bize göre hangi açıda (radyan) durduğunu hesapla (atan2)
        target_theta = math.atan2(dist_y, dist_x)

        # 5. Hareket Mantığı (Önce Dön, Sonra Git)
        
        # Açı farkı (hata) eşik değerden büyükse: Sadece DÖN
        if abs(target_theta - self.pose_.theta) > self.pose_threshold_angular:
            # Hedef açı ile mevcut açı farkını katsayı ile çarpıp açısal hız olarak ver (P-Kontrol)
            msg.angular.z = (target_theta - self.pose_.theta) * self.coeff
        
        # Yönümüz hedefe doğruysa:
        else:
            # Hedefe henüz varmadıysak: İLERİ GİT
            if distance >= self.pose_threshold_linear:
                msg.linear.x = distance * self.coeff # Uzaklık arttıkça hız artar
            
            # Hedefe vardıysak: DUR ve YAKALA
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                # Servisi çağırarak kaplumbağayı simülasyondan sil (yakala)
                self.call_catch_turtle_service(self.new_turtle_to_catch_.name) 
                self.new_turtle_to_catch_ = None # Hedefi sıfırla

                self.get_logger().info("Succes!")
        
        # Hazırlanan hız komutunu robota gönder
        self.publisher_.publish(msg)

    # Kaplumbağa yakalama servisini (Client) çağıran fonksiyon
    def call_catch_turtle_service(self, turtle_name):
        client_ = self.create_client(CatchTurtle, "/catch_turtle")
        
        # Servis aktif olana kadar bekle
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Wating for server - [Catch Turtle]")

        # Servis isteğini oluştur
        request = CatchTurtle.Request()
        request.name = turtle_name

        # İsteği asenkron (arka planda) gönder
        future = client_.call_async(request)
        
        # İşlem bittiğinde çalışacak callback'i ayarla.
        # 'partial' kullanarak callback'e 'turtle_name' parametresini de ekliyoruz.
        future.add_done_callback(partial(self.callback_call_catch_turtle, turtle_name=turtle_name))


    # Servisten cevap geldiğinde çalışacak fonksiyon
    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result() # Sonucu al (Genelde boş veya başarı mesajı döner)
            self.get_logger().info("Turtle : "+ turtle_name + " has been caught")  
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args) # ROS2 iletişimini başlat
    node = GoToLocationNode() # Düğümü oluştur
    rclpy.spin(node) # Düğümü canlı tut (callback'leri dinle)
    rclpy.shutdown() # Kapat

    
if __name__ == "__main__":
    main()