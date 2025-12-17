#!/usr/bin/env python3
import rclpy
import random
import math
from rclpy.node import Node

# Turtlesim'in kendi servisleri (Yaratma ve Öldürme)
from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from functools import partial

# Özel mesaj ve servis türleri
from turtlesim_interfaces.msg import Turtle
from turtlesim_interfaces.msg import TurtleArray
from turtlesim_interfaces.srv import CatchTurtle


class SpawnTurtleNode(Node): #1
    def __init__(self):
        # 1. Düğümü "spawn_turtle_node" adıyla başlatıyoruz
        super().__init__("spawn_turtle_node") 
        
        self.name_ = "turtle_" # Kaplumbağa isim ön eki (turtle_2, turtle_3 vb.)
        self.counter_ = 1      # İsimlendirme için sayaç
        self.new_turtles_ = [] # Şu an hayatta olan kaplumbağaların listesi

        # 2. İletişim Kurulumları
        # Canlı kaplumbağa listesini diğer nodlara (örn: avcı robota) duyurur
        self.new_turtle_publisher_ = self.create_publisher(TurtleArray, "new_turtles", 10)
        
        # Bir Servis Sunucusu (Server) oluşturur.
        # Avcı robot, bir kaplumbağayı yakaladığında bu servisi çağırır.
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)

        # 3. Zamanlayıcı
        # Her 5.0 saniyede bir yeni kaplumbağa yaratmak için fonksiyonu tetikler
        self.timer_ = self.create_timer(5.0, self.spawn_turtle)

    # Avcı robot "catch_turtle" servisini çağırdığında bu fonksiyon çalışır
    def callback_catch_turtle(self, request, response):
        # İsteği (request) yapan robotun gönderdiği isimdeki kaplumbağayı yok et (kill)
        self.call_kill_server(request.name)
        response.success = True # Başarılı cevabı döndür
        return response


    # Canlı kaplumbağa listesini yayınlayan yardımcı fonksiyon
    def publish_new_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.new_turtles_ # Listeyi mesaja yükle
        self.new_turtle_publisher_.publish(msg) # Yayınla


    # Rastgele konum belirleyip yaratma servisini çağıran ana fonksiyon
    def spawn_turtle(self):
        self.counter_ += 1
        turtle_name = self.name_ + str(self.counter_) # Örn: turtle_2
        
        # 0 ile 11 arasında rastgele X ve Y koordinatları (Turtlesim ekran boyutu)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2 * math.pi) # Rastgele yön
        
        # Turtlesim'in kendi 'spawn' servisine istek gönder
        self.call_spawn_turtle_server(x, y, theta, turtle_name)

    # Turtlesim /spawn servisine istek gönderen istemci (Client) fonksiyonu
    def call_spawn_turtle_server(self, x, y, theta, turtle_name):
        client_ = self.create_client(Spawn, "/spawn")
        
        # Servis hazır olana kadar bekle
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Wating for server - [Spawn Turtles]")

        # İstek paketini hazırla
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        # Asenkron çağrı yap
        future = client_.call_async(request)
        # İşlem bitince çalışacak callback'i ayarla (parametreleri de aktararak)
        future.add_done_callback(partial(self.callback_call_spawn_turtle, x=x, y=y, theta=theta, turtle_name=turtle_name))


    # /spawn servisi tamamlandığında (kaplumbağa yaratıldığında) çalışan fonksiyon
    def callback_call_spawn_turtle(self, future, x, y, theta, turtle_name):
        try:
            response = future.result()
            # Eğer isim boş değilse başarıyla yaratılmıştır
            if response.name != "":
                self.get_logger().info("Turtle : "+ response.name + "has been created")
                
                # Yeni kaplumbağayı kendi listemize (veritabanımıza) ekliyoruz
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.new_turtles_.append(new_turtle)
                
                # Listeyi güncellediğimiz için hemen yayınlıyoruz
                self.publish_new_turtles()
                
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    # Turtlesim /kill servisine istek gönderen fonksiyon (Kaplumbağayı siler)
    def call_kill_server(self, turtle_name):
        client_ = self.create_client(Kill, "/kill")
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Wating for server - [Kill Turtles]")

        request = Kill.Request()
        request.name = turtle_name

        future = client_.call_async(request)
        # Silme işlemi bitince callback_call_kill_turtle çalışacak
        future.add_done_callback(partial(self.callback_call_kill_turtle, turtle_name=turtle_name))


def main(args=None):
    rclpy.init(args=args) # ROS2 iletişimini başlat
    node = SpawnTurtleNode() # Düğümü oluştur
    rclpy.spin(node) # Düğümü canlı tut (callback'leri dinle)
    rclpy.shutdown() # Kapat

    
if __name__ == "__main__":
    main()