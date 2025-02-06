from geometry_msgs.msg import Twist
from pynput import keyboard  
from rclpy.node import Node
import threading
import rclpy

class VesselController(Node):
    def __init__(self):
        # Gemi kontrolü için node oluşturuluyor
        super().__init__('vesselController')  
        
        # Geminin hareketini yayınlayacak bir publisher
        self.publisher_ = self.create_publisher(Twist, '/vessel_a/cmd_vel', 10)  

        # Mevcut hızlar (lineer ve açısal)
        self.currentVelocity = 0.0
        self.currentAngularVelocity = 0.0

        # Uygulanan kuvvet ve tork değerleri (komutlara bağlı olarak değişir)
        self.appliedForce = 0.0
        self.appliedTorque = 0.0

        # Dinamik model parametreleri
        self.mass = 1.0  # Araç kütlesi
        self.inertia = 1.0  # Atalet momenti
        self.dt = 0.1  # Güncelleme periyodu (saniye)

        # Timer callback: her dt süresinde hızı güncelle
        self.timer = self.create_timer(self.dt, self.update)

        # Klavye dinleyicisini ayrı bir iş parçacığında çalıştırıyoruz.
        self.keyboardThread = threading.Thread(target=self.keyboardListener)
        # Daemon thread, ana iş parçacığı kapandığında otomatik olarak kapanır
        self.keyboardThread.daemon = True  
        self.keyboardThread.start()

    def update(self):
        # F = m * a  =>  a = F / m 
        acceleration = self.appliedForce / self.mass
        angularAcceleration = self.appliedTorque / self.inertia

        # Hız güncellemesi
        self.currentVelocity += acceleration * self.dt
        self.currentAngularVelocity += angularAcceleration * self.dt

        # hızları azaltma
        damping = 0.9  # Sönümleme faktörü
        self.currentVelocity *= damping
        self.currentAngularVelocity *= damping


        # Hesaplanan hızlara göre Twist mesajını oluşturup yayınlıyoruz
        twist = Twist()
        twist.linear.x = self.currentVelocity
        twist.angular.z = self.currentAngularVelocity
        self.publisher_.publish(twist)

    def keyboardListener(self):
        def onPress(key):
            try:
                # Yukarı ok tuşuna basılınca ileriye hareket
                if key == keyboard.Key.up:  
                    self.appliedForce = 5.0  
                # Aşağı ok tuşuna basılınca geri hareket
                elif key == keyboard.Key.down:  
                    self.appliedForce = -5.0  
                # Sol ok tuşuna basılınca sola dönme
                elif key == keyboard.Key.left: 
                    self.appliedTorque = 2.0  
                # Sağ ok tuşuna basılınca sağa dönme
                elif key == keyboard.Key.right:  
                    self.appliedTorque = -2.0  
            except AttributeError:
                pass

        def onRelease(key):
            self.appliedForce = 0.0
            self.appliedTorque = 0.0

            # ESC tuşuna basıldığında program sonlanır
            if key == keyboard.Key.esc:  
                return False

        # Klavyeyi dinle
        with keyboard.Listener(on_press=onPress, on_release=onRelease) as listener:
            listener.join()

    # Node kapatildiginda klavyeyide kapat
    def destroy_node(self):
        self.keyboardThread.join()  
        super().destroy_node() 

def main(args=None):
    rclpy.init(args=args)  
    vesselController = VesselController()  

    try:
        rclpy.spin(vesselController)  
    finally:
        vesselController.destroy_node()  
        rclpy.shutdown()  

# Ana fonksiyonu çalıştırıyoruz
if __name__ == '__main__':
    main()  
