from geometry_msgs.msg import Twist
from pynput import keyboard  
from rclpy.node import Node
import threading
import rclpy

class VesselController(Node):
    def __init__(self):
        super().__init__('vessel_controller')
        self.publisher_ = self.create_publisher(Twist, '/vessel_a/cmd_vel', 10)
        
        # Mevcut hızlar (lineer ve açısal)
        self.current_velocity = 0.0
        self.current_angular_velocity = 0.0

        # Uygulanan kuvvet ve tork değerleri (komutlara bağlı)
        self.applied_force = 0.0
        self.applied_torque = 0.0

        # Dinamik model parametreleri
        self.mass = 1.0         # Araç kütlesi
        self.inertia = 1.0      # Atalet momenti
        self.dt = 0.1           # Güncelleme periyodu (saniye)

        # Timer callback: her dt süresinde hızı güncelle
        self.timer = self.create_timer(self.dt, self.update)

        # Klavye dinleyicisini ayrı bir iş parçacığında çalıştırıyoruz.
        self.keyboard_thread = threading.Thread(target=self.keyboardListener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def update(self):
        """
        Uygulanan kuvvet ve tork değerlerine göre aracın hızını hesaplar ve Twist mesajını yayınlar.
        Basit bir Euler entegrasyonu kullanılarak hız güncellemesi yapılır.
        """
        # F = m * a  =>  a = F / m
        acceleration = self.applied_force / self.mass
        angular_acceleration = self.applied_torque / self.inertia

        # Hız güncellemesi
        self.current_velocity += acceleration * self.dt
        self.current_angular_velocity += angular_acceleration * self.dt

        # Basit bir sönümleme (frenleme, hava direnci vb.) ekleyelim
        damping = 0.9
        self.current_velocity *= damping
        self.current_angular_velocity *= damping

        # Hesaplanan hızlara göre Twist mesajını oluşturup yayınlayalım.
        twist = Twist()
        twist.linear.x = self.current_velocity
        twist.angular.z = self.current_angular_velocity
        self.publisher_.publish(twist)

    def keyboardListener(self):
        def onPress(key):
            try:
                if key == keyboard.Key.up:  
                    self.applied_force = 5.0  
                elif key == keyboard.Key.down:  
                    self.applied_force = -5.0  
                elif key == keyboard.Key.left: 
                    self.applied_torque = 2.0  
                elif key == keyboard.Key.right:  
                    self.applied_torque = -2.0  
            except AttributeError:
                pass

        def onRelease(key):
            self.applied_force = 0.0
            self.applied_torque = 0.0

            if key == keyboard.Key.esc:  
                return False

        with keyboard.Listener(on_press=onPress, on_release=onRelease) as listener:
            listener.join()

    def destroy_node(self):
        self.keyboard_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    vessel_controller = VesselController()

    try:
        rclpy.spin(vessel_controller)
    except KeyboardInterrupt:
        pass

    vessel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
