import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import math

class VesselController(Node):
    def __init__(self):
        super().__init__('vessel_controller')
        self.publisher_ = self.create_publisher(Twist, '/vessel_a/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/vessel_a/camera/image_raw',
            self.imageCallback,
            10
        )
        self.bridge = CvBridge()
        self.model = YOLO('weights/230_epochs/weights/best.pt')
        
        self.current_velocity = 0.1
        self.current_angular_velocity = 0.0
        self.applied_force = 0.0
        self.applied_torque = 0.0
        # arac kütlesi
        self.mass = 1.0
        # atalet
        self.inertia = 1.0
        # zaman adımı (frekansı)
        self.dt = 0.1

        self.timer = self.create_timer(self.dt, self.update)
        self.last_target_pair = None
        self.target_history = []  # Hedef geçmişi
        self.alpha = 0.2  # EMA ağırlık katsayısı

        self.prev_error_x = 0.0
        self.integral_x = 0.0
        # pid yapisi
        self.Kp_angular = 0.002  # Daha düşük kazanç
        self.Ki_angular = 0.0001  # Entegre kazancı
        self.Kd_angular = 0.001  # Türev kazancı

    def update(self):
        # hız ve yön güncelleme     
        acceleration = self.applied_force / self.mass
        angular_acceleration = self.applied_torque / self.inertia

        # ivme hesabı
        self.current_velocity += acceleration * self.dt
        self.current_angular_velocity += angular_acceleration * self.dt

        # yeni hiz belirleleme
        damping = 0.95  # Daha yüksek sönümleme faktörü
        self.current_velocity *= damping
        self.current_angular_velocity *= damping

        # ros2 mesajı yayinla
        twist = Twist()
        twist.linear.x = self.current_velocity
        twist.angular.z = self.current_angular_velocity
        self.publisher_.publish(twist)

    def adjustMovement(self, avg_x, ref_x, avg_y, ref_y):
        # hatayi hesapla
        error_x = avg_x - ref_x
        self.integral_x += error_x * self.dt
        derivative_x = (error_x - self.prev_error_x) / self.dt

        # PID ile verilmesi gereken kuvvet
        self.applied_torque = -(self.Kp_angular * error_x + self.Ki_angular * self.integral_x + self.Kd_angular * derivative_x)
        self.applied_force = 0.01 * (avg_y - ref_y)

        # Yön hatasını sınırlandırma (örn. -1 ile 1 arası)
        if self.applied_torque > 1.0:
            self.applied_torque = 1.0
        elif self.applied_torque < -1.0:
            self.applied_torque = -1.0

        self.prev_error_x = error_x

    def imageCallback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model.predict(frame, max_det=5, iou=0.6)

        ref_x, ref_y = frame.shape[1] // 2, (frame.shape[0] // 2 + 50)
        detections = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                detections.append((cx, cy))
                
                # Algılanan kutuları çiz
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # Orta noktaları işaretle
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        if len(detections) >= 2:
            # Y değerleri arasındaki farkı kontrol et
            max_y_diff = 50  # Y değerleri arasındaki maksimum farkı belirleyin (bu değeri ihtiyaca göre ayarlayın)
            detections.sort(key=lambda x: x[1], reverse=True)

            # Yüksek y değeri farkına sahip olan hedefleri eleyin
            if abs(detections[0][1] - detections[1][1]) < max_y_diff:
                self.last_target_pair = detections[0], detections[1]

                if self.target_history:
                    prev_x, prev_y = self.target_history[-1]
                    avg_x = self.alpha * ((detections[0][0] + detections[1][0]) // 2) + (1 - self.alpha) * prev_x
                    avg_y = self.alpha * ((detections[0][1] + detections[1][1]) // 2) + (1 - self.alpha) * prev_y
                else:
                    avg_x = (detections[0][0] + detections[1][0]) // 2
                    avg_y = (detections[0][1] + detections[1][1]) // 2

                self.target_history.append((avg_x, avg_y))
                if len(self.target_history) > 10:
                    self.target_history.pop(0)
            else:
                # Yüksek y değeri farkı olduğunda, hedefleri belirlemeyin ve default referans noktalarına dönün
                avg_x, avg_y = ref_x, ref_y
        elif len(detections) == 1:
            if self.target_history:
                avg_x, avg_y = self.target_history[-1]
            else:
                avg_x, avg_y = ref_x, ref_y
        else:
            if self.target_history:
                avg_x, avg_y = self.target_history[-1]
            else:
                avg_x, avg_y = ref_x, ref_y

        self.adjustMovement(avg_x, ref_x, avg_y, ref_y)

        # Hedeflenen noktayı işaretle
        if len(detections) >= 2 and abs(detections[0][1] - detections[1][1]) < max_y_diff:
            cv2.circle(frame, (int(avg_x), int(avg_y)), 7, (255, 0, 0), -1)
            cv2.line(frame, (ref_x, ref_y), (int(avg_x), int(avg_y)), (0, 255, 255), 2)
            if self.last_target_pair:
                cv2.line(frame, self.last_target_pair[0], self.last_target_pair[1], (0, 0, 255), 2)
        
        # Geminin ortasını çiz
        cv2.circle(frame, (ref_x, ref_y), 7, (255, 255, 0), -1)

        # Görüntüyü göster
        cv2.imshow('frame', frame)
        cv2.waitKey(1)



    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    vessel_controller = VesselController()
    
    try:
        rclpy.spin(vessel_controller)
    finally:
        vessel_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
