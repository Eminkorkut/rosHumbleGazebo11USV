from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.node import Node
import rclpy
import cv2


class VesselController(Node):
    def __init__(self):
        super().__init__('vesselController')
        # gemi hareketlerini yayınlamak için bir yayıncı oluştur
        self.publisher_ = self.create_publisher(Twist, '/vessel_a/cmd_vel', 10)
        # kameradan görüntü almak için bir abone oluştur
        self.subscription = self.create_subscription(
            Image,
            '/vessel_a/camera/image_raw',
            self.imageCallback,
            10
        )
        # cv2 kütüphanesini kullanmak için bir nesne oluştur
        self.bridge = CvBridge()
        # YOLO modelini yükle
        self.model = YOLO('weights/230_epochs/weights/best.pt')
        
        # gemi hareketleri için değişkenler
        self.currentVelocity = 0.1
        self.currentAngularVelocity = 0.0
        self.appliedForce = 0.0
        self.appliedTorque = 0.0
        # arac kütlesi
        self.mass = 1.0
        # atalet momenti
        self.inertia = 1.0
        # zaman adımı (frekansı)
        self.dt = 0.1

        # zamanlayıcı oluştur
        self.timer = self.create_timer(self.dt, self.update)
        self.lastTargetPair = None
        self.targetHistory = []  # Hedef geçmişi
        self.alpha = 0.2  # EMA ağırlık katsayısı

        self.prevErrorX = 0.0
        self.integralX = 0.0
        # pid yapisi
        self.KpAngular = 0.002  # Daha düşük kazanç
        self.KiAngular = 0.0001  # Entegre kazancı
        self.KdAngular = 0.001  # Türev kazancı

    def update(self):
        # hız ve yön güncelleme     
        acceleration = self.appliedForce / self.mass
        angularAcceleration = self.appliedTorque / self.inertia

        # ivme hesabı
        self.currentVelocity += acceleration * self.dt
        self.currentAngularVelocity += angularAcceleration * self.dt

        # yeni hiz belirleleme
        damping = 0.95  # Daha yüksek sönümleme faktörü
        self.currentVelocity *= damping
        self.currentAngularVelocity *= damping

        # ros2 mesajı yayinla
        twist = Twist()
        twist.linear.x = self.currentVelocity
        twist.angular.z = self.currentAngularVelocity
        self.publisher_.publish(twist)

    def adjustMovement(self, avgX, refX, avgY, refY):
        # hatayi hesapla
        errorX = avgX - refX
        self.integralX += errorX * self.dt
        derivative_x = (errorX - self.prevErrorX) / self.dt

        # PID ile verilmesi gereken kuvvet
        self.appliedTorque = -(self.KpAngular * errorX + self.KiAngular * self.integralX + self.KdAngular * derivative_x)
        self.appliedForce = 0.01 * (avgY - refY)

        # Yön hatasını sınırlandırma (örn. -1 ile 1 arası)
        if self.appliedTorque > 1.0:
            self.appliedTorque = 1.0
        elif self.appliedTorque < -1.0:
            self.appliedTorque = -1.0

        self.prevErrorX = errorX

    def imageCallback(self, msg):
        # Görüntüyü al
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # YOLO modelini kullanarak nesneleri algıla
        results = self.model.predict(frame, max_det=5, iou=0.6)

        # Geminin orta noktasını al
        refX, refY = frame.shape[1] // 2, (frame.shape[0] // 2 + 50)
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
            max_y_diff = 50  # Y değerleri arasındaki maksimum fark (yanlis hedefleme durumunu azaltma)
            # Y değerine göre sırala (bize en yakin dubalar y degeri olarak daha buyuktur)
            detections.sort(key=lambda x: x[1], reverse=True)

            # belirlenen dubalar arasinda y degeri kontrolu
            if abs(detections[0][1] - detections[1][1]) < max_y_diff:
                self.lastTargetPair = detections[0], detections[1]

                # listenin bos olup olmadigini kontrol et
                if self.targetHistory:
                    prev_x, prev_y = self.targetHistory[-1]
                    avgX = self.alpha * ((detections[0][0] + detections[1][0]) // 2) + (1 - self.alpha) * prev_x
                    avgY = self.alpha * ((detections[0][1] + detections[1][1]) // 2) + (1 - self.alpha) * prev_y
                # hedefin orta noktasini al
                else:
                    avgX = (detections[0][0] + detections[1][0]) // 2
                    avgY = (detections[0][1] + detections[1][1]) // 2

                # hedef gecmisine hedef ekle
                self.targetHistory.append((avgX, avgY))

                # belirli sayida gecmis tut
                if len(self.targetHistory) > 10:
                    self.targetHistory.pop(0)
            
            # belirlenen dubalar y araliginin disinda ise son hedefe git
            else:
                avgX, avgY = refX, refY
        
        # Bir hedef varsa, hedefe git
        elif len(detections) == 1:
            if self.targetHistory:
                avgX, avgY = self.targetHistory[-1]
            else:
                avgX, avgY = refX, refY
        # Hedef yoksa, son hedefe git
        else:
            if self.targetHistory:
                avgX, avgY = self.targetHistory[-1]
            else:
                avgX, avgY = refX, refY

        # Hareketi ayarla
        self.adjustMovement(avgX, refX, avgY, refY)

        # Hedeflenen kutular belirli şartlara uygunsa hedefi çiz
        if len(detections) >= 2 and abs(detections[0][1] - detections[1][1]) < max_y_diff:
            cv2.circle(frame, (int(avgX), int(avgY)), 7, (255, 0, 0), -1)
            cv2.line(frame, (refX, refY), (int(avgX), int(avgY)), (0, 255, 255), 2)
            if self.lastTargetPair:
                cv2.line(frame, self.lastTargetPair[0], self.lastTargetPair[1], (0, 0, 255), 2)
        
        # Geminin ortasını çiz
        cv2.circle(frame, (refX, refY), 7, (255, 255, 0), -1)

        # Görüntüyü göster
        cv2.imshow('frame', frame)
        cv2.waitKey(1)


    # node'u kapat
    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()


# main fonksiyonu
def main(args=None):
    rclpy.init(args=args)
    vessel_controller = VesselController()
    try:
        rclpy.spin(vessel_controller)
    finally:
        vessel_controller.destroy_node()
        rclpy.shutdown()

# main fonksiyonunu çalıştır
if __name__ == '__main__':
    main()
