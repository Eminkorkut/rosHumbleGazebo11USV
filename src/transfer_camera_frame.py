from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.node import Node
import rclpy
import time
import cv2
import os

class ImageSubscriber(Node):
    def __init__(self):
        # ROS2 node başlat
        super().__init__('imageSubscriber')
        
        # Kameradan görüntü almak için abone oluştur
        self.subscription = self.create_subscription(
            Image,
            '/vessel_a/camera/image_raw',
            self.imageCallback,
            10
        )
        
        # CvBridge nesnesi, ROS görüntülerini OpenCV formatına dönüstür
        self.bridge = CvBridge()
        
        # YOLO modelini dahil et
        self.model = YOLO('weights/230_epochs/weights/best.pt')  

    def imageCallback(self, msg):
        # ROS mesajını OpenCV formatına çeviriyoruz
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # nesne tanıma işlemi
        results = self.model(frame)
        
        # Modelin sonuçlarını kullanarak box çiziyoruz
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0]
                classId = int(box.cls[0])
                
                label = f'{self.model.names[classId]} {conf:.2f}' 

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  
                cv2.putText(frame, label, (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  

        cv2.imshow('frame', frame)  
        cv2.waitKey(1)  

# Ana fonksiyon
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  

# Ana fonksiyonu çalıştırıyoruz
if __name__ == '__main__':
    main()
