from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.node import Node
import numpy as np
import argparse
import torch
import rclpy
import cv2
import ast
import math

class usvController(Node):
    def __init__(self, args):
        super().__init__('usvController')
        self.args = args
        self.args.yoloDevice = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        # ROS2 publishers and subscribers
        self.publisher = self.create_publisher(Twist, args.publishTopic, 10)
        self.subscription = self.create_subscription(
            Image, args.subscribeTopic, self.imageCallBack, 10
        )
        
        # Initialize components
        self.initialize_cv_bridge()
        self.load_yolo_model()
        self.initialize_vehicle_parameters()
        self.initialize_pid_parameters()
        
        # Timer to update the vehicle's movement
        self.timer = self.create_timer(self.args.timeStep, self.update)
        
        # Navigation state
        self.targetHistory = []
        self.lastTargetPair = None
        self.current_frame = None
        self.latest_buoys = []
        
        # Safety parameters
        self.max_linear_velocity = 2.0
        self.max_angular_velocity = 1.0
        self.min_buoy_distance = 2.0  # meters
        
        self.get_logger().info("USV Controller initialized successfully")

    def initialize_cv_bridge(self):
        try:
            self.bridge = CvBridge()
        except Exception as e:
            self.get_logger().error(f'Error initializing CvBridge: {e}')
            raise

    def load_yolo_model(self):
        try:
            self.model = YOLO(self.args.yoloModelPath)
            self.get_logger().info(f'YOLO model loaded from: {self.args.yoloModelPath}')
        except Exception as e:
            self.get_logger().error(f'Error loading YOLO model: {e}')
            raise

    def initialize_vehicle_parameters(self):
        self.currentVelocity = 0.0
        self.currentAngularVelocity = 0.0
        self.appliedForce = 0.0
        self.appliedTorque = 0.0
        self.mass = self.args.vehicleMass
        self.inertia = self.args.vehicleInertia
        self.timeStep = self.args.timeStep

    def initialize_pid_parameters(self):
        self.prevErrorX = 0.0
        self.integralX = 0.0
        self.alpha = self.args.alpha
        self.kpAngular = self.args.kpAngular
        self.kiAngular = self.args.kiAngular
        self.kdAngular = self.args.kdAngular
        
        # Anti-windup için integral sınırları
        self.integral_max = 10.0
        self.integral_min = -10.0

    def update(self):
        """Ana kontrol döngüsü"""
        self.compute_new_velocities()
        self.publish_velocities()

    def compute_new_velocities(self):
        """Fizik tabanlı hız hesaplama"""
        # Sürüklenme kuvveti (drag force)
        drag_coefficient = 0.1
        drag_force = -drag_coefficient * self.currentVelocity * abs(self.currentVelocity)
        angular_drag = -drag_coefficient * self.currentAngularVelocity * abs(self.currentAngularVelocity)
        
        # Net kuvvet hesaplama
        net_force = self.appliedForce + drag_force
        net_torque = self.appliedTorque + angular_drag
        
        # İvme hesaplama
        acceleration = net_force / self.mass
        angularAcceleration = net_torque / self.inertia
        
        # Hız güncelleme
        self.currentVelocity += acceleration * self.timeStep
        self.currentAngularVelocity += angularAcceleration * self.timeStep
        
        # Hız sınırları
        self.currentVelocity = np.clip(self.currentVelocity, -self.max_linear_velocity, self.max_linear_velocity)
        self.currentAngularVelocity = np.clip(self.currentAngularVelocity, -self.max_angular_velocity, self.max_angular_velocity)

    def publish_velocities(self):
        """Hız komutlarını yayınla"""
        msg = Twist()
        msg.linear.x = float(self.currentVelocity)
        msg.angular.z = float(self.currentAngularVelocity)
        self.publisher.publish(msg)

    def adjustMovement(self, targetX: int, referenceX: int, targetY: int, referenceY: int):
        """Geliştirilmiş PID kontrolör"""
        if targetX is None or referenceX is None:
            self.appliedForce = 0.0
            self.appliedTorque = 0.0
            return
            
        # Açısal hata (yatay sapma)
        errorX = targetX - referenceX
        
        # Integral hesaplama (anti-windup ile)
        self.integralX += errorX * self.timeStep
        self.integralX = np.clip(self.integralX, self.integral_min, self.integral_max)
        
        # Türev hesaplama
        derivativeX = (errorX - self.prevErrorX) / self.timeStep
        
        # PID çıkışı
        pid_output = (self.kpAngular * errorX + 
                     self.kiAngular * self.integralX + 
                     self.kdAngular * derivativeX)
        
        # Torque sınırları
        self.appliedTorque = -np.clip(pid_output, -1.0, 1.0)
        
        # İleri hız kontrolü (mesafe tabanlı)
        distance_error = targetY - referenceY
        desired_speed = np.clip(distance_error * 0.01, 0.0, 0.5)
        
        # Güvenlik kontrolü - çok yakın dubalar varsa yavaşla
        if self.latest_buoys:
            min_distance = min([buoy[3] for buoy in self.latest_buoys]) / 1000.0  # mm to m
            if min_distance < self.min_buoy_distance:
                desired_speed *= 0.3  # Güvenlik için yavaşla
        
        self.appliedForce = desired_speed
        self.prevErrorX = errorX

    def calculate_distance_mm(self, width_px, focal_length_mm=3.18, real_width_mm=300, 
                            sensor_width_mm=5.6, image_width_px=1920):
        """Mesafe hesaplama - hata kontrolü ile"""
        try:
            if width_px <= 0:
                return float('inf')
                
            pixel_size = sensor_width_mm / image_width_px
            f_px = focal_length_mm / pixel_size
            distance_mm = (real_width_mm * f_px) / width_px
            
            # Mantıksız mesafe değerlerini filtrele
            if distance_mm > 50000 or distance_mm < 100:  # 50m - 10cm arası
                return float('inf')
                
            return distance_mm
        except (ZeroDivisionError, ValueError):
            return float('inf')

    def select_nearest_buoys(self, detectionObjectCenterList):
        """En yakın dubaları seç ve mesafe hesapla"""
        if not detectionObjectCenterList:
            return []
            
        buoy_scores = []
        
        for frame, centerX, centerY, area, width in detectionObjectCenterList:
            distance_mm = self.calculate_distance_mm(width)
            
            if distance_mm != float('inf'):
                buoy_scores.append((centerX, centerY, area, distance_mm))
                
                # Görselleştirme için mesafe yazısı
                distance_text = f"{distance_mm/1000:.1f}m"
                cv2.putText(frame, distance_text, (centerX, centerY - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Mesafeye göre sırala
        buoy_scores.sort(key=lambda x: x[3])
        
        # En fazla 5 duba döndür
        return buoy_scores[:5]

    def find_buoy_pairs(self, buoys):
        """Duba çiftlerini bul - geliştirilmiş algoritma"""
        if len(buoys) < 2:
            return []
            
        pairs = []
        maxYdifference = self.args.targetObjectsMaxDistance
        
        # Tüm duba çiftlerini kontrol et
        for i in range(len(buoys)):
            for j in range(i + 1, len(buoys)):
                buoy1, buoy2 = buoys[i], buoys[j]
                
                # Y koordinatı farkı kontrolü
                y_diff = abs(buoy1[1] - buoy2[1])
                if y_diff < maxYdifference:
                    # Çift kalitesi hesapla (mesafe ve y_diff kombinasyonu)
                    avg_distance = (buoy1[3] + buoy2[3]) / 2
                    quality_score = 1.0 / (avg_distance + y_diff)
                    
                    pairs.append((buoy1, buoy2, quality_score))
        
        # Kalite skoruna göre sırala
        pairs.sort(key=lambda x: x[2], reverse=True)
        
        return pairs

    def imageCallBack(self, msg: Image):
        """Ana görüntü işleme callback'i"""
        try:
            frame = self.convert_image(msg)
            self.current_frame = frame
            
            # Tekne merkezi
            mainBoatCenterX, mainBoatCenterY = self.get_boat_center(frame)
            
            # Nesne tespiti
            detectionObjectCenterList = self.detect_objects(frame)
            
            # En yakın dubaları seç
            nearest_buoys = self.select_nearest_buoys(detectionObjectCenterList)
            self.latest_buoys = nearest_buoys
            
            # Hedef pozisyon hesapla
            avgX, avgY = self.compute_target_position(nearest_buoys, mainBoatCenterX, mainBoatCenterY)
            
            # Hareket ayarla
            self.adjustMovement(avgX, mainBoatCenterX, avgY, mainBoatCenterY)
            
            # Görselleştir
            self.visualize(frame, nearest_buoys, avgX, avgY, mainBoatCenterX, mainBoatCenterY)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def convert_image(self, msg: Image):
        """ROS Image mesajını OpenCV formatına çevir"""
        try:
            return self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            raise

    def get_boat_center(self, frame):
        """Tekne merkezi (frame merkezi)"""
        height, width = frame.shape[:2]
        return width // 2, height // 2

    def detect_objects(self, frame):
        """YOLO ile nesne tespiti"""
        try:
            results = self.model.predict(
                frame,
                max_det=self.args.yoloMaxDetect,
                iou=self.args.yoloIou,
                conf=self.args.yoloConf,
                device=self.args.yoloDevice,
                verbose=self.args.yoloVerbose
            )
            
            detectionObjectCenterList = []
            
            for result in results:
                if result.boxes is not None:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        centerX = (x1 + x2) // 2
                        centerY = (y1 + y2) // 2
                        area = (x2 - x1) * (y2 - y1)
                        width = x2 - x1
                        
                        # Minimum boyut kontrolü
                        if width > 10 and (y2 - y1) > 10:
                            detectionObjectCenterList.append((frame, centerX, centerY, area, width))
                            self.draw_bounding_box(frame, x1, y1, x2, y2, centerX, centerY)
            
            return detectionObjectCenterList
            
        except Exception as e:
            self.get_logger().error(f'Error in object detection: {e}')
            return []

    def draw_bounding_box(self, frame, x1, y1, x2, y2, centerX, centerY):
        """Tespit edilen nesneler için bounding box çiz"""
        rect_color = self.args.detectObjectsRectangleColor
        circle_color = self.args.detectObjectsCircleColor
        
        cv2.rectangle(frame, (x1, y1), (x2, y2), rect_color, self.args.detectObjectRectangleThickness)
        cv2.circle(frame, (centerX, centerY), 5, circle_color, -1)

    def compute_target_position(self, nearest_buoys, mainBoatCenterX, mainBoatCenterY):
        """Hedef pozisyon hesaplama - geliştirilmiş algoritma"""
        if len(nearest_buoys) < 2:
            # Yetersiz duba - durma komutu
            self.appliedForce = 0.0
            self.appliedTorque = 0.0
            return None, None
        
        # Duba çiftlerini bul
        pairs = self.find_buoy_pairs(nearest_buoys)
        
        if not pairs:
            # Uygun çift bulunamadı - durma komutu
            self.appliedForce = 0.0
            self.appliedTorque = 0.0
            return None, None
        
        # En iyi çifti seç
        best_pair = pairs[0]
        first, second = best_pair[0], best_pair[1]
        
        # Çift arasındaki orta nokta
        avgX = (first[0] + second[0]) // 2
        avgY = (first[1] + second[1]) // 2
        
        # Exponential moving average ile yumuşatma
        if self.targetHistory:
            prevX, prevY = self.targetHistory[-1]
            avgX = int(self.alpha * avgX + (1 - self.alpha) * prevX)
            avgY = int(self.alpha * avgY + (1 - self.alpha) * prevY)
        
        # Hedef geçmişini güncelle
        self.targetHistory.append((avgX, avgY))
        self.targetHistory = self.targetHistory[-10:]  # Son 10 hedefi sakla
        
        # Son hedef çiftini kaydet
        self.lastTargetPair = ((first[0], first[1]), (second[0], second[1]))
        
        return avgX, avgY

    def visualize(self, frame, nearest_buoys, avgX, avgY, mainBoatCenterX, mainBoatCenterY):
        """Görselleştirme"""
        try:
            # Hedef varsa çiz
            if avgX is not None and avgY is not None:
                cv2.circle(frame, (int(avgX), int(avgY)), 7, self.args.targetCircleColor, -1)
                cv2.line(frame, (mainBoatCenterX, mainBoatCenterY), (int(avgX), int(avgY)), (0, 255, 255), 2)
                
                # Mesafe bilgisi
                distance = math.sqrt((avgX - mainBoatCenterX)**2 + (avgY - mainBoatCenterY)**2)
                cv2.putText(frame, f"Target: {distance:.0f}px", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Son hedef çifti
            if self.lastTargetPair:
                pt1, pt2 = self.lastTargetPair
                cv2.line(frame, (pt1[0], pt1[1]), (pt2[0], pt2[1]), (0, 0, 255), 2)
            
            # Tekne merkezi
            cv2.circle(frame, (mainBoatCenterX, mainBoatCenterY), 5, self.args.frameCenterCircleColor, -1)
            
            # Durum bilgileri
            cv2.putText(frame, f"Buoys: {len(nearest_buoys)}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Speed: {self.currentVelocity:.2f}", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Angular: {self.currentAngularVelocity:.2f}", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('USV Navigation', frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in visualization: {e}')

    def destroy_node(self):
        """Node'u temizle"""
        super().destroy_node()
        cv2.destroyAllWindows()


def parseOpt():
    """Komut satırı argümanlarını parse et"""
    parser = argparse.ArgumentParser(description='USV Navigation Controller')
    
    # YOLO model parameters
    parser.add_argument('--yoloModelPath', type=str, default='weights/230_epochs/weights/best.pt',
                       help='Path to the YOLO model')
    parser.add_argument('--yoloConf', type=float, default=0.5,
                       help='Confidence threshold for YOLO object detection (range: 0 to 1)')
    parser.add_argument('--yoloIou', type=float, default=0.6,
                       help='Intersection over Union (IoU) threshold for YOLO object detection')
    parser.add_argument('--yoloDevice', type=str, default='cuda',
                       help='Device to run YOLO model on (e.g., "cpu" or "cuda")')
    parser.add_argument('--yoloVerbose', type=bool, default=False,
                       help='Enable verbose mode for YOLO object detection')
    parser.add_argument('--yoloMaxDetect', type=int, default=10,
                       help='Maximum number of objects to detect using YOLO')
    
    # ROS 2 parameters
    parser.add_argument('--publishTopic', type=str, default='/vessel_a/cmd_vel',
                       help='Topic to publish vessel movements')
    parser.add_argument('--subscribeTopic', type=str, default='/vessel_a/camera/image_raw',
                       help='Topic to subscribe to get image from camera')
    
    # Visualization parameters
    parser.add_argument('--frameCenterCircleColor', type=str, default='(255, 0, 0)',
                       help='Color of the circle to show the center of the frame')
    parser.add_argument('--targetCircleColor', type=str, default='(255, 125, 125)',
                       help='Color of the circle to show the target')
    parser.add_argument('--detectObjectsRectangleColor', type=str, default='(0, 255, 0)',
                       help='Color of the rectangle to show the detected objects')
    parser.add_argument('--detectObjectRectangleThickness', type=int, default=2,
                       help='Thickness of the rectangle to show the detected objects')
    parser.add_argument('--detectObjectsCircleColor', type=str, default='(0, 0, 255)',
                       help='Color of the circle to show the detected objects')
    
    # Vehicle parameters
    parser.add_argument('--vehicleMass', type=float, default=1.0,
                       help='Mass of the vehicle')
    parser.add_argument('--vehicleInertia', type=float, default=1.0,
                       help='Inertia of the vehicle')
    parser.add_argument('--timeStep', type=float, default=0.1,
                       help='Time step (frequency)')
    
    # PID parameters
    parser.add_argument('--kpAngular', type=float, default=0.002,
                       help='Proportional gain')
    parser.add_argument('--kiAngular', type=float, default=0.0001,
                       help='Integral gain')
    parser.add_argument('--kdAngular', type=float, default=0.001,
                       help='Derivative gain')
    parser.add_argument('--alpha', type=float, default=0.2,
                       help='Alpha value for exponential moving average')
    parser.add_argument('--targetObjectsMaxDistance', type=int, default=50,
                       help='Maximum distance between the detected objects')
    
    args = parser.parse_args()
    
    # Convert string to tuple for colors
    try:
        args.frameCenterCircleColor = ast.literal_eval(args.frameCenterCircleColor)
        args.targetCircleColor = ast.literal_eval(args.targetCircleColor)
        args.detectObjectsRectangleColor = ast.literal_eval(args.detectObjectsRectangleColor)
        args.detectObjectsCircleColor = ast.literal_eval(args.detectObjectsCircleColor)
    except (ValueError, SyntaxError) as e:
        print(f"Error parsing color values: {e}")
        # Default colors if parsing fails
        args.frameCenterCircleColor = (255, 0, 0)
        args.targetCircleColor = (255, 125, 125)
        args.detectObjectsRectangleColor = (0, 255, 0)
        args.detectObjectsCircleColor = (0, 0, 255)
    
    return args


def main():
    """Ana fonksiyon"""
    try:
        rclpy.init()
        args = parseOpt()
        controller = usvController(args)
        
        print("USV Controller started. Press Ctrl+C to stop.")
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("Shutting down USV Controller...")
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        try:
            controller.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()