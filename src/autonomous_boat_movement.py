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

    def calculate_buoy_priority_score(self, centerX, centerY, area, distance_mm, frame_width, frame_height):
        """
        Duba öncelik skoru hesaplama - Birinci derece doğrusal denklem
        
        Score = a₁×D + a₂×A + a₃×Cx + a₄×Cy + a₅×W + a₆×H + a₇×R + Constant
        
        Değişkenler:
        - D: Mesafe (normalize edilmiş, 0-1 arası) - Yakın dubalar yüksek skor
        - A: Alan (normalize edilmiş, 0-1 arası) - Büyük dubalar yüksek skor  
        - Cx: X merkez pozisyonu (normalize edilmiş, -0.5 ile +0.5 arası)
        - Cy: Y merkez pozisyonu (normalize edilmiş, 0-1 arası) - Alt kısım yüksek skor
        - W: Genişlik oranı (normalize edilmiş, 0-1 arası)
        - H: Yükseklik oranı (normalize edilmiş, 0-1 arası)
        - R: Merkeze uzaklık (normalize edilmiş, 0-1 arası) - Merkeze yakın yüksek skor
        """
        
        # Katsayılar (manuel olarak optimize edilmiş)
        a1 = -0.4   # Mesafe katsayısı (negatif çünkü yakın dubalar tercih edilir)
        a2 = 0.25   # Alan katsayısı (büyük dubalar daha güvenilir)
        a3 = -0.05  # X pozisyon katsayısı (merkez tercih edilir)
        a4 = 0.15   # Y pozisyon katsayısı (alt kısım tercih edilir)
        a5 = 0.1    # Genişlik katsayısı
        a6 = 0.08   # Yükseklik katsayısı
        a7 = -0.2   # Merkeze uzaklık katsayısı (negatif)
        constant = 0.5  # Sabit terim
        
        # Değişkenleri normalize et
        # D: Mesafe (10m = max pratik mesafe olarak kabul)
        D = min(distance_mm / 10000.0, 1.0)  # 0-1 arası
        
        # A: Alan (frame'in %20'si max alan olarak kabul)
        max_area = frame_width * frame_height * 0.2
        A = min(area / max_area, 1.0)  # 0-1 arası
        
        # Cx: X pozisyonu (-0.5 ile +0.5 arası, merkez = 0)
        Cx = (centerX - frame_width/2) / frame_width
        
        # Cy: Y pozisyonu (0-1 arası, üst = 0, alt = 1)
        Cy = centerY / frame_height
        
        # W: Genişlik oranı (frame genişliğinin %50'si max)
        width = math.sqrt(area * 1.5)  # Yaklaşık genişlik
        W = min(width / (frame_width * 0.5), 1.0)
        
        # H: Yükseklik oranı (frame yüksekliğinin %50'si max)
        height = area / width if width > 0 else 0
        H = min(height / (frame_height * 0.5), 1.0)
        
        # R: Merkeze uzaklık (normalize edilmiş)
        center_distance = math.sqrt((centerX - frame_width/2)**2 + (centerY - frame_height/2)**2)
        max_distance = math.sqrt((frame_width/2)**2 + (frame_height/2)**2)
        R = center_distance / max_distance
        
        # Birinci derece doğrusal denklem
        score = (a1 * D + a2 * A + a3 * abs(Cx) + a4 * Cy + 
                a5 * W + a6 * H + a7 * R + constant)
        
        # Bonus faktörler
        # Çift duba potansiyeli bonusu (Y koordinatı benzer olanlar)
        pair_bonus = 0.0
        
        # Stabilite bonusu (düşük gürültü)
        stability_bonus = min(A * 0.1, 0.05)  # Büyük dubalar daha stabil
        
        # Final skor
        final_score = max(score + pair_bonus + stability_bonus, 0.0)
        
        return final_score, {
            'distance_norm': D,
            'area_norm': A, 
            'center_x_norm': Cx,
            'center_y_norm': Cy,
            'width_norm': W,
            'height_norm': H,
            'center_dist_norm': R,
            'raw_score': score,
            'stability_bonus': stability_bonus
        }

    def select_nearest_buoys(self, detectionObjectCenterList):
        """Gelişmiş duba seçimi - öncelik skoru ile"""
        if not detectionObjectCenterList:
            return []
            
        buoy_scores = []
        frame_height, frame_width = self.current_frame.shape[:2] if self.current_frame is not None else (480, 640)
        
        for frame, centerX, centerY, area, width in detectionObjectCenterList:
            distance_mm = self.calculate_distance_mm(width)
            
            if distance_mm != float('inf'):
                # Öncelik skoru hesapla
                priority_score, score_details = self.calculate_buoy_priority_score(
                    centerX, centerY, area, distance_mm, frame_width, frame_height
                )
                
                # Tuple format: (centerX, centerY, area, distance_mm, priority_score, score_details)
                buoy_scores.append((centerX, centerY, area, distance_mm, priority_score, score_details))
                
                # Görselleştirme için mesafe ve skor yazısı
                distance_text = f"{distance_mm/1000:.1f}m"
                score_text = f"S:{priority_score:.2f}"
                
                cv2.putText(frame, distance_text, (centerX, centerY - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                cv2.putText(frame, score_text, (centerX, centerY - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # Öncelik skoruna göre sırala (yüksekten düşüğe)
        buoy_scores.sort(key=lambda x: x[4], reverse=True)
        
        # En iyi 5 dubayı döndür (eski format için uyumlu)
        selected_buoys = []
        for i, (centerX, centerY, area, distance_mm, priority_score, score_details) in enumerate(buoy_scores[:5]):
            selected_buoys.append((centerX, centerY, area, distance_mm))
            
            # Debug bilgisi (isteğe bağlı)
            if i < 2:  # İlk 2 duba için detay
                self.get_logger().info(
                    f"Buoy {i+1}: Score={priority_score:.3f}, "
                    f"Dist={distance_mm/1000:.1f}m, "
                    f"Area={score_details['area_norm']:.2f}, "
                    f"Center=({score_details['center_x_norm']:.2f},{score_details['center_y_norm']:.2f})"
                )
        
        return selected_buoys

    def find_buoy_pairs(self, buoys):
        """Geliştirilmiş duba çifti bulma - skorlama sistemi ile"""
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
                    # Çift kalite skoru hesapla (birinci derece denklem)
                    pair_score = self.calculate_pair_quality_score(buoy1, buoy2, y_diff)
                    pairs.append((buoy1, buoy2, pair_score))
        
        # Kalite skoruna göre sırala
        pairs.sort(key=lambda x: x[2], reverse=True)
        
        return pairs
    
    def calculate_pair_quality_score(self, buoy1, buoy2, y_diff):
        """
        Duba çifti kalite skoru hesaplama - Birinci derece denklem
        
        Pair_Score = b₁×D_avg + b₂×D_diff + b₃×Y_diff + b₄×X_span + b₅×Size_sim + Constant
        
        Değişkenler:
        - D_avg: Ortalama mesafe (normalize, yakın çiftler tercih)
        - D_diff: Mesafe farkı (normalize, benzer mesafeler tercih) 
        - Y_diff: Y koordinat farkı (normalize, aynı seviyedeki dubalar tercih)
        - X_span: X koordinat açıklığı (normalize, uygun aralık tercih)
        - Size_sim: Boyut benzerliği (normalize, benzer boyutlar tercih)
        """
        
        # Katsayılar
        b1 = -0.3   # Ortalama mesafe (yakın tercih)
        b2 = -0.25  # Mesafe farkı (benzer mesafe tercih)
        b3 = -0.4   # Y farkı (aynı seviye tercih)
        b4 = 0.2    # X açıklığı (uygun aralık tercih)
        b5 = 0.15   # Boyut benzerliği (benzer boyut tercih)
        constant = 1.0
        
        # Değişkenleri hesapla ve normalize et
        
        # D_avg: Ortalama mesafe (10m = 1.0 olarak normalize)
        avg_distance = (buoy1[3] + buoy2[3]) / 2
        D_avg = min(avg_distance / 10000.0, 1.0)
        
        # D_diff: Mesafe farkı (5m max fark olarak normalize)
        distance_diff = abs(buoy1[3] - buoy2[3])
        D_diff = min(distance_diff / 5000.0, 1.0)
        
        # Y_diff: Y koordinat farkı (100px max fark olarak normalize)
        Y_diff_norm = min(y_diff / 100.0, 1.0)
        
        # X_span: X koordinat açıklığı (200px optimal açıklık)
        x_span = abs(buoy1[0] - buoy2[0])
        X_span = min(x_span / 200.0, 1.0)
        
        # Size_sim: Boyut benzerliği (alan farkı)
        area_diff = abs(buoy1[2] - buoy2[2])
        avg_area = (buoy1[2] + buoy2[2]) / 2
        Size_sim = area_diff / avg_area if avg_area > 0 else 1.0
        Size_sim = min(Size_sim, 1.0)
        
        # Birinci derece denklem
        pair_score = (b1 * D_avg + b2 * D_diff + b3 * Y_diff_norm + 
                     b4 * X_span + b5 * Size_sim + constant)
        
        # Bonus faktörler
        
        # Navigation channel bonus (dubalar arası açı uygunluğu)
        navigation_bonus = 0.0
        if 50 < x_span < 300 and y_diff < 30:  # İdeal navigation channel
            navigation_bonus = 0.2
        
        # Symmetry bonus (merkeze göre simetri)
        frame_center = 320  # Varsayılan merkez
        center_line = (buoy1[0] + buoy2[0]) / 2
        symmetry_error = abs(center_line - frame_center) / frame_center
        symmetry_bonus = max(0.1 - symmetry_error, 0)
        
        final_score = max(pair_score + navigation_bonus + symmetry_bonus, 0.0)
        
        return final_score

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