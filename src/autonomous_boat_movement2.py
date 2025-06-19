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


class usvController(Node):
    def __init__(self, args):
        super().__init__('usvController')
        self.args = args
        self.args.yoloDevice = 'cuda' if torch.cuda.is_available() else 'cpu'

        self.publisher = self.create_publisher(Twist, args.publishTopic, 10)
        self.subscription = self.create_subscription(
            Image,
            args.subscribeTopic,
            self.imageCallBack,
            10
        )

        self.initialize_cv_bridge()
        self.load_yolo_model()
        self.initialize_vehicle_parameters()
        self.initialize_pid_parameters()

        # Timer to update the vehicle's movement
        self.timer = self.create_timer(self.timeStep, self.update)

        self.targetHistory = []
        self.lastTargetPair = None

    def initialize_cv_bridge(self):
        try:
            self.bridge = CvBridge()
        except Exception as e:
            self.get_logger().error(f'Error initializing CvBridge: {e}')
            self.destroy_node()
            return

    def load_yolo_model(self):
        try:
            self.model = YOLO(self.args.yoloModelPath)
        except Exception as e:
            self.get_logger().error(f'Error loading YOLO model: {e}')
            self.destroy_node()
            return

    def initialize_vehicle_parameters(self):
        self.currentVelocity = 0.1
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

    def update(self):
        self.compute_new_velocities()
        self.publish_velocities()

    def compute_new_velocities(self):
        acceleration = self.appliedForce / self.mass
        angularAcceleration = self.appliedTorque / self.inertia

        # Update velocities
        self.currentVelocity += acceleration * self.timeStep
        self.currentAngularVelocity += angularAcceleration * self.timeStep

    def publish_velocities(self):
        msg = Twist()
        msg.linear.x = self.currentVelocity
        msg.angular.z = self.currentAngularVelocity
        self.publisher.publish(msg)

    def adjustMovement(self, targetX: int, referenceX: int, targetY: int, referenceY: int):
        errorX = targetX - referenceX
        self.integralX += errorX * self.timeStep
        derivativeX = (errorX - self.prevErrorX) / self.timeStep

        # Compute control signal
        pid_output = self.kpAngular * errorX + self.kiAngular * self.integralX + self.kdAngular * derivativeX
        self.appliedTorque = -np.clip(pid_output, -1.0, 1.0)
        self.appliedForce = 0.01 * (targetY - referenceY)

        self.prevErrorX = errorX

    def select_nearest_buoys(self, detectionObjectCenterList, focal_length_mm: float, real_width_mm: float, sensor_width_mm: float, image_width_px: float):
        buoy_scores = []

        pixel_size = sensor_width_mm / image_width_px  # mm/piksel
        f_px = focal_length_mm / pixel_size  # Odak uzaklığı (piksel)

        
        for frame, centerX, centerY, area, width in detectionObjectCenterList:
            distance_mm = (real_width_mm * f_px) / width
            buoy_scores.append((centerX, centerY, area, distance_mm))

            cv2.putText(frame, str(distance_mm), (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0))

        buoy_scores.sort(key=lambda x: x[3])
        return buoy_scores[:2]

    def imageCallBack(self, msg: Image):
        try:
            frame = self.convert_image(msg)
            mainBoatCenterX, mainBoatCenterY = self.get_boat_center(frame)

            detectionObjectCenterList = self.detect_objects(frame)
            nearest_buoys = self.select_nearest_buoys(detectionObjectCenterList, focal_length_mm=3.18, real_width_mm=300, sensor_width_mm= 5.6, image_width_px= 1920)
            avgX, avgY = self.compute_target_position(nearest_buoys, mainBoatCenterX, mainBoatCenterY)

            self.adjustMovement(avgX, mainBoatCenterX, avgY, mainBoatCenterY)
            self.visualize(frame, nearest_buoys, avgX, avgY, mainBoatCenterX, mainBoatCenterY)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def convert_image(self, msg: Image):
        return self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def get_boat_center(self, frame):
        height, width = frame.shape[:2]
        return width // 2, height // 2

    def detect_objects(self, frame):
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
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                centerX = (x1 + x2) // 2
                centerY = (y1 + y2) // 2
                area = (x2 - x1) * (y2 - y1)
                width = x2 - x1
                detectionObjectCenterList.append((frame, centerX, centerY, area, width))
                self.draw_bounding_box(frame, x1, y1, x2, y2, centerX, centerY)

        return detectionObjectCenterList

    def draw_bounding_box(self, frame, x1, y1, x2, y2, centerX, centerY):
        rect_color = self.args.detectObjectsRectangleColor
        circle_color = self.args.detectObjectsCircleColor
        cv2.rectangle(frame, (x1, y1), (x2, y2), rect_color, self.args.detectObjectRectangleThickness)
        cv2.circle(frame, (centerX, centerY), 5, circle_color, -1)

    def compute_target_position(self, nearest_buoys, mainBoatCenterX, mainBoatCenterY):
        maxYdifference = self.args.targetObjectsMaxDistance
        num_detections = len(nearest_buoys)

        if num_detections == 2:
            first, second = nearest_buoys
            sameY = abs(first[1] - second[1]) < maxYdifference
            if sameY:
                self.lastTargetPair = (first, second)
                avgX = (first[0] + second[0]) // 2
                avgY = (first[1] + second[1]) // 2

                if self.targetHistory:
                    prevX, prevY = self.targetHistory[-1]
                    avgX = int(self.alpha * avgX + (1 - self.alpha) * prevX)
                    avgY = int(self.alpha * avgY + (1 - self.alpha) * prevY)

                self.targetHistory.append((avgX, avgY))
                self.targetHistory = self.targetHistory[-10:]
            else:
                avgX, avgY = mainBoatCenterX, mainBoatCenterY
        elif num_detections == 1:
            avgX, avgY = self.targetHistory[-1] if self.targetHistory else (mainBoatCenterX, mainBoatCenterY)
        else:
            avgX, avgY = self.targetHistory[-1] if self.targetHistory else (mainBoatCenterX, mainBoatCenterY)

        return avgX, avgY

    def visualize(self, frame, nearest_buoys, avgX, avgY, mainBoatCenterX, mainBoatCenterY):
        num_detections = len(nearest_buoys)
        maxYdifference = self.args.targetObjectsMaxDistance

        if num_detections == 2 and abs(nearest_buoys[0][1] - nearest_buoys[1][1]) < maxYdifference:
            cv2.circle(frame, (int(avgX), int(avgY)), 7, self.args.targetCircleColor, -1)
            cv2.line(frame, (mainBoatCenterX, mainBoatCenterY), (int(avgX), int(avgY)), (0, 255, 255), 2)
            if self.lastTargetPair:
                pt1, pt2 = self.lastTargetPair
                cv2.line(frame, (pt1[0], pt1[1]), (pt2[0], pt2[1]), (0, 0, 255), 2)

        cv2.circle(frame, (mainBoatCenterX, mainBoatCenterY), 5, self.args.frameCenterCircleColor, -1)
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

# Parse command line arguments
def parseOpt():
    parser = argparse.ArgumentParser()
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
    parser.add_argument('--yoloMaxDetect', type=int, default=5,
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

    # Convert string to tuple
    args.frameCenterCircleColor = ast.literal_eval(args.frameCenterCircleColor)
    args.targetCircleColor = ast.literal_eval(args.targetCircleColor)
    args.detectObjectsRectangleColor = ast.literal_eval(args.detectObjectsRectangleColor)
    args.detectObjectsCircleColor = ast.literal_eval(args.detectObjectsCircleColor)

    return args

# Main function
def main():
    rclpy.init()
    args = parseOpt()
    controller = usvController(args)
    try:
        rclpy.spin(controller)
    finally:
        controller.destroy_node()
        rclpy.shutdown()

# Entry point
if __name__ == '__main__':
    main()
