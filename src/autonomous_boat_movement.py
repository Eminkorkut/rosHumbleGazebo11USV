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

        # Initialize CvBridge
        try:
            self.bridge = CvBridge()
        except Exception as e:
            self.get_logger().error(f'Error initializing CvBridge: {e}')
            self.destroy_node()
            return

        # Load YOLO model
        try :
            self.model = YOLO(args.yoloModelPath)
        except Exception as e:
            self.get_logger().error(f'Error loading YOLO model: {e}')
            self.destroy_node()
            return

        # Initialize vehicle parameters
        self.currentVelocity = 0.1
        self.currentAngularVelocity = 0.0
        self.appliedForce = 0.0
        self.appliedTorque = 0.0

        self.mass = args.vehicleMass
        self.inertia = args.vehicleInertia
        self.timeStep = args.timeStep

        # Timer to update the vehicle's movement
        self.timer = self.create_timer(self.timeStep, self.update)

        self.targetHistory = []
        self.lastTargetPair = None

        # PID controller parameters
        self.prevErrorX = 0.0
        self.integralX = 0.0
        self.alpha = args.alpha

        self.kpAngular = args.kpAngular
        self.kiAngular = args.kiAngular
        self.kdAngular = args.kdAngular

    def update(self):
        # Compute new velocities
        acceleration = self.appliedForce / self.mass
        angularAcceleration = self.appliedTorque / self.inertia

        # Update velocities
        self.currentVelocity += acceleration * self.timeStep
        self.currentAngularVelocity += angularAcceleration * self.timeStep

        # Publish new velocities
        msg = Twist()
        msg.linear.x = self.currentVelocity
        msg.angular.z = self.currentAngularVelocity
        self.publisher.publish(msg)

    def adjustMovement(self, targetX: int, referenceX: int, targetY: int, referenceY: int):
        # PID controller
        errorX = targetX - referenceX
        self.integralX += errorX * self.timeStep
        derivativeX = (errorX - self.prevErrorX) / self.timeStep

        # Compute control signal
        pid_output = self.kpAngular * errorX + self.kiAngular * self.integralX + self.kdAngular * derivativeX
        self.appliedTorque = -np.clip(pid_output, -1.0, 1.0)
        self.appliedForce = 0.01 * (targetY - referenceY)

        self.prevErrorX = errorX

    def imageCallBack(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = frame.shape[:2]
            mainBoatCenterX, mainBoatCenterY = width // 2, height // 2

            # Perform object detection using YOLO
            results = self.model.predict(
                frame,
                max_det=self.args.yoloMaxDetect,
                iou=self.args.yoloIou,
                conf=self.args.yoloConf,
                device=self.args.yoloDevice,
                verbose=self.args.yoloVerbose
            )

            # Draw bounding boxes and circles around detected objects
            detectionObjectCenterList = []
            rect_color = self.args.detectObjectsRectangleColor
            circle_color = self.args.detectObjectsCircleColor
            maxYdifference = self.args.targetObjectsMaxDistance

            # Draw bounding boxes and circles around detected objects
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    centerX = (x1 + x2) // 2
                    centerY = (y1 + y2) // 2
                    area = (x2 - x1) * (y2 - y1)
                    detectionObjectCenterList.append((centerX, centerY, area))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), rect_color, self.args.detectObjectRectangleThickness)
                    cv2.circle(frame, (centerX, centerY), 5, circle_color, -1)

            # Sort detected objects by area
            detectionObjectCenterList.sort(key=lambda x: x[2], reverse=True)
            num_detections = len(detectionObjectCenterList)

            # Compute average target position
            if num_detections >= 2:
                first, second = detectionObjectCenterList[0], detectionObjectCenterList[1]
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

            # Adjust movement based on target position
            self.adjustMovement(avgX, mainBoatCenterX, avgY, mainBoatCenterY)

            # Visualize target if conditions are met
            if num_detections >= 2 and abs(detectionObjectCenterList[0][1] - detectionObjectCenterList[1][1]) < maxYdifference:
                cv2.circle(frame, (int(avgX), int(avgY)), 7, self.args.targetCircleColor, -1)
                cv2.line(frame, (mainBoatCenterX, mainBoatCenterY), (int(avgX), int(avgY)), (0, 255, 255), 2)
                if self.lastTargetPair:
                    pt1, pt2 = self.lastTargetPair
                    cv2.line(frame, (pt1[0], pt1[1]), (pt2[0], pt2[1]), (0, 0, 255), 2)

            # Visualize the center of the frame
            cv2.circle(frame, (mainBoatCenterX, mainBoatCenterY), 5, self.args.frameCenterCircleColor, -1)
            cv2.imshow('frame', frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


    # Destroy the node
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
