from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.node import Node
import argparse
import torch
import rclpy
import cv2
import ast

class usvController(Node):
    def __init__(self, args):
        super().__init__('usvController')

        self.args = args

        self.args.yoloDevice = ['cuda' if torch.cuda.is_available() else 'cpu'][0]

        # create a publisher to publish vessel movements
        self.publisher = self.create_publisher(Twist, args.publishTopic, 10)

        # create a subscriber to get image from camera
        self.subscription = self.create_subscription(
            Image,
            args.subscribeTopic,
            self.imageCallBack,
            10
        )

        # create an object to use cv2 library
        self.bridge = CvBridge()
        # load YOLO model
        self.model = YOLO(args.yoloModelPath)

        # variables for vessel movements
        self.currentVelocity = 0.1
        self.currentAngularVelocity = 0.0
        self.appliedForce = 0.0
        self.appliedTorque = 0.0
        # mass of the vehicle
        self.mass = args.vehicleMass
        # inertia of the vehicle
        self.inertia = args.vehicleInertia
        # time step (frequency)
        self.timeStep = args.timeStep

        # create a timer to update the vehicle movements
        self.timer = self.create_timer(self.timeStep, self.update)

        # necessary variables
        self.targetHistory = [] 
        self.lastTargetPair = None

        self.prevErrorX = 0.0
        self.integralX = 0.0
        self.alpha = args.alpha

        # pid parameters
        self.kpAngular = args.kpAngular # proportional gain
        self.kiAngular = args.kiAngular # integral gain
        self.kdAngular = args.kdAngular # derivative gain 

    def update(self):
        acceleration = self.appliedForce / self.mass
        angularAcceleration = self.appliedTorque / self.inertia

        # calculate the velocity
        self.currentVelocity += acceleration * self.timeStep
        self.currentAngularVelocity += angularAcceleration * self.timeStep

        # create a Twist message to publish the vessel movements
        msg = Twist()
        msg.linear.x = self.currentVelocity
        msg.angular.z = self.currentAngularVelocity
        self.publisher.publish(msg)
        
    def adjustMovement(self, targetX, referenceX, targetY, referenceY):
        # calculate the error
        errorX = targetX - referenceX
        self.integralX += errorX * self.timeStep
        derivativeX = (errorX - self.prevErrorX) / self.timeStep

        # calculate the angular velocity with PID controller
        self.appliedTorque = -(self.kpAngular * errorX + self.kiAngular * self.integralX + self.kdAngular * derivativeX)
        self.appliedForce = 0.01 * (targetY - referenceY)

        # apply limits to the angular velocity
        if self.appliedTorque > 1.0:
            self.appliedTorque = 1.0
        elif self.appliedTorque < -1.0:
            self.appliedTorque = -1.0

        self.prevErrorX = errorX

    def imageCallBack(self, msg):
        # convert image message to cv2 image
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # detect objects in the image
        results = self.model.predict(frame, max_det=self.args.yoloMaxDetect, iou=self.args.yoloIou, conf=self.args.yoloConf, device=self.args.yoloDevice, verbose=self.args.yoloVerbose)

        # get the center of the main boat
        mainBoatCenterX = frame.shape[1] // 2
        mainBoatCenterY = frame.shape[0] // 2

        # list to store detected objects
        detectionObjectCenterList = []

        # iterate over the detected objects
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detectedObjectCenterX = (x1 + x2) // 2
                detectedObjectCenterY = (y1 + y2) // 2
                detectedObjectArea = (x2 - x1) * (y2 - y1)
                detectionObjectCenterList.append((detectedObjectCenterX, detectedObjectCenterY, detectedObjectArea))

                # draw the rectangle and circle around the detected objects
                cv2.rectangle(frame, (x1, y1), (x2, y2), self.args.detectObjectsRectangleColor, self.args.detectObjectRectangleThickness)
                cv2.circle(frame, (detectedObjectCenterX, detectedObjectCenterY), 5, self.args.detectObjectsCircleColor, -1)


        if len(detectionObjectCenterList) >= 2:
            # maximum distance between the detected objects
            maxYdifference = self.args.targetObjectsMaxDistance

            # sort the detected objects according to their y coordinates
            detectionObjectCenterList.sort(key=lambda x: x[2], reverse=True)

            if abs(detectionObjectCenterList[0][1] - detectionObjectCenterList[1][1]) < maxYdifference:
                self.lastTargetPair = detectionObjectCenterList[0], detectionObjectCenterList[1]

                if self.targetHistory:
                    prevX, prevY = self.targetHistory[-1]
                    avgX = self.alpha * ((detectionObjectCenterList[0][0] + detectionObjectCenterList[1][0]) // 2) + (1 - self.alpha) * prevX
                    avgY = self.alpha * ((detectionObjectCenterList[0][1] + detectionObjectCenterList[1][1]) // 2) + (1 - self.alpha) * prevY
                else:
                    avgX = (detectionObjectCenterList[0][0] + detectionObjectCenterList[1][0]) // 2
                    avgY = (detectionObjectCenterList[0][1] + detectionObjectCenterList[1][1]) // 2

                self.targetHistory.append((avgX, avgY))

                if len(self.targetHistory) > 10:
                    self.targetHistory.pop(0)
            else:
                avgX, avgY = mainBoatCenterX, mainBoatCenterY
        
        elif len(detectionObjectCenterList) == 1:
            if self.targetHistory:
                avgX, avgY = self.targetHistory[-1]
            else:
                avgX, avgY = mainBoatCenterX, mainBoatCenterY
        
        else:
            if self.targetHistory:
                avgX, avgY = self.targetHistory[-1]
            else:
                avgX, avgY = mainBoatCenterX, mainBoatCenterY


        # adjust the movement of the boat
        self.adjustMovement(avgX, mainBoatCenterX, avgY, mainBoatCenterY)

        # draw the line between the main boat and the target
        if len(detectionObjectCenterList) >= 2 and abs(detectionObjectCenterList[0][1] - detectionObjectCenterList[1][1]) < maxYdifference:
            cv2.circle(frame, (int(avgX), int(avgY)), 7, self.args.targetCircleColor, -1)
            cv2.line(frame, (mainBoatCenterX, mainBoatCenterY), (int(avgX), int(avgY)), (0, 255, 255), 2)
            if self.lastTargetPair:
                cv2.line(frame, 
                        (self.lastTargetPair[0][0], self.lastTargetPair[0][1]), 
                        (self.lastTargetPair[1][0], self.lastTargetPair[1][1]), 
                        (0, 0, 255), 
                        2)


        # draw the center of the main boat
        cv2.circle(frame, (mainBoatCenterX, mainBoatCenterY), 5, self.args.frameCenterCircleColor, -1)

        # show the frame
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

    def destroyNode(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def parseOpt():
    parser = argparse.ArgumentParser()
    # yolo model parameters
    parser.add_argument('--yoloModelPath', type=str, default='weights/230_epochs/weights/best.pt', help='Path to the YOLO model')
    parser.add_argument('--yoloConf', type=float, default=0.5, help='Confidence threshold for YOLO object detection (range: 0 to 1)')
    parser.add_argument('--yoloIou', type=float, default=0.6, help='Intersection over Union (IoU) threshold for YOLO object detection')
    parser.add_argument('--yoloDevice', type=str, default='cuda', help='Device to run YOLO model on (e.g., "cpu" or "cuda")')
    parser.add_argument('--yoloVerbose', type=bool, default=False, help='Enable verbose mode for YOLO object detection')
    parser.add_argument('--yoloMaxDetect', type=int, default=5, help='Maximum number of objects to detect using YOLO')
    # ros2 parameters
    parser.add_argument('--publishTopic', type=str, default='/vessel_a/cmd_vel', help='Topic to publish vessel movements')
    parser.add_argument('--subscribeTopic', type=str, default='/vessel_a/camera/image_raw', help='Topic to subscribe to get image from camera')
    # visualization parameters
    parser.add_argument('--frameCenterCircleColor', type=str, default='(255, 0, 0)', help='Color of the circle to show the center of the frame')
    parser.add_argument('--targetCircleColor', type=str, default='(255, 125, 125)', help='Color of the circle to show the target')
    parser.add_argument('--detectObjectsRectangleColor', type=str, default='(0, 255, 0)', help='Color of the rectangle to show the detected objects')
    parser.add_argument('--detectObjectRectangleThickness', type=int, default=2, help='Thickness of the rectangle to show the detected objects')
    parser.add_argument('--detectObjectsCircleColor', type=str, default='(0, 0, 255)', help='Color of the circle to show the detected objects')
    # usv parameters
    parser.add_argument('--vehicleMass', type=float, default=1.0, help='Mass of the vehicle')
    parser.add_argument('--vehicleInertia', type=float, default=1.0, help='Inertia of the vehicle')
    parser.add_argument('--timeStep', type=float, default=0.1, help='Time step (frequency)')
    parser.add_argument('--kpAngular', type=float, default=0.002, help='Proportional gain')
    parser.add_argument('--kiAngular', type=float, default=0.0001, help='Integral gain')
    parser.add_argument('--kdAngular', type=float, default=0.001, help='Derivative gain')
    parser.add_argument('--alpha', type=float, default=0.2, help='Alpha value for exponential moving average')
    parser.add_argument('--targetObjectsMaxDistance', type=int, default=50, help='Maximum distance between the detected objects')

    args = parser.parse_args()

    # Change the string values to tuple
    args.frameCenterCircleColor = ast.literal_eval(args.frameCenterCircleColor)
    args.targetCircleColor = ast.literal_eval(args.targetCircleColor)
    args.detectObjectsRectangleColor = ast.literal_eval(args.detectObjectsRectangleColor)
    args.detectObjectsCircleColor = ast.literal_eval(args.detectObjectsCircleColor)

    return args

# main function
def main():
    rclpy.init()
    args = parseOpt()
    controller = usvController(args)

    try:
        rclpy.spin(controller)
    finally:
        controller.destroyNode()
        rclpy.shutdown()

# run the main function
if __name__ == '__main__':
    main()
