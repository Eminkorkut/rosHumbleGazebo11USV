from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.node import Node
import rclpy
import cv2


class usvController(Node):
    def __init__(self):
        super().__init__('usvController')

        # create a publisher to publish vessel movements
        self.publisher = self.create_publisher(Twist, '/vessel_a/cmd_vel', 10)

        # create a subscriber to get image from camera
        self.subscription = self.create_subscription(
            Image,
            '/vessel_a/camera/image_raw',
            self.imageCallBack,
            10
        )

        # create an object to use cv2 library
        self.bridge = CvBridge()
        # load YOLO model
        self.model = YOLO('weights/230_epochs/weights/best.pt')


        # variables for vessel movements
        self.currentVelocity = 0.1
        self.currentAngularVelocity = 0.0
        self.appliedForce = 0.0
        self.appliedTorque = 0.0
        # mass of the vehicle
        self.mass = 1.0
        # inertia of the vehicle
        self.inertia = 1.0
        # time step (frequency)
        self.timeStep = 0.1


        self.timer = self.create_timer(self.timeStep, self.update)


        self.targetHistory = [] 
        self.lastTargetPair = None

        self.prevErrorX = 0.0
        self.integralX = 0.0
        self.alpha = 0.2


        # pid parameters
        self.kpAngular = 0.002 # proportional gain
        self.kiAngular = 0.0001 # integral gain
        self.kdAngular = 0.001 # derivative gain 


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
        results = self.model.predict(frame, max_det=5, iou=0.6)

        # get the center of the main boat
        mainBoatCenterX = frame.shape[1] // 2
        mainBoatCenterY = frame.shape[0] // 2

        # list to store detected objects
        detectionObjectCenterList = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detectedObjectCenterX = (x1 + x2) // 2
                detectedObjectCenterY = (y1 + y2) // 2
                detectionObjectCenterList.append((detectedObjectCenterX, detectedObjectCenterY))

                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (detectedObjectCenterX, detectedObjectCenterY), 5, (0, 0, 255), -1)


        if len(detectionObjectCenterList) >= 2:
            # maximum distance between the detected objects
            maxYdifference = 50

            # sort the detected objects according to their y coordinates
            detectionObjectCenterList.sort(key=lambda x: x[1], reverse=True)

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
            cv2.circle(frame, (int(avgX), int(avgY)), 7, (255, 0, 0), -1)
            cv2.line(frame, (mainBoatCenterX, mainBoatCenterY), (int(avgX), int(avgY)), (0, 255, 255), 2)
            if self.lastTargetPair:
                cv2.line(frame, self.lastTargetPair[0], self.lastTargetPair[1], (0, 0, 255), 2)


        # draw the center of the main boat
        cv2.circle(frame, (mainBoatCenterX, mainBoatCenterY), 5, (255, 0, 0), -1)

        # show the frame
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

    def destroynode(self):
        super().destroy_node()
        cv2.destroyAllWindows()


# main function to run the node
def main(args=None):
    rclpy.init(args=args)
    controller = usvController()
    try:
        rclpy.spin(controller)
    finally:
        controller.destroynode()
        rclpy.shutdown()

# if the script is run directly, execute the main function
if __name__ == '__main__':
    main()

