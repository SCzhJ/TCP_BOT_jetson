#!/home/nvidia/tcpb_ws/src/nav/scripts/ultralytics/venv/bin/python
import rospy
from std_msgs.msg import Float32, Int32, Bool
print(rospy.__file__)
from geometry_msgs.msg import Twist
from ultralytics import YOLO
import time
#import cv2

def move_robot(linear_x: float, linear_y: float, angular_z: float):
    # Create a Twist message and set x, y, and z linear velocities
    move_cmd = Twist()
    move_cmd.linear.x = linear_x  # Move forward at 0.5 m/s
    move_cmd.linear.y = linear_y
    move_cmd.linear.z = 0.0
    
    # Set angular velocities to zero (no rotation)
    move_cmd.angular.x = 1.0
    move_cmd.angular.y = 0.0
    move_cmd.angular.z = angular_z
    return move_cmd

pick = False
def pick_cb(msg):
    global pick 
    pick = msg.data

if __name__ == '__main__':
    rospy.init_node("obj_rec")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubDet = rospy.Publisher("/obj_det", Int32, queue_size=10)
    # pubDet.publish(Int32(100))
    subPick = rospy.Subscriber("pick", Bool, pick_cb, queue_size=10)
    pubPickDone = rospy.Publisher("pickDone", Bool, queue_size=10)
    pubSwitch = rospy.Publisher("Switch", Bool, queue_size=10)
    # pubPickDone.publish(Bool(True))
    #sub = rospy.Subscriber('/BeginPick', Bool, BeginPick_cb, queue_size=10)
    webCam = 0
    model = YOLO('/home/nvidia/tcpb_ws/src/nav/scripts/ultralytics/1_100.pt')
    
    results = model(webCam, show=False, conf=0.4, device='cuda', stream=True)

    xLine = 335
    yLine = 160
    XYFlag = 0
    timeInSeconds = rospy.Time.now().to_sec()
    noDetectionTime = rospy.Time.now().to_sec()
    switchSignal = False

    for result in results:
        if rospy.is_shutdown():
            break
        
        xCenter = -1
        yCenter = -1

        boxes = result.boxes
        centerPoints = []
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            xCenter = (x1 + x2) / 2
            yCenter = (y1 + y2) / 2
            centerPoints.append((xCenter, yCenter))
        
        if xCenter != -1 or yCenter != -1:
            pubDet.publish(Int32(100))
        else:
            pubDet.publish(Int32(0))

        if len(centerPoints) != 0:
            xDistance = abs(xLine - centerPoints[0][0])
            yDistance = abs(yLine - centerPoints[0][1])
            for centerPoint in centerPoints:
                if abs(xLine - centerPoint[0]) + abs(yLine - centerPoint[1]) < xDistance + yDistance:
                    xCenter = centerPoint[0]
                    yCenter = centerPoint[1]
                    xDistance = abs(xLine - centerPoint[0])
                    yDistance = abs(yLine - centerPoint[1])
        
        if rospy.Time.now().to_sec() - timeInSeconds >= 0.3:
            timeInSeconds = rospy.Time.now().to_sec()

            if xCenter != -1 and yCenter != -1 and pick == True:
                noDetectionTime = rospy.Time.now().to_sec()
                # if yCenter < yLine+20 and yCenter > yLine-20 and xCenter < xLine+20 and xCenter > xLine-20:
                #     print("     STOP")
                #     vel = move_robot(0, 0, 0)
                #     pub.publish(vel)
                #     pubPickDone.publish(Bool(True))
                #     pubPickDone.publish(Bool(True))
                #     pubPickDone.publish(Bool(True))
                #     pick = False
                # elif yCenter > yLine:
                #     print("     GO BACK, y position:", yCenter)
                #     vel = move_robot(-0.1, 0, 0)
                #     pub.publish(vel)
                # elif xCenter < xLine-5 and XYFlag == 0:
                #     print("     TURN LEFT, x position:", xCenter)
                #     vel = move_robot(0, 0, 0.2)
                #     pub.publish(vel)
                #     XYFlag = 1
                # elif xCenter > xLine+5 and XYFlag == 0:
                #     print("     TURN RIGHT, x position:", xCenter)
                #     vel = move_robot(0, 0, -0.2)
                #     pub.publish(vel)
                #     XYFlag = 1
                # elif XYFlag == 1:
                #     print("     GO FORWARD, y position:", yCenter)
                #     vel = move_robot(0.1, 0, 0)
                #     pub.publish(vel)
                #     XYFlag = 0
                if yCenter < yLine+20 and yCenter > yLine-20 and xCenter < xLine+20 and xCenter > xLine-20:
                    print("     STOP")
                    vel = move_robot(0, 0, 0)
                    vel.angular.x = 0
                    pub.publish(vel)
                    switchSignal = not switchSignal
                    pubSwitch.publish(Bool(switchSignal))
                    time.sleep(20)
                    pubPickDone.publish(Bool(True))
                    pubPickDone.publish(Bool(True))
                    pubPickDone.publish(Bool(True))
                    pick = False
                elif XYFlag == 1:
                    XYFlag = 0
                    if yCenter > yLine:
                        print("     GO BACK, y position:", yCenter)
                        vel = move_robot(-0.1, 0, 0)
                        pub.publish(vel)
                    else:
                        print("     GO FORWARD, y position:", yCenter)
                        vel = move_robot(0.1, 0, 0)
                        pub.publish(vel)
                elif XYFlag == 0:
                    XYFlag = 1
                    if xCenter < xLine-5:
                        print("     TURN LEFT, x position:", xCenter)
                        vel = move_robot(0, 0, 0.2)
                        pub.publish(vel)
                    elif xCenter > xLine+5:
                        print("     TURN RIGHT, x position:", xCenter)
                        vel = move_robot(0, 0, -0.2)
                        pub.publish(vel)
            else:
                print("     NO DETECTION, y position:", yCenter)
                if rospy.Time.now().to_sec() - noDetectionTime >= 0.5:
                    pubPickDone.publish(Bool(True))
                    pubPickDone.publish(Bool(True))
                    pubPickDone.publish(Bool(True))
                    pick = False

    vel = move_robot(0, 0, 0)
    pub.publish(vel)
