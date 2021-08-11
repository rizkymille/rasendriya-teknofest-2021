import rospy
from rasendriya.msg import Dropzone 
import cv2
import numpy as np
import time
import math
import imutils
import argparse
from imutils.video import VideoStream
from std_msgs.msg import Bool

vision_flag = False

def vision_flag_callback(vis_flag):
    global vision_flag
    vision_flag = vis_flag.data

def dropzone_detect():
    # camera resolution width and height parameters
    width = 400
    height = 400

    # parsing arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("integers", metavar='N', type=int, help='Video source address')
    args = parser.parse_args(rospy.myargv()[1:])

    if args.integers == -1:
        cam = VideoStream(usePiCamera=False).start()
    else:
        cam = VideoStream(src=args.integers).start()

    # initialize ros node
    rospy.init_node('vision_dropzone')

    # initialize ros publisher
    vision_result_pub = rospy.Publisher('/rasendriya/dropzone', Dropzone, queue_size=5)
    rate = rospy.Rate(15)
    vision_result = Dropzone()
    
    # initialize ros subscriber
    rospy.Subscriber("/rasendriya/vision_flag", Bool, vision_flag_callback)

    # set lower and upper hsv threshold in red
    lower = np.array([170, 127, 117], dtype='uint8')
    upper = np.array([179, 255, 255], dtype='uint8')

    while not rospy.is_shutdown():
        
        rospy.loginfo_once("Vision program ready")

        if (vision_flag):
            rospy.loginfo_once("Vision program launched. Starting target detection")
            # pre process
            img = cam.read()
            img = imutils.resize(img, width=400)
            img_disp = img.copy()
            
            #img = imutils.resize(img, width=400)
            blur = cv2.GaussianBlur(img, (7, 7), 0)

            # color filtering
            frame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            frame = cv2.inRange(frame, lower, upper)
            frame = cv2.bitwise_and(blur, blur, mask=frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # circle detection using hough transform
            circles = cv2.HoughCircles(frame, method=cv2.HOUGH_GRADIENT, dp=1.5, minDist=200,
                param1=100, param2=30,
                minRadius=5, maxRadius=120)

            largest_circle_radius = 0
            largest_circle_center = None

            # display bounding circle
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    center = (i[0], i[1])

                    # circle outline
                    radius = (i[2])

                    if radius > largest_circle_radius:
                        largest_circle_radius = radius
                        largest_circle_center = center

            # transform pixel coordinate system to screen coordinate system
            if largest_circle_center is not None:
                vision_result.x = int(largest_circle_center[0] - width/2)
                vision_result.y = int(height/2 - largest_circle_center[1])
            else:
                vision_result.x = 3000
                vision_result.y = 3000
        
            vision_result_pub.publish(vision_result)

        rate.sleep()

if __name__ == "__main__":
    try:
        dropzone_detect()
    except rospy.ROSInterruptException:
            pass
