import rospy
from rasendriya.msg import Dropzone 
import cv2
import numpy as np
import time
import math
import imutils
import argparse
from imutils.video import VideoStream
from std_msgs.msg import Int8

# camera resolution width and height parameters
width = 640
height = 480
vision_flag = -1;

def vision_flag_callback(vis_flag):
    vision_flag = vis_flag.data

def dropzone_detect():
    # parsing arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("integers", metavar='N', type=int, help='Video source address')
    args = parser.parse_args(rospy.myargv()[1:])

    if args.integers >= 0:
        cam = VideoStream(src=args.integers).start()
    elif args.integers == -1:
        cam = VideoStream(usePiCamera=False).start()

    # initialize ros node
    rospy.init_node('vision_dropzone')

    # initialize ros publisher
    pub = rospy.Publisher('dropzone_detector', Dropzone, queue_size=3)
    rate = rospy.Rate(30)
    msg = Dropzone()
    
    # initialize ros subscriber
    rospy.Subscriber("vision_flag", Int8, vision_flag_callback)
    
    # spin
    rospy.spin()

    time.sleep(2.)

    # set lower and upper hsv threshold in red
    lower = np.array([170, 127, 117], dtype='uint8')
    upper = np.array([179, 255, 255],  dtype='uint8')

    while not (rospy.is_shutdown() and vision_flag == -1):
        
        # pre process
        img = cam.read()
        img_disp = img.copy()
        
        #img = imutils.resize(img, width=400)
        blur = cv2.GaussianBlur(img, (7, 7), 0)

        # color filtering
        frame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        frame = cv2.inRange(frame, lower, upper)
        frame = cv2.bitwise_and(blur, blur, mask=frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # edge detection using canny
        canny = cv2.Canny(frame, 50, 240)
        canny = cv2.dilate(canny, np.ones((5,5)), iterations=1)

        # circle detection using hough transform
        circles = cv2.HoughCircles(canny, cv2.HOUGH_GRADIENT, 1, 200,
            param1=100, param2=30,
            minRadius=5, maxRadius=120)

        # display bounding circle
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])

                # circle center
                cv2.circle(img_disp, center, 1, (255,255,255), 3)
                cv2.putText(img_disp, "center", (i[0] - 20, i[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # circle outline
                radius = (i[2])
                cv2.circle(img_disp, center, radius, (0,255,0), 3)

                # transform pixel coordinate system to screen coordinate system
                cX = i[0] - width/2
                cY = height/2 - i[1]
                cAngle = math.degrees(math.atan2(cY, cX))
                msg.x_dropzone = cX
                msg.y_dropzone = cY
                msg.center_angle = cAngle

        cv2.imshow("Camera", img)
        #cv2.imshow("Mask", frame)
        #cv2.imshow("Canny Edge", canny)

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
	try:
		dropzone_detect()
	except rospy.ROSInterruptException:
		pass
