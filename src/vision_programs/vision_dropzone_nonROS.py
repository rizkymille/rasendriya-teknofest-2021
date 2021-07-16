import cv2
import numpy as np
import time
import imutils
import argparse
import math
from imutils.video import VideoStream

# camera resolution width and height parameters
width = 640
height = 480

''' uncomment to tune color
def trackbar_callback(val):
    """
    Trackbar callback do nothing
    """
    pass

uncomment to tune color
def get_threshold():
    """
    Generate threshold values
    """
    lh = cv2.getTrackbarPos('Lower H', 'Image')
    uh = cv2.getTrackbarPos('Upper H', 'Image')

    ls = cv2.getTrackbarPos('Lower S', 'Image')
    us = cv2.getTrackbarPos('Upper S', 'Image')
    
    lv = cv2.getTrackbarPos('Lower V', 'Image')
    uv = cv2.getTrackbarPos('Upper V', 'Image')
    
    return np.array([lh, ls, lv], dtype='uint8'),\
        np.array([uh, us, uv], dtype='uint8')

def trackbar():
    cv2.namedWindow('Image')
    cv2.createTrackbar('Lower H', 'Image', 0, 179, trackbar_callback)
    cv2.createTrackbar('Upper H', 'Image', 0, 179, trackbar_callback)
    cv2.createTrackbar('Lower S', 'Image', 0, 255, trackbar_callback)
    cv2.createTrackbar('Upper S', 'Image', 0, 255, trackbar_callback)
    cv2.createTrackbar('Lower V', 'Image', 0, 255, trackbar_callback)
    cv2.createTrackbar('Upper V', 'Image', 0, 255, trackbar_callback) '''

def dropzone_detect():
    # parsing arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("integers", metavar='N', type=int, help='Video source address')
    args = parser.parse_args()

    if args.integers >= 0:
        cam = VideoStream(src=args.integers).start()
    elif args.integers == -1:
        cam = VideoStream(usePiCamera=False).start()
    
    time.sleep(2.)

    # trackbar() # uncomment to tune HSV range

    # set lower and upper hsv threshold in red
    lower = np.array([170, 127, 117], dtype='uint8')
    upper = np.array([179, 255, 255],  dtype='uint8')

    while True:
        # pre process
        img = cam.read()
        img_disp = img.copy()
        #img = imutils.resize(img, width=400)
        blur = cv2.GaussianBlur(img, (7, 7), 0)

        # color filtering
        # lower, upper = get_threshold() # uncomment to tune HSV range
        frame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        frame = cv2.inRange(frame, lower, upper)
        frame = cv2.bitwise_and(blur, blur, mask=frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # edge detection using canny
        canny = cv2.Canny(frame, 50, 240)
        canny = cv2.dilate(canny, np.ones((5,5)), iterations=1)

        # circle detection using hough transform
        circles = cv2.HoughCircles(canny, method=cv2.HOUGH_GRADIENT, dp=1.5, minDist=200,
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
                print("x coordinate:", cX)
                print("y coordinate", cY)
                print("angle", cAngle)

        cv2.imshow("Camera", img_disp)
        cv2.imshow("Mask", frame)
        cv2.imshow("Canny Edge", canny)
    
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    dropzone_detect()