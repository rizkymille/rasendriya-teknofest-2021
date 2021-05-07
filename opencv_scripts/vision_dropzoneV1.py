import cv2
import numpy as np
import time
import imutils
from imutils.video import VideoStream

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
    
    cam = VideoStream(usePiCamera=False).start()
    time.sleep(2.)

    # trackbar() # uncomment to tune HSV range

    # set lower and upper hsv threshold
    # in red
    lower = np.array([140, 127, 117], dtype='uint8')
    upper = np.array([179, 255, 255],  dtype='uint8')

    while True:
    	# pre process
        img = cam.read()
        img = imutils.resize(img, width=400)

        # color filtering
        # lower, upper = get_threshold() # uncomment to tune HSV range
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        frame = cv2.GaussianBlur(frame, (11, 11), 0)
        frame = cv2.inRange(frame, lower, upper)
        frame = cv2.bitwise_and(img, img, mask=frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # edge detection using canny
        canny = cv2.Canny(frame, 50, 240)

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
        	    cv2.circle(img, center, 1, (255,255,255), 3)
        	    cv2.putText(img, "center", (i[0] - 20, i[1] - 20),
        	    	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        	    # circle outline
        	    radius = (i[2])
        	    cv2.circle(img, center, radius, (0,255,0), 3)

        cv2.imshow("Camera", img)
        cv2.imshow("Mask", frame)
        cv2.imshow("Canny Edge", canny)
    
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    dropzone_detect()
