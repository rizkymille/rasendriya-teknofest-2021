import cv2
import numpy as np 
import time

from imutils import resize, grab_contours
from imutils.video import VideoStream

def trackbar_callback(val):
    """
    Trackbar callback do nothing
    """
    pass

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

def color_detection():
    """
    Detect color with given threshold in HSV-space
    then take the largest blob
    """
    vs = VideoStream(usePiCamera=False).start()
    time.sleep(2.)

    # create trackbar
    cv2.namedWindow('Image')
    cv2.createTrackbar('Lower H', 'Image', 0, 179, trackbar_callback)
    cv2.createTrackbar('Upper H', 'Image', 0, 179, trackbar_callback)
    cv2.createTrackbar('Lower S', 'Image', 0, 255, trackbar_callback)
    cv2.createTrackbar('Upper S', 'Image', 0, 255, trackbar_callback)
    cv2.createTrackbar('Lower V', 'Image', 0, 255, trackbar_callback)
    cv2.createTrackbar('Upper V', 'Image', 0, 255, trackbar_callback)

    while True:
        # (get) update threshold values
        lower, upper = get_threshold()

        # preprocess
        original = vs.read()
        original = resize(original, width=400)

        frame = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
        frame = cv2.GaussianBlur(frame, (11, 11), 0)

        # color detection
        frame = cv2.inRange(frame, lower, upper)

        # find largest blob if exist
        # then compute the center
        cnts = cv2.findContours(frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = grab_contours(cnts)

        if len(cnts) != 0:
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)

            try:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

            except ZeroDivisionError:
                continue

            cv2.circle(original, (cx, cy), 7, (255, 255, 255), -1)
            cv2.putText(original, "center", (cx - 20, cy - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # show image
        cv2.imshow("Image", original)
        cv2.imshow("Mask", frame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    color_detection()