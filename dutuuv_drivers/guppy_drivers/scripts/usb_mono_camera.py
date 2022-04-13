#!/usr/bin/env python3
import cv2 as cv
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

if __name__ == "__main__":
    rospy.init_node('usb_mono_camera')
    img_pub = rospy.Publisher('img_raw', Image, queue_size=1)
    bridge = CvBridge()
    cap = cv.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if frame is None:
            print('None')
            break
        cv.imshow('video', frame)
        if cv.waitKey(10) == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()


