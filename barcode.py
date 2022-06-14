#!/usr/bin/env python
from matplotlib.pyplot import bar
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar


def callback_image(msg):
    global _image
    _image = CvBridge().imgmsg_to_cv2(msg, "bgr8")


if __name__ == "__main__":
    rospy.init_node("barcode")
    rospy.loginfo("barcode demo start!")

    _image = None
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_image)
    rospy.wait_for_message("/camera/rgb/image_raw", Image)

    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()

        gray = cv2.cvtColor(_image, cv2.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(gray)
        for barcode in barcodes:
            x, y, w, h = barcode.rect
            t = barcode.type
            s = barcode.data.decode("UTF-8")
            cv2.rectangle(_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(_image, "%s %s" % (t, s), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            rospy.loginfo("found it")        

        cv2.imshow("frame", _image)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break

    rospy.loginfo("barcode demo end!")
