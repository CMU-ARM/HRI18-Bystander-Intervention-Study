import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import (
    Image
)

class TableSetup(object):

    def _image_callback(self, img):
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        #add tablet image
        cv2.rectangle(cv_image,(300,150),(700,400),(0,0,255))
        #rebroadcast
        self.reimage_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def __init__(self):
        self.reimage_publisher = rospy.Publisher('/table_setup_img',Image, queue_size=1)
        self.bridge = CvBridge()
        rospy.Subscriber('kinect2/qhd/image_color_rect',Image, self._image_callback)

def main():
    rospy.init_node("table_setup_node")
    ts = TableSetup()
    rospy.spin()
    

if __name__ == '__main__':
    main()