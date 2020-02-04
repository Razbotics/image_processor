#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_processor.srv import point, pointResponse

class webCamSubscriber():
    def __init__(self):
        rospy.init_node("web_cam_subscriber", anonymous=True)

        self.success = False
        self.image_start = False
        self.cropped_image_start = False
        self.point_width = rospy.get_param("~px")
        self.point_height = rospy.get_param("~py")
        rospy.loginfo("Waiting for server to start!")
        rospy.wait_for_service('point')
        self.point_service_setter()
        rospy.Subscriber("/web_cam/image_raw", Image, self.image_callback)
        rospy.Subscriber("/web_cam/service_cropped", Image, self.cropped_image_callback)
        self.bridge = CvBridge()

        while not rospy.is_shutdown() and self.success:
            if self.image_start and self.cropped_image_start:
                self.image_viewer()
            

    def image_callback(self, image_data):
        self.cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        self.image_start = True

    def cropped_image_callback(self, image_data):
        self.cv_image_cropped = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        self.cropped_image_start = True

    def image_viewer(self):
        cv2.imshow('web_cam_original', self.cv_image)
        cv2.imshow('web_cam_service_cropped', self.cv_image_cropped)
        cv2.waitKey(1)

    def point_service_setter(self):

        try:
            point_set_req = rospy.ServiceProxy('point', point)
            resp = point_set_req(self.point_width, self.point_height)
            if resp.result:
                rospy.loginfo("Points were set successfully")
                self.success = True
            else:
                rospy.logerr("Points set unsuccessfull, Out of bounds make sure points are in limits!")

        except rospy.ServiceException, err:
            print "Service call failed: %s"%err


if __name__=="__main__":
    webCamSubscriber()