#!/usr/bin/env python
import rospy
import cv2
import actionlib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_processor.srv import point, pointResponse
import image_processor.msg 

class webCamPublisher():
    def __init__(self):
        rospy.init_node("web_cam_publisher", anonymous=True)
        self._feedback = image_processor.msg.cropFeedback()
        self. _result = image_processor.msg.cropResult()

        self.frame_pub = rospy.Publisher("/web_cam/image_raw", Image, queue_size=10)
        self.cropped_frame_pub = rospy.Publisher("/web_cam/service_cropped", Image, queue_size=10)
        self.bridge = CvBridge()
        self.success = False
        self.cam = cv2.VideoCapture(0)
        ret, self.frame = self.cam.read()
        self.curr_height, self.curr_width = self.frame.shape[0:2]
        self.point_height = self.curr_height/2
        self.point_width = self.curr_width/2
        self.crop_decrement = 100
        self._action_name = "cropper"

        self._as = actionlib.SimpleActionServer(self._action_name, image_processor.msg.cropAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Action Server started for cropping image!")

        rospy.Service('point', point, self.service_callback)
        rospy.loginfo("Server started waiting for image points width and height!")
        #while not self.success and not rospy.is_shutdown():
         #   continue

        while not rospy.is_shutdown():
            self.publish_frame()


    def publish_frame(self):
        _, self.frame = self.cam.read()
        self.height = rospy.get_param("~res_height")
        self.width = rospy.get_param("~res_width")

        start_column = self.point_width - self.width/2
        end_column = self.point_width + self.width/2
        start_row = self.point_height - self.height/2
        end_row = self.point_height+ self.height/2

        croppedImage = self.frame[start_row:end_row, start_column:end_column]
        img_msg = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
        cropped_img_msg = self.bridge.cv2_to_imgmsg(croppedImage, "bgr8")
        #cv2.imshow('web_cam_original', self.frame)
        #cv2.waitKey(1)
        self.frame_pub.publish(img_msg)
        self.cropped_frame_pub.publish(cropped_img_msg)
        

    def service_callback(self, req):
        rospy.loginfo("point requested width: %d, height: %d",req.width, req.height)
        if(req.width < self.width/2 or req.width > (self.curr_width - self.width/2)):
            return pointResponse(False)
        elif(req.height < self.height/2 or req.height > (self.curr_height - self.height/2)):
            return pointResponse(False)
        else:
            self.point_height = req.height
            self.point_width = req.width
            self.success = True
            return pointResponse(True)

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, cropping image to %d crop size, with %d decrement' % (self._action_name, goal.crop_size, self.crop_decrement))

        # start executing the action
        for size in range(400, goal.crop_size, -self.crop_decrement):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            start_column = self.curr_width/2 - size/2
            end_column = self.curr_width/2 + size/2
            start_row = self.curr_height/2 - size/2
            end_row = self.curr_height/2 + size/2

            cropped_img = self.frame[start_row:end_row, start_column:end_column]
            cv2.imshow('web_cam_action_cropped', cropped_img)
            cv2.waitKey(1)
            img_msg = self.bridge.cv2_to_imgmsg(cropped_img, "bgr8")

            self._feedback.feedback_image = img_msg
            self._feedback.curr_width = size
            self._feedback.curr_height = size
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        if success:
            cv2.destroyWindow('web_cam_action_cropped')
            self._result.result_image = self._feedback.feedback_image
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__=='__main__':
    webCamPublisher()
