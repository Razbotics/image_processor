#!/usr/bin/env python
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_processor.srv import point, pointResponse
import actionlib
import image_processor.msg


def cropActionClient(_crop_size):
    client = actionlib.SimpleActionClient('cropper', image_processor.msg.cropAction)
    rospy.loginfo("Waiting for action server to start!")
    client.wait_for_server()
    goal = image_processor.msg.cropGoal(crop_size = _crop_size)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    if len(sys.argv) == 2:
        crop_size = int(sys.argv[1])
    else:
        rospy.logwarn("Enter crop size as argument!")
        sys.exit(1)
    try:
        rospy.init_node('crop_action_client')
        bridge = CvBridge()
        result = cropActionClient(crop_size)
        cv_image = bridge.imgmsg_to_cv2(result.result_image, "bgr8")
        cv2.imshow('web_cam_action_cropped', cv_image)
        cv2.waitKey(2000)
        
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        rospy.logwarn("program interrupted before completion")
