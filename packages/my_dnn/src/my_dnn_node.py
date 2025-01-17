#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from image_processing.rectification import Rectify
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


import torch
print("torch is imported successfully")
print("cuda available?", torch.cuda.is_available())


class DNNNode(DTROS):
    """
        A template node to run a pytorch model.
       
    """
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(DNNNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.LOCALIZATION
        )
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")
        self.bridge = CvBridge()
        self.camera_info = None
        self.Rectify = None


        # Camera subscribers:
        camera_topic = f'/{self.veh}/camera_node/image/compressed'
        self.camera_feed_sub = rospy.Subscriber(
            camera_topic,
            CompressedImage,
            self.run_dnn,
            queue_size=1,
            buff_size=2**24
        )

        camera_info_topic = f'/{self.veh}/camera_node/camera_info'
        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic,
            CameraInfo,
            self.getCameraInfo,
            queue_size=1
        )

        self.image_pub = rospy.Publisher(f'/{self.veh}/rectified_image', Image, queue_size=10)
        
        self.log("Initialized!")

    def getCameraInfo(self, cam_msg):
        if (self.camera_info == None):
            self.camera_info = cam_msg
            self.Rectify = Rectify(self.camera_info)
        return

    def run_dnn(self, image_msg):
        """
            Image callback.
            Args:
                msg_encoder (:obj:`Compressed`) encoder ROS message.
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr('Could not decode image: %s' % e)
            return

        new_cam, rectified_img = self.Rectify.rectify_full(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(rectified_img, "bgr8"))
        except ValueError as e:
            self.logerr('Could not decode image: %s' % e)
            return

        gray_img = cv2.cvtColor(rectified_img, cv2.COLOR_RGB2GRAY)        
        camera_params = (new_cam[0, 0], new_cam[1, 1], new_cam[0, 2], new_cam[1, 2])
        


if __name__ == "__main__":
    # Initialize the node
    at_pose_node = DNNNode(node_name='DNNNode')
    # Keep it spinning
    rospy.spin()
