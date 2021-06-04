#!/usr/bin/env python3
from typing import Tuple, Optional, List

import math
import cv2
import numpy as np
import rospy
import tf2_ros
import tf_conversions
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType, TopicType
from geometry_msgs.msg import TransformStamped
from image_processing.rectification import Rectify
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from my_dnn.msg import Int32MultiArrayStamped
from tf.transformations import *


__TAG_ID_DICT = {32: 32, 33: 31, 65: 61, 31: 33, 57: 57, 61: 65, 10: 11, 11: 10, 9: 9, 24: 26, 25: 25, 26: 24}


def fetch_tag_id(tag):
    return __TAG_ID_DICT[tag.tag_id]


class DNNNode(DTROS):
    """
        Computes an estimate of the Duckiebot pose using the wheel encoders.
        Args:
            node_name (:obj:`str`): a unique, descriptive name for the ROS node
        Configuration:

        Publisher:
            ~/rectified_image (:obj:`Image`): The rectified image
            ~at_localization (:obj:`PoseStamped`): The computed position broadcasted in TFs
        Subscribers:
            ~/camera_node/image/compressed (:obj:`CompressedImage`): Observation from robot
       
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

        # self.log("{}".format((rospy.Time.now()- image_msg.header.stamp).to_sec()))
        

        camera_params = (new_cam[0, 0], new_cam[1, 1], new_cam[0, 2], new_cam[1, 2])
        # t = rospy.Time.now()
        detected_tags = self.at_detector.detect(gray_img, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.065)
        # self.log("{}".format((rospy.Time.now()- t).to_sec()))
        detected_tag_ids = list(map(lambda x: fetch_tag_id(x), detected_tags))
        array_for_pub = Int32MultiArrayStamped()
        array_for_pub.array.data = detected_tag_ids
        array_for_pub.header = image_msg.header

        # self.log("{}".format((rospy.Time.now()- image_msg.header.stamp).to_sec()))
        
        for tag_id, tag in zip(detected_tag_ids, detected_tags):
            # self.log('detected {}: ({})'.format(tag_id, (rospy.Time.now() - image_msg.header.stamp).to_sec()))
            self._broadcast_detected_tag(image_msg, tag_id, tag)

        # I think we need to publish this after broadcasting TFs to make sure all 
        # the TFs are up to date when sensor fusion node starts processing them.
        # Since the sensor fusion node gets a callback on this topic (detected_tags)
        # and when the message arrives at the sensor fusion node, 
        # all the TFs should have been already updated.
        # Otherwise sensor fusion node could process the previous frames.
        self.tag_pub.publish(array_for_pub)


if __name__ == "__main__":
    # Initialize the node
    at_pose_node = DNNNode(node_name='DNNNode')
    # Keep it spinning
    rospy.spin()
