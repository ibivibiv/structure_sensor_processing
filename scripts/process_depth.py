#!/usr/bin/env python
import rospy
# This is the tool that marshals images into OpenCV
from cv_bridge import CvBridge, CvBridgeError 
# Import some stock ROS message types.
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
# import some utils.
import numpy as np
import cv
import SimpleCV as scv
import copy as copy

class ProcessDepth:
    def __init__(self):
        # Allows conversion between numpy arrays and ROS sensor_msgs/Image
        self.bridge = CvBridge() 

        # Allow our topics to be dynamic.
        self.input_camera_topic = rospy.get_param('~input_camera_topic', '/camera/depth/image_rect')
        self.output_camera_topic = rospy.get_param('~output_camera_topic', '/processed')

        # WE are going to publish a debug image as it comes in.
        self.pub = rospy.Publisher(self.output_camera_topic, Image,queue_size=10)
        rospy.Subscriber(self.input_camera_topic, Image, self._process_depth_img)
        # run the node
        self._run()

    # Keep the node alive
    def _run(self):
        rospy.spin()
            
    def _process_depth_img(self,input):
        # convert our image to CV2 numpy format from ROS format
        latest_image = self.bridge.imgmsg_to_cv2(input)

        if( latest_image is not None ):
            try:
                # convert the image to SimpleCV
                dmin = np.nanmin(latest_image)
                dmax = np.nanmax(latest_image)
                latest_image = latest_image - dmin
                sf = 255.0/(dmax-dmin)
                latest_image = sf*latest_image
                temp = latest_image.astype(np.uint8)
                img = scv.Image(temp, cv2image=True, colorSpace=scv.ColorSpace.RGB)
                img = img.threshold(128)
                # convert SimpleCV to CV2 Numpy Format
                cv2img = img.getNumpyCv2()
                # Convert Cv2 numpy to ROS format
                img_msg = self.bridge.cv2_to_imgmsg(cv2img, "bgr8")
                # publish the topic.
                self.pub.publish(self.bridge.cv2_to_imgmsg(cv2img, "bgr8"))
            except CvBridgeError, e:
                print e

# Boilerplate node spin up. 
if __name__ == '__main__':
    try:
        rospy.init_node('ProcessDepth')
        p = ProcessDepth()
    except rospy.ROSInterruptException:
        pass
