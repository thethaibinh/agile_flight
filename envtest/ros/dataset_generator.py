#!/usr/bin/python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import os
import numpy as np

class DatasetGeneratorNode:
    def __init__(self):
        print("Initializing dataset_generator_node...")
        rospy.init_node('dataset_generator_node', anonymous=False)
        self.cv_bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/kingfisher/dodgeros_pilot/unity/depth", Image, self.img_callback,
                                        queue_size=1, tcp_nodelay=True)
        if not os.path.exists('./dataset/images'):
            os.makedirs('./dataset/images')
        if not os.path.exists('./dataset/labels'):
            os.makedirs('./dataset/labels')
        print("Initialization completed!")

    def img_callback(self, img_data):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.cv_bridge.imgmsg_to_cv2(img_data, desired_encoding='passthrough')
            # cv2_img = cv2_img * 65535               #Scale value to uint8 range 0-255
            # cv2_img = cv2_img.astype(np.uint16)
            # Save your OpenCV2 image as a png
            path = './dataset/images'
            cv2.imwrite(os.path.join(path, 'SFC_' + str(img_data.header.stamp.to_sec()) + '.png'), cv2_img)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    dataset_generator_node = DatasetGeneratorNode()
    rospy.spin()
