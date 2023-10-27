#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import data_processing.image as im
import data_processing.capture as capture

class ImageProcessorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_subscribers = []
        self.image_topics = ["/micasense/image_red", "/micasense/image_red_edge", "/micasense/image_near_infrared"]
        self.aligned_publisher = []
        self.images = [None] * len(self.image_topics)
        self.images_set = [False] * len(self.image_topics)

        for idx, topic in enumerate(self.image_topics):
            self.image_subscribers.append(rospy.Subscriber(topic, Image, callback=self.image_callback, callback_args=(idx, topic)))
        
        for topic in self.image_topics:
            self.aligned_publisher.append(rospy.Publisher(topic + "_aligned", Image, queue_size=10))
        self.aligning = False

    def image_callback(self, data, args):
        idx, topic = args
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            self.images[idx] = im.Image(data=cv_image, index=idx, band_name=topic)
            self.images_set[idx] = True

            self.process_images()
        except Exception as e:
            rospy.logerr("Error converting image: %s", str(e))

    def reset_images(self):
        self.images = [None] * len(self.image_topics)
        self.images_set = [False] * len(self.image_topics)

    def process_images(self):
        if all(self.images_set) and not self.aligning:
            self.aligning = True
            rospy.loginfo("All images received")
            cap = capture.Capture(self.images)
            rospy.loginfo("capture aligned")
            self.reset_images()
            self.aligning = False

            for i in range(len(self.images_set)):
                self.aligned_publisher[i].publish(self.bridge.cv2_to_imgmsg(cap.aligned_images[i], "mono8"))
            rospy.loginfo("capture published")


if __name__ == "__main__":
    rospy.init_node("image_processor_node")
    node = ImageProcessorNode()
    rospy.loginfo("Image processor node running")

    rate = rospy.Rate(10)  # 10 Hz (adjust as needed)

    while not rospy.is_shutdown():
        rate.sleep()