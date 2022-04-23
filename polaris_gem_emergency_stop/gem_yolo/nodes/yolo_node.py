import sys
from typing import Optional
import cv2
import torch
import rospy
import numpy as np
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image


class YoloDetector:
    def __init__(self, pub_annotated_img: Optional[rospy.Publisher] = None,):
        self.bridge = CvBridge()
        self.pub = pub_annotated_img

        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

    def img_callback(self, data: Image):
        try:
            rospy.loginfo("Image Callback")
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            cv_image = np.asarray(cv_image)
            yolo_detection = self.model(cv_image, size=data.height)
            bbox_img = yolo_detection.render()
            bbox_img = cv2.resize(*bbox_img, (data.width, data.height))

            self.pub.publish(self.bridge.cv2_to_imgmsg(bbox_img, "rgb8"))
        except CvBridgeError as e:
            rospy.logerr(e)


def main(argv):
    rospy.init_node('yolo_node', anonymous=True)

    pub_annotated_image = rospy.Publisher("annotated_image/yolo", Image, queue_size=1)
    net = YoloDetector(pub_annotated_image)

    _ = rospy.Subscriber(
        "/mako_1/mako_1/image_raw",  # /gem/front_single_camera/image_raw
        Image, net.img_callback, queue_size=1
    )
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down LaneNet node")


if __name__ == '__main__':
    sys.exit(main(rospy.myargv()) or 0)
