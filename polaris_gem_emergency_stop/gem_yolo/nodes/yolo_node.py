import sys
import torch
from torch import hub
import rospy
import numpy as np
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image


class YoloDetector:
    def __init__(self, pub_annotated_img: rospy.Publisher, src_dir):
        self.detection = None
        self.bridge = CvBridge()
        self.pub = pub_annotated_img

        self.model = torch.hub.load(src_dir, model="yolov5x6", source="local")

    def img_callback(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            cv_image = np.asarray(cv_image)
            yolo_detection = self.model(cv_image, size=data.height)
            self.detection = yolo_detection
            bbox_img = yolo_detection.render()

            rospy.loginfo(yolo_detection.print())
            self.pub.publish(self.bridge.cv2_to_imgmsg(*bbox_img, "rgb8"))
        except CvBridgeError as e:
            rospy.logerr(e)


def main(argv):
    rospy.init_node('yolo_node', anonymous=True)
    is_simulator = rospy.get_param("~is_simulator", False)
    src_dir = rospy.get_param("~models_path")

    pub_annotated_image = rospy.Publisher("annotated_image/yolo", Image, queue_size=1)
    net = YoloDetector(pub_annotated_image, src_dir)

    _ = rospy.Subscriber(
        "/mako_1/mako_1/image_raw" if not is_simulator else "/gem/front_single_camera/image_raw",
        Image, net.img_callback, queue_size=1
    )
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Yolo node")


if __name__ == '__main__':
    sys.exit(main(rospy.myargv()) or 0)
