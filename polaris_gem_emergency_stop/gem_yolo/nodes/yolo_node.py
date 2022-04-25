#!/usr/bin/env python3
import sys
import torch
from torch import hub
import rospy
import numpy as np
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image


class YoloDetector:
    DEPTH_SCALE = 0.001
    BRAKE_THRESHOLD = 10

    def __init__(self, pub_annotated_img: rospy.Publisher, src_dir: str, is_simulator: bool):
        self.depth = None
        self.detection = None
        self.bridge = CvBridge()
        self.img_pub = pub_annotated_img

        self.is_simulator = is_simulator
        self.model = torch.hub.load(src_dir, model="yolov5x6", source="local")
        self.trigger_class = [0, 9, 12]  # [person, traffic light, stop sign]

        if self.is_simulator:
            from ackermann_msgs.msg import AckermannDrive

            self.brake_cmd = AckermannDrive()
            self.brake_cmd.speed = 0
            self.brake_cmd.acceleration = 0
            self.brake_pub = rospy.Publisher("ackermann_cmd", AckermannDrive, queue_size=1)
        else:
            from pacmod_msgs.msg import PacmodCmd

            self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
            self.brake_cmd = PacmodCmd()
            self.brake_cmd.enable = True
            self.brake_cmd.clear = False
            self.brake_cmd.ignore = False

    def engage_break(self):
        self.brake_cmd.f64_cmd = 0.6
        self.brake_pub.publish(self.brake_cmd)
        rospy.loginfo("Brake Engaged!")

    def release_brake(self):
        self.brake_cmd.f64_cmd = 0
        self.brake_pub.publish(self.brake_cmd)
        rospy.loginfo("Brake Released!")

    def depth_callback(self, data: Image):
        try:
            if not self.detection:
                return

            depth = self.bridge.imgmsg_to_cv2(data, data.encoding) * self.DEPTH_SCALE

            detection_cls = self.detection.xyxy[:, -1]
            detection_mask = np.isin(detection_cls, self.trigger_class)
            detection_bbox = self.detection.xyxy[detection_mask].astype(np.int32)
            detection_depth = np.array([np.median(depth[y1:y2, x1:x2]) for x1, y1, x2, y2 in detection_bbox])

            if np.any(detection_mask) or np.any(detection_depth < self.BRAKE_THRESHOLD):
                rospy.loginfo(f"Detected Dangerous Object at {np.min(detection_depth)} meters!")
                self.engage_break()
            else:
                self.release_brake()

        except CvBridgeError as e:
            rospy.logerr(e)

    def img_callback(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            cv_image = np.asarray(cv_image)
            yolo_detection = self.model(cv_image, size=data.height)
            self.detection = yolo_detection
            self.detection.xyxy, *_ = self.detection.xyxy.cpu().numpy()

            yolo_detection.print()
            bbox_img = yolo_detection.render()
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(*bbox_img, "rgb8"))
        except CvBridgeError as e:
            rospy.logerr(e)


def main(argv):
    rospy.init_node('yolo_node', anonymous=True)
    is_simulator = rospy.get_param("~is_simulator", False)
    src_dir = rospy.get_param("~src_path",
                              "/home/gem/gta-project/src/POLARIS_GEM_e2/polaris_gem_emergency_stop/gem_yolo/src/")

    pub_annotated_image = rospy.Publisher("annotated_image/yolo", Image, queue_size=1)
    net = YoloDetector(pub_annotated_image, src_dir, is_simulator)

    _ = rospy.Subscriber(
        "/camera/color/image_raw",
        Image, net.img_callback, queue_size=1
    )
    _ = rospy.Subscriber(
        "/camera/aligned_depth_to_color/image_raw",
        Image, net.depth_callback, queue_size=1
    )
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Yolo node")


if __name__ == '__main__':
    sys.exit(main(rospy.myargv()) or 0)
