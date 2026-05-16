#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class RgbToMonoBridge:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.input_topic = rospy.get_param("~input_topic", "/camera/color/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/rvio2_bridge/cam0/image_raw")
        queue_size = int(rospy.get_param("~queue_size", 1))

        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=queue_size)
        self.subscriber = rospy.Subscriber(self.input_topic, Image, self.callback, queue_size=queue_size)

        rospy.loginfo("rgb_to_mono_bridge forwarding %s -> %s", self.input_topic, self.output_topic)

    def callback(self, msg: Image) -> None:
        try:
            mono = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(2.0, "rgb_to_mono_bridge conversion failed: %s", exc)
            return

        mono_msg = self.bridge.cv2_to_imgmsg(mono, encoding="mono8")
        mono_msg.header = msg.header
        self.publisher.publish(mono_msg)


def main() -> None:
    rospy.init_node("rgb_to_mono_bridge")
    RgbToMonoBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
