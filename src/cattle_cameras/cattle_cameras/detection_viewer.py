"""Display the YOLO debug image stream in an OpenCV window.

yolo_ros' debug_node already draws boxes/labels/masks onto the frame and
publishes it on `<namespace>/dbg_image`. This node just subscribes to that
topic and shows it, so you can watch detections live without rqt.

Run one per camera, pointing `image_topic` at that camera's debug image:

    ros2 run cattle_cameras detection_viewer \
        --ros-args -p image_topic:=/camera_12345678/yolo/dbg_image
"""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2


class DetectionViewer(Node):
    def __init__(self):
        super().__init__("detection_viewer")

        self.declare_parameter("image_topic", "/yolo/dbg_image")
        self.topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )

        self.bridge = CvBridge()
        self.window = self.topic
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)

        self.sub = self.create_subscription(Image, self.topic, self.on_image, 10)
        self.get_logger().info(f"Viewing detections on {self.topic}")

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - log and skip bad frames
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        cv2.imshow(self.window, frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
