import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.bridge = CvBridge()
        self.topic_windows = [
            ('/camera_sensor/image_raw', 'Camera View'),
            ('/camera/image_raw', 'Camera View'),
            ('/stop_detection_debug', 'CV Processed View'),
            ('/stop_detection_mask', 'Stop Detection Mask'),
        ]
        self.image_subscriptions = [
            self.create_subscription(Image, topic, self.make_callback(window_name), 10)
            for topic, window_name in self.topic_windows
        ]
        self.timer = self.create_timer(0.03, self.spin_windows)
        for window_name in sorted({window_name for _, window_name in self.topic_windows}):
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 480, 360)
        self.get_logger().info('Image viewer started for camera, processed, and mask topics')

    def make_callback(self, window_name):
        def cb_image(msg):
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv2.imshow(window_name, image)
            except Exception as exc:
                self.get_logger().error(f'Image viewer error for {window_name}: {exc}')

        return cb_image

    def spin_windows(self):
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
