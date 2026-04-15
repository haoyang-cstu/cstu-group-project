import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.declare_parameter('camera_topic', '/camera_sensor/image_raw')
        self.declare_parameter('fallback_camera_topic', '/camera/image_raw')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        fallback_camera_topic = self.get_parameter('fallback_camera_topic').get_parameter_value().string_value
        camera_topics = list(dict.fromkeys([camera_topic, fallback_camera_topic]))
        self.camera_subscriptions = [
            self.create_subscription(Image, topic, self.cb_image, 10)
            for topic in camera_topics
        ]
        self.pub_detected = self.create_publisher(Bool, '/stop_detected', 10)
        self.pub_debug = self.create_publisher(Image, '/stop_detection_debug', 10)
        self.pub_mask = self.create_publisher(Image, '/stop_detection_mask', 10)
        self.declare_parameter('min_area', 60)
        self.min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        self.get_logger().info(
            f'Image processor started on {camera_topics}. Publishing debug to /stop_detection_debug and /stop_detection_mask'
        )

    def cb_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # red color mask (two ranges for hue wrap-around)
        lower1 = np.array([0, 80, 50])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160, 80, 50])
        upper2 = np.array([179, 255, 255])
        m1 = cv2.inRange(hsv, lower1, upper1)
        m2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(m1, m2)

        # morphological ops
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        debug_img = cv_img.copy()
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            # approximate shape
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            
            # draw all contours in green
            cv2.drawContours(debug_img, [cnt], 0, (0, 255, 0), 2)
            
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / float(h) if h else 0.0

            # The simulated sign is a red, near-square marker; accept square-to-octagon contours.
            if 4 <= len(approx) <= 10 and 0.55 <= aspect_ratio <= 1.8:
                detected = True
                # draw detected stop signs in red with thickness
                cv2.drawContours(debug_img, [approx], 0, (0, 0, 255), 3)
                # add text label
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.putText(debug_img, 'STOP', (cx - 20, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        msg_out = Bool()
        msg_out.data = detected
        self.pub_detected.publish(msg_out)
        
        # publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.pub_debug.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing debug image: {e}')
        
        # publish mask
        try:
            mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_msg = self.bridge.cv2_to_imgmsg(mask_3ch, encoding='bgr8')
            self.pub_mask.publish(mask_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing mask: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
