import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np

class Weed3DDetector(Node):
    def __init__(self):
        super().__init__('weed_3d_detector_node')
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.camera_info_received = False

        # Subscribers
        self.create_subscription(Image, '/camera/image', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth_image', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.position_publisher = self.create_publisher(PointStamped, '/weed_position', 10)
        self.marker_publisher = self.create_publisher(Marker, '/weed_marker', 10)

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info('Camera Intrinsics Received')

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_if_ready()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process_if_ready()

    def process_if_ready(self):
        if self.rgb_image is None or self.depth_image is None or not self.camera_info_received:
            return

        # --- Segment Green (Weed) in RGB ---
        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # --- Find Largest Contour ---
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().info('No weed segment found')
            return

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return
        cx_pixel = int(M['m10'] / M['m00'])
        cy_pixel = int(M['m01'] / M['m00'])

        # --- Get Depth at (cx_pixel, cy_pixel) ---
        depth = float(self.depth_image[cy_pixel, cx_pixel])
        if depth == 0.0 or np.isnan(depth):
            self.get_logger().warn('Invalid depth at weed centroid')
            return

        # --- Compute 3D Coordinates ---
        X = float((cx_pixel - self.cx) * depth / self.fx)
        Y = float((cy_pixel - self.cy) * depth / self.fy)
        Z = float(depth)

        self.get_logger().info(f"Weed 3D Position: X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m")

        # --- Publish Position as PointStamped ---
        point_msg = PointStamped()
        point_msg.header.stamp.sec = 0
        point_msg.header.stamp.nanosec = 0
        point_msg.header.frame_id = 'camera_link_optical'
        point_msg.point.x = X
        point_msg.point.y = Y
        point_msg.point.z = Z
        self.position_publisher.publish(point_msg)

        # --- Publish Marker for RViz ---
        marker = Marker()
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.header.frame_id = 'camera_link_optical'
        marker.ns = "weed_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point_msg.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)

        # --- Optional Visualization ---
        vis_image = self.rgb_image.copy()
        cv2.drawContours(vis_image, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(vis_image, (cx_pixel, cy_pixel), 5, (255, 0, 0), -1)
        cv2.imshow('Weed Detection', vis_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Weed3DDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
