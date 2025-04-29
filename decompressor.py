#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ImageDecompressor(Node):
    def __init__(self):
        super().__init__('image_decompressor')

        # --- QoS Profile to match the Unity Publisher ---
        # Based on `ros2 topic info -v /camera/color/image/compressed`
        qos_profile_subscriber = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, # KEEP_LAST is common, try changing if needed
            depth=1 # Depth 1 is common for volatile sensor data
        )

        # --- Subscription ---
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image/compressed', # Input topic
            self.listener_callback,
            qos_profile=qos_profile_subscriber) # Use the custom QoS
        self.subscription  # prevent unused variable warning

        # --- Publisher ---
        # Use default reliable QoS for derived image topics unless needed otherwise
        self.publisher = self.create_publisher(
            Image,
            '/camera/color/image_raw', # Output topic
            10) # Standard QoS depth for publishers

        # --- OpenCV Bridge ---
        self.bridge = CvBridge()
        self.get_logger().info('Image Decompressor node started.')

    def listener_callback(self, msg):
        self.get_logger().debug('Received compressed image format: "%s"' % msg.format)
        try:
            # Decompress the image using OpenCV
            # np.frombuffer is faster than np.fromstring or np.array
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # Use IMREAD_COLOR for RGB

            if cv_image is None:
                self.get_logger().error('cv2.imdecode failed! Check image format/data.')
                return

            # Convert OpenCV image back to ROS Image message
            # Common encodings: "bgr8" (OpenCV default), "rgb8"
            # Check what RViz/RTAB-Map expects. If they need rgb8, you might need:
            # cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # raw_image_msg = self.bridge.cv2_to_imgmsg(cv_image_rgb, encoding="rgb8")

            raw_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

            # Copy header timestamp and frame_id
            raw_image_msg.header = msg.header

            # Publish the raw image
            self.publisher.publish(raw_image_msg)

        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: %s' % str(e))
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    image_decompressor = ImageDecompressor()
    rclpy.spin(image_decompressor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_decompressor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
