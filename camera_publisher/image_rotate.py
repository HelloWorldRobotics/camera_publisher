#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.parameter import Parameter


class ImageRotateNode(Node):
    def __init__(self):
        super().__init__('image_rotate_node')
        
        # Declare parameters with default values
        self.declare_parameter('input_image_topic', '')
        self.declare_parameter('output_image_topic', '')
        self.declare_parameter('rotation_angle', 0)
        self.declare_parameter('use_compressed', False)
        
        # Get parameters
        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.rotation_angle = self.get_parameter('rotation_angle').value
        self.use_compressed = self.get_parameter('use_compressed').value
        
        self.get_logger().info("Initializing with parameters:")
        self.get_logger().info(f"Input image topic: {self.input_image_topic}")
        self.get_logger().info(f"Output image topic: {self.output_image_topic}")
        self.get_logger().info(f"Rotation angle: {self.rotation_angle}")
        self.get_logger().info(f"Using compressed: {self.use_compressed}")
        
        # Create CV bridge
        self.bridge = CvBridge()

        self.image_subscription = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            10)
        
        # Create subscribers based on compression type
        if self.use_compressed:
            self.image_subscription = self.create_subscription(
                CompressedImage,
                self.input_image_topic + '/compressed',
                self.compressed_image_callback,
                10)
        
        # Always create raw image publisher
        self.image_publisher = self.create_publisher(
            Image,
            self.output_image_topic,
            10)
            
        # Optionally create additional compressed publisher
        if self.use_compressed:
            self.compressed_publisher = self.create_publisher(
                CompressedImage,
                self.output_image_topic + '/compressed',
                10)
            
    def get_rotation_mode(self):
        rotation_modes = {
            0: None,
            90: cv2.ROTATE_90_CLOCKWISE,
            180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_COUNTERCLOCKWISE
        }
        return rotation_modes.get(self.rotation_angle)

    def compressed_image_callback(self, msg):
        try:
            self.get_logger().debug(f"Received compressed image with timestamp: {msg.header.stamp}")
            
            # Convert compressed image to CV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.get_logger().debug(f"Decoded image shape: {cv_image.shape}")
            
            # Rotate image
            rotation_mode = self.get_rotation_mode()
            if rotation_mode is not None:
                rotated_image = cv2.rotate(cv_image, rotation_mode)
                self.get_logger().debug(f"Rotated image by {self.rotation_angle} degrees")
            else:
                rotated_image = cv_image
                self.get_logger().debug("No rotation applied (angle = 0)")
            
            self.get_logger().debug(f"Rotated image shape: {rotated_image.shape}")
            
            # Publish raw image
            raw_msg = self.bridge.cv2_to_imgmsg(rotated_image, encoding='bgr8')
            raw_msg.header = msg.header
            self.image_publisher.publish(raw_msg)
            self.get_logger().debug("Published rotated raw image")
            
            # Also publish compressed if enabled
            if self.use_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = msg.format
                compressed_msg.data = np.array(cv2.imencode('.jpg', rotated_image)[1]).tobytes()
                self.compressed_publisher.publish(compressed_msg)
                self.get_logger().debug("Published rotated compressed image")
            
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {str(e)}')

    def image_callback(self, msg):
        try:
            self.get_logger().debug(f"Received raw image with timestamp: {msg.header.stamp}")
            
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            self.get_logger().debug(f"Converted image shape: {cv_image.shape}")
            
            # Rotate image
            rotation_mode = self.get_rotation_mode()
            if rotation_mode is not None:
                rotated_image = cv2.rotate(cv_image, rotation_mode)
                self.get_logger().debug(f"Rotated image by {self.rotation_angle} degrees")
            else:
                rotated_image = cv_image
                self.get_logger().debug("No rotation applied (angle = 0)")
            
            self.get_logger().debug(f"Rotated image shape: {rotated_image.shape}")
            
            # Publish raw image
            rotated_msg = self.bridge.cv2_to_imgmsg(rotated_image, encoding=msg.encoding)
            rotated_msg.header = msg.header
            self.image_publisher.publish(rotated_msg)
            self.get_logger().debug("Published rotated raw image")
            
            # Also publish compressed if enabled
            if self.use_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = np.array(cv2.imencode('.jpg', rotated_image)[1]).tobytes()
                self.compressed_publisher.publish(compressed_msg)
                self.get_logger().debug("Published rotated compressed image")
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageRotateNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
