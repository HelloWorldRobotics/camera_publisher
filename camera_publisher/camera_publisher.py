import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import cv2
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.declare_parameter('flip_image', True)
        self.declare_parameter('publish_compressed', False)
        self.declare_parameter('camera_device', '/dev/buggy_cam_back')
        self.declare_parameter('calibration_file', 'config/back.yaml')
        self.declare_parameter('frame_id', 'camera_back_link')
        self.declare_parameter('uitm_topics', False)
        self.declare_parameter('uitm_calibration_file', 'config/uitm/back.yaml')
        
        self.flip_image = self.get_parameter('flip_image').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.camera_device = self.get_parameter('camera_device').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.uitm_topics = self.get_parameter('uitm_topics').value
        self.uitm_calibration_file = self.get_parameter('uitm_calibration_file').value
        
        try:
            package_share_dir = get_package_share_directory('camera_publisher')
            calibration_file_path = os.path.join(package_share_dir, self.calibration_file)
            
            with open(calibration_file_path, 'r') as file:
                calibration_data = yaml.safe_load(file)
                
            camera_info = calibration_data['camera_info']
            self.camera_matrix = camera_info['camera_matrix']['data']
            self.distortion_coeffs = camera_info['distortion_coefficients']['data']
            self.rectification_matrix = camera_info['rectification_matrix']['data']
            self.projection_matrix = camera_info['projection_matrix']['data']
            
            self.get_logger().info('Successfully loaded camera calibration')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load camera calibration: {str(e)}')
            self.camera_matrix = [
                751.748554, 0.000000, 656.307374,
                0.000000, 754.534212, 368.645691,
                0.000000, 0.000000, 1.000000
            ]
            self.distortion_coeffs = [-0.351268, 0.087101, 0.000186, -0.003012, 0.000000]
            self.rectification_matrix = [
                1.000000, 0.000000, 0.000000,
                0.000000, 1.000000, 0.000000,
                0.000000, 0.000000, 1.000000
            ]
            self.projection_matrix = [
                494.303589, 0.000000, 647.948443, 0.000000,
                0.000000, 685.485107, 370.209981, 0.000000,
                0.000000, 0.000000, 1.000000, 0.000000
            ]
        
        self.publisher_compressed = self.create_publisher(CompressedImage, 'image/compressed', 10)
        self.publisher_raw = self.create_publisher(Image, 'image/raw', 10)
        self.publisher_camera_info = self.create_publisher(CameraInfo, 'camera_info', 10)
        
        try:
            self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)  # Use the parameter here
            if not self.cap.isOpened():
                raise ValueError(f"Failed to open camera at {self.camera_device}")
            
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            
            self.cap.set(cv2.CAP_PROP_FPS, 60)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)

        except Exception as e:
            self.get_logger().warn(f"Exception: {str(e)}")
            raise

        # Verify settings
        actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
        self.get_logger().info(f"Camera format: {fourcc_str}")
        self.get_logger().info(f"Camera resolution: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        self.get_logger().info(f"Camera FPS: {self.cap.get(cv2.CAP_PROP_FPS)}")

        self.fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        
        # Fallback calibration parameters
        self.camera_matrix = [
            751.748554, 0.000000, 656.307374,
            0.000000, 754.534212, 368.645691,
            0.000000, 0.000000, 1.000000
        ]
        
        self.distortion_coeffs = [-0.351268, 0.087101, 0.000186, -0.003012, 0.000000]
        
        self.rectification_matrix = [
            1.000000, 0.000000, 0.000000,
            0.000000, 1.000000, 0.000000,
            0.000000, 0.000000, 1.000000
        ]
        
        self.projection_matrix = [
            494.303589, 0.000000, 647.948443, 0.000000,
            0.000000, 685.485107, 370.209981, 0.000000,
            0.000000, 0.000000, 1.000000, 0.000000
        ]

        if self.uitm_topics:
            self.uitm_publisher_raw = self.create_publisher(Image, 'uitm/image/raw', 10)
            self.uitm_publisher_compressed = self.create_publisher(CompressedImage, 'uitm/image/compressed', 10)
            self.uitm_publisher_camera_info = self.create_publisher(CameraInfo, 'uitm/camera_info', 10)
            
            try:
                uitm_calibration_file_path = os.path.join(package_share_dir, self.uitm_calibration_file)
                with open(uitm_calibration_file_path, 'r') as file:
                    uitm_calibration_data = yaml.safe_load(file)
                    
                uitm_camera_info = uitm_calibration_data['camera_info']
                self.uitm_camera_matrix = uitm_camera_info['camera_matrix']['data']
                self.uitm_distortion_coeffs = uitm_camera_info['distortion_coefficients']['data']
                self.uitm_rectification_matrix = uitm_camera_info['rectification_matrix']['data']
                self.uitm_projection_matrix = uitm_camera_info['projection_matrix']['data']
                
                self.get_logger().info('Successfully loaded UITM camera calibration')
                
            except Exception as e:
                self.get_logger().error(f'Failed to load UITM camera calibration: {str(e)}')
                self.uitm_camera_matrix = self.camera_matrix
                self.uitm_distortion_coeffs = self.distortion_coeffs
                self.uitm_rectification_matrix = self.rectification_matrix
                self.uitm_projection_matrix = self.projection_matrix

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            if self.flip_image:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            
            current_time = self.get_clock().now().to_msg()
            
            msg_raw = Image()
            msg_raw.header.stamp = current_time
            msg_raw.header.frame_id = self.frame_id
            msg_raw.height, msg_raw.width = frame.shape[:2]
            msg_raw.encoding = 'bgr8'
            msg_raw.data = frame.tobytes()
            msg_raw.step = frame.shape[1] * 3
            self.publisher_raw.publish(msg_raw)
            self.get_logger().debug('Publishing raw image')
            
            if self.publish_compressed:
                msg_compressed = CompressedImage()
                msg_compressed.header.stamp = current_time
                msg_compressed.header.frame_id = self.frame_id
                msg_compressed.format = "jpeg"
                msg_compressed.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 40])[1].tobytes()
                self.publisher_compressed.publish(msg_compressed)
                self.get_logger().debug('Publishing compressed image')
            
            camera_info_msg = CameraInfo()
            camera_info_msg.header.stamp = current_time
            camera_info_msg.header.frame_id = self.frame_id
            camera_info_msg.height = 720
            camera_info_msg.width = 1280
            
            camera_info_msg.k = self.camera_matrix
            camera_info_msg.d = self.distortion_coeffs
            camera_info_msg.r = self.rectification_matrix
            camera_info_msg.p = self.projection_matrix
            
            self.publisher_camera_info.publish(camera_info_msg)
            
            if self.uitm_topics:
                # Resize frame to 640x480 for UITM
                uitm_frame = cv2.resize(frame, (640, 480))
                
                # Publish raw UITM image
                uitm_msg_raw = Image()
                uitm_msg_raw.header.stamp = current_time
                uitm_msg_raw.header.frame_id = self.frame_id
                uitm_msg_raw.height, uitm_msg_raw.width = uitm_frame.shape[:2]
                uitm_msg_raw.encoding = 'bgr8'
                uitm_msg_raw.data = uitm_frame.tobytes()
                uitm_msg_raw.step = uitm_frame.shape[1] * 3
                self.uitm_publisher_raw.publish(uitm_msg_raw)
                
                # Publish compressed UITM image
                uitm_msg_compressed = CompressedImage()
                uitm_msg_compressed.header.stamp = current_time
                uitm_msg_compressed.header.frame_id = self.frame_id
                uitm_msg_compressed.format = "jpeg"
                uitm_msg_compressed.data = cv2.imencode('.jpg', uitm_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
                self.uitm_publisher_compressed.publish(uitm_msg_compressed)
                
                # Publish UITM camera info
                uitm_camera_info_msg = CameraInfo()
                uitm_camera_info_msg.header.stamp = current_time
                uitm_camera_info_msg.header.frame_id = self.frame_id
                uitm_camera_info_msg.height = 480
                uitm_camera_info_msg.width = 640
                
                uitm_camera_info_msg.k = self.uitm_camera_matrix
                uitm_camera_info_msg.d = self.uitm_distortion_coeffs
                uitm_camera_info_msg.r = self.uitm_rectification_matrix
                uitm_camera_info_msg.p = self.uitm_projection_matrix
                
                self.uitm_publisher_camera_info.publish(uitm_camera_info_msg)

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
