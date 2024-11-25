from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    back_camera = Node(
            package='camera_publisher',
            executable='camera_publisher',
            name='camera_back',
            namespace='camera_back',
            output='screen',
            parameters=[
                {
                    'frame_id': 'camera2/camera_optical_link',
                    'parent_frame_id': 'sensor_kit_base_link',
                    'camera_device': '/dev/buggy_cam_back',
                    'calibration_file': 'config/back.yaml',
                    'flip_image': True,
                    'publish_compressed': True,
                    'uitm_topics': False,
                    'uitm_calibration_file': 'config/uitm/back.yaml'
                }
            ]
    )
    
    left_camera = Node(
        package='camera_publisher',
        executable='camera_publisher',
        name='camera_left',
        namespace='camera_left',
        output='screen',
        parameters=[
            {
                'frame_id': 'camera1/camera_optical_link',
                'parent_frame_id': 'sensor_kit_base_link',
                'camera_device': '/dev/buggy_cam_left',
                'calibration_file': 'config/left.yaml',
                'flip_image': False,
                'publish_compressed': True,
                'uitm_topics': False,
                'uitm_calibration_file': 'config/uitm/left.yaml'
            }
        ]
    )

    right_camera = Node(
        package='camera_publisher',
        executable='camera_publisher',
        name='camera_right',
        namespace='camera_right',
        output='screen',
        parameters=[
            {
                'frame_id': 'camera0/camera_optical_link',
                'parent_frame_id': 'sensor_kit_base_link',
                'camera_device': '/dev/buggy_cam_right',
                'calibration_file': 'config/right.yaml',
                'flip_image': False,
                'publish_compressed': True,
                'uitm_topics': False,
                'uitm_calibration_file': 'config/uitm/right.yaml'
            }
        ]
    )

    front_camera = Node(
        package='camera_publisher',
        executable='camera_publisher',
        name='camera_front',
        namespace='camera_front',
        output='screen',
        parameters=[
            {
                'frame_id': 'camera3/camera_optical_link',
                'parent_frame_id': 'sensor_kit_base_link',
                'camera_device': '/dev/buggy_cam_front',
                'calibration_file': 'config/back.yaml',
                'flip_image': False,
                'publish_compressed': True,
                'uitm_topics': False,
                'uitm_calibration_file': 'config/uitm/back.yaml'
            }
        ]
    )


    return LaunchDescription([
        back_camera,
        # left_camera,
        # right_camera,
        # front_camera
    ])
