from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('robocup_vision'),
        'model',
        'best.xml'
    )

    config_dir = os.path.join(
        get_package_share_directory('robocup_vision'),
        'config',
        'camera_config.yaml'
    )

    info_dir = os.path.join(
        get_package_share_directory('robocup_vision'),
        'config',
        'camera_info_config.yaml'
    )

    # YAML 설정 파일 로드
    with open(config_dir, 'r') as file:
        config_params = yaml.safe_load(file)
        camera_name = config_params['/**']['ros__parameters']['camera_name']
        # topic = config_params['/**']['ros__parameters']['topic']
        # viewer_enabled = config_params['/**']['ros__parameters'].get('viewer', False)

    node_name = f'pan_tilt_camera_node_{camera_name}'
    # image_topic = f'{camera_name}{topic}'

    pan_tilt_camera_node = Node(
        package='robocup_vision',
            executable='pan_tilt_camera_node',
            name=node_name,
            output='screen',
            parameters=[config_dir, info_dir]
    )

    detection_node = Node(
        package='robocup_vision',
        executable='detection_node',
        name='detection_node',
        output='screen',
        parameters=[
            {'model_xml': model_path}
        ]
    )

    refiner_node = Node(
        package='robocup_vision',
        executable='refiner_node',
        name='refiner_node',
        output='screen'
    )

    pan_tilt_node = Node(
        package='robocup_vision',
        executable='pan_tilt_node',
        name='pan_tilt_node',
        output='screen'
    )

    return LaunchDescription([
        pan_tilt_camera_node,
        detection_node,
        refiner_node,
        pan_tilt_node
    ])
