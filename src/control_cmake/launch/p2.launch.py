from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bridge_config_path = 'src/control_cmake/src/domain_manage.yaml'

    domain_bridge = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_manage',
        output='screen',
        arguments=[bridge_config_path],
        additional_env={'ROS_DOMAIN_ID': '100'}
    )
    
    main_control = Node(
        package='control_cmake',
        executable='main_p2_controller',
        name='main_p2_controller',
        output='screen',
        additional_env={'ROS_DOMAIN_ID': '100'}
    )

    stanley_cav_01 = Node(
        package='control_cmake',
        executable='stanley_p2_node',
        name='stanley_p2_controller',
        namespace='CAV_01', # 중요: 토픽 앞에 /CAV_01/을 붙여줌
        output='screen',
        parameters=[{
            'csv_path_1': 'tool/JSp2Lane1.csv',
            'csv_path_2': 'tool/JSp2Lane2.csv',
            'k_gain': 1.3,
            'max_steer': 0.9,
            'center_to_front': 0.17,
            'wheelbase': 0.33,
            'steer_gain': 1.0,
            'forward_step': 15,
            'target_speed': 1.3
        }],
        additional_env={'ROS_DOMAIN_ID': '1'}
    )

    return LaunchDescription([
        domain_bridge,
        main_control,
        stanley_cav_01,
    ])