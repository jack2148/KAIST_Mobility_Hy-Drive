from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # domain_bridge yaml파일 실행
    bridge_config_path = 'src/control_cmake/src/domain_manage.yaml' 

    domain_bridge = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_manage',
        output='screen',
        arguments=[bridge_config_path],
        additional_env={'ROS_DOMAIN_ID': '100'} 
    )
    
    # 1. 메인 컨트롤러 (ID 100)
    main_control = Node(
        package='control_cmake',
        executable='main_controller',
        name='main_controller',
        output='screen',
        additional_env={'ROS_DOMAIN_ID': '100'}
    )

    # 2. Stanley 노드 1 (ID 1)
    stanley_cav_01 = Node(
        package='control_cmake',
        executable='stanley_node',
        name='stanley_controller_01', # 이름 구분
        namespace='CAV_01',           # 토픽 구분 (/CAV_01/...)
        output='screen',
        parameters=[{'csv_path': 'tool/p1_1.csv'}],
        additional_env={'ROS_DOMAIN_ID': '1'}
    )

    # 3. Stanley 노드 2 (ID 2)
    stanley_cav_02 = Node(
        package='control_cmake',
        executable='stanley_node',
        name='stanley_controller_02', # 이름 구분
        namespace='CAV_02',           # 토픽 구분 (/CAV_02/...)
        output='screen',
        parameters=[{'csv_path': 'tool/p1_2.csv'}],
        additional_env={'ROS_DOMAIN_ID': '2'}
    )

    return LaunchDescription([
        domain_bridge,
        main_control,
        stanley_cav_01,
        stanley_cav_02,
    ])
