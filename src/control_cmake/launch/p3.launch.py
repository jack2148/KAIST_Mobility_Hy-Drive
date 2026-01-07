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

    # 2. Stanley 노드 cav1 (ID 1)
    stanley_cav_01 = Node(
        package='control_cmake',
        executable='stanley_node',
        name='stanley_controller_01', # 이름 구분
        namespace='CAV_01',           # 토픽 구분 (/CAV_01/...)
        output='screen',
        parameters=[{'csv_path': 'tool/cav1p3.csv'}],
        additional_env={'ROS_DOMAIN_ID': '1'},
        remappings=[
            ('/cmd_stop', '/CAV_01/cmd_stop')
        ]
    )

    # 3. Stanley 노드 cav2 (ID 2)
    stanley_cav_02 = Node(
        package='control_cmake',
        executable='stanley_node',
        name='stanley_controller_02', 
        namespace='CAV_02',           
        output='screen',
        parameters=[{'csv_path': 'tool/cav2p3.csv'}],
        additional_env={'ROS_DOMAIN_ID': '2'},
        remappings=[
            ('/cmd_stop', '/CAV_02/cmd_stop')
        ]
    )

    # 4. Stanley 노드 cav3 (ID 3)
    stanley_cav_03 = Node(
        package='control_cmake',
        executable='stanley_node',
        name='stanley_controller_02', 
        namespace='CAV_03',         
        output='screen',
        parameters=[{'csv_path': 'tool/cav3.csv'}],
        additional_env={'ROS_DOMAIN_ID': '3'},
        remappings=[
            ('/cmd_stop', '/CAV_03/cmd_stop')
        ]
    )

    # 5. Stanley 노드 cav4 (ID 4)
    stanley_cav_04 = Node(
        package='control_cmake',
        executable='stanley_node',
        name='stanley_controller_04', 
        namespace='CAV_04',           
        output='screen',
        parameters=[{'csv_path': 'tool/cav4.csv'}],
        additional_env={'ROS_DOMAIN_ID': '4'},
        remappings=[
            ('/cmd_stop', '/CAV_04/cmd_stop')
        ]
    )
    return LaunchDescription([
        domain_bridge,
        main_control,
        stanley_cav_01,
        stanley_cav_02,
        stanley_cav_03,
        stanley_cav_04,
    ])
