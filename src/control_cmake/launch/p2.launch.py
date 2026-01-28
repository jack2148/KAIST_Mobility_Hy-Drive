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
    
    # 문제 2용 메인 컨트롤러 (main_p2_controller로 가정)
    main_control = Node(
        package='control_cmake',
        executable='main_p2_controller', 
        name='main_controller',
        output='screen',
        additional_env={'ROS_DOMAIN_ID': '100'}
    )

    # 문제 2용 Stanley 노드 실행
    stanley_cav_01 = Node(
        package='control_cmake',
        executable='stanley_p2_node', # 위 CMakeLists.txt에서 만든 실행파일 이름
        name='stanley_p2_controller', 
        namespace='CAV_01',           
        output='screen',
        parameters=[{
            'csv_path_1': 'tool/JSp2Lane1.csv',  # 안쪽(추월) 경로
            'csv_path_2': 'tool/JSp2Lane2.csv',  # 바깥쪽(주행) 경로  slop2.csv
            'merge_csv_path': 'tool/JSp2merge.csv',
            'k_gain': 1.3
        }],
        additional_env={'ROS_DOMAIN_ID': '1'},
        remappings=[
            ('/cmd_stop', '/CAV_01/cmd_stop'),
            # 메인 컨트롤러가 보내는 신호와 내 차의 토픽을 연결
            ('/cmd_lane', '/CAV_01/cmd_lane'),
            ('/cmd_speed', '/CAV_01/cmd_speed')
        ]
    )

    return LaunchDescription([
        domain_bridge,
        main_control,
        stanley_cav_01,
    ])