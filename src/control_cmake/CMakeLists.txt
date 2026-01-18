cmake_minimum_required(VERSION 3.5)
project(control_cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # std_msgs 추가
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
<<<<<<< chan
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
=======
>>>>>>> local

# =========================================================
# 1. 기존 노드 (문제 1, 3용)
# =========================================================
add_executable(stanley_node src/control_stanley.cpp)
ament_target_dependencies(stanley_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(main_controller src/main.cpp)
ament_target_dependencies(main_controller
  rclcpp
  geometry_msgs
<<<<<<< chan
  std_msgs       # [필수] Bool 메시지 사용
)

=======
  std_msgs       
)

# =========================================================
# 2. 문제 2용 노드 (control_p2.cpp / main_p2.cpp)
# =========================================================
add_executable(stanley_p2_node src/control_p2.cpp)
ament_target_dependencies(stanley_p2_node 
  rclcpp 
  geometry_msgs 
  std_msgs
)

add_executable(main_p2_controller src/main_p2.cpp)
ament_target_dependencies(main_p2_controller 
  rclcpp 
  geometry_msgs 
  std_msgs
)

# =========================================================
# [설치 설정] 실행 파일 (TARGETS)
# =========================================================
install(TARGETS
  stanley_node
  main_controller
  stanley_p2_node
  main_p2_controller
  DESTINATION lib/${PROJECT_NAME}
)
>>>>>>> local

# =========================================================
# [설치 설정] 데이터 파일 (CSV, YAML, Launch)
# =========================================================


if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()


if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/domain_manage.yaml")
  install(FILES src/domain_manage.yaml
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

<<<<<<< chan
install(TARGETS
  stanley_node
  main_controller
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
=======
ament_package()
>>>>>>> local
