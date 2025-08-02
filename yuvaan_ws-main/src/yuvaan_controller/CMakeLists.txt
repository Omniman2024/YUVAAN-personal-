cmake_minimum_required(VERSION 3.8)  ## 3.0.2
project(yuvaan_controller)

find_package(ament_cmake REQUIRED)  ## catkin 
find_package(rclcpp REQUIRED)  ## roscpp
find_package(rclpy REQUIRED)  ##rospy
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  ## message_generation

rosidl_generate_interfaces(${YUVAAN}
	"msg/yuvaan.msg"
)
## add_message_files(FILES yuvaan.msg)
## add_message_files(FILES z.msg)

##generate_messages(DEPENDENCIES std_msgs)
ament_package()

install(DIRECTORY
	launch
	DESTINATION share/${YUVAAN}/
)

install(PROGRAMS
	DESTINATION lib/${YUVAAN}
)
