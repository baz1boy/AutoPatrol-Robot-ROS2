cmake_minimum_required(VERSION 3.8)
project(autopatrol_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# only use the necessary typesupport
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpeechText.srv"
)

ament_package()

