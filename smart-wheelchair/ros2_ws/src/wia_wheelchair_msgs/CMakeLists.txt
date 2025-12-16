cmake_minimum_required(VERSION 3.8)
project(wia_wheelchair_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Declare the message files
set(msg_files
  "msg/WheelchairState.msg"
  "msg/MotorCommand.msg"
  "msg/MotorStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/CollisionWarning.msg"
  "msg/JoystickInput.msg"
)

# Declare the service files
set(srv_files
  "srv/SetMode.srv"
  "srv/GetMode.srv"
  "srv/SetSpeedLimit.srv"
  "srv/SetProfile.srv"
)

# Declare the action files
set(action_files
  "action/GoToPose.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
