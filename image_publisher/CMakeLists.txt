cmake_minimum_required(VERSION 3.8)
project(image_publisher)

# # Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
# # enforcing cleaner code.
add_definitions(-Wall -Werror)

# # Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN license_detector::ImagePublisher
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package()
