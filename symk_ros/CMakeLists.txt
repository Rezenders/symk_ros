cmake_minimum_required(VERSION 3.8)
project(symk_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# add_executable(fast-downward src/fast-downward.cpp)
# target_include_directories(fast-downward PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(fast-downward PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS fast-downward
#   DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  symk_ros/fast-downward.py
  DESTINATION lib/${PROJECT_NAME}
)

set(py_cmd "python build.py release")
execute_process(
  COMMAND python build.py release
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)
install(DIRECTORY
  builds
  DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
