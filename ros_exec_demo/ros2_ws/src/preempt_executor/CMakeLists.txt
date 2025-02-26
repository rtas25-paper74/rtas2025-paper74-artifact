cmake_minimum_required(VERSION 3.5)
project(preempt_executor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(simple_timer REQUIRED)

add_library(primes_workload src/primes_workload.cpp)
target_include_directories(primes_workload PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(primes_workload
  simple_timer
)

add_library(test_nodes src/test_nodes.cpp)
target_include_directories(test_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_nodes
  primes_workload
)
ament_target_dependencies(test_nodes
  rclcpp std_msgs
)

add_library(preempt_executor  src/preempt_executor.cpp)
target_include_directories(preempt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(preempt_executor
  primes_workload
)
ament_target_dependencies(preempt_executor
  rclcpp #rclc
)

add_executable(run src/run.cpp)
target_link_libraries(run
  preempt_executor test_nodes
)
ament_target_dependencies(run
  rclcpp simple_timer
)


install(TARGETS
  preempt_executor primes_workload test_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  run
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(preempt_executor primes_workload test_nodes)

ament_package()
