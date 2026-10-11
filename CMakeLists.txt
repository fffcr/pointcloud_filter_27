cmake_minimum_required(VERSION 3.8)
project(livox_filter_27)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# macOS（RoboStack / pixi）：环境里不设 DYLD_LIBRARY_PATH，运行时动态库解析
# 完全依赖二进制自身的 rpath。RoboStack 预编译的包带 @loader_path/.. ，但 colcon
# 编出来的目标默认不写任何 rpath，于是启动时报：
#   dyld: Library not loaded: @rpath/libmessage_filters.dylib
#   (librclcpp.dylib 等同理，它们都在 $CONDA_PREFIX/lib 下)
# 这里显式把 conda 环境的 lib 目录写进 rpath。
if(APPLE)
  if(DEFINED ENV{CONDA_PREFIX})
    set(CMAKE_INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib")
  endif()
  # 兜底：把链接期用到的库目录也加进来
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
#输入改成 PointCloud2 后不再依赖 livox_ros_driver2，需要 CustomMsg 时再打开
# find_package(livox_ros_driver2 REQUIRED)

# 法向量估计走 OMP 并行，一定要开 OpenMP，否则 NormalEstimationOMP 会退化甚至编不过
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters features search kdtree omp)

include_directories(
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_node
  src/slope_filter.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  pcl_conversions
  # livox_ros_driver2
)

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

if(APPLE)
  # ROS 的消息包会把 Python 绑定库（*__rosidl_generator_py.dylib，由 rclcpp 等
  # 经 interface target 传递进来）一起带上链接行。这些是用
  # `-undefined dynamic_lookup` 编的 Python 扩展模块，符号本该由 Python 宿主
  # 进程提供；链进纯 C++ 节点后 dyld 解析不到 _PyExc_RuntimeError 会直接崩：
  #   dyld: symbol not found in flat namespace '_PyExc_RuntimeError'
  #
  # 注意：只在 CMake 层过滤 LINK_LIBRARIES 是没用的——大部分 py 绑定是经由
  # interface target 的 INTERFACE_LINK_LIBRARIES 展开进来的，过滤不掉。
  # 交给链接器：-dead_strip_dylibs 会丢掉输出里没有任何符号被引用的 dylib，
  # 正好把用不到的 Python 绑定清干净，其余依赖不受影响。
  target_link_options(${PROJECT_NAME}_node PRIVATE "-Wl,-dead_strip_dylibs")
endif()

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
