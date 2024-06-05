cmake_minimum_required(VERSION 3.5)
project(visionproject)

# ROS 패키지와 의존성 찾기
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  image_transport
  cv_bridge
  image_geometry
)

# OpenCV 패키지 찾기
find_package(OpenCV REQUIRED)

# PCL 패키지 찾기
find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)

# catkin 특정 설정 추가
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions sensor_msgs image_transport cv_bridge
  DEPENDS PCL OpenCV
)

# 헤더 파일 경로 포함
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 실행 파일 추가 및 라이브러리 링크
add_executable(${PROJECT_NAME}_node src/pointcloud_processor.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 추가적인 컴파일러 플래그 설정 (선택적)
add_definitions(${PCL_DEFINITIONS})

