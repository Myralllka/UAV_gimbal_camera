cmake_minimum_required(VERSION 3.1.2)
project(mrs_gimbal_camera)

set(FILESNAME GimbalCameraNodelet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(apriltag_DIR /home/mrs/april_tag_workspace)

add_compile_options(-Wall -Wextra -pedantic -Werror -Werror=vla)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        std_msgs
        mrs_lib
        apriltag_ros
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp mrs_lib
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        )

add_library(${FILESNAME} src/${FILESNAME}.cpp)

add_dependencies(${FILESNAME} ${${FILESNAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${FILESNAME} ${catkin_LIBRARIES})
target_link_libraries (${FILESNAME} Eigen3::Eigen)
