cmake_minimum_required(VERSION 2.8.3)
project(raposang_odometry)

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    tf
    roscpp
    idmind_motorsboard
)

catkin_package(
    INCLUDE_DIRS
        src/include
    CATKIN_DEPENDS
        nav_msgs
        tf
        roscpp
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        src/include
)

add_executable(raposang_odometry_node
        src/raposang_odometry_node.cpp
)

# Link against the catkin libraries
target_link_libraries(raposang_odometry_node ${catkin_LIBRARIES})


add_dependencies(raposang_odometry_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install the executable
install(TARGETS raposang_odometry_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})