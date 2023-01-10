cmake_minimum_required(VERSION 2.8.12)
project(leph_viewer_rhio)

find_package(catkin REQUIRED
    libzmq_catkin
    rhio_common
    rhio_client
    leph_plot
    lepg_utils
)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
        libzmq_catkin
        rhio_common
        rhio_client
        leph_plot
        leph_utils
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_compile_options(-std=c++17 -Wno-invalid-partial-specialization)

add_library(${PROJECT_NAME}
    src/InterfaceCLI.cpp
    src/TreeNode.cpp
    src/TreeValue.cpp
    src/WinBase.cpp
    src/WinTree.cpp
    src/WinPlot.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ncurses
)

add_executable(rhioCursesLog
    bin/rhioCursesLog.cpp
)
target_link_libraries(rhioCursesLog
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ncurses
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS rhioCursesLog DESTINATION bin)

