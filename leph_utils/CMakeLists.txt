cmake_minimum_required(VERSION 2.8.12)
project(leph_utils)

find_package(catkin REQUIRED
    rospack
    roscpp
)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
        rospack
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_compile_options(-std=c++17)

add_library(${PROJECT_NAME}
    src/Chrono.cpp
    src/Filesystem.cpp
    src/SpaceMouse.cpp
    src/ViveTracking.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    atomic
)

#Test names
set(TESTS_FILES
    testChrono
    testTimeSeriesBuffer
    testFilesystem
    testImporterCSV
)

#Build tests
if (CATKIN_ENABLE_TESTING)
    foreach (TEST ${TESTS_FILES})
        add_executable(${TEST} test/${TEST}.cpp)
        target_link_libraries(${TEST} 
            ${PROJECT_NAME}
            ${catkin_LIBRARIES}
        )
    endforeach ()
endif (CATKIN_ENABLE_TESTING)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

