cmake_minimum_required(VERSION 2.8.12)
project(leph_plot)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

add_compile_options(-std=c++17)

add_library(${PROJECT_NAME}
    src/PlotGlobal.cpp
)

#Test names
set(TESTS_FILES
    testPlot
    testStream
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

