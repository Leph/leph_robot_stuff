cmake_minimum_required(VERSION 2.8.12)
project(leph_maths)

find_package(catkin REQUIRED
    leph_plot
    leph_utils
)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
        leph_plot
        leph_utils
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_compile_options(-std=c++17)

add_library(${PROJECT_NAME}
    src/Polynomial.cpp
    src/Spline.cpp
    src/SplineLinear.cpp
    src/SplineCubic.cpp
    src/SplineQuintic.cpp
    src/LinearRegression.cpp
    src/PolyFit.cpp
    src/Polytope.cpp
    src/RecursiveLeastSquares.cpp
    src/TrajectoryBangBangAcc.cpp
    src/FilterPoseCommand.cpp
    src/DifferentialDrive.cpp
    src/PoseCommandProcessing.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES} 
)

#Test names
set(TESTS_FILES
    testTimeSeries
    testLinearRegression
    testAutoRegressive
    testFilters
    testControllerProportionalBarrier
    testRLS
    testSoftMax
    testVelocityDifferentiation
    testTrajectory
    testAxisAngle
    testFilterPoseCommand
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

