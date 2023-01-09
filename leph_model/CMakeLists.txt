cmake_minimum_required(VERSION 2.8.12)
project(leph_model)

find_package(catkin REQUIRED
    urdf
    rbdl_catkin
    leph_maths
    leph_plot
    leph_utils
)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
        urdf
        rbdl_catkin
        leph_maths
        leph_plot
        leph_utils
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
)
link_directories(
    ${CMAKE_INSTALL_PREFIX}/lib/
)

add_compile_options(-std=c++17 -Wno-invalid-partial-specialization)

add_library(${PROJECT_NAME}
    src/Model.cpp
    src/RBDLRootUpdate.cpp
    src/InverseKinematics.cpp
    src/ModelHumanoidFixed.cpp
    src/ForwardSimulation.cpp
    src/InverseDynamics.cpp
    src/InverseDynamicsSimplified.cpp
    src/MultiContactIKStaticID.cpp
    src/PinocchioInterface.cpp
    src/ActuatorSEA.cpp
    src/SEIKOBimanual.cpp
    src/SEIKOFloatingBase.cpp
    src/SEIKOWrapper.cpp
    src/SEIKOTalos.cpp
    src/SEIKOTiago.cpp
    src/LegIKTalos.cpp
)
target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES} 
    ${pinocchio_LIBRARIES} 
)

#Test names
set(TESTS_FILES
    testModel
    testImportState
    testPinocchio
    testWrenchTransform
    testCentroidal
    testJacobian
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

