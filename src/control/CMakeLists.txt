add_library(control STATIC
    controller.cpp)

target_include_directories(control PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/mujoco-3.6.0/include
    ${CMAKE_SOURCE_DIR}/include/rbdl
    /opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3
)

target_link_libraries(control PUBLIC
    robot
    planning
    Eigen3::Eigen
)
