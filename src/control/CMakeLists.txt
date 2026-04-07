add_library(control STATIC #.a 파일로 생성,
    controller.cpp)

target_include_directories(control PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/mujoco-3.6.0/include
    /usr/local/include/rbdl
)

target_link_libraries( control PUBLIC
    robot
    planning
    Eigen3::Eigen
)