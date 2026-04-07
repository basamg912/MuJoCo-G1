add_library( robot robotmodel.cpp
)

target_include_directories(robot PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/mujoco-3.6.0/include
    /usr/local/include
)

target_link_libraries(robot PUBLIC
    Eigen3::Eigen
    -lrbdl
    -lrbdl_urdfreader
)