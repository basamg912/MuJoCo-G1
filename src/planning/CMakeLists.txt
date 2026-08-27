add_library(planning STATIC
    trajectory.cpp
)

target_include_directories(planning PUBLIC
    ${CMAKE_SOURCE_DIR}/mujoco-3.6.0/include
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(planning PUBLIC
    Eigen3::Eigen
)



