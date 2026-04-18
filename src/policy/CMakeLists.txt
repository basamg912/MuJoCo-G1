add_library(policy STATIC
    policy.cpp
)

target_include_directories(policy PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/onnxruntime/include
)

target_link_directories(policy PUBLIC
    ${CMAKE_SOURCE_DIR}/onnxruntime/lib
)

target_link_libraries(policy PUBLIC
    Eigen3::Eigen
    onnxruntime
)
