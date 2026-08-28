add_library(policy STATIC
    policy.cpp
)
add_library(attention_policy STATIC
    attention_policy.cpp
)

target_link_directories(policy PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/onnxruntime/include
)

target_link_libraries(policy PUBLIC
    Eigen3::Eigen
    onnxruntime
)
target_link_directories(attention_policy PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/onnxruntime/include
)

target_link_libraries(attention_policy PUBLIC
    Eigen3::Eigen
    onnxruntime
)
