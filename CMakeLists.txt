cmake_minimum_required(VERSION 3.15)
project(simulation)

set(CMAKE_CXX_STANDARD 17)

# Platform-specific linker flags
if(NOT APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-no-as-needed")
endif()

# Find dependencies
# macOS: Homebrew에 Eigen 5.x와 3.x가 공존 — 3.x를 강제 지정해야 RBDL과 호환됨
if(APPLE)
  find_package(Eigen3 3.3 REQUIRED
    PATHS "/opt/homebrew/Cellar/eigen/3.4.0_1"
    NO_DEFAULT_PATH)
else()
  find_package(Eigen3 3.3 REQUIRED)
endif()
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED HINTS /opt/homebrew/lib/cmake/glfw3)
find_package(GLEW REQUIRED)

message(STATUS "Eigen3 : " ${EIGEN3_INCLUDE_DIR}/Eigen)
message(STATUS "current : " ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "Platform: " ${CMAKE_SYSTEM_NAME})

add_definitions(-DMJSIMULATE_STATIC)

# Set MUJOCO root
set(MUJOCO_HOME ${CMAKE_SOURCE_DIR}/mujoco-3.6.0/)
message(STATUS "MUJOCO_HOME: " ${MUJOCO_HOME})

# RBDL: prefer project-local lib, fall back to system
set(RBDL_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
set(RBDL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/rbdl)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mujoco-3.6.0/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mujoco-3.6.0/simulate
    ${GLEW_INCLUDE_DIRS}
    ${RBDL_INCLUDE_DIR}
    /opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/
    ${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime/lib
)

add_subdirectory(src/control)
add_subdirectory(src/log)
add_subdirectory(src/planning)
add_subdirectory(src/robot)
add_subdirectory(src/utils)
add_subdirectory(src/policy)
add_subdirectory(src/observation)

# Simulate source files (shared between Linux and macOS)
set(COM_SRC
    src/main.cc
    ${MUJOCO_HOME}/simulate/glfw_dispatch.cc
    ${MUJOCO_HOME}/simulate/glfw_adapter.cc
    ${MUJOCO_HOME}/simulate/simulate.cc
    ${MUJOCO_HOME}/simulate/platform_ui_adapter.cc
)

# MuJoCo library
if(APPLE)
    set(MUJOCO_LIB ${MUJOCO_HOME}/lib/libmujoco.dylib)
    set(MUJOCO_ELASTICITY_PLUGIN ${MUJOCO_HOME}/bin/mujoco_plugin/libelasticity.dylib)

    # macOS-specific: CoreVideo/GLFW integration
    list(APPEND COM_SRC
        ${MUJOCO_HOME}/simulate/glfw_corevideo.mm
        ${MUJOCO_HOME}/simulate/macos_gui.mm
    )

    # Enable Objective-C++ for .mm files
    enable_language(OBJCXX)
    set(CMAKE_OBJCXX_STANDARD 17)
else()
    set(MUJOCO_LIB
        ${MUJOCO_HOME}/lib/libmujoco.so
        ${MUJOCO_HOME}/lib/libmujoco.so.3.6.0
    )
    set(MUJOCO_ELASTICITY_PLUGIN ${MUJOCO_HOME}/bin/mujoco_plugin/libelasticity.so)
    find_package(X11 REQUIRED)
endif()

set(COM_LIB
    ${MUJOCO_LIB}
    glfw
)

add_executable(run ${COM_SRC})

target_link_libraries(run
    control
    ${COM_LIB}
    ${CMAKE_DL_LIBS}
    OpenGL::GL
    GLEW::glew
    ${OPENGL_LIBRARIES}
    policy
    observation
)

if(NOT APPLE)
    target_link_libraries(run
        ${X11_LIBRARIES}
        OpenGL::EGL
    )
endif()

if(APPLE)
    target_link_libraries(run
        "-framework CoreVideo"
        "-framework Cocoa"
        "-framework IOKit"
    )
    # macOS: set rpath for dylibs
    set_target_properties(run PROPERTIES
        BUILD_RPATH "${MUJOCO_HOME}/lib;${CMAKE_SOURCE_DIR}/onnxruntime/lib"
        INSTALL_RPATH "${MUJOCO_HOME}/lib;${CMAKE_SOURCE_DIR}/onnxruntime/lib"
    )
endif()

add_custom_command(TARGET run POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${MUJOCO_HOME}/bin/mujoco_plugin
    $<TARGET_FILE_DIR:run>/mujoco_plugin
)
