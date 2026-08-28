cmake_minimum_required(VERSION 3.10)
project(TEST_PROJECT)

# test
message(STATUS "$ENV{HOME}")
set(my_name "tkdyun")
message(STATUS "${my_name}")
set(MUJOCO_DIR "$ENV{HOME}/work/mujoco/")

# set(mujoco_DIR "${MUJOCO_DIR}")
# list(APPEND CMAKE_PREFIX_PATH "${mujoco_DIR}")


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mujoco REQUIRED)

add_executable(run main.cpp)

target_link_libraries(run mujoco::mujoco)
