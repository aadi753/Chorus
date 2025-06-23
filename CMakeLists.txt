cmake_minimum_required(VERSION 3.10)
project(Chorus)

find_package(Python3 COMPONENTS Interpreter Development NumPy)
find_package(Eigen3 REQUIRED)

include_directories(
    ${Python3_INCLUDE_DIRS}
    include
)

add_library(Chorus SHARED
    src/otg.cpp
    src/multi_dof_otg.cpp
)

add_executable(otg_test
    bin/otg_test.cpp
)
target_link_libraries(otg_test PUBLIC
    Chorus
    ${Python3_LIBRARIES}
)

add_executable(multi_dof_otg_test
    bin/test_multi_dof_otg.cpp
)

target_link_libraries(multi_dof_otg_test PUBLIC
    Chorus
    ${Python3_LIBRARIES}
)