cmake_minimum_required(VERSION 3.10)
project(Chorus)

# for matplotlibcppp
find_package(Python3 COMPONENTS Interpreter Development NumPy)
find_package(Eigen3 REQUIRED)

include_directories(
    ${Python3_INCLUDE_DIRS}
    include
)

# main chorus module
add_library(Chorus SHARED
    src/otg.cpp
    src/multi_dof_otg.cpp
)

# single dof otg test
add_executable(otg_test
    bin/otg_test.cpp
)
target_link_libraries(otg_test PUBLIC
    Chorus
    ${Python3_LIBRARIES}
)

# multi dof otg test
add_executable(multi_dof_otg_test
    bin/test_multi_dof_otg.cpp
)

target_link_libraries(multi_dof_otg_test PUBLIC
    Chorus
    ${Python3_LIBRARIES}
)

# test where only targets are updated
add_executable(example_multi_dof_target_update
    bin/example_multi_dof_target_update.cpp
)

target_link_libraries(example_multi_dof_target_update PUBLIC
    Chorus
    ${Python3_LIBRARIES}
)

# test where targets and contraints are updated
add_executable(example_multi_dof_target_and_constraints_update
    bin/example_multi_dof_target_and_constraints_update.cpp
)

target_link_libraries(example_multi_dof_target_and_constraints_update PUBLIC
    Chorus
    ${Python3_LIBRARIES}
)

# test where only constraints are updated
add_executable(example_multi_dof_constraints_update
    bin/example_multi_dof_constraints_update.cpp
)

target_link_libraries(example_multi_dof_constraints_update PUBLIC
    Chorus
    ${Python3_LIBRARIES}
)