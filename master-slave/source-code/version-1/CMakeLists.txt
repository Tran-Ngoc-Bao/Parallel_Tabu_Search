cmake_minimum_required(VERSION 3.16)
project(master_slave_v1 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MPI REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(master_slave_v1
    src/main.cpp
    src/input.cpp
    src/master.cpp
    src/worker.cpp
)

target_include_directories(master_slave_v1 PRIVATE src)
target_link_libraries(master_slave_v1 PRIVATE MPI::MPI_CXX OpenMP::OpenMP_CXX)
