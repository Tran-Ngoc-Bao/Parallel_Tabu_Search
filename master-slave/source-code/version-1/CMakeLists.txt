cmake_minimum_required(VERSION 3.16)
project(master_slave_v1 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
    CLI11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG        v2.3.2
)
FetchContent_MakeAvailable(CLI11)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

find_package(MPI REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(master_slave_v1
    src/main.cpp
    src/common.cpp
    src/input.cpp
    src/config.cpp
    src/master.cpp
    src/worker.cpp
    src/solutions.cpp
)

target_include_directories(master_slave_v1 PRIVATE src)
target_link_libraries(master_slave_v1 PRIVATE
    MPI::MPI_CXX
    OpenMP::OpenMP_CXX
    CLI11::CLI11
    nlohmann_json::nlohmann_json
)
