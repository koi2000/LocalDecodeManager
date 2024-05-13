cmake_minimum_required(VERSION 3.16)


set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.3/bin/nvcc")
project(LocalDecodeManager LANGUAGES  CXX C CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wno-unused-result -lpthread")


include_directories(
    ./include
    /usr/include
    /usr/local/include
    ${CUDA_INCLUDE_DIRS}
)
find_package(OpenMP REQUIRED)
find_package(CUDA REQUIRED)

add_executable(MCGAL_Test tools/MCGAL_Test.cpp)
target_link_libraries(MCGAL_Test /usr/local/lib/libMCGAL.so)
