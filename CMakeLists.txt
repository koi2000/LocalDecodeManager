cmake_minimum_required(VERSION 3.16)


set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.3/bin/nvcc")
project(LocalDecodeManager LANGUAGES  CXX C CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -Wno-unused-result -lpthread")


include_directories(
    ./include
    /usr/include
    /usr/local/include
    ${CUDA_INCLUDE_DIRS}
)

set(MINICGAL_SOURCE_DIR "deps/MCGAL/Core")

file(GLOB_RECURSE MINI_CGAL_FILES
    "${MINICGAL_SOURCE_DIR}/include/*.h"
    "${MINICGAL_SOURCE_DIR}/*.cpp"
)

file(GLOB_RECURSE LOCAL_COMPRESS_FILES
    "src/include/*.h"
    "src/*.cpp"
)

find_package(OpenMP REQUIRED)
find_package(CUDA REQUIRED)

# add_executable(MCGAL_Test tools/MCGAL_Test.cpp)
# target_link_libraries(MCGAL_Test /usr/local/lib/libMCGAL.so)

add_executable(LocalEncoderTest tools/LocalEncoderTest.cpp ${LOCAL_COMPRESS_FILES})
# target_link_libraries(LocalEncoderTest /usr/local/lib/libMCGAL.so)

add_executable(LocalDecoderTest tools/LocalDecoderTest.cpp ${LOCAL_COMPRESS_FILES})
# target_link_libraries(LocalDecoderTest /usr/local/lib/libMCGAL.so)
