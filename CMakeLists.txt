cmake_minimum_required(VERSION 2.8.4)
project(QuickFusing)

set(CMAKE_CXX_STANDARD 11)

set( CMAKE_CXX_FLAGS "-std=c++11 -O3  -lpython2")
set(CUDA_NVCC_FLAGSS ${CUDA_NVCC_FLAGS} --gpu-architecture sm_20)



find_package(CUDA REQUIRED)
include(FindCUDA)
find_package(Eigen3 REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ./include
        /usr/local/cuda/include
        /home/steve/Cpp_Extent/Cpp_Extent
)

FILE(GLOB SOURCES "*.cu" "*.cpp" "*.c" "*.h")

set(SOURCE_FILES main.cpp)
add_executable(QuickFusing ${SOURCE_FILES} )


###CUDA SETTING

cuda_add_executable(cuda_test cuda_test.cu)