cmake_minimum_required(VERSION 2.8.4)
project(QuickFusing)

set(CMAKE_CXX_STANDARD 11)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3  -lpython2.7")
#set(CUDA_NVCC_FLAGSS ${CUDA_NVCC_FLAGS} --gpu-architecture sm_20)



#find_package(CUDA REQUIRED)
#include(FindCUDA)
find_package(Eigen3 REQUIRED)

find_package(PythonLibs 2.7 REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ./include
        /usr/local/cuda/include
        /home/steve/Cpp_Extent/Cpp_Extent
        ${PYTHON_INDCLUDE_PATH}
        /usr/include
        /usr/local/include
)

link_directories(
        /usr/local/lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu
)

#FILE(GLOB SOURCES "*.cu" "*.cpp" "*.c" "*.h")

set(SOURCE_FILES main.cpp include/EKF.hpp)
add_executable(QuickFusing ${SOURCE_FILES} )

target_link_libraries(QuickFusing ${PYTHON_LIBRARIES})


# For test

add_executable(fun_test func_test.cpp)
target_link_libraries(fun_test ${PYTHON_LIBRARIES})


###CUDA SETTING

#cuda_add_executable(cuda_test cuda_test.cu)