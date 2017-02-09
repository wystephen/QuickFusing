cmake_minimum_required(VERSION 2.8.4)
project(QuickFusing)

#set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TOOL Release)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3  -lpython2.7")
#set(CUDA_NVCC_FLAGSS ${CUDA_NVCC_FLAGS} --gpu-architecture sm_20)



#find_package(CUDA REQUIRED)
#include(FindCUDA)
find_package(Eigen3 REQUIRED)

#find_package(PythonLibs 3.0 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)

#find_package(Sophus REQUIRED)

find_package(OpenMP)

if(OPENMP_FOUND)
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ./include
        /usr/local/cuda/include
        /home/steve/Cpp_Extent/Cpp_Extent
        ${PYTHON_INDCLUDE_PATH}
        /usr/include
        /usr/local/include
#        ${Sophus_INCLUDE_DIRS}
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


add_executable(resultevaluation ResultEvaluation.cpp)
target_link_libraries(resultevaluation ${PYTHON_LIBRARIES})


add_executable(pf_test pf_test.cpp)
#target_link_libraries(pf_test ${PYTHON_LIBRARIES})
target_link_libraries(pf_test ${PYTHON_LIBRARIES} ${Sophus_LIBRARIES})
#
#
#add_executable(SophusTest SophusTest.cpp)
#target_link_libraries(SophusTest ${Sophus_LIBRARIES})

###CUDA SETTING

#cuda_add_executable(cuda_test cuda_test.cu)