cmake_minimum_required(VERSION 2.8.4)
project(QuickFusing)


set(CMAKE_CXX_STANDARD 14)


set(CMAKE_BUILD_TOOL Release)
#set(CMAKE_BUILD_TOOL Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -lpthread")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -S -O -fno-omit-frame-pointer -g  -lpthread -Wreturn-type -Wsign-compare")
#set(CUDA_NVCC_FLAGSS ${CUDA_NVCC_FLAGS} --gpu-architecture sm_20)



#find_package(CUDA REQUIRED)
#include(FindCUDA)
find_package(Eigen3 REQUIRED)

#find_package(PythonLibs 3.0 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)

find_package(Sophus REQUIRED)
set(Sophus_LIBRARIES libSophus.so)
find_package(OpenMP)
find_package(BLAS)
find_package(LAPACK)


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# G2O
SET(G2O_ROOT /usr/local/include/g2o)
FIND_PACKAGE(G2O)
FIND_PACKAGE(CSparse)


if (OPENMP_FOUND)
        message("OPENMP FOUND")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

SET(G2O_ROOT /usr/local/include/g2o)
FIND_PACKAGE(G2O)
FIND_PACKAGE(CSparse)
FIND_PACKAGE(Cholmod)


# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ./include
        /usr/local/cuda/include
        /home/steve/Cpp_Extent/Cpp_Extent
        ${PYTHON_INDCLUDE_PATH}
        /usr/include
        /usr/local/include
        ${Sophus_INCLUDE_DIRS}
        ${BLAS_INCLUDE_DIRS}
        /usr/local/include
        ${CSPARSE_INCLUDE_DIR}
        ${G2O_INCLUDE_DIR}
        ${Cholmod_INCLUDE_DIR}
        ${SuiteSparse_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}

)

link_directories(
        /usr/local/lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ./include
)


SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator
        g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
        g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
        g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper
        g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg
        g2o_types_data g2o_types_sim3 cxsparse
        g2o_types_slam3d_addons g2o_solver_cholmod cholmod )


#FILE(GLOB SOURCES "*.cu" "*.cpp" "*.c" "*.h")

#set(SOURCE_FILES main.cpp include/EKF.hpp)
#add_executable(QuickFusing ${SOURCE_FILES} )
#target_link_libraries(QuickFusing ${PYTHON_LIBRARIES})

add_executable(gt graph_test.cpp)
add_executable(gt_new graph_test_new.cpp)
add_executable(fast_demo FastUwbDemo.cpp)

target_link_libraries(gt ${G2O_LIBS} ${CSPARSE_LIBRARY} ${PYTHON_LIBRARIES} )
target_link_libraries(gt_new ${G2O_LIBS} ${CSPARSE_LIBRARY} ${PYTHON_LIBRARIES} )
target_link_libraries(fast_demo ${G2O_LIBS} ${CSPARSE_LIBRARY} ${PYTHON_LIBRARIES} )


# for only imu mag
add_executable(ImuOnlyGraph IMUOnly-graph_test.cpp)
target_link_libraries(ImuOnlyGraph ${Sophus_LIBRARIES} ${G2O_LIBS} ${CSPARSE_LIBRARY} ${PYTHON_LIBRARIES} ${SuiteSparse_LIBRARIES})


# imu mag fusion based on gtsam
add_executable(ImuMagFusion ImuMagFusion.cpp)
target_link_libraries(ImuMagFusion
        ${Sophus_LIBRARIES} ${G2O_LIBS} ${CSPARSE_LIBRARY} ${PYTHON_LIBRARIES} ${SuiteSparse_LIBRARIES}
        -ltbb ${GTSAM_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(OnlyImuTest OnlyImuTestGTSAM.cpp)
target_link_libraries(OnlyImuTest
        ${Sophus_LIBRARIES} ${G2O_LIBS} ${CSPARSE_LIBRARY} ${PYTHON_LIBRARIES} ${SuiteSparse_LIBRARIES}
        -ltbb ${GTSAM_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(dw DemoUseResult.cpp)
#target_link_libraries(dw ${G2O_LIBS} ${Cholmod_LIBRARIES} ${CSPARSE_LIBRARY} ${PYTHON_LIBRARIES} ${SuiteSparse_LIBRARIES} )
target_link_libraries(dw ${Sophus_LIBRARIES} ${G2O_LIBS} ${CSPARSE_LIBRARY} ${PYTHON_LIBRARIES} ${SuiteSparse_LIBRARIES} )



# For test

add_executable(fun_test func_test.cpp)
target_link_libraries(fun_test ${PYTHON_LIBRARIES})

#
#add_executable(resultevaluation ResultEvaluation.cpp)
#target_link_libraries(resultevaluation ${PYTHON_LIBRARIES})

add_executable(pf_test pf_test.cpp)
target_link_libraries(pf_test ${PYTHON_LIBRARIES} ${Sophus_LIBRARIES} ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
#
add_executable(SophusTest SophusTest.cpp)
target_link_libraries(SophusTest ${Sophus_LIBRARIES})

add_executable(pdr_pf_test pdr_pf_test.cpp)
target_link_libraries(pdr_pf_test ${PPYTHON_LIBRARIES} ${Sophus_LIBRARIES} ${BLAS_LIBRARIES})
