cmake_minimum_required(VERSION 3.5)
project(KGMT LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
    include 
    ${EIGEN3_INCLUDE_DIRS} 
    ${BOOST_INCLUDE_DIRS}  
    ${CMAKE_SOURCE_DIR}/src 
    ${CUDA_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
)

set(SOURCE_FILES
    src/planners/KGMT.cu
    src/planners/Planner.cu
    src/collisionCheck/collisionCheck.cu
    src/graphs/Graph.cu
    src/helper/helper.cu
    src/statePropagator/statePropagator.cu
    src/cpu/collisionCheck/CollisionCheck.cpp
    src/cpu/planners/OMPL_Planner.cpp
    src/cpu/planners/ModRRT.cpp;
    src/cpu/planners/ModPDST.cpp;
    src/cpu/planners/ModEST.cpp;
    src/cpu/planners/ModSyclopEST.cpp;
    src/cpu/decomposition/ModGridDecomposition.cpp;
)
add_library(KGMT_lib ${SOURCE_FILES})


if (OpenMP_CXX_FOUND)
    target_link_libraries(KGMT_lib PUBLIC OpenMP::OpenMP_CXX)
endif()


# -- GPU EXECUTABLES --
add_executable(main examples/gpu/main.cu)
target_link_libraries(main
    PUBLIC KGMT_lib
    ${CUDA_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
)

add_executable(benchMain examples/gpu/benchMain.cu)
target_link_libraries(benchMain
    PUBLIC KGMT_lib
    ${CUDA_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
)

add_executable(executionTimeMain examples/gpu/executionTimeMain.cu)
target_link_libraries(executionTimeMain
    PUBLIC KGMT_lib
    ${CUDA_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
)

add_executable(pcBenchMain examples/gpu/pcBenchMain.cu)
target_link_libraries(pcBenchMain
    PUBLIC KGMT_lib
    ${CUDA_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
)

set_target_properties(KGMT_lib PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(main PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(benchMain PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(executionTimeMain PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(pcBenchMain PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


# -- CPU EXECUTABLES --
add_executable(est_cpu examples/cpu/est_cpu.cpp)
target_link_libraries(est_cpu
    PUBLIC KGMT_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
    ${OMPL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

add_executable(rrt_cpu examples/cpu/rrt_cpu.cpp)
target_link_libraries(rrt_cpu
    PUBLIC KGMT_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
    ${OMPL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

add_executable(rrtParallel_cpu examples/cpu/rrtParallel_cpu.cpp)
target_link_libraries(rrtParallel_cpu
    PUBLIC KGMT_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
    ${OMPL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

add_executable(omplParallelBench_cpu examples/cpu/omplParallelBench_cpu.cpp)
target_link_libraries(omplParallelBench_cpu
    PUBLIC KGMT_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
    ${OMPL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

add_executable(pdst_cpu examples/cpu/pdst_cpu.cpp)
target_link_libraries(pdst_cpu
    PUBLIC KGMT_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
    ${OMPL_LIBRARIES}
    OpenMP::OpenMP_CXX
)


set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -diag-suppress=20012,20013,20014,20015 --expt-relaxed-constexpr --ptxas-options=-v")
set_property(TARGET main PROPERTY CUDA_ARCHITECTURES 86)
set_property(TARGET benchMain PROPERTY CUDA_ARCHITECTURES 86)
set_property(TARGET KGMT_lib PROPERTY CUDA_ARCHITECTURES 86)
add_definitions(-DEIGEN_NO_DEBUG)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-unused-function")
endif()
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996 /wd4244 /wd4267")
endif()
add_definitions(-D_MWAITXINTRIN_H_INCLUDED)
add_definitions(-D_FORCE_INLINES)