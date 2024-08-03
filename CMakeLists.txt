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

# Add the source files for your project
set(SOURCE_FILES
    src/planners/KGMT.cu
    src/planners/Planner.cu
    src/collisionCheck/collisionCheck.cu
    src/graphs/Graph.cu
    src/helper/helper.cu
    src/statePropagator/statePropagator.cu
    src/cpu/collisionCheck/CollisionCheck.cpp
    src/cpu/planners/OMPL_Planner.cpp
)

# Add a library target for your non-CUDA code
add_library(KGMT_lib ${SOURCE_FILES})

# Add your CUDA executable
add_executable(main examples/gpu/main.cu)

# Link the library and other dependencies with the CUDA executable
target_link_libraries(main
    KGMT_lib
    ${CUDA_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
)

# Add your CUDA executable
add_executable(benchMain examples/gpu/benchMain.cu)

# Link the library and other dependencies with the CUDA executable
target_link_libraries(benchMain
    KGMT_lib
    ${CUDA_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
)

add_executable(est_cpu examples/cpu/est_cpu.cpp)

# Link the library and other dependencies with the CUDA executable
target_link_libraries(est_cpu
    KGMT_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
    ${OMPL_LIBRARIES}
)

add_executable(rrt_cpu examples/cpu/rrt_cpu.cpp)

# Link the library and other dependencies with the CUDA executable
target_link_libraries(rrt_cpu
    KGMT_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
    ${OMPL_LIBRARIES}
)

add_executable(pdst_cpu examples/cpu/pdst_cpu.cpp)

# Link the library and other dependencies with the CUDA executable
target_link_libraries(pdst_cpu
    KGMT_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    Python3::Python
    Python3::NumPy
    ${OMPL_LIBRARIES}
)


# Ensure CUDA separable compilation
set_target_properties(KGMT_lib PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(main PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(benchMain PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# Suppress specific CUDA warnings and enable detailed information
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -diag-suppress=20012,20013,20014,20015 --expt-relaxed-constexpr --ptxas-options=-v")

# Set CUDA architectures
set_property(TARGET main PROPERTY CUDA_ARCHITECTURES 86)
set_property(TARGET benchMain PROPERTY CUDA_ARCHITECTURES 86)
set_property(TARGET KGMT_lib PROPERTY CUDA_ARCHITECTURES 86)

# Suppress all warnings from Eigen
add_definitions(-DEIGEN_NO_DEBUG)

# Additional configurations for suppressing warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-unused-function")
endif()

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996 /wd4244 /wd4267")
endif()

# Suppress warnings from CUDA headers
add_definitions(-D_MWAITXINTRIN_H_INCLUDED)
add_definitions(-D_FORCE_INLINES)
