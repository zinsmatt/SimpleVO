cmake_minimum_required(VERSION 2.8)
project(SimpleVO)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################
# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)

# G2O
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
# include_directories(${GLOG_INCLUDE_DIRS})

# # gtest
# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})


set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o::core g2o::stuff g2o::types_sba g2o::solver_csparse g2o::csparse_extension
        # ${GTEST_BOTH_LIBRARIES}
        glog::glog
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        Sophus::Sophus
        )


add_executable(SimpleVO
main.cpp
camera.h
camera.cpp
frame.cpp
frame.h
feature.cpp
feature.h
map_point.cpp
map_point.h
map.cpp
map.h
frontend.cpp
frontend.h
common.h
g2o_types.h
)

target_link_libraries(SimpleVO ${THIRD_PARTY_LIBS})
