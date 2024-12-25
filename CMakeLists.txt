cmake_minimum_required(VERSION 2.8.3)
project(imu_utils)


# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Pangolin REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

include_directories("./")


set(ACC_LIB_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/acc_lib/allan_acc.cpp
    ${PROJECT_SOURCE_DIR}/acc_lib/fitallan_acc.cpp
    )
set(GYR_LIB_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/gyr_lib/allan_gyr.cpp
    ${PROJECT_SOURCE_DIR}/gyr_lib/fitallan_gyr.cpp
    )

add_library(imu_calibr SHARED imu_calibr.cpp ${GYR_LIB_SOURCE_FILES} ${ACC_LIB_SOURCE_FILES} )
target_link_libraries(imu_calibr ${CERES_LIBRARIES} yaml-cpp)
set_target_properties(imu_calibr PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  

# only for Orbrecc
# obsensor sdk
set(OrbbecSDK_ROOT_DIR ${CMAKE_SOURCE_DIR}/SDK)
set(OrbbecSDK_LIBRARY_DIRS ${OrbbecSDK_ROOT_DIR}/lib)
set(OrbbecSDK_INCLUDE_DIR ${OrbbecSDK_ROOT_DIR}/include)
include_directories(${OrbbecSDK_INCLUDE_DIR})
link_directories(${OrbbecSDK_LIBRARY_DIRS})




add_executable(main 
  main.cpp
)
target_link_libraries(main imu_calibr yaml-cpp OrbbecSDK)


