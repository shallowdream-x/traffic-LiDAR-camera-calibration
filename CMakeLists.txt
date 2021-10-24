cmake_minimum_required (VERSION 2.8)
project (edge_align)


set(CMAKE_CXX_STANDARD 14)


set(PCL_DIR "/usr/local/pcl-1.8.1/share/pcl-1.8")
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 3 REQUIRED )
find_package(PCL REQUIRED)


include_directories(
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ROOT_DIR}/yaml-cpp/include
  )

LINK_DIRECTORIES(${ROOT_DIR}/yaml-cpp/lib)

set( CMAKE_CXX_FLAGS "-fpermissive -std=c++11 -O3 -mavx" )

add_executable(main src/main.cpp)
target_link_libraries(main ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES} libyaml-cpp.a)

