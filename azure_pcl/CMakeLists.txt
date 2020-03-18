find_package(Boost REQUIRED)
find_package(k4a REQUIRED)
find_package(k4arecord REQUIRED)
find_package(PCL 1.10 CONFIG COMPONENTS REQUIRED common visualization kdtree segmentation filters)

set(_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE _headers RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${_include_dir}/*.h*)

add_executable(${PROJECT_NAME} ${_headers} ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/kinect_reader.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/point_cloud_utils.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_clustering.cpp)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
target_include_directories(${PROJECT_NAME} PRIVATE ${_include_dir} PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})
unset(_include_dir)
target_link_libraries(${PROJECT_NAME} Boost::boost k4a::k4a k4a::k4arecord ${PCL_LIBRARIES})
