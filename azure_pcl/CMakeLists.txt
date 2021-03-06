find_package(Boost 1.73 REQUIRED)
find_package(k4a REQUIRED)
find_package(k4arecord REQUIRED)
find_package(PCL 1.10 COMPONENTS REQUIRED common visualization kdtree segmentation filters)

set(_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE _headers RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${_include_dir}/*.h*)

add_executable(${PROJECT_NAME} ${_headers} 
${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/src/kinect_reader.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/src/point_cloud_utils.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_clustering.cpp)



target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
target_include_directories(${PROJECT_NAME} PRIVATE ${_include_dir} PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})
target_precompile_headers(${PROJECT_NAME} PRIVATE ${_include_dir}/pch.h)
target_link_libraries(${PROJECT_NAME} PRIVATE Boost::headers PRIVATE ${PCL_LIBRARIES} PRIVATE k4a::k4arecord PRIVATE k4a::k4a)

if(UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE udev) 
endif()

unset(_include_dir)

