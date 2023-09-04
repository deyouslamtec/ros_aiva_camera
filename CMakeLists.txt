#  minimum required cmake version: 3.0.2
cmake_minimum_required( VERSION 3.0.2 )

project( depthsense_publisher )
add_definitions( -std=c++14 )

set( THIRD_PARTY_LIBS 
	depthsense )

include_directories(${PROJECT_SOURCE_DIR}/include)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
	link_directories(${PROJECT_SOURCE_DIR}/lib/x86_64/)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm64")
	link_directories(${PROJECT_SOURCE_DIR}/lib/linux-arm64-v8a/)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm")
	link_directories(${PROJECT_SOURCE_DIR}/lib/linux-armeabi-v7a/)
endif()

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	cv_bridge
	image_transport
	camera_info_manager
	sensor_msgs
	nodelet
	dynamic_reconfigure)

find_package(OpenCV)

find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(cfg/DepthsensePublisher.cfg)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED src/depthsense_publisher_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${THIRD_PARTY_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(depthsense_publishe_node src/depthsense_publisher.cpp)
target_link_libraries(depthsense_publishe_node ${catkin_LIBRARIES})

set_target_properties(depthsense_publishe_node PROPERTIES OUTPUT_NAME depthsense_publisher)

install(TARGETS depthsense_publishe_node ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
