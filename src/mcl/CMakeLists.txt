cmake_minimum_required(VERSION 2.8)
project(gmapping)

add_compile_options(-std=c++11)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
	kobuki_msgs)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs)

# Download matplotplus library on the go if does not exist
find_package(Matplot++ QUIET)
if(NOT Matplot++_FOUND)
include(FetchContent)
FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master) # or whatever tag you want
        
        FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
endif()

catkin_package()

include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(map
    src/map.h
    src/map.cpp
)

target_link_libraries(map ${catkin_LIBRARIES})
target_link_libraries(map matplot)

add_dependencies(map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add the publisher example
add_executable(main src/main.cpp)
target_link_libraries(main map matplot ${catkin_LIBRARIES} ${OpenCV_LIB})

