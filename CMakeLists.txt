cmake_minimum_required( VERSION 2.8 )
project(YGZ-Stereo)

# uncommont this to use release
set( CMAKE_BUILD_TYPE "Release" )
#set(CMAKE_BUILD_TYPE "Debug")


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# thirdparty libs 
find_package(OpenCV REQUIRED)
MESSAGE("OPENCV PATH: ${OpenCV_INCLUDE_DIRS}")

#find_package(Qt5 REQUIRED)
find_package(Qt5Widgets REQUIRED)


set(CPP_SOURCES src/main.cpp src/mainwindow.cpp)
set(CPP_HEADERS include/mainwindow.h)

include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(${Qt5Widgets_DEFINITIONS})

qt5_wrap_ui(UIS_HEADERS src/mainwindow.ui)

qt5_wrap_cpp(MOC_SOURCES ${CPP_SOURCES} ${CPP_HEADERS})

qt5_add_resources(RESOURCES_RCC ${RESOURCE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} -std=c++11")

set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# octomap
find_package(catkin REQUIRED COMPONENTS
                                cv_bridge
                                image_transport
                                roscpp
                                sensor_msgs
				geometry_msgs
                                nav_msgs
                                std_msgs
                                octomap_msgs
                                octomap_rviz_plugins
                                visualization_msgs
                                tf)

include_directories(
${OpenCV_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
include
)


# thirdparty
set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Qt5Widgets_LIBRARIES}
        ${catkin_LIBRARIES}
)


add_executable(run ${CPP_SOURCES} ${UIS_HEADERS} ${MOC_SOURCES} ${catkin_INCLUDE_DIRS})

target_link_libraries(run
        ${THIRD_PARTY_LIBS}
        )

qt5_use_modules(run Widgets)
