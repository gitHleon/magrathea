cmake_minimum_required(VERSION 3.4)
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(STATUS "Magrathea requires an out-of-source build.")
endif()

project(Magrathea)
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#
# Setup Qt stuff
#
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(QT5_DIR /usr/local/Cellar/qt/5.13.0)
list(APPEND CMAKE_PREFIX_PATH ${QT5_DIR})
set(Qt5Widgets_DIR ${QT5_DIR}/lib/cmake/Qt5Core)
set(Qt5Gui_DIR ${QT5_DIR}/lib/cmake/Qt5Gui)
set(Qt5Widgets_DIR ${QT5_DIR}/lib/cmake/Qt5Widgets)
set(Qt5Multimedia_DIR ${QT5_DIR}/lib/cmake/Qt5Multimedia)
set(Qt5MultimediaWidgets_DIR ${QT5_DIR}/lib/cmake/Qt5MultimediaWidgets)

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Multimedia CONFIG REQUIRED)
find_package(Qt5MultimediaWidgets CONFIG REQUIRED)
find_package(Qt5SerialPort CONFIG REQUIRED)


#
# Find OpenCV
#
find_package( OpenCV REQUIRED )


#
# Create alglib library
#
set(alglib_DIR alglib-3.15.0/cpp/src)
include_directories(${alglib_DIR})
FILE(GLOB alglib_SOURCES ${alglib_DIR}/*.cpp)	
add_library(alglib STATIC ${alglib_SOURCES})

# Test flatness
add_executable(test-flatness test-flatness.cc flatness.cc alglibtools.cc)
target_link_libraries(test-flatness alglib)	

set(magrathea_SOURCES
	magrathea.ui
	main.cpp 
	magrathea.cpp 
	calibrator.cpp 
	focus_finder.cpp 
	MotionHandler.cpp 
	Fiducial_finder.cpp 
	fiducial_locations.cpp 
	verticalalignmenttool.cpp 
	flatness.cc
	alglibtools.cc
	)
add_executable(magrathea ${magrathea_SOURCES})
# CHOSE IFIC MOTION CONTROLLER
#target_compile_definitions(magrathea PUBLIC -DACSC=1 -DVALENCIA=1)
# Use the Widgets module from Qt 5
target_link_libraries(magrathea 
	Qt5::Core 
	Qt5::Gui 
	Qt5::Widgets 
	Qt5::Multimedia 
	Qt5::MultimediaWidgets 
	Qt5::SerialPort
	${OpenCV_LIBS} alglib)
	
	
