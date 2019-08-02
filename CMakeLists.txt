cmake_minimum_required(VERSION 3.4)
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(STATUS "Magrathea requires an out-of-source build.")
endif()

project(Magrathea)
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -pedantic")

#
# Find OpenCV
#
find_package( OpenCV REQUIRED )

#
# Create xlnt for reading XLSX files
#
set( STATIC ON CACHE BOOL "XNLT static library" FORCE)
set( TESTS OFF CACHE BOOL "disable tests" FORCE)
add_subdirectory(xlnt)
find_library(xlnt_LIB xlnt PATHS ${CMAKE_CURRENT_BINARY_DIR}/xlnt/source)
set(xlnt_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/xlnt/include)
if 	(WIN32)
	set(xlnt_LIB  ${CMAKE_CURRENT_BINARY_DIR}/xlnt/source/libxlnt.a)
endif (WIN32)

#
# Create alglib library
#
set(alglib_DIR alglib-3.15.0/cpp/src)
FILE(GLOB alglib_SOURCES ${alglib_DIR}/*.cpp)
add_library(alglib STATIC ${alglib_SOURCES})

#
# Create Geom library
#
set(geomlib_SOURCES MatrixTransform.cc Point.cc)
add_library(geomlib STATIC ${geomlib_SOURCES})
target_include_directories(geomlib PUBLIC ${OpenCV_INCLUDE_DIRS})

# Set include directories
include_directories(${alglib_DIR} ${xlnt_INCLUDE})

# Test flatness
add_executable(test-flatness test-flatness.cc flatness.cc alglibtools.cc)
target_link_libraries(test-flatness alglib  ${xlnt_LIB})
add_dependencies (test-flatness xlnt)

# Test transforms
add_executable(test-matrix test-matrix.cc PetalCoordinates.cc)
target_link_libraries(test-matrix geomlib ${xlnt_LIB})
target_include_directories(test-matrix PUBLIC ${OpenCV_INCLUDE_DIRS} )
add_dependencies (test-matrix xlnt)

# Test Fiducial Finder
add_executable(test-f-finder test_f_finder.cc Fiducial_finder.cpp logger.cc )
target_include_directories(test-f-finder PUBLIC ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(test-f-finder ${OpenCV_LIBS} geomlib alglib)


#
# Setup Qt stuff
#
if (WIN32)
	set(QT5_DIR d:/Qt/5.9.6/mingw53_32)
elseif (APPLE)
	set(QT5_DIR /usr/local/Cellar/qt/5.13.0)
endif (WIN32)
list(APPEND CMAKE_PREFIX_PATH ${QT5_DIR})
set(Qt5Widgets_DIR ${QT5_DIR}/lib/cmake/Qt5Core)
set(Qt5Gui_DIR ${QT5_DIR}/lib/cmake/Qt5Gui)
set(Qt5Widgets_DIR ${QT5_DIR}/lib/cmake/Qt5Widgets)
set(Qt5Multimedia_DIR ${QT5_DIR}/lib/cmake/Qt5Multimedia)
set(Qt5MultimediaWidgets_DIR ${QT5_DIR}/lib/cmake/Qt5MultimediaWidgets)


# Find the QtWidgets library
find_package(Qt5 COMPONENTS Core Gui Widgets Multimedia MultimediaWidgets SerialPort REQUIRED)


# Include the QJoysticks
if (UNIX)
	find_package(SDL2 )
elseif (WIN32)
	set(SDL2_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/QJoysticks-master/lib/SDL/include)
	message( "SDL include ${SDL2_INCLUDE_DIRS}")
endif (UNIX)
set(qjoystick_DIR QJoysticks-master)
set(qjoystick_INCLUDES ${qjoystick_DIR}/src)
set(qjoystick_SOURCES
	${qjoystick_DIR}/src/QJoysticks.cpp
   	${qjoystick_DIR}/src/QJoysticks/SDL_Joysticks.cpp
    ${qjoystick_DIR}/src/QJoysticks/VirtualJoystick.cpp
    ${qjoystick_DIR}/src/QJoysticks/Android_Joystick.cpp
)
set(QT_RESOURCES ${qjoystick_DIR}/etc/resources/qjoysticks-res.qrc)

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)


#
# The StreamViewer test
#
add_executable(test-viewer test-stream-viewer.cc StreamViewer.cc logger.cc)
target_link_libraries(test-viewer
	Qt5::Core
	Qt5::Gui
	Qt5::Widgets)
target_include_directories(test-viewer PUBLIC ${Qt5Widgets_INCLUDE_DIRS})


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
	logger.cc
	PetalCoordinates.cc
	QPetalLocator.cpp
	CameraView.cpp
	StreamViewer.cc
	${qjoystick_SOURCES}
	${QT_RESOURCES}
	)
if (WIN32)
  set(magrathea_SOURCES ${magrathea_SOURCES} ACSCMotionHandler.cpp)
endif (WIN32)

add_executable(magrathea ${magrathea_SOURCES})

# CHOSE IFIC MOTION CONTROLLER
if (WIN32)
	target_compile_definitions(magrathea PUBLIC -DACSC=1 -DVALENCIA=1)
	set(ACSC_LIB D:/Code/magrathea/ACSC/C_CPP/ACSCL_x86.LIB)
endif (WIN32)

target_include_directories(magrathea PUBLIC
						   ${Qt5Widgets_INCLUDE_DIRS}
						   ${qjoystick_INCLUDES} ${SDL2_INCLUDE_DIRS})
target_link_libraries(magrathea
	Qt5::Core
	Qt5::Gui
	Qt5::Widgets
	Qt5::Multimedia
	Qt5::MultimediaWidgets
	Qt5::SerialPort
	geomlib ${OpenCV_LIBS} alglib ${xlnt_LIB}
	${ACSC_LIB})
add_dependencies (magrathea xlnt)
