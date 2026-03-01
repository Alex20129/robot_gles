CMAKE_MINIMUM_REQUIRED(VERSION 3.20)

PROJECT(robot_gles)

SET(CMAKE_CXX_FLAGS "-Wall -Wpedantic")

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)

FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

LINK_LIBRARIES(Qt5::Core Qt5::Gui Qt5::Widgets)

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}
	main.cpp
	geometryengine.hpp
	geometryengine.cpp
	robot.hpp
	robot.cpp
	controlswidget.hpp controlswidget.cpp controlswidget.ui
	robotviewwidget.hpp robotviewwidget.cpp robotviewwidget.ui
	${CMAKE_PROJECT_NAME}.qrc)
