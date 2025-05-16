cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(PointCloud LANGUAGES CXX)

include(qt.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt6の必要なモジュール
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)

# OpenGL
find_package(OpenGL REQUIRED)

# PCL
find_package(PCL 1.13 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# SDL
find_package(SDL3 CONFIG REQUIRED)

qt_standard_project_setup()

set(PROJECT_SOURCES
    main.cpp
    PointCloud.cpp
    PointCloud.h
    setupController.h
)

qt_add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

set_target_properties(${PROJECT_NAME}
    PROPERTIES
        WIN32_EXECUTABLE TRUE
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        OpenGL::GL
        ${PCL_LIBRARIES}
    PRIVATE 
        SDL3::SDL3
)
