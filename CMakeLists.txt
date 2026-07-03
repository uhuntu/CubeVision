cmake_minimum_required(VERSION 3.21)
project(CubeVision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(OpenCV REQUIRED)

add_executable(CubeVision
    src/main.cpp
    src/MainWindow.cpp
    src/MainWindow.h
)

target_link_libraries(CubeVision PRIVATE Qt6::Widgets ${OpenCV_LIBS})
