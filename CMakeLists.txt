cmake_minimum_required(VERSION 3.21)
project(CubeVision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio aruco calib3d)

add_executable(CubeVision
    src/main.cpp
    src/MainWindow.cpp
    src/MainWindow.h
)

target_link_libraries(CubeVision PRIVATE Qt6::Widgets ${OpenCV_LIBS})

add_executable(GenerateCharuco EXCLUDE_FROM_ALL tools/generate_charuco.cpp)
target_link_libraries(GenerateCharuco PRIVATE ${OpenCV_LIBS})

if(WIN32)
    get_target_property(QT_CORE_LIBRARY Qt6::Core IMPORTED_LOCATION)
    get_filename_component(QT_RUNTIME_DIR "${QT_CORE_LIBRARY}" DIRECTORY)

    find_program(WINDEPLOYQT_EXECUTABLE
        NAMES windeployqt6 windeployqt
        HINTS "${QT_RUNTIME_DIR}"
        REQUIRED
    )

    set(DEPLOY_DIR "${CMAKE_BINARY_DIR}/deploy")

    add_custom_target(deploy
        COMMAND "${CMAKE_COMMAND}" -E rm -rf "${DEPLOY_DIR}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${DEPLOY_DIR}"
        COMMAND "${CMAKE_COMMAND}" -E copy "$<TARGET_FILE:CubeVision>" "${DEPLOY_DIR}/"
        COMMAND "${WINDEPLOYQT_EXECUTABLE}"
            --no-translations
            --dir "${DEPLOY_DIR}"
            "$<TARGET_FILE:CubeVision>"
        COMMAND "${CMAKE_COMMAND}"
            "-DEXECUTABLE=$<TARGET_FILE:CubeVision>"
            "-DDESTINATION=${DEPLOY_DIR}"
            "-DRUNTIME_DIR=${QT_RUNTIME_DIR}"
            -P "${CMAKE_SOURCE_DIR}/cmake/DeployRuntime.cmake"
        DEPENDS CubeVision
        COMMENT "Deploying CubeVision runtime dependencies"
        VERBATIM
    )
endif()
