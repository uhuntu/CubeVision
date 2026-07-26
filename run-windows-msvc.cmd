@echo off
setlocal

set "APP=%~dp0build-msvc\deploy\CubeVision.exe"

if not exist "%APP%" (
    echo CubeVision MSVC build has not been deployed yet.
    echo Run: cmake --build build-msvc --target deploy --config Release
    exit /b 1
)

start "" "%APP%"
