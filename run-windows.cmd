@echo off
setlocal

set "APP=%~dp0build\deploy\CubeVision.exe"

if not exist "%APP%" (
    echo CubeVision has not been deployed yet.
    echo Run: cmake --build build --target deploy
    exit /b 1
)

start "" "%APP%"
