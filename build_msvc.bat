@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
C:\Qt\Tools\CMake_64\bin\cmake.exe --build build-msvc --target CubeVision --config Release --parallel 8
