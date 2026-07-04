# CubeVision Qt v0.1

Requirements:

- Linux or Windows
- C++17 compiler
- CMake 3.21+
- Qt 6
- OpenCV 4.x

Build:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

On Windows with MSYS2 installed in the user profile, add the UCRT64
toolchain to `PATH` first:

```powershell
$env:PATH = "$env:USERPROFILE\msys64\ucrt64\bin;$env:PATH"
```

Create a self-contained Windows directory before launching the application
outside that configured terminal:

```powershell
cmake --build build --target deploy
.\run-windows.cmd
```

Current features:

- Qt window
- OpenCV camera
- ArUco DICT_5X5_100 detection
- Live preview

Roadmap:

v0.2 Face labels
v0.3 Pose
v0.4 BT cube
