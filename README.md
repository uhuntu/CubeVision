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
- Selectable ArUco DICT_5X5_100 and DICT_4X4_100 detection
- Camera calibration using an 8x11 ChArUco board with 20 mm squares and
  15 mm DICT_5X5_100 markers
- Projected 3D surface normal using 15 mm (5x5) or 10 mm (4x4) markers
- Live preview
- Color-coded CubeNet marker overlays
- Six-face cube-state scanning with marker ID validation
- Offline two-phase solving with step-by-step move guidance
- Live 3D cube-state visualization with active-move highlighting

Camera calibration:

1. Print `output/pdf/charuco-8x11-20mm-15mm-a4.pdf` at 100% scale. Do not use
   "fit to page". The source SVG is in `calibration/`.
2. Enable **Calibration mode**.
3. Capture at least 10 views at different positions, distances, and angles.
   Each view needs at least 12 visible ChArUco corners; partial views are valid.
4. Select **Calibrate**. The parameters are saved in the user's application
   configuration directory and loaded on future runs.

Roadmap:

v0.2 Face labels
v0.3 Pose
v0.4 BT cube
