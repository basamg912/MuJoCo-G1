# MuJoCo-G1-Locomotion

## Preview

https://github.com/user-attachments/assets/a0d3b031-77c5-44d4-a3ba-be1764bdba3a



## About

MuJoCo simulation for Unitree G1 humanoid locomotion.
Ported from [ARC-KIST/MuJoCo-franka-panda](https://github.com/ARC-KIST/MuJoCo-franka-panda) to support macOS Apple Silicon.

## Requirements

- macOS (Apple Silicon) or Linux x86-64
- CMake 3.16+
- MuJoCo 3.6.0
- ONNX Runtime 1.24.4

**macOS only:**
```bash
brew install glfw glew eigen
```

## Build

```bash
mkdir build && cd build

# macOS
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DEigen3_DIR=/opt/homebrew/Cellar/eigen/3.4.0_1/share/eigen3/cmake

# Linux
cmake .. -DCMAKE_BUILD_TYPE=Release

make -j$(nproc || sysctl -n hw.ncpu)
```

## Run

```bash
./build/run
```

## macOS Porting Notes

See [PORTING_MACOS.md](PORTING_MACOS.md) for detailed macOS porting guide.
