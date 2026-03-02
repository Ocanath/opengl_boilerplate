# OpenGL Boilerplate

OpenGL 4.6 renderer with Bullet3 physics, deferred shading, ImGui, and VLP-16 LiDAR visualisation.

## Linux dependencies

Most libraries (GLFW, GLM, Assimp, ImGui, Bullet3) are vendored submodules. You only need
system packages for OpenGL, the windowing system headers, and standard build tools.

### Ubuntu / Debian

```bash
# Build tools
sudo apt install build-essential cmake git

# OpenGL — pick whichever exists on your distro version:
#   Ubuntu 20.04+  →  libgl-dev
#   Ubuntu 18.04 / older Debian  →  libgl1-mesa-dev
# Both pull in mesa-common-dev (headers) and the libGL.so symlink CMake needs.
sudo apt install libgl-dev libgl1-mesa-dev mesa-common-dev 2>/dev/null || \
sudo apt install libgl1-mesa-dev mesa-common-dev

# X11 windowing headers (required by GLFW's X11 backend)
sudo apt install \
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev

# Wayland headers (required by GLFW's Wayland backend — on by default)
sudo apt install \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols

# zlib (used by Assimp's internal zip reader)
sudo apt install zlib1g-dev
```

### Fedora / RHEL

```bash
sudo dnf install cmake gcc-c++ git \
    mesa-libGL-devel \
    libX11-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel \
    wayland-devel libxkbcommon-devel wayland-protocols-devel \
    zlib-devel
```

### Arch Linux

```bash
sudo pacman -S cmake base-devel git \
    mesa \
    libx11 libxrandr libxinerama libxcursor libxi \
    wayland libxkbcommon wayland-protocols \
    zlib
```

## Build

```bash
git submodule update --init --recursive
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./opengl_boilerplate
```

## Controls

| Input | Action |
|---|---|
| W / A / S / D | Move horizontally |
| Space / LCtrl | Move up / down |
| Mouse | Look |
| Escape | Toggle mouse capture |
| 1 / 2 / 3 / 4 | Select ability |
| LMB | Fire primary |
| RMB | Fire secondary |
| Scroll | Ability parameter |

## VLP-16 LiDAR

Open the **LiDAR** window (top-right), enter the UDP port your sensor streams to
(default `2381`), and click **Connect**. Red cubes appear at each return point.
Adjust *Buffer Depth* to persist more revolutions and *Max Pts/Frame* to control box count.
