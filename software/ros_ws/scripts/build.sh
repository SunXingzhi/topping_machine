#!/usr/bin/env bash
# 一键构建：先确保 libuvc 私有前缀存在，再 colcon build ROS 包。
# 用法：software/ros_ws/scripts/build.sh

set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="${WS}/third_party/install"

# 保持与 Conda 等外部工具链隔离（沿用验证环境的做法）。
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONHOME
unset PYTHONPATH

source /opt/ros/humble/setup.bash

# --- 1. libuvc（submodule）→ 私有前缀，已装过则跳过 ---
if [ ! -f "${PREFIX}/lib/pkgconfig/libuvc.pc" ]; then
  if [ ! -f "${WS}/third_party/libuvc/CMakeLists.txt" ]; then
    echo "libuvc submodule 未初始化，先执行：git submodule update --init --recursive" >&2
    exit 1
  fi
  cmake -S "${WS}/third_party/libuvc" -B "${WS}/third_party/libuvc/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${PREFIX}"
  cmake --build "${WS}/third_party/libuvc/build" -j"$(nproc)"
  cmake --install "${WS}/third_party/libuvc/build"
else
  echo "libuvc 已就绪：${PREFIX}"
fi

# --- 2. ROS2 colcon 构建 ---
source "${WS}/scripts/setup_env.sh"
cd "${WS}"
colcon build \
  --base-paths src \
  --event-handlers console_direct+ \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++
