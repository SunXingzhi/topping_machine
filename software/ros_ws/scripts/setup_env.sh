#!/usr/bin/env bash
# 上位机 ROS2 工作区环境：把 libuvc 私有前缀接入 pkg-config / cmake / 链接器 / 动态加载器。
# 用法：source software/ros_ws/scripts/setup_env.sh

_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_PREFIX="${_WS}/third_party/install"

export PKG_CONFIG_PATH="${_PREFIX}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
export CMAKE_PREFIX_PATH="${_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
# astra_camera 的 CMake 只向链接器传 -luvc，不传 pkg-config 的 -L 目录，
# 必须靠 LIBRARY_PATH 让 ld 找到私有前缀里的库（链接阶段）。
export LIBRARY_PATH="${_PREFIX}/lib${LIBRARY_PATH:+:${LIBRARY_PATH}}"
# 运行时让 astra_camera_node 加载这份 libuvc.so.0。
export LD_LIBRARY_PATH="${_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
