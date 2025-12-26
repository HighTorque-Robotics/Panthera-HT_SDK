#!/bin/bash
set -euo pipefail

# 从已有源码目录配置、编译并安装 C++ SDK。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_PROJECT_DIR="${ROOT_DIR}/panthera_cpp"
PROJECT_DIR="${1:-${DEFAULT_PROJECT_DIR}}"
BUILD_DIR="${PROJECT_DIR}/build"

if [[ "${PROJECT_DIR}" != /* ]]; then
  # 支持传入相对路径，统一转换为基于仓库根目录的绝对路径。
  PROJECT_DIR="${ROOT_DIR}/${PROJECT_DIR}"
fi

if [ ! -d "${PROJECT_DIR}" ]; then
  printf '未找到 panthera_cpp 目录：%s。\n' "${PROJECT_DIR}"
  exit 1
fi

printf '正在配置并编译...\n'
pushd "${PROJECT_DIR}" >/dev/null
mkdir -p "${BUILD_DIR}"
pushd "${BUILD_DIR}" >/dev/null

# 运行 CMake 生成构建文件并执行并行编译
cmake ..
make -j8
printf '配置并编译完成\n'

printf '正在安装...\n'
sudo make install
popd >/dev/null
popd >/dev/null
printf '安装完成。\n'
