#!/bin/bash
set -euo pipefail

# 安装 Python 接口及其依赖。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="${ROOT_DIR}/panthera_cpp"
PYTHON_ROOT="${ROOT_DIR}/panthera_python"

printf '正在安装系统依赖...\n'
sudo apt-get update
sudo apt-get install -y \
    cmake \
    python3-dev \
    python3-pip \
    liblcm-dev \
    libyaml-cpp-dev \
    libserialport-dev

printf '正在安装 Python 依赖...\n'
if [ -z "${VIRTUAL_ENV:-}" ] && [ -z "${CONDA_PREFIX:-}" ]; then
    printf '检测到处于全局环境，退出脚本。\n'
    exit 1
else
    CONDA_NAME="${CONDA_DEFAULT_ENV:-}"
    if [ -n "${CONDA_PREFIX:-}" ] && { [ "${CONDA_NAME}" = "base" ] || [ "$(basename "${CONDA_PREFIX}")" = "base" ]; }; then
        printf '检测到处于 Conda base 环境，退出脚本。\n'
        exit 1
    fi
    python3 -m pip install pybind11 numpy pyyaml pin
fi
printf 'Python 依赖安装完成。\n'

printf '请选择安装方式:\n'
printf '  1) 使用 pip 开发模式安装\n'
printf '  2) 使用 CMake 构建\n'
read -r -p '请输入选项 (1/2): ' INSTALL_CHOICE

case "${INSTALL_CHOICE}" in
    1)
        printf '正在以开发模式安装 Python 包...\n'
        pushd "${PYTHON_ROOT}" >/dev/null
        python3 -m pip install -e .
        popd >/dev/null
        ;;
    2)
        printf '正在使用 CMake 构建 Python 包...\n'
        pushd "${PYTHON_ROOT}" >/dev/null
        mkdir -p build
        pushd build >/dev/null
        cmake ..
        make
        popd >/dev/null
        popd >/dev/null
        ;;
    *)
        printf '未识别的选项，退出。\n'
        exit 1
        ;;
esac

printf 'Python 接口处理完成。\n'
