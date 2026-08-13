#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_ROOT="${PANTHERA_BUILD_ROOT:-${ROOT_DIR}/build/python_wheel}"
STAGE_DIR="${BUILD_ROOT}/stage"
MOTOR_BUILD_DIR="${BUILD_ROOT}/motor_cpp"
PY_SRC_DIR="${BUILD_ROOT}/python_src"
WHEEL_DIR="${PANTHERA_WHEEL_DIR:-${ROOT_DIR}/dist}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}"

if [[ "${1:-}" == "--clean" ]]; then
    rm -rf "${BUILD_ROOT}" "${WHEEL_DIR}"
fi

command -v cmake >/dev/null 2>&1 || {
    echo "cmake is required but was not found on PATH." >&2
    exit 1
}

command -v pkg-config >/dev/null 2>&1 || {
    echo "pkg-config is required to build bundled LCM but was not found on PATH." >&2
    echo "On Ubuntu, install system build dependencies with:" >&2
    echo "  sudo apt-get install -y pkg-config libglib2.0-dev libserialport-dev libyaml-cpp-dev" >&2
    exit 1
}

pkg-config --exists glib-2.0 || {
    echo "glib-2.0 development files are required to build bundled LCM." >&2
    echo "On Ubuntu, install system build dependencies with:" >&2
    echo "  sudo apt-get install -y pkg-config libglib2.0-dev libserialport-dev libyaml-cpp-dev" >&2
    exit 1
}

command -v "${PYTHON_BIN}" >/dev/null 2>&1 || {
    echo "${PYTHON_BIN} is required but was not found on PATH." >&2
    exit 1
}

"${PYTHON_BIN}" -m pip --version >/dev/null 2>&1 || {
    echo "${PYTHON_BIN} -m pip is required." >&2
    exit 1
}

"${PYTHON_BIN}" -c "import pybind11" >/dev/null 2>&1 || {
    echo "pybind11 is required in the selected Python environment." >&2
    echo "Install it with: ${PYTHON_BIN} -m pip install pybind11" >&2
    exit 1
}

"${PYTHON_BIN}" -c "import wheel" >/dev/null 2>&1 || {
    echo "wheel is required in the selected Python environment." >&2
    echo "Install it with: ${PYTHON_BIN} -m pip install wheel" >&2
    exit 1
}

mkdir -p "${BUILD_ROOT}" "${STAGE_DIR}" "${WHEEL_DIR}"

echo "==> Building and installing motor C++ SDK into ${STAGE_DIR}"
cmake \
    -S "${ROOT_DIR}/panthera_cpp/motor_cpp" \
    -B "${MOTOR_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${STAGE_DIR}"

cmake --build "${MOTOR_BUILD_DIR}" --target install --parallel "${JOBS}"

echo "==> Preparing staged Python source tree at ${PY_SRC_DIR}"
rm -rf "${PY_SRC_DIR}"
mkdir -p "${PY_SRC_DIR}"

cp "${ROOT_DIR}/panthera_python/CMakeLists.txt" "${PY_SRC_DIR}/"
cp "${ROOT_DIR}/panthera_python/setup.py" "${PY_SRC_DIR}/"
cp "${ROOT_DIR}/panthera_python/pyproject.toml" "${PY_SRC_DIR}/"
cp "${ROOT_DIR}/panthera_python/README.md" "${PY_SRC_DIR}/"
cp "${ROOT_DIR}/panthera_python/requirements.txt" "${PY_SRC_DIR}/"
cp -R "${ROOT_DIR}/panthera_python/src" "${PY_SRC_DIR}/src"
cp -R "${ROOT_DIR}/panthera_python/hightorque_robot" "${PY_SRC_DIR}/hightorque_robot"
cp -R "${ROOT_DIR}/panthera_python/scripts" "${PY_SRC_DIR}/scripts"
cp -R "${ROOT_DIR}/panthera_python/robot_param" "${PY_SRC_DIR}/robot_param"
cp -R "${ROOT_DIR}/panthera_python/Panthera-HT_description" "${PY_SRC_DIR}/Panthera-HT_description"
cp -R "${ROOT_DIR}/panthera_python/images" "${PY_SRC_DIR}/images"

echo "==> Copying staged SDK shared libraries into the Python package"
find "${STAGE_DIR}/lib" -maxdepth 1 \( -type f -o -type l \) \( -name "*.so" -o -name "*.so.*" \) \
    -exec cp -L {} "${PY_SRC_DIR}/hightorque_robot/" \;

echo "==> Building Python wheel into ${WHEEL_DIR}"
(
    cd "${PY_SRC_DIR}"
    CMAKE_PREFIX_PATH="${STAGE_DIR}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}" \
        "${PYTHON_BIN}" -m pip wheel . --no-deps --no-build-isolation -w "${WHEEL_DIR}"
)

echo "==> Wheel build complete"
ls -1 "${WHEEL_DIR}"/hightorque_robot-*.whl
