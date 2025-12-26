cmake_minimum_required(VERSION 3.5)
project(serial_cmake)

# 包含版本信息
include(cmake/version.cmake)

# 包含安装逻辑
include(GNUInstallDirs)

# 加载包配置
include(CMakePackageConfigHelpers)

# 源码文件
set(SERIAL_SRC
    src/serial.cc
    src/impl/unix.cc
    src/impl/list_ports/list_ports_linux.cc
)

# 建立库
add_library(${PROJECT_NAME} SHARED ${SERIAL_SRC})

# 头文件目录
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 链接底层库
target_link_libraries(${PROJECT_NAME}
    PUBLIC rt pthread
)

# 安装：头文件
install(DIRECTORY include/serial
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# 安装：库文件
install(TARGETS ${PROJECT_NAME}
    EXPORT serial_cmakeTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# 导出Targets
install(EXPORT serial_cmakeTargets
    FILE serial_cmakeTargets.cmake
    NAMESPACE serial::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/serial_cmake
)

# 生成Config文件
configure_package_config_file(
    cmake/serial_cmakeConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/serial_cmakeConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/serial_cmake
)

# 安装Config
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/serial_cmakeConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/serial_cmake
)
