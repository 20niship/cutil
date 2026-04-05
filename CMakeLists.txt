cmake_minimum_required(VERSION 3.14)
project(CutilProject LANGUAGES CXX)

# C++ 標準設定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ビルドオプション
option(BUILD_TESTS "ビルドテスト" ON)
option(NANOBIND "nanobind を使用" OFF)

# 共通設定
set(CUTIL_SOURCES "")
set(CUTIL_HEADERS "")

# cutil ヘッダー
file(GLOB CUTIL_HEADERS_RECURSIVE "${CMAKE_CURRENT_SOURCE_DIR}/cutil/*.hpp")
list(APPEND CUTIL_HEADERS ${CUTIL_HEADERS_RECURSIVE})

# 外部ライブラリ
find_package(Threads REQUIRED)

# テスト用
if(BUILD_TESTS)
    enable_testing()
    add_executable(tests
        tests/test_ref.cpp
        tests/test_string.cpp
    )
    target_include_directories(tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/cutil
        ${CMAKE_CURRENT_SOURCE_DIR}/ext
    )
    target_link_libraries(tests PRIVATE Threads::Threads)
    add_test(NAME Tests COMMAND tests)
endif()

# メインライブラリ（ヘッダーベース）
add_library(cutil INTERFACE)
target_include_directories(cutil INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/cutil
    ${CMAKE_CURRENT_SOURCE_DIR}/ext
)

# nanobind 設定
if(NANOBIND)
    find_package(nanobind REQUIRED)
    target_link_libraries(cutil INTERFACE nanobind::nanobind)
endif()

# インストール設定
include(GNUInstallDirs)
install(TARGETS cutil
    EXPORT cutilTargets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# 構成ファイル出力
export(EXPORT cutilTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/cutilTargets.cmake"
    NAMESPACE cutil::
)

# 開発者向け
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 警告レベル
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
