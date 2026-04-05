cmake_minimum_required(VERSION 3.12)
project(tests)

# テスト用ソースファイル
set(TEST_SOURCES
    test_string.cpp
)

# テスト実行ファイル
add_executable(test_runner ${TEST_SOURCES})

# インクルードディレクトリ
target_include_directories(test_runner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../cutil
    ${CMAKE_CURRENT_SOURCE_DIR}/../ext
)

# ライブラリリンク
target_link_libraries(test_runner PRIVATE
    cutil
    Threads::Threads
)

# doctest の設定
include(CTest)
add_test(NAME StrTests COMMAND test_runner)
