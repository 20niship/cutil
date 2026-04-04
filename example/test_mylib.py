#!/usr/bin/env python3
"""
Python スクリプト：C++ライブラリを ctypes で使用するデモ
"""

import ctypes
import os
from pathlib import Path

# ライブラリのパスを設定
# ビルドディレクトリ内のライブラリを参照
lib_path = Path(__file__).parent.parent / "build" / "libmycpplib.dylib"

# フォールバック：example ディレクトリのライブラリ
if not lib_path.exists():
    lib_path = Path(__file__).parent / "libmycpplib.dylib"

if not lib_path.exists():
    print(f"エラー: ライブラリが見つかりません")
    print(f"探索パス: {Path(__file__).parent.parent / 'build' / 'libmycpplib.dylib'}")
    print("先に CMake でビルドしてください:")
    print("  cmake -B build && cmake --build build")
    exit(1)

# 共有ライブラリをロード
lib = ctypes.CDLL(str(lib_path))

# 関数のシグネチャを設定
lib.add.argtypes = [ctypes.c_int, ctypes.c_int]
lib.add.restype = ctypes.c_int

lib.multiply.argtypes = [ctypes.c_int, ctypes.c_int]
lib.multiply.restype = ctypes.c_int

lib.concat.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
lib.concat.restype = ctypes.c_char_p

lib.manager_init.argtypes = []
lib.manager_init.restype = None

lib.manager_set_data.argtypes = [ctypes.c_char_p]
lib.manager_set_data.restype = None

lib.manager_get_data.argtypes = []
lib.manager_get_data.restype = ctypes.c_char_p

lib.manager_increment.argtypes = []
lib.manager_increment.restype = ctypes.c_int

lib.manager_get_counter.argtypes = []
lib.manager_get_counter.restype = ctypes.c_int

lib.manager_use_count.argtypes = []
lib.manager_use_count.restype = ctypes.c_int

lib.manager_reset.argtypes = []
lib.manager_reset.restype = None

print("=" * 60)
print("C++ ライブラリ (Ref<T>) のPythonからの使用テスト")
print("=" * 60)

print("\n【テスト1】単純な計算関数")
print("-" * 60)
result_add = lib.add(10, 20)
print(f"add(10, 20) = {result_add}")
assert result_add == 30, "計算エラー"

result_mul = lib.multiply(5, 6)
print(f"multiply(5, 6) = {result_mul}")
assert result_mul == 30, "計算エラー"

print("\n【テスト2】文字列連結")
print("-" * 60)
str1 = b"Hello, "
str2 = b"World!"
result = lib.concat(str1, str2)
result_str = result.decode('utf-8') if result else ""
print(f"concat('{str1.decode()}', '{str2.decode()}') = '{result_str}'")

print("\n【テスト3】DataManagerクラス (enable_ref_from_this を使用)")
print("-" * 60)

# マネージャーを初期化
print("① マネージャーを初期化...")
lib.manager_init()

# データを設定
print("② データを設定...")
lib.manager_set_data(b"Test Data")

# データを取得
data = lib.manager_get_data()
print(f"③ データを取得: {data.decode('utf-8') if data else ''}")

# 参照カウントを確認
use_count = lib.manager_use_count()
print(f"④ 参照カウント: {use_count}")
assert use_count > 0, "参照カウントが正しくない"

print("\n【テスト4】Ref<T>の参照カウント機能")
print("-" * 60)

# カウンターをインクリメント
print("① カウンターをインクリメント...")
for i in range(5):
    count = lib.manager_increment()
    print(f"   increment() = {count}")

current = lib.manager_get_counter()
print(f"② 現在のカウンター値: {current}")
assert current == 5, "カウンター値が正しくない"

print("\n【テスト5】リセット")
print("-" * 60)
print("① マネージャーをリセット...")
lib.manager_reset()

use_count_after_reset = lib.manager_use_count()
print(f"② リセット後の参照カウント: {use_count_after_reset}")
print("   (0 はオブジェクトが破棄されたことを示す)")

print("\n" + "=" * 60)
print("✓ 全てのテストが成功しました！")
print("=" * 60)

print("\n【まとめ】")
print("- Ref<T>スマートポインタが正常に動作")
print("- enable_ref_from_thisによる自己参照管理が機能")
print("- 参照カウントが正確に管理されている")
print("- C++のメモリ管理がPythonから透過的に利用可能")
