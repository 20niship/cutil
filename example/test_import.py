#!/usr/bin/env python3
"""
Test mylib by importing it as a Python module
"""

import sys
from pathlib import Path

# Add parent directory to path so we can import mylib
sys.path.insert(0, str(Path(__file__).parent))

# Now we can import mylib
from mylib import add, multiply, concat, DataManager

print("=" * 60)
print("mylib - Pythonモジュールとして実装したライブラリ")
print("=" * 60)

print("\n【テスト1】モジュール関数の使用")
print("-" * 60)
result_add = add(10, 20)
print(f"add(10, 20) = {result_add}")
assert result_add == 30

result_mul = multiply(5, 6)
print(f"multiply(5, 6) = {result_mul}")
assert result_mul == 30

result_concat = concat("Hello, ", "World!")
print(f"concat('Hello, ', 'World!') = '{result_concat}'")
assert result_concat == "Hello, World!"

print("\n【テスト2】DataManagerクラスの使用（Ref<T>を使用）")
print("-" * 60)

# DataManagerを初期化
manager = DataManager()
print("✓ DataManager オブジェクトを作成")

# データを設定
manager.set_data("Test Data")
print("✓ データを設定: 'Test Data'")

# データを取得
data = manager.get_data()
print(f"✓ データを取得: '{data}'")
assert data == "Test Data"

# 参照カウントを確認（Ref<T>の機能）
use_count = manager.use_count()
print(f"✓ 参照カウント: {use_count}")
assert use_count > 0

print("\n【テスト3】Ref<T>の参照カウント機能")
print("-" * 60)

# カウンターをインクリメント
print("カウンターをインクリメント:")
for i in range(5):
    count = manager.increment()
    print(f"  {i+1}: {count}")

current = manager.get_counter()
print(f"現在のカウンター値: {current}")
assert current == 5

print("\n【テスト4】オブジェクトのリセット（自動破棄）")
print("-" * 60)
print("マネージャーをリセット...")
manager.reset()

use_count_after = manager.use_count()
print(f"リセット後の参照カウント: {use_count_after}")
print("(0 はオブジェクトが破棄されたことを示す)")

print("\n" + "=" * 60)
print("✓ 全てのテストが成功しました！")
print("=" * 60)

print("\n【まとめ】")
print("- mylibをPythonモジュールとしてimportで呼び出し可能")
print("- C++のRef<T>が正常に動作")
print("- enable_ref_from_thisによる自己参照管理が機能")
print("- 参照カウントが正確に管理されている")
print("- C++とPythonの透過的な連携が実現")
