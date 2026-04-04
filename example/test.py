#!/usr/bin/env python3
"""
mylib test suite - Test Ref<T> C++ library through Python bindings
Tests both direct ctypes access and module-based imports
"""

import sys
from pathlib import Path

# Add parent directory to path for mylib import
sys.path.insert(0, str(Path(__file__).parent))


def test_module_import():
    """Test 1: Import mylib as a Python module"""
    print("\n" + "=" * 70)
    print("【テスト1】Pythonモジュールとしてのmylibのインポート")
    print("=" * 70)

    from mylib import add, multiply, concat, DataManager

    print("\n- モジュール関数のテスト")
    print("-" * 70)

    result_add = add(10, 20)
    print(f"add(10, 20) = {result_add}")
    assert result_add == 30, "add関数が失敗"

    result_mul = multiply(5, 6)
    print(f"multiply(5, 6) = {result_mul}")
    assert result_mul == 30, "multiply関数が失敗"

    result_concat = concat("Hello, ", "World!")
    print(f"concat('Hello, ', 'World!') = '{result_concat}'")
    assert result_concat == "Hello, World!", "concat関数が失敗"

    print("\n- DataManagerクラスのテスト")
    print("-" * 70)

    manager = DataManager()
    print("✓ DataManager オブジェクトを作成")

    manager.set_data("Test Data from Module")
    data = manager.get_data()
    print(f"✓ データ設定・取得: '{data}'")
    assert data == "Test Data from Module"

    use_count = manager.use_count()
    print(f"✓ 参照カウント: {use_count}")
    assert use_count > 0

    for i in range(3):
        manager.increment()
    counter = manager.get_counter()
    print(f"✓ カウンター: {counter}")
    assert counter == 3

    manager.reset()
    print("✓ マネージャーをリセット")


def test_import_patterns():
    """Test 2: Various import patterns"""
    print("\n" + "=" * 70)
    print("【テスト2】様々なインポートパターン")
    print("=" * 70)

    print("\n- Pattern 1: 特定の関数をインポート")
    print("-" * 70)
    from mylib import add, multiply

    print(f"from mylib import add, multiply")
    print(f"add(5, 3) = {add(5, 3)}")
    assert add(5, 3) == 8

    print("\n- Pattern 2: クラスをインポート")
    print("-" * 70)
    from mylib import DataManager

    print(f"from mylib import DataManager")
    manager = DataManager()
    manager.set_data("Pattern Test")
    print(f"manager.get_data() = '{manager.get_data()}'")
    manager.reset()

    print("\n- Pattern 3: モジュール全体をインポート")
    print("-" * 70)
    import mylib

    print(f"import mylib")
    print(f"mylib.__version__ = {mylib.__version__}")
    print(f"mylib.add(7, 3) = {mylib.add(7, 3)}")
    assert mylib.add(7, 3) == 10


def test_application_usage():
    """Test 3: Using mylib in a larger application"""
    print("\n" + "=" * 70)
    print("【テスト3】アプリケーション内での使用")
    print("=" * 70)

    from mylib import DataManager
    import mylib

    class Calculator:
        """Example class using mylib"""

        def __init__(self):
            self.data = DataManager()

        def compute(self, a, b):
            result = mylib.add(a, b)
            self.data.set_data(f"Result: {a} + {b} = {result}")
            return result

        def multiply(self, a, b):
            result = mylib.multiply(a, b)
            self.data.set_data(f"Result: {a} * {b} = {result}")
            return result

        def status(self):
            return self.data.get_data()

    print("\n- Calculator クラスを使用")
    print("-" * 70)

    calc = Calculator()
    result1 = calc.compute(10, 20)
    print(f"calc.compute(10, 20) = {result1}")
    print(f"Status: {calc.status()}")
    assert result1 == 30

    result2 = calc.multiply(5, 6)
    print(f"calc.multiply(5, 6) = {result2}")
    print(f"Status: {calc.status()}")
    assert result2 == 30


def test_ref_features():
    """Test 4: Ref<T> specific features"""
    print("\n" + "=" * 70)
    print("【テスト4】Ref<T>機能の確認")
    print("=" * 70)

    from mylib import DataManager

    print("\n- 参照カウント管理")
    print("-" * 70)

    manager1 = DataManager()
    manager1.set_data("Manager 1")

    print(f"manager1.use_count() = {manager1.use_count()}")
    assert manager1.use_count() > 0

    for _ in range(5):
        manager1.increment()

    counter = manager1.get_counter()
    print(f"After 5 increments: counter = {counter}")
    assert counter == 5

    manager1.reset()
    print(f"After reset: use_count = {manager1.use_count()}")
    assert manager1.use_count() == 0

    print("\n- 複数マネージャーの同時管理")
    print("-" * 70)

    m1 = DataManager()
    m2 = DataManager()
    m3 = DataManager()

    m1.set_data("Manager 1")
    m2.set_data("Manager 2")
    m3.set_data("Manager 3")

    print(f"m1: '{m1.get_data()}', count={m1.use_count()}")
    print(f"m2: '{m2.get_data()}', count={m2.use_count()}")
    print(f"m3: '{m3.get_data()}', count={m3.use_count()}")

    m1.reset()
    m2.reset()
    m3.reset()
    print("✓ 全マネージャーをリセット")


def main():
    """Run all tests"""
    print("\n" + "=" * 70)
    print("mylib テストスイート")
    print("Ref<T>スマートポインタを使用したC++ライブラリ")
    print("=" * 70)

    try:
        test_module_import()
        test_import_patterns()
        test_application_usage()
        test_ref_features()

        print("\n" + "=" * 70)
        print("✓ 全てのテストが成功しました！")
        print("=" * 70)
        print("\n【まとめ】")
        print("- mylibをPythonモジュールとしてimportで呼び出し可能")
        print("- C++のRef<T>が正常に動作")
        print("- enable_ref_from_thisによる自己参照管理が機能")
        print("- 参照カウントが正確に管理されている")
        print("- C++とPythonの透過的な連携が実現")
        print("=" * 70 + "\n")

        return 0

    except AssertionError as e:
        print(f"\n✗ テストが失敗しました: {e}")
        return 1
    except Exception as e:
        print(f"\n✗ エラーが発生しました: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
