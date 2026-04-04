#!/usr/bin/env python3
"""
mylib demo - Various import patterns and usage examples
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

print("=" * 70)
print("mylib Demo - Pythonモジュールとしての様々な使い方")
print("=" * 70)

# Pattern 1: Import specific functions
print("\n【パターン1】特定の関数をインポート")
print("-" * 70)
from mylib import add, multiply, concat

print(f"from mylib import add, multiply, concat")
print(f"add(5, 3) = {add(5, 3)}")
print(f"multiply(4, 7) = {multiply(4, 7)}")
print(f"concat('Python', ' Rules') = '{concat('Python', ' Rules')}'")

# Pattern 2: Import class
print("\n【パターン2】クラスをインポート")
print("-" * 70)
from mylib import DataManager

print(f"from mylib import DataManager")
manager = DataManager()
manager.set_data("Hello from Python")
print(f"manager.get_data() = '{manager.get_data()}'")
print(f"manager.use_count() = {manager.use_count()}")

# Pattern 3: Import entire module
print("\n【パターン3】モジュール全体をインポート")
print("-" * 70)
import mylib

print(f"import mylib")
print(f"mylib.__version__ = {mylib.__version__}")
print(f"mylib.__all__ = {mylib.__all__}")
print(f"Available functions: {', '.join([name for name in dir(mylib) if not name.startswith('_')])}")

# Pattern 4: Using as module in a larger application
print("\n【パターン4】アプリケーション内での使用")
print("-" * 70)

class Calculator:
    """Pythonクラスがmylibを使用する例"""

    def __init__(self):
        self.data = mylib.DataManager()

    def compute(self, a, b):
        """Compute and store result"""
        result = mylib.add(a, b)
        self.data.set_data(f"Result: {a} + {b} = {result}")
        return result

    def multiply(self, a, b):
        """Multiply using mylib"""
        result = mylib.multiply(a, b)
        self.data.set_data(f"Result: {a} * {b} = {result}")
        return result

    def status(self):
        """Get current status"""
        return self.data.get_data()

calc = Calculator()
print(f"calc.compute(10, 20) = {calc.compute(10, 20)}")
print(f"Status: {calc.status()}")

calc.multiply(5, 6)
print(f"After multiply: {calc.status()}")

# Pattern 5: Direct C++ interaction
print("\n【パターン5】Ref<T>機能の確認")
print("-" * 70)

manager1 = DataManager()
manager1.set_data("Manager 1")

print(f"manager1.use_count() = {manager1.use_count()}")
for _ in range(3):
    manager1.increment()
print(f"After 3 increments: counter = {manager1.get_counter()}")

manager1.reset()
print(f"After reset: use_count = {manager1.use_count()}")

print("\n" + "=" * 70)
print("✓ デモ完了 - mylibの全機能が正常に動作")
print("=" * 70)
