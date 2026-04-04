#!/usr/bin/env python3
"""
mylib - C++ ライブラリへのPythonバインディング
ctypes を使用してC++共有ライブラリをPythonから呼び出し
"""

import ctypes
import os
import sys
from pathlib import Path

# ライブラリのパスを設定
_lib_path = Path(__file__).parent.parent / "build" / "libmycpplib.dylib"

if not _lib_path.exists():
    raise ImportError(f"libmycpplib.dylib not found at {_lib_path}. "
                      f"Please run 'cd build && ninja'")

# 共有ライブラリをロード
_lib = ctypes.CDLL(str(_lib_path))

# ====== Function Definitions ======

def add(a: int, b: int) -> int:
    """Add two numbers"""
    _lib.add.argtypes = [ctypes.c_int, ctypes.c_int]
    _lib.add.restype = ctypes.c_int
    return _lib.add(a, b)


def multiply(a: int, b: int) -> int:
    """Multiply two numbers"""
    _lib.multiply.argtypes = [ctypes.c_int, ctypes.c_int]
    _lib.multiply.restype = ctypes.c_int
    return _lib.multiply(a, b)


def concat(a: str, b: str) -> str:
    """Concatenate two strings"""
    _lib.concat.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    _lib.concat.restype = ctypes.c_char_p
    result = _lib.concat(a.encode(), b.encode())
    return result.decode() if result else ""


# ====== DataManager Class ======

class DataManager:
    """DataManager - C++ class with Ref<T> and enable_ref_from_this"""

    def __init__(self):
        """Initialize DataManager"""
        _lib.manager_init.argtypes = []
        _lib.manager_init.restype = None
        _lib.manager_init()

    def set_data(self, data: str) -> None:
        """Set data"""
        _lib.manager_set_data.argtypes = [ctypes.c_char_p]
        _lib.manager_set_data.restype = None
        _lib.manager_set_data(data.encode())

    def get_data(self) -> str:
        """Get data"""
        _lib.manager_get_data.argtypes = []
        _lib.manager_get_data.restype = ctypes.c_char_p
        result = _lib.manager_get_data()
        return result.decode() if result else ""

    def increment(self) -> int:
        """Increment counter"""
        _lib.manager_increment.argtypes = []
        _lib.manager_increment.restype = ctypes.c_int
        return _lib.manager_increment()

    def get_counter(self) -> int:
        """Get counter value"""
        _lib.manager_get_counter.argtypes = []
        _lib.manager_get_counter.restype = ctypes.c_int
        return _lib.manager_get_counter()

    def use_count(self) -> int:
        """Get reference count (Ref<T> feature)"""
        _lib.manager_use_count.argtypes = []
        _lib.manager_use_count.restype = ctypes.c_int
        return _lib.manager_use_count()

    def reset(self) -> None:
        """Reset manager (destroys object)"""
        _lib.manager_reset.argtypes = []
        _lib.manager_reset.restype = None
        _lib.manager_reset()


# ====== Module Info ======

__version__ = "0.1.0"
__all__ = ["add", "multiply", "concat", "DataManager"]
