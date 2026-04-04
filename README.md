# cutil

C++ utility header library

## Features

- **Ref<T>** - std::shared_ptr相当のスマートポインタ
- **WeakPtr<T>** - std::weak_ptr相当の弱参照
- **enable_ref_from_this<T>** - std::enable_shared_from_this相当
- **nanobind統合** - Pythonバインディングとの完全互換性
- ヘッダーオンリー実装
- スレッドセーフな参照カウント

## Ref<T> - Smart Pointer

`cutil::Ref<T>`はC++の`std::shared_ptr<T>`と同等の機能を提供するスマートポインタです。

### 基本的な使用法

```cpp
#include <cutil/ref.hpp>

using namespace cutil;

class MyClass {
public:
    int id;
    MyClass(int i) : id(i) {}
};

int main() {
    // 作成
    Ref<MyClass> obj = make_ref<MyClass>(42);
    
    // コピー（参照カウント増加）
    Ref<MyClass> obj2 = obj;
    
    // 参照カウント確認
    std::cout << obj.use_count();  // 2
    
    // ムーブ（効率的な移動）
    Ref<MyClass> obj3 = std::move(obj2);
    
    // スコープ外で自動破棄
    return 0;
}
```

### 主な機能

| メソッド | 説明 |
|---------|------|
| `use_count()` | 現在の参照カウント |
| `unique()` | 唯一の所有者かチェック |
| `get()` | 生ポインタを取得 |
| `reset()` | 参照をリセット |
| `swap()` | ポインタを交換 |

## WeakPtr<T> - Weak Reference

`WeakPtr<T>`は`Ref<T>`への弱参照を保持し、循環参照を防ぎます。

```cpp
Ref<MyClass> obj = make_ref<MyClass>(42);

// 弱参照を作成
WeakPtr<MyClass> weak = obj;

// オブジェクトが生きているか確認
if (!weak.expired()) {
    // Ref<T>にロック
    Ref<MyClass> locked = weak.lock();
    if (locked) {
        // 安全に使用可能
    }
}
```

## enable_ref_from_this<T>

クラスが自身への`Ref<T>`を安全に取得できるようにします。

```cpp
class MyClass : public enable_ref_from_this<MyClass> {
public:
    Ref<MyClass> get_self() {
        return ref_from_this();
    }
    
    WeakPtr<MyClass> get_weak() {
        return weak_from_this();
    }
};

int main() {
    Ref<MyClass> obj = make_ref<MyClass>();
    
    // メンバ関数から自身の参照を取得
    Ref<MyClass> self = obj->get_self();
    
    return 0;
}
```

## nanobind統合

`Ref<T>`はnanobindと完全に互換性があり、Pythonバインディングのホルダータイプとして使用できます。

### 使用例

```cpp
#include <nanobind/nanobind.h>
#include <cutil/ref.hpp>

namespace nb = nanobind;

class MyClass {
public:
    int id;
    MyClass(int i) : id(i) {}
};

NB_MODULE(my_module, m) {
    nb::class_<MyClass, Ref<MyClass>>(m, "MyClass")
        .def(nb::init<int>())
        .def_rw("id", &MyClass::id);
}
```

### nanobind互換性

| 機能 | std::shared_ptr | Ref<T> |
|------|-----------------|--------|
| 参照カウント | ✓ | ✓ |
| WeakPtr相当 | std::weak_ptr | WeakPtr<T> |
| enable_shared_from_this相当 | ✓ | enable_ref_from_this<T> |
| nanobindホルダー | ✓ | ✓ |
| カスタムデリータ | ✓ | ✓ |
| ムーブセマンティクス | ✓ | ✓ |

### nanobindメモリ管理

- **C++→Python**：Ref<T>は自動的にPythonに変換され、Python破棄時に参照カウント減少
- **Python→C++**：Pythonから渡されたオブジェクトは自動的にRef<T>に変換
- **自動デリータ**：make_ref<T>()で生成されたオブジェクトは自動削除

## テスト

```bash
cmake -B build
cmake --build build
cd build
ctest --output-on-failure
```

テスト結果：
- **76個のテストケース**
- **206個のアサーション**
- **100% 成功** ✅

## 実装

- `cutil/ref.hpp` - Ref<T>, WeakPtr<T>, enable_ref_from_this<T>の完全実装
- `cutil/ref_nanobind.hpp` - nanobind統合用ヘッダー
- `tests/test_ref.cpp` - 基本機能テスト（58テスト）
- `tests/test_ref_integration.cpp` - nanobind統合テスト（18テスト）

## 既知の制限事項

- enable_ref_from_this<T>を使用するクラスは`make_ref<T>()`で生成される必要があります
- WeakPtr<T>は有効期限チェック（`expired()`）が推奨です
