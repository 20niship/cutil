<h1> cutil -- C++ utility header library</h1>

- [Features](#features)
- [ビルド](#ビルド)
- [Ref\<T\> / WeakPtr\<T\> / enable\_ref\_from\_this\<T\>](#reft--weakptrt--enable_ref_from_thist)
- [Prop / PropInfo](#prop--propinfo)
- [Vec / Mat / Quat](#vec--mat--quat)
- [Str / Path](#str--path)
- [Range / Rect / Rect3D](#range--rect--rect3d)
- [dictionary / hash\_map / variant](#dictionary--hash_map--variant)
- [ObjectPool\<T\>](#objectpoolt)
- [Octree](#octree)
- [Color / logger](#color--logger)


## Features

- ヘッダーオンリー実装
- **Ref<T> / WeakPtr<T> / enable_ref_from_this<T>** - std::shared_ptr/weak_ptr相当のスマートポインタ、nanobind完全互換
- **Prop / PropInfo** - offsetofルールベースのリフレクション。任意構造体のdump/load_to、バイナリ/JSONシリアライズ
- **Vec / Mat / Quat** - GLM/Godot風APIのN次元ベクトル・行列・クォータニオン（SSE2 SIMD対応）
- **Str / Path** - SSO付き文字列、ファイルパス型
- **Range / Rect / Rect3D** - 2D/3D幾何プリミティブ
- **dictionary / hash_map / variant** - 汎用コンテナ
- **ObjectPool** - チャンク連結リストによるオブジェクトプール
- **Octree** - 空間分割木
- **Color / logger** - 補助ユーティリティ

## ビルド

CMakeでヘッダーオンリーライブラリとして構成。`include`ディレクトリに`cutil/`を追加するだけでも使用可能。

```bash
cmake -B build
cmake --build build
```

テスト実行（doctest使用、`tests/`配下）:

```bash
cd build
ctest --output-on-failure
# もしくは直接実行
./tests/tests_main
```

## Ref&lt;T&gt; / WeakPtr&lt;T&gt; / enable_ref_from_this&lt;T&gt;

`std::shared_ptr`/`std::weak_ptr`/`enable_shared_from_this`相当のスマートポインタ。nanobindのホルダータイプとしても使用可能。

<details>
<summary>使用例を見る</summary>

`cutil::Ref<T>`は`std::shared_ptr<T>`と同等の機能を提供するスマートポインタ。`WeakPtr<T>`で循環参照を防止できる。

```cpp
#include <cutil/ref.hpp>

using namespace cutil;

class MyClass : public enable_ref_from_this<MyClass> {
public:
    int id;
    MyClass(int i) : id(i) {}

    Ref<MyClass> get_self() { return ref_from_this(); }
};

int main() {
    Ref<MyClass> obj = make_ref<MyClass>(42);
    Ref<MyClass> obj2 = obj;              // 参照カウント増加
    std::cout << obj.use_count();         // 2

    WeakPtr<MyClass> weak = obj;          // 弱参照（循環参照を防止）
    if (!weak.expired()) {
        Ref<MyClass> locked = weak.lock();
    }

    Ref<MyClass> obj3 = std::move(obj2);  // ムーブ
    return 0;
}
```

| メソッド | 説明 |
|---------|------|
| `use_count()` | 現在の参照カウント |
| `unique()` | 唯一の所有者かチェック |
| `get()` | 生ポインタを取得 |
| `reset()` | 参照をリセット |
| `swap()` | ポインタを交換 |
| `weak.expired()` / `weak.lock()` | 有効期限チェック / Ref\<T\>へロック |

nanobind統合例:

```cpp
#include <nanobind/nanobind.h>
#include <cutil/ref.hpp>

namespace nb = nanobind;

NB_MODULE(my_module, m) {
    nb::class_<MyClass, cutil::Ref<MyClass>>(m, "MyClass")
        .def(nb::init<int>())
        .def_rw("id", &MyClass::id);
}
```

- **既知の制限**: `enable_ref_from_this<T>`を使うクラスは`make_ref<T>()`で生成する必要がある

</details>

## Prop / PropInfo

offsetofルールベースのリフレクションシステム。任意のC++構造体をキー付きの動的な値コンテナ`Prop`へdump/load_toでき、バイナリ/JSON形式へシリアライズできる。

<details>
<summary>使用例を見る</summary>

`Prop`はキー付きの動的な値コンテナ、`PropInfo`はoffsetofベースの構造体スキーマ。

```cpp
#include <cutil/prop.hpp>
#include <cutil/prop_io.hpp>

using namespace cutil;

// 動的な値コンテナとして使う
Prop p;
p.set<float>("speed", 3.5f);
p.set<Vec3f>("pos", Vec3f(1, 2, 3));
float speed = p.get<float>("speed");

// バイナリ/JSONへシリアライズ
std::vector<uint8_t> bytes;
prop_dump_binary(p, bytes);
Prop restored;
prop_load_binary(restored, bytes);

std::string json;
prop_dump_json(p, json);
```

外部構造体をoffsetofルールでdump/load_toする例:

```cpp
struct Model3D { Vec3f pos; Str name; bool visible = false; };

const PropInfo& Model3DInfo() {
  static const PropInfo rule = {
    {"pos",     offsetof(Model3D, pos),     prop_info_of<Vec3f>()},
    {"name",    offsetof(Model3D, name),    prop_info_of<Str>()},
    {"visible", offsetof(Model3D, visible), prop_info_of<bool>()},
  };
  return rule;
}

Model3D a;
Prop p;
p.dump(&a, &Model3DInfo());   // 構造体 → Prop
Model3D b;
p.load_to(&b, &Model3DInfo()); // Prop → 構造体
```

- 壊れたバイナリ/JSONを読み込んでもクラッシュせず安全に失敗するよう設計（`tests/test_prop_fuzz.cpp`でfuzzテスト済み）

</details>

## Vec / Mat / Quat

GLM/Godot風APIのN次元ベクトル・行列・クォータニオン。SSE2 SIMDに対応。

<details>
<summary>使用例を見る</summary>

`NVec<N, T>`はN次元ベクトル（`Vec3f`, `Vec4f`等はエイリアス）、`Mat<Rows, Cols, T>`は行列、`Quat<T>`はクォータニオン。

```cpp
#include <cutil/vec.hpp>
#include <cutil/mat.hpp>
#include <cutil/quaternion.hpp>

using namespace cutil;

Vec3f a(1, 2, 3), b(4, 5, 6);
Vec3f c = a + b * 2.0f;
float d = dot(a, b);
Vec3f n = normalize(a);

Mat4f m = Mat4f::identity();

Quat<float> q = Quat<float>::from_axis_angle(Vec3f(0, 1, 0), 3.14 / 2);
```

</details>

## Str / Path

SSO（Small String Optimization）付き文字列型と、ファイルパス操作用の型。

<details>
<summary>使用例を見る</summary>

```cpp
#include <cutil/string.hpp>
#include <cutil/path.hpp>

using namespace cutil;

Str s("hello");
s += " world";

Path p("assets/model.fbx");
Str ext = p.extension();     // ".fbx"
Str name = p.filename();     // "model.fbx"
Path dir = p.parent();       // "assets"
```

</details>

## Range / Rect / Rect3D

2D/3D向けの範囲・矩形・AABB（軸並行境界ボックス）。

<details>
<summary>使用例を見る</summary>

```cpp
#include <cutil/rect.hpp>
#include <cutil/rect3d.hpp>

using namespace cutil;

Range r(0, 10);
Rect rect(0, 1, 0, 1);           // xmin, xmax, ymin, ymax
Rect3D bbox(Vec3f(0, 0, 0), Vec3f(1, 1, 1)); // min, max
```

</details>

## dictionary / hash_map / variant

文字列キーの辞書、カスタムハッシュマップ、型安全なUnion型といった汎用コンテナ。

<details>
<summary>使用例を見る</summary>

`dictionary<Value>`は文字列キーの辞書、`hash_map<Key, Value>`はカスタムハッシュマップ、`variant<Types...>`は`std::variant`相当。

```cpp
#include <cutil/dictionary.hpp>
#include <cutil/hash_map.hpp>
#include <cutil/variant.hpp>

using namespace cutil;

dictionary<int> d;
d.insert("hp", 100);
if (d.contains("hp")) { /* ... */ }

hash_map<std::string, int> hm;
hm["score"] = 42;

variant<int, float, std::string> v(3.14f);
```

</details>

## ObjectPool&lt;T&gt;

チャンク連結リスト構造のオブジェクトプール。`create()`で`Ref<T>`を返し、参照が全て解放されるとスロットがプールへ返却される。

<details>
<summary>使用例を見る</summary>

```cpp
#include <cutil/pool.hpp>

using namespace cutil;

ObjectPool<MyClass, 64> pool;
Ref<MyClass> obj = pool.create(42); // コンストラクタ引数を転送
```

- **注意**: `ObjectPool`はそこから生成した全`Ref<T>`より長生きしなければならない

</details>

## Octree

固定深度のボクセル空間分割木。近傍探索・範囲探索用。

<details>
<summary>使用例を見る</summary>

```cpp
#include <cutil/octree.hpp>

using namespace cutil;

Octree2<MyPointData> tree;
tree.setVolume(0, 100, 0, 100, 0, 100);
```

</details>

## Color / logger

RGB⇔HSV変換などの色空間ヘルパーと、レベル付きログマクロ。

<details>
<summary>使用例を見る</summary>

```cpp
#include <cutil/color.hpp>
#include <cutil/logger.hpp>

double hue = cutil::RGB2H(Vector3b(255, 0, 0));

LOGI << "info message";
LOGW << "warning message";
LOGE << "error message";
```

</details>
