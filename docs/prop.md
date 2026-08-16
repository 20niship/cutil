# Prop / PropInfo の使い方

`cutil::Prop` / `cutil::PropInfo` は Blender の DNA/RNA に相当するシリアライズシステムです。
`Prop`(DNA相当)は生バイナリバッファとフィールド一覧を持つ動的プロパティコンテナ、
`PropInfo`(RNA相当)は「どのフィールドがどこにあり、どんな型・意味を持つか」を記述するメタデータです。
これらを使うと、ゲームエンジンや動画編集ソフトのようなアプリケーションで、独自のC++構造体を
そのままバイナリ/JSON形式でプロジェクトファイルへ保存・復元できます。

対象ファイル:

- `cutil/prop.hpp` — `Prop`/`PropInfo`本体
- `cutil/prop_registry.hpp` — `CustomSlot`/`CustomTypeRegistry`(任意の型の拡張登録)
- `cutil/prop_io.hpp` — JSON/バイナリでのdump/load(ファイルI/O相当)
- `tests/entity.hpp` — `Model`/`Mesh`を使った実装例

## 1. 基本: Propを動的プロパティバッグとして使う

`Prop`はキー(名前)と値のペアを持つ辞書のように使えます。対応する型は
`bool/int32_t/float/Str/Path/Vec3f/Vec4f/Quat<float>/Range/Rect/Rect3D/std::vector<uint8_t>(Binary)`と、
子`Prop`(`PropType::Nested`)です。

```cpp
#include <cutil/prop.hpp>
using namespace cutil;

Prop p;
p.set<int32_t>("hp", 100);
p.set<float>("speed", 3.5f);
p.set<Vec3f>("pos", Vec3f(1, 2, 3));
p.set<Str>("name", Str("player"));

int32_t hp = p.get<int32_t>("hp");
bool has_name = p.contains("name");
p.erase("hp");

// 子Propとして階層構造を持たせることもできる
Prop child;
child.set<int32_t>("level", 1);
p.set_child("stats", child);
Prop& stats = p.get_child("stats");
```

`set<T>()`と同じ型で再度`set`すると値が上書きされ、違う型で呼ぶと例外(`std::logic_error`)になります。

## 2. PropInfo::Data と PropInfo

`PropInfo`は複数フィールド分の`PropInfo::Data`(1フィールドのメタデータ)を保持するコンテナです。

```cpp
struct PropInfo {
  struct Data {
    char name[32];        // フィールド名
    char label[64];        // UI表示名
    char desc[256];         // 説明
    PropType type;           // Bool/Int/Float/Str/... など
    size_t offset, size;      // 構造体内でのoffset/size
    bool is_pointer;            // Str/Binary/Nested/RefListなど非トリビアルコピーが必要か
    uint32_t version;             // per-fieldのバージョン(互換性チェック用)
    // widget/min_value/max_value/drag_speed/flags など UI 用メタ情報も持てる
  };

  char id[32];       // このスキーマ自体の名前(例: "Model")。任意
  uiVector<Data> infos;
};
```

`Prop`自身が内部で持つ`infos_`も、外部C++構造体を説明する「ルール」も、どちらも同じ`PropInfo`型です。
前者のoffsetは`Prop::data_`内でのoffset、後者のoffsetは呼び出し側構造体のメモリ内でのoffsetを指します。

## 3. 外部C++構造体との相互変換: dump() / load_to()

任意のC++構造体を`Prop`と相互変換したい場合、`offsetof`ベースの「ルール」を型ごとに一度だけ静的に作ります。

```cpp
struct Model3D {
  Vec3f pos;
  Str name;
  bool visible = false;
};

const PropInfo& Model3DInfo() {
  static const PropInfo rule = {
      {"pos", PropType::Vec3, offsetof(Model3D, pos), sizeof(Model3D::pos), false},
      {"name", PropType::Str, offsetof(Model3D, name), sizeof(Model3D::name), true},
      {"visible", PropType::Bool, offsetof(Model3D, visible), sizeof(Model3D::visible), false},
  };
  return rule;
}

Model3D a;
a.pos = Vec3f(1, 2, 3);

Prop p;
p.dump(&a, &Model3DInfo());       // 構造体 → Prop

Model3D b;
p.load_to(&b, &Model3DInfo());    // Prop → 構造体(bへ復元)
```

`is_pointer`がtrueの型(`Str`/`Path`/`Binary`/`Nested`/`Custom`)は、`Prop`側にライブオブジェクトとして
placement-newで構築され、`load_to()`では既存オブジェクトへのコピー代入で書き戻されます。

## 4. 参照フィールド(Ref<T> / vector<Ref<T>>)を扱う: PropType::Ref / RefList

シーングラフのようにオブジェクト同士が`cutil::Ref<T>`/`cutil::WeakPtr<T>`で参照し合っている場合、
参照先の実体をコピーするのではなく「生きたポインタ」だけを`Prop`に持たせたいことがあります。
これを`PropType::Ref`(単一参照)・`PropType::RefList`(複数参照)で実現します。

```cpp
PropInfo::Data::make_ref<Model>("parent", offsetof(Model, parent));          // WeakPtr<Model> 用
PropInfo::Data::make_ref_list<Mesh>("meshes", offsetof(Model, meshes));      // vector<Ref<Mesh>> 用
```

- dump時: 参照先の生ポインタ(`T*`)を`Prop`のバッファへ格納する。
- load_to時: 格納された生ポインタから`T::weak_from_this()`/`ref_from_this()`(`enable_ref_from_this<T>`のAPI)を
  使って正規の`WeakPtr<T>`/`Ref<T>`を再構築する。

**重要な制約**: ここで保持されるのは生のメモリアドレスです。同一プロセス内で参照先オブジェクトが
生きている間だけ有効な「ライブスナップショット」であり、ファイルへdump/loadしてプロセスをまたいで
復元しても意味を持ちません(詳細はセクション6)。

## 5. get_propinfo() パターンと dump()/load() ラッパー

自分の型に対して、上記の「ルール」を`static const PropInfo* get_propinfo()`という静的メンバ関数として
持たせておくと、呼び出し側はルールを意識せず使えます。`tests/entity.hpp`の`Model`/`Mesh`が実装例です。

```cpp
class Mesh final : public enable_ref_from_this<Mesh> {
public:
  int vertex_count = 0;

  static const PropInfo* get_propinfo() {
    static const PropInfo rule = {
        {"vertex_count", PropType::Int, offsetof(Mesh, vertex_count), sizeof(int), false},
    };
    return &rule;
  }

  Prop dump() const { Prop p; p.dump(this, get_propinfo()); return p; }
  bool load(const Prop& p) { return p.load_to(this, get_propinfo()); }
};
```

```cpp
auto mesh = Mesh::Create(42);
Prop p = mesh->dump();

auto restored = Mesh::Create(0);
restored->load(p);
// restored->vertex_count == 42
```

`Model::get_propinfo()`は`name`(Str)・`position`(Vec3扱いのfloat[3])に加えて、
`parent`/`meshes`/`children`もRef/RefListとして含んでいるため、`dump()`/`load()`だけで
親子関係を保ったままモデルを複製・復元できます。

## 6. カスタムエンジンでのプロジェクト保存(`cutil/prop_io.hpp`)

`Prop`をファイルへ保存・復元するには`prop_io.hpp`のヘルパーを使います。2系統あります。

### JSON(可読・フォールバック用)

```cpp
#include <cutil/prop_io.hpp>

std::string json_text;
prop_dump_json(prop, json_text);   // 全フィールドを人間可読なJSONへ

Prop loaded;
prop_load_json(loaded, json_text); // JSONから復元
```

### バイナリ(高速・本番用)

```cpp
std::vector<uint8_t> bytes;
prop_dump_binary(prop, bytes);     // POD型は直接コピー、Str/Binary/Nested等はBlobBlockへ

Prop loaded;
prop_load_binary(loaded, bytes);   // 読み込み。フォーマット不一致・per-fieldバージョン不一致は
                                    // 自動的にファイル内蔵のJSONブロックへフォールバックする
```

バイナリファイルには常にJSONブロックが併載されるため(`format_version`不一致や壊れたデータでも)、
1ファイルだけで新旧互換のフォールバックが効きます。フォールバック処理を自前で用意したい場合は
`prop_load_binary(prop, bytes, fallback)`のように第3引数に`bool(Prop&, const std::vector<uint8_t>&)`を渡せます。

### エンジン内での典型的な流れ

```cpp
// 保存
Prop p = model->dump();
std::vector<uint8_t> bytes;
prop_dump_binary(p, bytes);
// bytesをファイルへ書き込む(std::ofstreamなど)

// 読み込み
// bytesをファイルから読み込む
Prop loaded;
prop_load_binary(loaded, bytes);
auto model = Model::Create();
loaded.load_to(model.get(), Model::get_propinfo());
```

### 注意: 参照フィールド(Ref/RefList)はファイル保存には向かない

`PropType::Ref`/`RefList`は生ポインタをそのまま値として保持するため、
`prop_dump_json`/`prop_dump_binary`でファイルへ書き出すこと自体は失敗しませんが、
別プロセス・別実行で読み込んでもアドレスは無意味になります。そのため:

- `Ref`は同一プロセス内でのJSON dump/loadでは往復しますが(アドレスは有効なため)、ファイルをまたいだ
  永続化には使えません。
- `RefList`はJSON/バイナリいずれの読み込みでも意図的にスキップされ、フィールドごと復元されません
  (`prop_load_json`/`prop_load_binary`内で明示的にガードされているため、例外や破損は起きません)。

つまり `Ref`/`RefList` は「同一プロセス内でシーングラフのスナップショットを取って`dump()`/`load()`で
コピー・復元する」用途(アンドゥ履歴、クリップボード、ランタイムでの複製など)には使えますが、
プロジェクトファイルへの永続的な参照保存(別の起動時にも親子関係を復元する)には未対応です。
永続化したい場合は、各オブジェクトに一意なIDを持たせて`Ref`の代わりにID(整数)をフィールドとして
持たせ、load後に自前でID→オブジェクトの解決を行う設計にする必要があります(未実装)。
