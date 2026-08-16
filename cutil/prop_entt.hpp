#pragma once

#include <functional>
#include <string>
#include <typeindex>
#include <vector>

#include <cutil/dictionary.hpp>
#include <cutil/entt.hpp>
#include <cutil/prop.hpp>
#include <cutil/prop_io.hpp>

// per-fieldバージョンフォールバックは新規実装せず、既存のprop_dump_binary/prop_load_binaryにそのまま乗せる(entt_save_binary/entt_load_binary参照)。

namespace cutil {

// 型名(文字列) -> dump/loadルールのレジストリ。CustomTypeRegistryと同じ発想。
struct ComponentOps {
  std::type_index type_index = std::type_index(typeid(void));
  std::function<void(EnttManager&, Prop&)> dump_all;       // mgr.get<T>()の全要素をlist(空のProp)へ書き込む
  std::function<void(const Prop&, EnttManager&)> load_all; // listからTを復元しmgr.add<T>()で戻す
};

class ComponentRegistry {
public:
  static ComponentRegistry& instance() {
    static ComponentRegistry inst;
    return inst;
  }

  void register_type(const std::string& name, const ComponentOps& ops) { ops_.put(name, ops); }

  [[nodiscard]] const dictionary<ComponentOps>& all() const { return ops_; }

private:
  dictionary<ComponentOps> ops_;
};

// get_propinfo()を持たないleaf型向けのデフォルトルール(register_component_typeのget_rule省略時に使う)。
template <typename T> const PropInfo* single_field_propinfo() {
  static const PropInfo rule = {
      {"value", prop_type_traits<T>::type, 0, sizeof(T), prop_type_traits<T>::is_pointer},
  };
  return &rule;
}

// offsetofベースのPropInfoルールを持つ型(構造体 or leaf型)を登録する。
template <typename T> void register_component_type(const std::string& name, const PropInfo* (*get_rule)() = &single_field_propinfo<T>) {
  ComponentOps ops;
  ops.type_index = std::type_index(typeid(T));
  ops.dump_all   = [get_rule](EnttManager& mgr, Prop& list) {
    auto elems = mgr.get<T>();
    list.set<int32_t>("count", static_cast<int32_t>(elems.size()));
    for(size_t i = 0; i < elems.size(); i++) {
      Prop elem;
      elem.dump(elems[i], get_rule());
      list.set_child(std::to_string(i).c_str(), elem);
    }
  };
  ops.load_all = [get_rule](const Prop& list, EnttManager& mgr) {
    int32_t count = list.contains("count") ? list.get<int32_t>("count") : 0;
    for(int32_t i = 0; i < count; i++) {
      std::string key = std::to_string(i);
      if(!list.contains(key.c_str())) continue; // 壊れたデータでも安全にスキップ
      T tmp{};
      (void)list.get_child(key.c_str()).load_to(&tmp, get_rule());
      mgr.add(tmp);
    }
  };
  ComponentRegistry::instance().register_type(name, ops);
}

// offsetofで表現できないコンテナ型用。事前にregister_custom_type<T>(custom_type_name)が必要。
template <typename T> void register_component_type_custom(const std::string& name, const std::string& custom_type_name) {
  ComponentOps ops;
  ops.type_index = std::type_index(typeid(T));
  ops.dump_all   = [custom_type_name](EnttManager& mgr, Prop& list) {
    auto elems = mgr.get<T>();
    list.set<int32_t>("count", static_cast<int32_t>(elems.size()));
    for(size_t i = 0; i < elems.size(); i++) {
      list.set_custom(std::to_string(i).c_str(), custom_type_name.c_str(), *elems[i]);
    }
  };
  ops.load_all = [custom_type_name](const Prop& list, EnttManager& mgr) {
    int32_t count = list.contains("count") ? list.get<int32_t>("count") : 0;
    for(int32_t i = 0; i < count; i++) {
      std::string key = std::to_string(i);
      if(!list.contains(key.c_str())) continue;
      mgr.add(list.get_custom<T>(key.c_str()));
    }
  };
  ComponentRegistry::instance().register_type(name, ops);
}

// 登録済みの全コンポーネント型をProp階層へ書き出す/読み戻す。
inline void entt_dump(EnttManager& mgr, Prop& out) {
  for(const auto& [name, ops] : ComponentRegistry::instance().all()) {
    Prop list;
    ops.dump_all(mgr, list);
    out.set_child(name.c_str(), list);
  }
}

inline void entt_load(const Prop& in, EnttManager& mgr) {
  for(const auto& [name, ops] : ComponentRegistry::instance().all()) {
    if(!in.contains(name.c_str())) continue; // 未登録 or このセーブに無かった型はスキップ
    ops.load_all(in.get_child(name.c_str()), mgr);
  }
}

// プロジェクトファイル相当のエントリポイント。フォールバックはprop_load_binaryにそのまま委譲する。
inline bool entt_save_binary(EnttManager& mgr, std::vector<uint8_t>& out) {
  Prop root;
  entt_dump(mgr, root);
  return prop_dump_binary(root, out);
}

inline bool entt_load_binary(EnttManager& mgr, const std::vector<uint8_t>& bytes, const PropLoadFallback& fallback = nullptr) {
  Prop root;
  if(!prop_load_binary(root, bytes, fallback)) return false;
  entt_load(root, mgr);
  return true;
}

} // namespace cutil
