#include <cutil/ref.hpp>
#include <iostream>
#include <string>

using namespace cutil;

// グローバルなマネージャーオブジェクト
Ref<class DataManager> g_manager;

// C インターフェース（Python からアクセス可能）

class DataManager : public enable_ref_from_this<DataManager> {
public:
  std::string data;
  int counter = 0;

  DataManager() { std::cout << "DataManager created" << std::endl; }

  ~DataManager() { std::cout << "DataManager destroyed" << std::endl; }

  void set_data(const char* d) {
    data = d;
    std::cout << "Data set: " << data << std::endl;
  }

  const char* get_data() { return data.c_str(); }

  int increment() {
    counter++;
    return counter;
  }

  int get_counter() { return counter; }

  Ref<DataManager> get_self() { return ref_from_this(); }
};

// C インターフェース
extern "C" {
// マネージャー初期化
void manager_init() { g_manager = make_ref<DataManager>(); }

// データを設定
void manager_set_data(const char* data) {
  if(g_manager) {
    g_manager->set_data(data);
  }
}

// データを取得
const char* manager_get_data() {
  if(g_manager) {
    return g_manager->get_data();
  }
  return "";
}

// カウンター増加
int manager_increment() {
  if(g_manager) {
    return g_manager->increment();
  }
  return -1;
}

// カウンター取得
int manager_get_counter() {
  if(g_manager) {
    return g_manager->get_counter();
  }
  return -1;
}

// 参照カウント取得
int manager_use_count() {
  if(g_manager) {
    return g_manager.use_count();
  }
  return 0;
}

// リセット
void manager_reset() { g_manager.reset(); }

// 単純な関数
int add(int a, int b) { return a + b; }

int multiply(int a, int b) { return a * b; }

// 文字列連結
const char* concat(const char* a, const char* b) {
  static std::string result;
  result = std::string(a) + std::string(b);
  return result.c_str();
}
}
