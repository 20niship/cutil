#include <cutil/ref.hpp>
#include <iostream>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
using namespace cutil;

// 単純な計算クラス
class Calculator {
public:
  int value = 0;

  Calculator() = default;
  Calculator(int v) : value(v) {}

  int add(int n) {
    value += n;
    return value;
  }

  int subtract(int n) {
    value -= n;
    return value;
  }

  int multiply(int n) {
    value *= n;
    return value;
  }

  int get_value() const { return value; }

  void reset() { value = 0; }
};

// enable_ref_from_this を使ったクラス
class DataManager : public enable_ref_from_this<DataManager> {
private:
  std::string data;

public:
  DataManager(const std::string& initial = "") : data(initial) {}

  void set_data(const std::string& d) {
    data = d;
    std::cout << "DataManager: data set to '" << data << "'" << std::endl;
  }

  std::string get_data() const { return data; }

  Ref<DataManager> get_self() { return ref_from_this(); }

  std::string process() { return "Processing: " + data; }
};

// WeakPtr を使ったObserver
class Observer {
private:
  WeakPtr<DataManager> target;
  std::string name;

public:
  Observer(const std::string& n) : name(n) {}

  void attach(const Ref<DataManager>& manager) {
    target = manager;
    std::cout << "Observer '" << name << "' attached" << std::endl;
  }

  bool is_target_alive() const { return !target.expired(); }

  std::string get_target_data() {
    if(auto mgr = target.lock()) {
      return "Observer '" + name + "' sees: " + mgr->get_data();
    }
    return "Observer '" + name + "' target is dead";
  }

  std::string get_name() const { return name; }
};

// Python モジュール定義
NB_MODULE(mylib, m) {
  m.doc() = "Example library using Ref<T> and nanobind";

  // Calculator クラス
  nb::class_<Calculator>(m, "Calculator")
    .def(nb::init<>())
    .def(nb::init<int>())
    .def("add", &Calculator::add, "Add value")
    .def("subtract", &Calculator::subtract, "Subtract value")
    .def("multiply", &Calculator::multiply, "Multiply value")
    .def("get_value", &Calculator::get_value, "Get current value")
    .def("reset", &Calculator::reset, "Reset to 0")
    .def_rw("value", &Calculator::value, "Current value");

  // DataManager クラス
  nb::class_<DataManager>(m, "DataManager").def(nb::init<>()).def(nb::init<const std::string&>()).def("set_data", &DataManager::set_data).def("get_data", &DataManager::get_data).def("get_self", &DataManager::get_self, "Get self reference").def("process", &DataManager::process);

  // Observer クラス
  nb::class_<Observer>(m, "Observer").def(nb::init<const std::string&>()).def("attach", &Observer::attach, "Attach to DataManager").def("is_target_alive", &Observer::is_target_alive).def("get_target_data", &Observer::get_target_data).def("get_name", &Observer::get_name);

  // フリー関数
  m.def("add_numbers", [](int a, int b) { return a + b; }, "Add two numbers");
  m.def("multiply_numbers", [](int a, int b) { return a * b; }, "Multiply two numbers");
  m.def("greet", [](const std::string& name) { return "Hello, " + name + "!"; }, "Greet someone");
}
