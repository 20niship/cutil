#include <cutil/ref.hpp>
#include <cutil/string.hpp>
#include <cutil/string_nanobind.hpp>
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

// StringProcessor クラス - Str を使用
class StringProcessor {
private:
  Str data;

public:
  StringProcessor(const Str& initial = Str()) : data(initial) {}

  // Str を受け取る関数
  void set_data(const Str& d) {
    data = d;
    std::cout << "StringProcessor: data set to '" << data.c_str() << "'" << std::endl;
  }

  Str get_data() const { return data; }

  // Str を返す関数
  Str uppercase() const { return data.to_upper(); }

  Str lowercase() const { return data.to_lower(); }

  Str reversed() const { return data.reverse(); }

  Str capitalized() const { return data.capitalize(); }

  // Str を引数に取る関数
  bool contains_text(const Str& text) const { return data.contains(text); }

  Str replace_text(const Str& what, const Str& with) const { return data.replace(what, with); }

  size_t count_text(const Str& text) const { return data.count(text); }

  std::vector<Str> split_text(const Str& delim) const { return data.split(delim); }
};

// Str を使ったフリー関数
Str process_string(const Str& input) { return Str("Processed: ") + input.to_upper(); }

Str concat_strings(const Str& a, const Str& b) { return a + " " + b; }

std::string get_upper_as_std_string(const Str& s) { return s.to_upper().to_std_string(); }

Str reverse_and_uppercase(const Str& s) { return s.reverse().to_upper(); }

Str extract_filename(const Str& path) { return path.get_file(); }

// Python モジュール定義
NB_MODULE(mylib, m) {
  m.doc() = "Example library using Ref<T>, Str, and nanobind";

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

  // StringProcessor クラス - Str のデモ
  nb::class_<StringProcessor>(m, "StringProcessor")
    .def(nb::init<>())
    .def(nb::init<const Str&>())
    .def("set_data", &StringProcessor::set_data, "Set data (accepts Python str)")
    .def("get_data", &StringProcessor::get_data, "Get data (returns Python str)")
    .def("uppercase", &StringProcessor::uppercase, "Return uppercase version")
    .def("lowercase", &StringProcessor::lowercase, "Return lowercase version")
    .def("reversed", &StringProcessor::reversed, "Return reversed string")
    .def("capitalized", &StringProcessor::capitalized, "Return capitalized string")
    .def("contains_text", &StringProcessor::contains_text, "Check if contains text")
    .def("replace_text", &StringProcessor::replace_text, "Replace text")
    .def("count_text", &StringProcessor::count_text, "Count occurrences")
    .def("split_text", &StringProcessor::split_text, "Split by delimiter");

  // フリー関数
  m.def("add_numbers", [](int a, int b) { return a + b; }, "Add two numbers");
  m.def("multiply_numbers", [](int a, int b) { return a * b; }, "Multiply two numbers");
  m.def("greet", [](const std::string& name) { return "Hello, " + name + "!"; }, "Greet someone");

  // Str を使ったフリー関数
  m.def("process_string", &process_string, "Process a string (uppercase with prefix)");
  m.def("concat_strings", &concat_strings, "Concatenate two strings");
  m.def("get_upper_as_std_string", &get_upper_as_std_string, "Get uppercase version as std::string");
  m.def("reverse_and_uppercase", &reverse_and_uppercase, "Reverse and uppercase");
  m.def("extract_filename", &extract_filename, "Extract filename from path");
}
