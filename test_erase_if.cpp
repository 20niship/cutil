#include <iostream>
#include <string>
#include <cutil/dictionary.hpp>

using cutil::dictionary;

int main() {
  dictionary<int> dict;
  dict.insert("a", 1);
  dict.insert("b", 2);
  dict.insert("c", 3);
  dict.insert("d", 4);
  
  std::cout << "Before erase_if:" << std::endl;
  for(const auto& [k, v] : dict) {
    std::cout << k << "=" << v << " ";
  }
  std::cout << "\nSize: " << dict.size() << std::endl;
  
  size_t erased = dict.erase_if([](const std::string& key, int value) {
    bool remove = (value % 2 == 0);
    std::cout << "Checking " << key << "=" << value << " -> " << (remove ? "ERASE" : "KEEP") << std::endl;
    return remove;
  });
  
  std::cout << "\nAfter erase_if (erased " << erased << " items):" << std::endl;
  for(const auto& [k, v] : dict) {
    std::cout << k << "=" << v << " ";
  }
  std::cout << "\nSize: " << dict.size() << std::endl;
  
  std::cout << "\nContains checks:" << std::endl;
  std::cout << "a: " << dict.contains("a") << std::endl;
  std::cout << "b: " << dict.contains("b") << std::endl;
  std::cout << "c: " << dict.contains("c") << std::endl;
  std::cout << "d: " << dict.contains("d") << std::endl;
  
  return 0;
}
