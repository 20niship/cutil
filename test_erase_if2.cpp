#include <iostream>
#include <string>
#include <cutil/dictionary.hpp>

using cutil::dictionary;

int main() {
  dictionary<int> dict;
  dict.insert("x", 1);
  dict.insert("y", 3);
  dict.insert("z", 5);
  
  std::cout << "Before erase_if:" << std::endl;
  std::cout << "Size: " << dict.size() << std::endl;
  for(const auto& [k, v] : dict) {
    std::cout << k << "=" << v << " ";
  }
  std::cout << std::endl;
  
  size_t erased = dict.erase_if([](const std::string& key, int value) {
    std::cout << "Check " << key << "=" << value << " > 2? ";
    bool remove = (value > 2);
    std::cout << (remove ? "YES" : "NO") << std::endl;
    return remove;
  });
  
  std::cout << "\nAfter erase_if (erased " << erased << " items):" << std::endl;
  std::cout << "Size: " << dict.size() << std::endl;
  for(const auto& [k, v] : dict) {
    std::cout << k << "=" << v << " ";
  }
  std::cout << std::endl;
  
  return 0;
}
