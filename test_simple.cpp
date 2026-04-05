#include <iostream>
#include <string>
#include <cutil/hash_map.hpp>

using cutil::hash_map;

int main() {
  hash_map<std::string, int> map;
  
  // Test insert
  auto [it, inserted] = map.insert("key1", 42);
  std::cout << "After insert, size: " << map.size() << std::endl;
  std::cout << "it->first: " << it->first << ", it->second: " << it->second << std::endl;
  
  // Test at
  std::cout << "at('key1'): " << map.at("key1") << std::endl;
  
  // Test operator[] on existing key
  std::cout << "Before op[], get value: " << map.at("key1") << std::endl;
  int val = map["key1"];
  std::cout << "After op[], returned value: " << val << std::endl;
  
  return 0;
}
