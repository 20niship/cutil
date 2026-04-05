#include <iostream>
#include <string>
#include <cutil/hash_map.hpp>

using cutil::hash_map;

int main() {
  hash_map<std::string, int> map;
  
  // Test insert
  auto [it, inserted] = map.insert("key1", 42);
  std::cout << "inserted: " << inserted << std::endl;
  std::cout << "it->first: " << it->first << ", it->second: " << it->second << std::endl;
  
  // Test at
  std::cout << "at('key1'): " << map.at("key1") << std::endl;
  
  // Test operator[]
  std::cout << "operator[]('key1'): " << map["key1"] << std::endl;
  
  // Test second operator[]
  auto [it2, inserted2] = map.insert("key2", 100);
  std::cout << "operator[]('key2'): " << map["key2"] << std::endl;
  
  return 0;
}
