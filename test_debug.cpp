#include <iostream>
#include <string>
#include <cutil/hash_map.hpp>

using cutil::hash_map;

int main() {
  hash_map<std::string, int> map;
  
  // First insert
  {
    auto [it, inserted] = map.insert("key1", 42);
    std::cout << "First insert: inserted=" << inserted << ", it->second=" << it->second << std::endl;
  }
  
  // Second insert of default value (like in operator[])
  {
    auto [it, inserted] = map.insert("key1", int());  // int() = 0
    std::cout << "Second insert (default): inserted=" << inserted << ", it->second=" << it->second << std::endl;
  }
  
  // Check at
  std::cout << "at('key1'): " << map.at("key1") << std::endl;
  
  return 0;
}
