#include <iostream>
#include <string>
#include <cutil/hash_map.hpp>

using cutil::hash_map;

int main() {
  hash_map<std::string, int> map;
  map.insert("a", 1);
  map.insert("b", 2);
  map.insert("c", 3);
  map.insert("d", 4);
  
  std::cout << "After insert, contains:" << std::endl;
  std::cout << "a: " << map.contains("a") << std::endl;
  std::cout << "b: " << map.contains("b") << std::endl;
  std::cout << "c: " << map.contains("c") << std::endl;
  std::cout << "d: " << map.contains("d") << std::endl;
  
  // Erase d
  map.erase("d");
  
  std::cout << "\nAfter erase(d), contains:" << std::endl;
  std::cout << "a: " << map.contains("a") << std::endl;
  std::cout << "c: " << map.contains("c") << std::endl;
  std::cout << "d: " << map.contains("d") << std::endl;
  
  // Erase b
  map.erase("b");
  
  std::cout << "\nAfter erase(b), contains:" << std::endl;
  std::cout << "a: " << map.contains("a") << std::endl;
  std::cout << "c: " << map.contains("c") << std::endl;
  
  return 0;
}
