#include <iostream>
#include <utility>
#include <cstring>

struct Node {
  int key;
  int value;
  size_t hash_code;
  bool occupied;
};

int main() {
  Node n;
  n.key = 10;
  n.value = 20;
  
  using Pair = std::pair<const int, int>;
  
  std::cout << "Node size: " << sizeof(Node) << ", align: " << alignof(Node) << std::endl;
  std::cout << "Pair size: " << sizeof(Pair) << ", align: " << alignof(Pair) << std::endl;
  std::cout << "Key offset: " << offsetof(Node, key) << std::endl;
  std::cout << "Value offset: " << offsetof(Node, value) << std::endl;
  std::cout << "Pair first offset: " << offsetof(Pair, first) << std::endl;
  std::cout << "Pair second offset: " << offsetof(Pair, second) << std::endl;
  
  Pair& p = *reinterpret_cast<Pair*>(std::addressof(n));
  std::cout << "Pair.first: " << p.first << ", .second: " << p.second << std::endl;
  
  return 0;
}
