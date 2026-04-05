#include "doctest.h"
#include <cutil/hash_map.hpp>

using cutil::hash_map;

TEST_SUITE("hash_map - Basic Operations") {
  TEST_CASE("Default constructor creates empty map") {
    hash_map<std::string, int> map;
    CHECK(map.empty());
    CHECK(map.size() == 0);
  }

  TEST_CASE("Insert single element") {
    hash_map<std::string, int> map;
    auto [it, inserted] = map.insert("key1", 42);
    CHECK(inserted);
    CHECK(map.size() == 1);
    CHECK(map.at("key1") == 42);
  }

  TEST_CASE("Insert duplicate key updates value") {
    hash_map<std::string, int> map;
    map.insert("key1", 10);
    auto [it, inserted] = map.insert("key1", 20);
    CHECK(!inserted);
    CHECK(map.size() == 1);
    CHECK(map.at("key1") == 20);
  }

  TEST_CASE("Find existing element") {
    hash_map<std::string, int> map;
    map.insert("key1", 42);
    auto it = map.find("key1");
    CHECK(it != map.end());
    CHECK(it->second == 42);
  }

  TEST_CASE("Find non-existing element returns end") {
    hash_map<std::string, int> map;
    map.insert("key1", 42);
    auto it = map.find("key2");
    CHECK(it == map.end());
  }

  TEST_CASE("at() throws on non-existing key") {
    hash_map<std::string, int> map;
    CHECK_THROWS(map.at("nonexistent"));
  }

  TEST_CASE("Operator[] inserts default value for new key") {
    hash_map<std::string, int> map;
    CHECK(map["key1"] == 0);
    CHECK(map.size() == 1);
  }

  TEST_CASE("Operator[] accesses existing value") {
    hash_map<std::string, int> map;
    map.insert("key1", 42);
    CHECK(map["key1"] == 42);
  }

  TEST_CASE("Contains") {
    hash_map<std::string, int> map;
    map.insert("key1", 42);
    CHECK(map.contains("key1"));
    CHECK(!map.contains("key2"));
  }

  TEST_CASE("Erase existing element") {
    hash_map<std::string, int> map;
    map.insert("key1", 42);
    bool erased = map.erase("key1");
    CHECK(erased);
    CHECK(map.size() == 0);
    CHECK(!map.contains("key1"));
  }

  TEST_CASE("Erase non-existing element returns false") {
    hash_map<std::string, int> map;
    bool erased = map.erase("nonexistent");
    CHECK(!erased);
  }

  TEST_CASE("Clear") {
    hash_map<std::string, int> map;
    map.insert("key1", 1);
    map.insert("key2", 2);
    map.insert("key3", 3);
    CHECK(map.size() == 3);
    map.clear();
    CHECK(map.empty());
    CHECK(map.size() == 0);
  }
}

TEST_SUITE("hash_map - Copy and Move") {
  TEST_CASE("Copy constructor") {
    hash_map<std::string, int> map1;
    map1.insert("key1", 42);
    map1.insert("key2", 100);

    hash_map<std::string, int> map2(map1);
    CHECK(map2.size() == 2);
    CHECK(map2.at("key1") == 42);
    CHECK(map2.at("key2") == 100);
  }

  TEST_CASE("Copy assignment") {
    hash_map<std::string, int> map1;
    map1.insert("key1", 42);

    hash_map<std::string, int> map2;
    map2.insert("other", 999);
    map2 = map1;

    CHECK(map2.size() == 1);
    CHECK(map2.at("key1") == 42);
    CHECK(!map2.contains("other"));
  }

  TEST_CASE("Move constructor") {
    hash_map<std::string, int> map1;
    map1.insert("key1", 42);
    map1.insert("key2", 100);

    hash_map<std::string, int> map2(std::move(map1));
    CHECK(map2.size() == 2);
    CHECK(map2.at("key1") == 42);
  }

  TEST_CASE("Move assignment") {
    hash_map<std::string, int> map1;
    map1.insert("key1", 42);

    hash_map<std::string, int> map2;
    map2 = std::move(map1);
    CHECK(map2.size() == 1);
    CHECK(map2.at("key1") == 42);
  }
}

TEST_SUITE("hash_map - Iteration") {
  TEST_CASE("Iterate over elements") {
    hash_map<std::string, int> map;
    map.insert("a", 1);
    map.insert("b", 2);
    map.insert("c", 3);

    int sum = 0;
    for(auto& item : map) {
      sum += item.second;
    }
    CHECK(sum == 6);
  }

  TEST_CASE("Const iteration") {
    hash_map<std::string, int> map;
    map.insert("x", 10);
    map.insert("y", 20);

    const auto& const_map = map;
    int sum = 0;
    for(const auto& item : const_map) {
      sum += item.second;
    }
    CHECK(sum == 30);
  }

  TEST_CASE("Empty map iteration") {
    hash_map<std::string, int> map;
    CHECK(map.begin() == map.end());
  }
}

TEST_SUITE("hash_map - Rehashing") {
  TEST_CASE("Automatic rehashing on load factor") {
    hash_map<int, int> map;
    size_t initial_capacity = map.capacity();

    // Insert many elements to trigger rehash
    for(int i = 0; i < 20; ++i) {
      map.insert(i, i * 10);
    }

    CHECK(map.size() == 20);
    // Capacity should increase
    CHECK(map.capacity() >= initial_capacity);

    // All elements should still be accessible
    for(int i = 0; i < 20; ++i) {
      CHECK(map.at(i) == i * 10);
    }
  }
}

TEST_SUITE("hash_map - Edge Cases") {
  TEST_CASE("Multiple types") {
    hash_map<int, std::string> map;
    map.insert(1, "one");
    map.insert(2, "two");
    map.insert(3, "three");

    CHECK(map.at(1) == "one");
    CHECK(map.at(2) == "two");
    CHECK(map.at(3) == "three");
  }

  TEST_CASE("Empty string as key") {
    hash_map<std::string, int> map;
    map.insert("", 42);
    CHECK(map.at("") == 42);
    CHECK(map.contains(""));
  }

  TEST_CASE("Large values") {
    hash_map<std::string, std::vector<int>> map;
    std::vector<int> large_vec(1000);
    for(size_t i = 0; i < 1000; ++i) {
      large_vec[i] = i;
    }
    map.insert("large", large_vec);
    CHECK(map.at("large").size() == 1000);
  }
}
