#include "doctest.h"
#include <cutil/dictionary.hpp>

using cutil::dictionary;

TEST_SUITE("dictionary - Basic Operations") {
  TEST_CASE("Default constructor creates empty dictionary") {
    dictionary<int> dict;
    CHECK(dict.empty());
    CHECK(dict.size() == 0);
  }

  TEST_CASE("Insert element") {
    dictionary<int> dict;
    auto [it, inserted] = dict.insert("key1", 42);
    CHECK(inserted);
    CHECK(dict.size() == 1);
    CHECK(dict.at("key1") == 42);
  }

  TEST_CASE("Insert duplicate key updates value") {
    dictionary<int> dict;
    dict.insert("key1", 10);
    auto [it, inserted] = dict.insert("key1", 20);
    CHECK(!inserted);
    CHECK(dict.size() == 1);
    CHECK(dict.at("key1") == 20);
  }

  TEST_CASE("Find element") {
    dictionary<int> dict;
    dict.insert("key1", 42);
    auto it = dict.find("key1");
    CHECK(it != dict.end());
    CHECK(it->second == 42);
  }

  TEST_CASE("Find non-existing returns end") {
    dictionary<int> dict;
    auto it = dict.find("nonexistent");
    CHECK(it == dict.end());
  }

  TEST_CASE("Contains") {
    dictionary<int> dict;
    dict.insert("key1", 42);
    CHECK(dict.contains("key1"));
    CHECK(!dict.contains("key2"));
  }

  TEST_CASE("Count") {
    dictionary<int> dict;
    dict.insert("key1", 42);
    CHECK(dict.count("key1") == 1);
    CHECK(dict.count("nonexistent") == 0);
  }

  TEST_CASE("Operator[]") {
    dictionary<int> dict;
    dict["key1"] = 42;
    CHECK(dict.at("key1") == 42);
    CHECK(dict["key1"] == 42);
  }

  TEST_CASE("at() throws on missing key") {
    dictionary<int> dict;
    CHECK_THROWS(dict.at("nonexistent"));
  }

  TEST_CASE("Erase element") {
    dictionary<int> dict;
    dict.insert("key1", 42);
    bool erased = dict.erase("key1");
    CHECK(erased);
    CHECK(dict.size() == 0);
    CHECK(!dict.contains("key1"));
  }

  TEST_CASE("Erase non-existing returns false") {
    dictionary<int> dict;
    bool erased = dict.erase("nonexistent");
    CHECK(!erased);
  }

  TEST_CASE("Clear") {
    dictionary<int> dict;
    dict.insert("key1", 1);
    dict.insert("key2", 2);
    dict.insert("key3", 3);
    dict.clear();
    CHECK(dict.empty());
    CHECK(dict.size() == 0);
  }
}

TEST_SUITE("dictionary - Copy and Move") {
  TEST_CASE("Copy constructor") {
    dictionary<int> dict1;
    dict1.insert("a", 1);
    dict1.insert("b", 2);

    dictionary<int> dict2(dict1);
    CHECK(dict2.size() == 2);
    CHECK(dict2.at("a") == 1);
    CHECK(dict2.at("b") == 2);
  }

  TEST_CASE("Copy assignment") {
    dictionary<int> dict1;
    dict1.insert("a", 1);

    dictionary<int> dict2;
    dict2.insert("x", 99);
    dict2 = dict1;

    CHECK(dict2.size() == 1);
    CHECK(dict2.at("a") == 1);
    CHECK(!dict2.contains("x"));
  }

  TEST_CASE("Move constructor") {
    dictionary<int> dict1;
    dict1.insert("a", 1);
    dict1.insert("b", 2);

    dictionary<int> dict2(std::move(dict1));
    CHECK(dict2.size() == 2);
    CHECK(dict2.at("a") == 1);
  }

  TEST_CASE("Move assignment") {
    dictionary<int> dict1;
    dict1.insert("a", 1);

    dictionary<int> dict2;
    dict2 = std::move(dict1);
    CHECK(dict2.size() == 1);
    CHECK(dict2.at("a") == 1);
  }

  TEST_CASE("Initializer list constructor") {
    dictionary<int> dict({{"a", 1}, {"b", 2}, {"c", 3}});
    CHECK(dict.size() == 3);
    CHECK(dict.at("a") == 1);
    CHECK(dict.at("b") == 2);
    CHECK(dict.at("c") == 3);
  }
}

TEST_SUITE("dictionary - Iteration") {
  TEST_CASE("Forward iteration") {
    dictionary<int> dict;
    dict.insert("a", 1);
    dict.insert("b", 2);
    dict.insert("c", 3);

    int sum = 0;
    for(auto& item : dict) {
      sum += item.second;
    }
    CHECK(sum == 6);
  }

  TEST_CASE("Const iteration") {
    dictionary<std::string> dict;
    dict.insert("x", "hello");
    dict.insert("y", "world");

    int count = 0;
    for(const auto& item : dict) {
      CHECK(!item.first.empty());
      CHECK(!item.second.empty());
      ++count;
    }
    CHECK(count == 2);
  }

  TEST_CASE("cbegin/cend") {
    dictionary<int> dict;
    dict.insert("a", 1);
    dict.insert("b", 2);

    auto it = dict.cbegin();
    CHECK(it != dict.cend());
  }
}

TEST_SUITE("dictionary - Helper Methods") {
  TEST_CASE("keys()") {
    dictionary<int> dict;
    dict.insert("a", 1);
    dict.insert("b", 2);
    dict.insert("c", 3);

    auto keys = dict.keys();
    CHECK(keys.size() == 3);
    CHECK(std::find(keys.begin(), keys.end(), "a") != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), "b") != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), "c") != keys.end());
  }

  TEST_CASE("values()") {
    dictionary<int> dict;
    dict.insert("a", 1);
    dict.insert("b", 2);
    dict.insert("c", 3);

    auto values = dict.values();
    CHECK(values.size() == 3);
    CHECK(std::find(values.begin(), values.end(), 1) != values.end());
    CHECK(std::find(values.begin(), values.end(), 2) != values.end());
    CHECK(std::find(values.begin(), values.end(), 3) != values.end());
  }

  TEST_CASE("get() with default value") {
    dictionary<int> dict;
    dict.insert("a", 42);

    CHECK(dict.get("a", 0) == 42);
    CHECK(dict.get("b", 0) == 0);
    CHECK(dict.get("c", 999) == 999);
  }

  TEST_CASE("merge()") {
    dictionary<int> dict1;
    dict1.insert("a", 1);
    dict1.insert("b", 2);

    dictionary<int> dict2;
    dict2.insert("b", 20);  // Override
    dict2.insert("c", 3);

    dict1.merge(dict2);
    CHECK(dict1.size() == 3);
    CHECK(dict1.at("a") == 1);
    CHECK(dict1.at("b") == 20);  // Updated
    CHECK(dict1.at("c") == 3);
  }

  TEST_CASE("put()") {
    dictionary<std::string> dict;
    auto& val1 = dict.put("x", "hello");
    CHECK(val1 == "hello");

    auto& val2 = dict.put("x", "world");
    CHECK(val2 == "world");
    CHECK(dict.size() == 1);
  }

  TEST_CASE("erase_if()") {
    dictionary<int> dict;
    // Fill with enough items to avoid linear probe issues
    for(int i = 0; i < 10; ++i) {
      dict.insert(std::to_string(i), i);
    }

    size_t erased = dict.erase_if([](const std::string& key, int value) { return value % 3 == 0; });

    CHECK(erased == 4);  // 0, 3, 6, 9
    CHECK(dict.size() == 6);
  }
}

TEST_SUITE("dictionary - Complex Value Types") {
  TEST_CASE("Store vectors as values") {
    dictionary<std::vector<int>> dict;
    dict.insert("nums", {1, 2, 3, 4, 5});

    auto& vec = dict.at("nums");
    CHECK(vec.size() == 5);
    CHECK(vec[0] == 1);
    CHECK(vec[4] == 5);
  }

  TEST_CASE("Store strings as values") {
    dictionary<std::string> dict;
    dict.insert("greeting", "Hello");
    dict.insert("farewell", "Goodbye");

    CHECK(dict.at("greeting") == "Hello");
    CHECK(dict.at("farewell") == "Goodbye");
  }

  TEST_CASE("Iterator erase") {
    dictionary<int> dict;
    dict.insert("a", 1);
    dict.insert("b", 2);
    dict.insert("c", 3);

    for(auto it = dict.begin(); it != dict.end();) {
      if(it->second == 2) {
        it = dict.erase(it);
      } else {
        ++it;
      }
    }

    CHECK(dict.size() == 2);
    CHECK(!dict.contains("b"));
  }
}
