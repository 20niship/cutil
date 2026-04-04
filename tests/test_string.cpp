#include "doctest.h"
#include <cutil/string.hpp>

using cutil::Str;

TEST_SUITE("Str - Constructors") {
  TEST_CASE("Default constructor creates empty string") {
    Str s;
    CHECK(s.empty());
    CHECK(s.size() == 0);
    CHECK(s.c_str()[0] == '\0');
  }

  TEST_CASE("Constructor from C string") {
    Str s("hello");
    CHECK(s.size() == 5);
    CHECK(s == "hello");
  }

  TEST_CASE("Constructor from C string with length") {
    Str s("hello world", 5);
    CHECK(s.size() == 5);
    CHECK(s == "hello");
  }

  TEST_CASE("Constructor from std::string_view") {
    std::string_view sv("test");
    Str s(sv);
    CHECK(s.size() == 4);
    CHECK(s == "test");
  }

  TEST_CASE("Constructor from char repetition") {
    Str s(5, 'a');
    CHECK(s.size() == 5);
    CHECK(s == "aaaaa");
  }

  TEST_CASE("Copy constructor") {
    Str original("copy test");
    Str copy(original);
    CHECK(copy == original);
    CHECK(copy.c_str() != original.c_str()); // Different memory
  }

  TEST_CASE("Move constructor") {
    Str original("move test");
    const char* original_ptr = original.c_str();
    Str moved(std::move(original));
    CHECK(moved == "move test");
    CHECK(original.empty());
  }

  TEST_CASE("SSO with short string (<=22 chars)") {
    Str s("short"); // 5 chars
    CHECK(s.size() == 5);
    CHECK(s.capacity() == 22);
  }

  TEST_CASE("Heap allocation with long string (>22 chars)") {
    Str s("this is a very long string that exceeds sso capacity");
    CHECK(s.size() > 22);
    CHECK(s.capacity() > 22);
  }
}

TEST_SUITE("Str - Basic Operations") {
  TEST_CASE("Size and capacity") {
    Str s("test");
    CHECK(s.size() == 4);
    CHECK(s.length() == 4);
    CHECK(s.capacity() >= 4);
  }

  TEST_CASE("Empty check") {
    Str empty;
    Str nonempty("a");
    CHECK(empty.empty());
    CHECK(empty.is_empty());
    CHECK(!nonempty.empty());
  }

  TEST_CASE("Element access with operator[]") {
    Str s("hello");
    CHECK(s[0] == 'h');
    CHECK(s[4] == 'o');
  }

  TEST_CASE("Element access with at()") {
    Str s("hello");
    CHECK(s.at(0) == 'h');
    CHECK_THROWS(s.at(10));
  }

  TEST_CASE("Front and back") {
    Str s("hello");
    CHECK(s.front() == 'h');
    CHECK(s.back() == 'o');
  }

  TEST_CASE("Push and pop back") {
    Str s("hi");
    s.push_back('!');
    CHECK(s == "hi!");
    s.pop_back();
    CHECK(s == "hi");
  }

  TEST_CASE("Append operations") {
    Str s("hello");
    s.append(" world");
    CHECK(s == "hello world");
  }

  TEST_CASE("Operator += string") {
    Str s("hello");
    s += " world";
    CHECK(s == "hello world");
  }

  TEST_CASE("Operator += char") {
    Str s("hi");
    s += '!';
    CHECK(s == "hi!");
  }

  TEST_CASE("Operator + creates new string") {
    Str s1("hello");
    Str s2 = s1 + " world";
    CHECK(s2 == "hello world");
    CHECK(s1 == "hello"); // Original unchanged
  }

  TEST_CASE("Clear") {
    Str s("content");
    s.clear();
    CHECK(s.empty());
  }

  TEST_CASE("Reserve and resize") {
    Str s("hi");
    s.reserve(100);
    CHECK(s.capacity() >= 100);

    s.resize(5, 'x');
    CHECK(s.size() == 5);
    CHECK(s == "hixxx");
  }
}

TEST_SUITE("Str - Comparison") {
  TEST_CASE("Equality comparison") {
    Str s1("hello");
    Str s2("hello");
    Str s3("world");

    CHECK(s1 == s2);
    CHECK(s1 != s3);
    CHECK(s1 == "hello");
    CHECK(s1 != "world");
  }

  TEST_CASE("Relational comparison") {
    Str s1("apple");
    Str s2("banana");

    CHECK(s1 < s2);
    CHECK(s1 <= s2);
    CHECK(!(s1 > s2));
    CHECK(!(s1 >= s2));
  }

  TEST_CASE("Case-insensitive comparison") {
    Str s1("Hello");
    Str s2("hello");
    CHECK(s1.casecmp_to(s2) == 0);
  }
}

TEST_SUITE("Str - Search") {
  TEST_CASE("Find substring") {
    Str s("hello world");
    CHECK(s.find("world") == 6);
    CHECK(s.find("xyz") == Str::npos);
    CHECK(s.find("hello", 0) == 0);
  }

  TEST_CASE("Reverse find") {
    Str s("hello hello");
    CHECK(s.rfind("hello") == 6);
  }

  TEST_CASE("Contains") {
    Str s("hello world");
    CHECK(s.contains("world"));
    CHECK(!s.contains("xyz"));
  }

  TEST_CASE("Case-insensitive contains") {
    Str s("Hello World");
    CHECK(s.containsn("world"));
    CHECK(s.containsn("HELLO"));
  }

  TEST_CASE("Begins with") {
    Str s("hello world");
    CHECK(s.begins_with("hello"));
    CHECK(!s.begins_with("world"));
  }

  TEST_CASE("Ends with") {
    Str s("hello world");
    CHECK(s.ends_with("world"));
    CHECK(!s.ends_with("hello"));
  }

  TEST_CASE("Count occurrences") {
    Str s("banana");
    CHECK(s.count("a") == 3);
    CHECK(s.count("an") == 2);
    CHECK(s.count("xyz") == 0);
  }
}

TEST_SUITE("Str - Substring & Slicing") {
  TEST_CASE("Substr") {
    Str s("hello world");
    CHECK(s.substr(0, 5) == "hello");
    CHECK(s.substr(6) == "world");
  }

  TEST_CASE("Left") {
    Str s("hello");
    CHECK(s.left(3) == "hel");
    CHECK(s.left(100) == "hello");
  }

  TEST_CASE("Right") {
    Str s("hello");
    CHECK(s.right(3) == "llo");
    CHECK(s.right(100) == "hello");
  }

  TEST_CASE("Insert") {
    Str s("hello");
    Str result = s.insert(2, "**");
    CHECK(result == "he**llo");
  }

  TEST_CASE("Erase") {
    Str s("hello");
    Str result = s.erase(1, 3);
    CHECK(result == "ho");
  }
}

TEST_SUITE("Str - Transformations") {
  TEST_CASE("to_upper and to_lower") {
    Str s("HeLLo");
    CHECK(s.to_upper() == "HELLO");
    CHECK(s.to_lower() == "hello");
  }

  TEST_CASE("Capitalize") {
    Str s("hello");
    CHECK(s.capitalize() == "Hello");
  }

  TEST_CASE("Reverse") {
    Str s("hello");
    CHECK(s.reverse() == "olleh");
  }

  TEST_CASE("Repeat") {
    Str s("ab");
    CHECK(s.repeat(3) == "ababab");
    CHECK(Str().repeat(5) == Str());
  }

  TEST_CASE("Replace") {
    Str s("hello world");
    CHECK(s.replace("world", "there") == "hello there");
    CHECK(s.replace("l", "L") == "heLLo worLd");
  }

  TEST_CASE("Case-insensitive replace") {
    Str s("Hello World");
    CHECK(s.replacen("hello", "hi") == "hi World");
  }

  TEST_CASE("to_camel_case") {
    Str s("hello_world_test");
    CHECK(s.to_camel_case() == "helloWorldTest");
  }

  TEST_CASE("to_pascal_case") {
    Str s("hello_world_test");
    CHECK(s.to_pascal_case() == "HelloWorldTest");
  }

  TEST_CASE("to_snake_case") {
    Str s("HelloWorld");
    CHECK(s.to_snake_case() == "hello_world");
  }
}

TEST_SUITE("Str - Trim") {
  TEST_CASE("strip_edges") {
    Str s("  hello  ");
    CHECK(s.strip_edges() == "hello");
  }

  TEST_CASE("lstrip and rstrip") {
    Str s("  hello  ");
    CHECK(s.lstrip() == "hello  ");
    CHECK(s.rstrip() == "  hello");
  }

  TEST_CASE("trim_prefix and trim_suffix") {
    Str s("prefix_content_suffix");
    CHECK(s.trim_prefix("prefix_") == "content_suffix");
    CHECK(s.trim_suffix("_suffix") == "prefix_content");
  }
}

TEST_SUITE("Str - Split & Join") {
  TEST_CASE("Split with space") {
    Str s("hello world test");
    auto parts = s.split(" ");
    CHECK(parts.size() == 3);
    CHECK(parts[0] == "hello");
    CHECK(parts[1] == "world");
    CHECK(parts[2] == "test");
  }

  TEST_CASE("Split with allow_empty=false") {
    Str s("a,,b");
    auto parts = s.split(",", false);
    CHECK(parts.size() == 2);
    CHECK(parts[0] == "a");
    CHECK(parts[1] == "b");
  }

  TEST_CASE("Split with maxsplit") {
    Str s("a:b:c:d");
    auto parts = s.split(":", true, 2);
    CHECK(parts.size() == 3);
    CHECK(parts[2] == "c:d");
  }

  TEST_CASE("Join") {
    std::vector<Str> parts{"hello", "world", "test"};
    Str result = Str::join(" ", parts);
    CHECK(result == "hello world test");
  }
}

TEST_SUITE("Str - Number Conversion") {
  TEST_CASE("to_int") {
    Str s("42");
    CHECK(s.to_int() == 42);

    Str s_neg("-100");
    CHECK(s_neg.to_int() == -100);
  }

  TEST_CASE("to_float and to_double") {
    Str s("3.14");
    CHECK(doctest::Approx(s.to_float()) == 3.14f);
    CHECK(doctest::Approx(s.to_double()) == 3.14);
  }

  TEST_CASE("is_valid_int") {
    CHECK(Str("42").is_valid_int());
    CHECK(!Str("42.5").is_valid_int());
    CHECK(!Str("abc").is_valid_int());
  }

  TEST_CASE("is_valid_float") {
    CHECK(Str("3.14").is_valid_float());
    CHECK(Str("42").is_valid_float());
    CHECK(!Str("abc").is_valid_float());
  }

  TEST_CASE("is_valid_identifier") {
    CHECK(Str("valid_name").is_valid_identifier());
    CHECK(Str("_leading_underscore").is_valid_identifier());
    CHECK(!Str("123start").is_valid_identifier());
    CHECK(!Str("with-dash").is_valid_identifier());
  }

  TEST_CASE("num static method") {
    Str s = Str::num(3.14159, 2);
    CHECK(s == "3.14");
  }

  TEST_CASE("num_int64") {
    CHECK(Str::num_int64(255, 16) == "ff");
    CHECK(Str::num_int64(10, 2) == "1010");
  }

  TEST_CASE("Padding") {
    Str s("42");
    CHECK(s.pad_zeros(5) == "00042");
    CHECK(s.lpad(5, '0') == "00042");
    CHECK(s.rpad(5, '*') == "42***");
  }
}

TEST_SUITE("Str - Path Operations") {
  TEST_CASE("get_base_dir") {
    CHECK(Str("/path/to/file.txt").get_base_dir() == "/path/to");
    CHECK(Str("file.txt").get_base_dir() == ".");
  }

  TEST_CASE("get_file") {
    CHECK(Str("/path/to/file.txt").get_file() == "file.txt");
    CHECK(Str("file.txt").get_file() == "file.txt");
  }

  TEST_CASE("get_extension") {
    CHECK(Str("file.txt").get_extension() == ".txt");
    CHECK(Str("/path/to/file.tar.gz").get_extension() == ".gz");
    CHECK(Str("no_extension").get_extension() == Str());
  }

  TEST_CASE("get_basename") {
    CHECK(Str("/path/to/file.txt").get_basename() == "file");
    CHECK(Str("file.txt").get_basename() == "file");
  }

  TEST_CASE("is_absolute_path and is_relative_path") {
    CHECK(Str("/absolute/path").is_absolute_path());
    CHECK(Str("relative/path").is_relative_path());
  }

  TEST_CASE("path_join") {
    CHECK((Str("/path/to") / "file.txt") == "/path/to/file.txt");
    CHECK(Str("dir").path_join("file") == "dir/file");
  }

  TEST_CASE("simplify_path") {
    CHECK(Str("/path/./to/../file.txt").simplify_path() == "/path/file.txt");
    CHECK(Str("a/b/../c").simplify_path() == "a/c");
  }
}

TEST_SUITE("Str - Conversions") {
  TEST_CASE("operator std::string_view") {
    Str s("test");
    std::string_view sv = static_cast<std::string_view>(s);
    CHECK(sv == "test");
  }

  TEST_CASE("to_std_string") {
    Str s("convert");
    std::string std_s = s.to_std_string();
    CHECK(std_s == "convert");
  }
}

TEST_SUITE("Str - Utility") {
  TEST_CASE("hash") {
    Str s1("test");
    Str s2("test");
    Str s3("other");

    CHECK(s1.hash() == s2.hash());
    CHECK(s1.hash() != s3.hash());
  }

  TEST_CASE("std::hash integration") {
    std::hash<Str> hasher;
    Str s("test");
    size_t h = hasher(s);
    CHECK(h == s.hash());
  }

  TEST_CASE("similarity") {
    Str s1("hello");
    Str s2("hallo");
    double sim = s1.similarity(s2);
    CHECK(sim > 0.0);
    CHECK(sim < 1.0);

    CHECK(s1.similarity(s1) > 0.9); // Nearly identical
  }
}

TEST_SUITE("Str - Assignment Operators") {
  TEST_CASE("Copy assignment") {
    Str s1("original");
    Str s2("temp");
    s2 = s1;
    CHECK(s2 == "original");
    CHECK(s1 == "original");
  }

  TEST_CASE("Move assignment") {
    Str s1("original");
    Str s2("temp");
    s2 = std::move(s1);
    CHECK(s2 == "original");
    CHECK(s1.empty());
  }

  TEST_CASE("Assignment from C string") {
    Str s;
    s = "hello";
    CHECK(s == "hello");
  }

  TEST_CASE("Swap") {
    Str s1("first");
    Str s2("second");
    s1.swap(s2);
    CHECK(s1 == "second");
    CHECK(s2 == "first");
  }
}

TEST_SUITE("Str - Edge Cases") {
  TEST_CASE("Empty string operations") {
    Str empty;
    CHECK(empty.empty());
    CHECK(empty.size() == 0);
    CHECK(empty.find("test") == Str::npos);
    CHECK(!empty.contains("anything"));
  }

  TEST_CASE("SSO to Heap conversion") {
    Str s("short"); // SSO
    s += " this will exceed sso capacity and force heap allocation";
    CHECK(s.contains("exceed"));
  }

  TEST_CASE("Null pointer handling in constructors") {
    Str s(nullptr);
    CHECK(s.empty());
  }

  TEST_CASE("Unicode handling") {
    // Basic support - treating as bytes
    Str s("こんにちは"); // Japanese characters
    CHECK(s.size() > 0);
  }
}

TEST_SUITE("Str - Memory Safety") {
  TEST_CASE("No double free on multiple moves") {
    Str s1("test");
    Str s2(std::move(s1));
    Str s3(std::move(s2));
    CHECK(s3 == "test");
  }

  TEST_CASE("Safe resize with SSO threshold crossing") {
    Str s("small");
    for(int i = 0; i < 20; ++i) {
      s += "x";
    }
    CHECK(s.size() > 22);
  }
}
