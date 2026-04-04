#pragma once

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace cutil {

// ============================================================================
// Str - C言語スタイルのメモリ管理 + SSO (Small String Optimization)
// ============================================================================

class Str {
public:
  static constexpr size_t SSO_CAPACITY   = 22;               // 22文字まで + null terminator
  static constexpr size_t HEAP_THRESHOLD = SSO_CAPACITY + 1; // 23文字以上はheap
  static constexpr size_t npos           = static_cast<size_t>(-1);

private:
  // Union: SSO時はスタック、heap時はポインタベース
  union Storage {
    struct {
      char* ptr;
      size_t size;
      size_t capacity;
    } heap;
    struct {
      char buf[SSO_CAPACITY + 1]; // +1 for null terminator
      uint8_t remaining;          // SSO_CAPACITY - size (0xFF = heap flag)
    } sso;
  } storage_;

public:
  // ========== Constructors & Assignment ==========

  // Default constructor - empty SSO string
  Str() noexcept { init_sso_empty(); }

  // Constructor from C string
  explicit Str(const char* s) {
    if(!s) {
      init_sso_empty();
      return;
    }
    size_t len = std::strlen(s);
    init_from_buffer(s, len);
  }

  // Constructor from C string with length
  Str(const char* s, size_t len) {
    if(!s || len == 0) {
      init_sso_empty();
      return;
    }
    init_from_buffer(s, len);
  }

  // Constructor from std::string_view
  Str(std::string_view sv) {
    if(sv.empty()) {
      init_sso_empty();
      return;
    }
    init_from_buffer(sv.data(), sv.length());
  }

  // Constructor from char repetition
  Str(size_t n, char c) {
    if(n == 0) {
      init_sso_empty();
      return;
    }
    if(n <= SSO_CAPACITY) {
      std::memset(storage_.sso.buf, c, n);
      storage_.sso.buf[n]    = '\0';
      storage_.sso.remaining = SSO_CAPACITY - n;
    } else {
      allocate_heap(n);
      std::memset(storage_.heap.ptr, c, n);
      storage_.heap.ptr[n] = '\0';
      storage_.heap.size   = n;
    }
  }

  // Copy constructor
  Str(const Str& other) {
    if(other.is_sso()) {
      // SSO copy: just copy the union
      storage_ = other.storage_;
    } else {
      // Heap copy: allocate and copy
      allocate_heap(other.size());
      std::memcpy(storage_.heap.ptr, other.storage_.heap.ptr, other.size());
      storage_.heap.ptr[other.size()] = '\0';
      storage_.heap.size              = other.size();
    }
  }

  // Move constructor
  Str(Str&& other) noexcept {
    if(other.is_sso()) {
      // SSO move: copy buffer, reset source
      storage_ = other.storage_;
      other.init_sso_empty();
    } else {
      // Heap move: steal pointer
      storage_ = other.storage_;
      other.init_sso_empty();
    }
  }

  // Destructor
  ~Str() {
    if(!is_sso()) {
      std::free(storage_.heap.ptr);
    }
  }

  // Copy assignment
  Str& operator=(const Str& other) {
    if(this != &other) {
      Str temp(other);
      swap(temp);
    }
    return *this;
  }

  // Move assignment
  Str& operator=(Str&& other) noexcept {
    if(this != &other) {
      if(!is_sso()) {
        std::free(storage_.heap.ptr);
      }
      if(other.is_sso()) {
        storage_ = other.storage_;
        other.init_sso_empty();
      } else {
        storage_ = other.storage_;
        other.init_sso_empty();
      }
    }
    return *this;
  }

  // Assign from C string
  Str& operator=(const char* s) {
    if(!s) {
      clear();
      return *this;
    }
    return operator=(Str(s));
  }

  // Assign from std::string_view
  Str& operator=(std::string_view sv) { return operator=(Str(sv)); }

  void swap(Str& other) noexcept { std::swap(storage_, other.storage_); }

  // ========== Capacity & Access ==========

  [[nodiscard]] size_t size() const noexcept {
    if(is_sso()) {
      return SSO_CAPACITY - storage_.sso.remaining;
    } else {
      return storage_.heap.size;
    }
  }

  [[nodiscard]] size_t length() const noexcept { return size(); }

  [[nodiscard]] size_t capacity() const noexcept {
    if(is_sso()) {
      return SSO_CAPACITY;
    } else {
      return storage_.heap.capacity;
    }
  }

  [[nodiscard]] bool empty() const noexcept { return size() == 0; }

  [[nodiscard]] bool is_empty() const noexcept { return size() == 0; }

  [[nodiscard]] const char* data() const noexcept { return is_sso() ? storage_.sso.buf : storage_.heap.ptr; }

  [[nodiscard]] const char* c_str() const noexcept { return data(); }

  char& operator[](size_t i) noexcept { return const_cast<char*>(data())[i]; }

  const char& operator[](size_t i) const noexcept { return data()[i]; }

  char& at(size_t i) {
    if(i >= size()) {
      throw std::out_of_range("Str::at");
    }
    return const_cast<char*>(data())[i];
  }

  const char& at(size_t i) const {
    if(i >= size()) {
      throw std::out_of_range("Str::at");
    }
    return data()[i];
  }

  [[nodiscard]] char front() const noexcept { return data()[0]; }

  [[nodiscard]] char back() const noexcept { return data()[size() - 1]; }

  // ========== Iterators ==========

  const char* begin() const noexcept { return data(); }
  const char* end() const noexcept { return data() + size(); }
  const char* cbegin() const noexcept { return begin(); }
  const char* cend() const noexcept { return end(); }

  char* begin() noexcept { return const_cast<char*>(data()); }
  char* end() noexcept { return const_cast<char*>(data()) + size(); }

  // ========== Modification ==========

  void clear() noexcept {
    if(!is_sso()) {
      std::free(storage_.heap.ptr);
    }
    init_sso_empty();
  }

  void reserve(size_t new_capacity) {
    if(new_capacity <= capacity()) {
      return;
    }

    if(is_sso()) {
      // Convert SSO to heap
      const char* old_data = storage_.sso.buf;
      size_t old_size      = size();
      allocate_heap(new_capacity);
      std::memcpy(storage_.heap.ptr, old_data, old_size);
      storage_.heap.ptr[old_size] = '\0';
      storage_.heap.size          = old_size;
    } else {
      // Reallocate heap
      char* new_ptr = static_cast<char*>(std::realloc(storage_.heap.ptr, new_capacity + 1));
      if(!new_ptr) {
        throw std::bad_alloc();
      }
      storage_.heap.ptr      = new_ptr;
      storage_.heap.capacity = new_capacity;
    }
  }

  void resize(size_t new_size, char fill = ' ') {
    size_t old_size = size();

    if(new_size < old_size) {
      // Truncate
      if(is_sso()) {
        storage_.sso.buf[new_size] = '\0';
        storage_.sso.remaining     = SSO_CAPACITY - new_size;
      } else {
        storage_.heap.ptr[new_size] = '\0';
        storage_.heap.size          = new_size;
      }
    } else if(new_size > old_size) {
      // Extend
      reserve(new_size);
      char* ptr = const_cast<char*>(data());
      std::memset(ptr + old_size, fill, new_size - old_size);
      ptr[new_size] = '\0';

      if(is_sso()) {
        storage_.sso.remaining = SSO_CAPACITY - new_size;
      } else {
        storage_.heap.size = new_size;
      }
    }
  }

  void push_back(char c) {
    size_t old_size = size();
    resize(old_size + 1);
    const_cast<char*>(data())[old_size] = c;
  }

  void pop_back() noexcept {
    if(size() > 0) {
      if(is_sso()) {
        size_t s                = size();
        storage_.sso.buf[s - 1] = '\0';
        storage_.sso.remaining  = SSO_CAPACITY - (s - 1);
      } else {
        storage_.heap.size--;
        storage_.heap.ptr[storage_.heap.size] = '\0';
      }
    }
  }

  void append(std::string_view sv) {
    if(sv.empty()) return;
    size_t old_size = size();
    resize(old_size + sv.size());
    std::memcpy(const_cast<char*>(data()) + old_size, sv.data(), sv.size());
  }

  void append(const char* s) {
    if(!s) return;
    append(std::string_view(s));
  }

  Str& operator+=(std::string_view sv) {
    append(sv);
    return *this;
  }

  Str& operator+=(const char* s) {
    append(s);
    return *this;
  }

  Str& operator+=(const Str& s) {
    append(std::string_view(s.c_str()));
    return *this;
  }

  Str& operator+=(char c) {
    push_back(c);
    return *this;
  }

  [[nodiscard]] Str operator+(std::string_view sv) const {
    Str result(*this);
    result += sv;
    return result;
  }

  [[nodiscard]] Str operator+(const Str& other) const { return operator+(std::string_view(other.c_str())); }

  [[nodiscard]] Str operator+(const char* s) const { return operator+(std::string_view(s ? s : "")); }

  [[nodiscard]] Str operator+(char c) const {
    Str result(*this);
    result += c;
    return result;
  }

  // ========== Comparison ==========

  [[nodiscard]] bool operator==(std::string_view other) const noexcept {
    if(size() != other.size()) return false;
    return std::memcmp(data(), other.data(), size()) == 0;
  }

  [[nodiscard]] bool operator==(const char* other) const noexcept { return operator==(std::string_view(other ? other : "")); }

  [[nodiscard]] bool operator==(const Str& other) const noexcept { return operator==(std::string_view(other.c_str())); }

  [[nodiscard]] bool operator!=(std::string_view other) const noexcept { return !operator==(other); }

  [[nodiscard]] bool operator!=(const char* other) const noexcept { return !operator==(other); }

  [[nodiscard]] bool operator!=(const Str& other) const noexcept { return !operator==(other); }

  [[nodiscard]] bool operator<(const Str& other) const noexcept { return std::strcmp(c_str(), other.c_str()) < 0; }

  [[nodiscard]] bool operator<=(const Str& other) const noexcept { return std::strcmp(c_str(), other.c_str()) <= 0; }

  [[nodiscard]] bool operator>(const Str& other) const noexcept { return std::strcmp(c_str(), other.c_str()) > 0; }

  [[nodiscard]] bool operator>=(const Str& other) const noexcept { return std::strcmp(c_str(), other.c_str()) >= 0; }

  [[nodiscard]] int casecmp_to(const Str& other) const noexcept {
    const char* p1 = c_str();
    const char* p2 = other.c_str();
    while(*p1 && *p2) {
      int c1 = std::tolower(static_cast<unsigned char>(*p1));
      int c2 = std::tolower(static_cast<unsigned char>(*p2));
      if(c1 != c2) return c1 - c2;
      p1++;
      p2++;
    }
    return static_cast<unsigned char>(*p1) - static_cast<unsigned char>(*p2);
  }

  // ========== Conversions ==========

  [[nodiscard]] explicit operator std::string_view() const noexcept { return std::string_view(data(), size()); }

  [[nodiscard]] explicit operator std::string() const { return std::string(data(), size()); }

  [[nodiscard]] std::string to_std_string() const { return std::string(data(), size()); }

  // ========== Search ==========

  [[nodiscard]] size_t find(std::string_view needle, size_t from = 0) const noexcept {
    if(needle.empty() || from >= size()) {
      return npos;
    }
    const char* pos = std::strstr(c_str() + from, needle.data());
    if(!pos) return npos;
    return pos - c_str();
  }

  [[nodiscard]] size_t find(const char* needle, size_t from = 0) const noexcept { return find(std::string_view(needle ? needle : ""), from); }

  [[nodiscard]] size_t rfind(std::string_view needle, size_t from = npos) const noexcept {
    if(needle.empty()) return npos;
    if(from == npos) from = size();

    for(size_t i = (from < size() ? from : size() - 1);; --i) {
      if(i + needle.size() > size()) {
        if(i == 0) break;
        --i;
        continue;
      }
      if(std::memcmp(data() + i, needle.data(), needle.size()) == 0) {
        return i;
      }
      if(i == 0) break;
      --i;
    }
    return npos;
  }

  [[nodiscard]] size_t rfind(const char* needle, size_t from = npos) const noexcept { return rfind(std::string_view(needle ? needle : ""), from); }

  [[nodiscard]] bool contains(std::string_view needle) const noexcept { return find(needle) != npos; }

  [[nodiscard]] bool contains(const char* needle) const noexcept { return contains(std::string_view(needle ? needle : "")); }

  [[nodiscard]] bool contains(const Str& needle) const noexcept { return contains(std::string_view(needle.c_str())); }

  [[nodiscard]] bool containsn(std::string_view needle) const noexcept {
    // Case-insensitive contains
    if(needle.empty()) return true;
    Str needle_lower(needle);
    needle_lower   = needle_lower.to_lower();
    Str self_lower = to_lower();
    return self_lower.contains(std::string_view(needle_lower.c_str()));
  }

  [[nodiscard]] bool containsn(const Str& needle) const noexcept { return containsn(std::string_view(needle.c_str())); }

  [[nodiscard]] bool begins_with(std::string_view prefix) const noexcept {
    if(prefix.size() > size()) return false;
    return std::memcmp(data(), prefix.data(), prefix.size()) == 0;
  }

  [[nodiscard]] bool begins_with(const char* prefix) const noexcept { return begins_with(std::string_view(prefix ? prefix : "")); }

  [[nodiscard]] bool ends_with(std::string_view suffix) const noexcept {
    if(suffix.size() > size()) return false;
    return std::memcmp(data() + size() - suffix.size(), suffix.data(), suffix.size()) == 0;
  }
  [[nodiscard]] bool ends_with(const char* suffix) const noexcept { return ends_with(std::string_view(suffix ? suffix : "")); }
  [[nodiscard]] size_t count(std::string_view needle) const noexcept {
    if(needle.empty()) return 0;
    size_t count = 0;
    size_t pos   = 0;
    while((pos = find(needle, pos)) != npos) {
      count++;
      pos += needle.size();
    }
    return count;
  }

  [[nodiscard]] size_t count(const char* needle) const noexcept { return count(std::string_view(needle ? needle : "")); }
  [[nodiscard]] size_t count(const Str& needle) const noexcept { return count(std::string_view(needle.c_str())); }

  [[nodiscard]] std::vector<Str> split(const Str& delim, bool allow_empty = true, size_t maxsplit = 0) const { return split(std::string_view(delim.c_str()), allow_empty, maxsplit); }

  [[nodiscard]] Str replace(const Str& what, const Str& by) const { return replace(std::string_view(what.c_str()), std::string_view(by.c_str())); }

  [[nodiscard]] Str replacen(const Str& what, const Str& by) const { return replacen(std::string_view(what.c_str()), std::string_view(by.c_str())); }

  // ========== Substr & Slicing ==========

  [[nodiscard]] Str substr(size_t from, size_t len = npos) const {
    if(from >= size()) {
      return Str();
    }
    if(len == npos || from + len > size()) {
      len = size() - from;
    }
    return Str(data() + from, len);
  }

  [[nodiscard]] Str left(size_t n) const noexcept {
    if(n > size()) n = size();
    return Str(data(), n);
  }

  [[nodiscard]] Str right(size_t n) const noexcept {
    if(n > size()) n = size();
    return Str(data() + size() - n, n);
  }

  [[nodiscard]] Str insert(size_t pos, std::string_view what) const {
    if(pos > size()) pos = size();
    Str result(data(), pos);
    result += what;
    result += Str(data() + pos);
    return result;
  }

  [[nodiscard]] Str insert(size_t pos, const char* what) const { return insert(pos, std::string_view(what ? what : "")); }

  [[nodiscard]] Str erase(size_t pos, size_t len = npos) const {
    if(pos >= size()) return Str(*this);
    if(len == npos || pos + len > size()) {
      len = size() - pos;
    }
    Str result(data(), pos);
    result += Str(data() + pos + len);
    return result;
  }

  // ========== String Transformation ==========

  [[nodiscard]] Str to_upper() const {
    Str result(*this);
    for(size_t i = 0; i < result.size(); ++i) {
      result[i] = std::toupper(static_cast<unsigned char>(result[i]));
    }
    return result;
  }

  [[nodiscard]] Str to_lower() const {
    Str result(*this);
    for(size_t i = 0; i < result.size(); ++i) {
      result[i] = std::tolower(static_cast<unsigned char>(result[i]));
    }
    return result;
  }

  [[nodiscard]] Str capitalize() const {
    Str result(*this);
    if(!result.empty()) {
      result[0] = std::toupper(static_cast<unsigned char>(result[0]));
    }
    return result;
  }

  [[nodiscard]] Str reverse() const {
    Str result(*this);
    std::reverse(result.begin(), result.end());
    return result;
  }

  [[nodiscard]] Str repeat(size_t n) const {
    if(n == 0 || empty()) return Str();
    Str result;
    for(size_t i = 0; i < n; ++i) {
      result += *this;
    }
    return result;
  }

  [[nodiscard]] Str replace(std::string_view what, std::string_view by) const {
    if(what.empty()) return Str(*this);

    Str result;
    size_t last_pos = 0;
    size_t pos      = find(what, last_pos);

    while(pos != npos) {
      result += substr(last_pos, pos - last_pos);
      result += by;
      last_pos = pos + what.size();
      pos      = find(what, last_pos);
    }
    result += substr(last_pos);
    return result;
  }

  [[nodiscard]] Str replace(const char* what, const char* by) const { return replace(std::string_view(what ? what : ""), std::string_view(by ? by : "")); }

  [[nodiscard]] Str replacen(std::string_view what, std::string_view by) const {
    // Case-insensitive replace
    Str self_lower = to_lower();
    Str what_lower(what);
    what_lower = what_lower.to_lower();

    Str result;
    size_t last_pos = 0;
    size_t pos      = self_lower.find(std::string_view(what_lower.c_str()), last_pos);

    while(pos != npos) {
      result += substr(last_pos, pos - last_pos);
      result += by;
      last_pos = pos + what.size();
      pos      = self_lower.find(std::string_view(what_lower.c_str()), last_pos);
    }
    result += substr(last_pos);
    return result;
  }

  [[nodiscard]] Str replacen(const char* what, const char* by) const { return replacen(std::string_view(what ? what : ""), std::string_view(by ? by : "")); }

  // ========== Trim ==========

  [[nodiscard]] Str strip_edges(std::string_view chars = " \t\n\r\f\v") const {
    size_t start = 0;
    while(start < size() && is_char_in(data()[start], chars)) {
      start++;
    }
    if(start >= size()) return Str();

    size_t end = size() - 1;
    while(end > start && is_char_in(data()[end], chars)) {
      end--;
    }

    return substr(start, end - start + 1);
  }

  [[nodiscard]] Str strip_edges(const char* chars = " \t\n\r\f\v") const { return strip_edges(std::string_view(chars)); }

  [[nodiscard]] Str lstrip(std::string_view chars = " \t\n\r\f\v") const {
    size_t start = 0;
    while(start < size() && is_char_in(data()[start], chars)) {
      start++;
    }
    return substr(start);
  }

  [[nodiscard]] Str lstrip(const char* chars = " \t\n\r\f\v") const { return lstrip(std::string_view(chars)); }

  [[nodiscard]] Str rstrip(std::string_view chars = " \t\n\r\f\v") const {
    if(empty()) return Str();

    size_t end = size() - 1;
    while(end < size() && is_char_in(data()[end], chars)) {
      if(end == 0) return Str();
      end--;
    }
    return substr(0, end + 1);
  }

  [[nodiscard]] Str rstrip(const char* chars = " \t\n\r\f\v") const { return rstrip(std::string_view(chars)); }

  [[nodiscard]] Str trim_prefix(std::string_view prefix) const {
    if(begins_with(prefix)) {
      return substr(prefix.size());
    }
    return Str(*this);
  }

  [[nodiscard]] Str trim_prefix(const char* prefix) const { return trim_prefix(std::string_view(prefix ? prefix : "")); }

  [[nodiscard]] Str trim_suffix(std::string_view suffix) const {
    if(ends_with(suffix)) {
      return substr(0, size() - suffix.size());
    }
    return Str(*this);
  }

  [[nodiscard]] Str trim_suffix(const char* suffix) const { return trim_suffix(std::string_view(suffix ? suffix : "")); }

  // ========== Split & Join ==========

  [[nodiscard]] std::vector<Str> split(std::string_view delim = " ", bool allow_empty = true, size_t maxsplit = 0) const {
    std::vector<Str> result;
    if(empty() || delim.empty()) {
      if(allow_empty || !empty()) result.push_back(*this);
      return result;
    }

    size_t last_pos    = 0;
    size_t split_count = 0;

    for(size_t pos = find(delim, last_pos); pos != npos && (maxsplit == 0 || split_count < maxsplit); pos = find(delim, last_pos)) {
      Str part = substr(last_pos, pos - last_pos);
      if(allow_empty || !part.empty()) {
        result.push_back(part);
      }
      last_pos = pos + delim.size();
      split_count++;
    }

    Str final_part = substr(last_pos);
    if(allow_empty || !final_part.empty()) {
      result.push_back(final_part);
    }

    return result;
  }

  [[nodiscard]] std::vector<Str> split(const char* delim = " ", bool allow_empty = true, size_t maxsplit = 0) const { return split(std::string_view(delim ? delim : ""), allow_empty, maxsplit); }

  static Str join(std::string_view delim, const std::vector<Str>& parts) {
    if(parts.empty()) return Str();

    Str result = parts[0];
    for(size_t i = 1; i < parts.size(); ++i) {
      result += delim;
      result += parts[i];
    }
    return result;
  }

  static Str join(const char* delim, const std::vector<Str>& parts) { return join(std::string_view(delim ? delim : ""), parts); }

  // ========== Number Conversion ==========

  [[nodiscard]] int64_t to_int() const {
    char* end      = nullptr;
    int64_t result = std::strtoll(c_str(), &end, 10);
    return result;
  }

  [[nodiscard]] float to_float() const { return std::strtof(c_str(), nullptr); }

  [[nodiscard]] double to_double() const { return std::strtod(c_str(), nullptr); }

  [[nodiscard]] bool is_valid_int() const {
    if(empty()) return false;
    char* end = nullptr;
    std::strtoll(c_str(), &end, 10);
    return *end == '\0';
  }

  [[nodiscard]] bool is_valid_float() const {
    if(empty()) return false;
    char* end = nullptr;
    std::strtod(c_str(), &end);
    return *end == '\0';
  }

  [[nodiscard]] bool is_valid_identifier() const {
    if(empty()) return false;
    if(!std::isalpha(static_cast<unsigned char>(front())) && front() != '_') {
      return false;
    }
    for(size_t i = 1; i < size(); ++i) {
      char c = data()[i];
      if(!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
        return false;
      }
    }
    return true;
  }

  static Str num(double f, int decimals = 2) {
    char buf[256];
    if(decimals < 0) {
      std::snprintf(buf, sizeof(buf), "%g", f);
    } else {
      std::snprintf(buf, sizeof(buf), "%.*f", decimals, f);
    }
    return Str(buf);
  }

  static Str num_int64(int64_t n, int base = 10) {
    if(base < 2 || base > 36) return Str("0");
    if(n == 0) return Str("0");

    const char* digits = "0123456789abcdefghijklmnopqrstuvwxyz";
    bool negative      = n < 0;
    if(negative) n = -n;

    char buf[128];
    int pos  = 127;
    buf[pos] = '\0';

    while(n > 0) {
      buf[--pos] = digits[n % base];
      n /= base;
    }

    if(negative) {
      buf[--pos] = '-';
    }

    return Str(buf + pos);
  }

  [[nodiscard]] Str pad_zeros(size_t n) const { return lpad(n, '0'); }

  [[nodiscard]] Str lpad(size_t len, char c) const {
    if(size() >= len) return Str(*this);
    Str padding(len - size(), c);
    return padding + *this;
  }

  [[nodiscard]] Str rpad(size_t len, char c) const {
    if(size() >= len) return Str(*this);
    Str padding(len - size(), c);
    return *this + padding;
  }

  // ========== Path Operations ==========

  [[nodiscard]] Str get_base_dir() const {
    size_t pos = rfind("/");
    if(pos == npos) {
#ifdef _WIN32
      pos = rfind("\\");
      if(pos == npos) {
        return Str(".");
      }
#else
      return Str(".");
#endif
    }
    return substr(0, pos);
  }

  [[nodiscard]] Str get_file() const {
    size_t pos = rfind("/");
    if(pos == npos) {
#ifdef _WIN32
      pos = rfind("\\");
      if(pos == npos) {
        return Str(*this);
      }
#else
      return Str(*this);
#endif
    }
    return substr(pos + 1);
  }

  [[nodiscard]] Str get_extension() const {
    size_t pos = rfind(".");
    if(pos == npos || pos == 0) {
      return Str();
    }
    // Check if there's a directory separator after the dot
    size_t sep_pos = rfind("/");
#ifdef _WIN32
    size_t sep_pos2 = rfind("\\");
    if(sep_pos2 != npos && sep_pos2 > sep_pos) {
      sep_pos = sep_pos2;
    }
#endif
    if(sep_pos != npos && sep_pos > pos) {
      return Str();
    }
    return substr(pos);
  }

  [[nodiscard]] Str get_basename() const {
    Str file   = get_file();
    size_t pos = file.rfind(".");
    if(pos == npos) {
      return file;
    }
    return file.substr(0, pos);
  }

  [[nodiscard]] bool is_absolute_path() const {
    if(empty()) return false;
#ifdef _WIN32
    if(size() >= 2 && data()[1] == ':') {
      return true;
    }
#endif
    return data()[0] == '/';
  }

  [[nodiscard]] bool is_relative_path() const { return !is_absolute_path(); }

  [[nodiscard]] Str path_join(std::string_view other) const {
    if(empty()) return Str(other);
    if(other.empty()) return Str(*this);

    Str result(*this);
    if(result.back() != '/' && result.back() != '\\') {
      result += '/';
    }

    // Skip leading separators in other
    size_t start = 0;
    while(start < other.size() && (other[start] == '/' || other[start] == '\\')) {
      start++;
    }

    result += other.substr(start);
    return result;
  }

  [[nodiscard]] Str path_join(const char* other) const { return path_join(std::string_view(other ? other : "")); }

  [[nodiscard]] Str operator/(std::string_view other) const { return path_join(other); }

  [[nodiscard]] Str operator/(const char* other) const { return path_join(other); }

  [[nodiscard]] Str simplify_path() const {
    auto parts = split("/", false);

    // Process . and ..
    std::vector<Str> simplified;
    for(const auto& part : parts) {
      if(part == "..") {
        if(!simplified.empty() && simplified.back() != "..") {
          simplified.pop_back();
        } else if(is_relative_path()) {
          simplified.push_back(Str(".."));
        }
      } else if(part != "." && !part.empty()) {
        simplified.push_back(part);
      }
    }

    Str result;
    if(is_absolute_path()) {
      result = "/";
    }
    for(size_t i = 0; i < simplified.size(); ++i) {
      if(i > 0 || is_absolute_path()) {
        result += "/";
      }
      result += simplified[i];
    }

    return result.empty() ? Str(".") : result;
  }

  // ========== Case Transformations (Godot API) ==========

  [[nodiscard]] Str to_camel_case() const {
    auto parts = split("_-", false);
    if(parts.empty()) return Str();

    Str result = parts[0].to_lower();
    for(size_t i = 1; i < parts.size(); ++i) {
      result += parts[i].capitalize();
    }
    return result;
  }

  [[nodiscard]] Str to_pascal_case() const {
    auto parts = split("_-", false);
    if(parts.empty()) return Str();

    Str result;
    for(const auto& part : parts) {
      result += part.capitalize();
    }
    return result;
  }

  [[nodiscard]] Str to_snake_case() const {
    Str result;
    for(size_t i = 0; i < size(); ++i) {
      char c = data()[i];
      if(std::isupper(static_cast<unsigned char>(c))) {
        if(i > 0 && data()[i - 1] != '_') {
          result += '_';
        }
        result += std::tolower(static_cast<unsigned char>(c));
      } else {
        result += c;
      }
    }
    return result;
  }

  // ========== Utility ==========

  [[nodiscard]] size_t hash() const noexcept {
    // FNV-1a
    size_t h = 14695981039346656037ULL;
    for(size_t i = 0; i < size(); ++i) {
      h ^= static_cast<unsigned char>(data()[i]);
      h *= 1099511628211ULL;
    }
    return h;
  }

  [[nodiscard]] double similarity(const Str& other) const noexcept {
    // Bigram-based similarity
    if(empty() && other.empty()) return 1.0;
    if(empty() || other.empty()) return 0.0;

    // Count bigrams
    std::vector<std::pair<char, char>> bigrams1, bigrams2;
    for(size_t i = 0; i + 1 < size(); ++i) {
      bigrams1.push_back({data()[i], data()[i + 1]});
    }
    for(size_t i = 0; i + 1 < other.size(); ++i) {
      bigrams2.push_back({other.data()[i], other.data()[i + 1]});
    }

    // Count matching bigrams
    size_t matches = 0;
    for(const auto& b1 : bigrams1) {
      for(size_t j = 0; j < bigrams2.size(); ++j) {
        if(bigrams2[j] == b1) {
          matches++;
          bigrams2.erase(bigrams2.begin() + j);
          break;
        }
      }
    }

    return 2.0 * matches / (bigrams1.size() + bigrams2.size());
  }

private:
  // ========== Internal Helpers ==========

  bool is_sso() const noexcept { return storage_.sso.remaining != 0xFF; }

  void init_sso_empty() noexcept {
    storage_.sso.buf[0]    = '\0';
    storage_.sso.remaining = SSO_CAPACITY;
  }

  void init_from_buffer(const char* data, size_t len) {
    if(len <= SSO_CAPACITY) {
      std::memcpy(storage_.sso.buf, data, len);
      storage_.sso.buf[len]  = '\0';
      storage_.sso.remaining = SSO_CAPACITY - len;
    } else {
      allocate_heap(len);
      std::memcpy(storage_.heap.ptr, data, len);
      storage_.heap.ptr[len] = '\0';
      storage_.heap.size     = len;
    }
  }

  void allocate_heap(size_t needed_capacity) {
    size_t new_capacity = std::max(needed_capacity, SSO_CAPACITY * 2);
    char* ptr           = static_cast<char*>(std::malloc(new_capacity + 1));
    if(!ptr) {
      throw std::bad_alloc();
    }
    storage_.heap.ptr      = ptr;
    storage_.heap.capacity = new_capacity;
    storage_.heap.size     = 0;
  }

  static bool is_char_in(char c, std::string_view charset) noexcept { return charset.find(c) != std::string_view::npos; }
};

// Standard Library Integration is done at namespace scope (below)

} // namespace cutil

// ============================================================================
// Standard Library Integration
// ============================================================================

inline std::ostream& operator<<(std::ostream& os, const cutil::Str& s) {
  os << s.c_str();
  return os;
}

inline std::istream& operator>>(std::istream& is, cutil::Str& s) {
  std::string tmp;
  is >> tmp;
  s = cutil::Str(tmp);
  return is;
}

// Hash specialization
namespace std {
template <> struct hash<cutil::Str> {
  size_t operator()(const cutil::Str& s) const noexcept { return s.hash(); }
};
} // namespace std
