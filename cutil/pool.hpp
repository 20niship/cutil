#pragma once

#include <assert.h>
#include <cstring>
#include <ctime>
#include <iostream>
#include <stdlib.h>

template <typename T, int LEN = 1024> struct Pool {
  struct Chunk {
    T items[LEN];
    Chunk* next;
  };
  class iterator : public std::iterator<std::input_iterator_tag, T> {
  public:
    Chunk* chunk;
    unsigned pos;
    iterator() {}
    ~iterator() {}
    iterator(Chunk* chunk, unsigned pos) : chunk(chunk), pos(pos) {}
    inline iterator(const iterator& it) : chunk(it.chunk), pos(it.pos) {}
    inline iterator& operator++() {
      ++pos;
      if(pos == LEN && chunk->next) {
        pos = 0;
        chunk = chunk->next;
      }
      return *this;
    }
    inline iterator operator++(int) {
      iterator tmp(*this);
      operator++();
      return tmp;
    }
    inline bool operator==(const iterator& rhs) { return chunk == rhs.chunk && pos == rhs.pos; }
    inline bool operator!=(const iterator& rhs) { return !(chunk == rhs.chunk && pos == rhs.pos); }
    inline T& operator*() { return chunk->items[pos]; }
    inline T& operator->() { return chunk->items[pos]; }
  };

private:
  Chunk* beg_m;
  Chunk* end_m;
  unsigned pos_m;
  unsigned chunk_num;

public:
  void init() {
    beg_m = (Chunk*)malloc(sizeof(Chunk));
    beg_m->next = NULL;
    end_m = beg_m;
    chunk_num = 1;
  }

  void clear() {
    Chunk* cur = beg_m;
    chunk_num = 1;
    while(cur != end_m) {
      Chunk* tmp = cur->next;
      free(cur);
      cur = tmp;
    }
  }

  Pool() : pos_m(0) { init(); }
  ~Pool() { clear(); }
  unsigned size() { return LEN * (chunk_num - 1) + pos_m; }

  unsigned countElements() {
    unsigned count = 0;
    Chunk* cur = beg_m;
    while(cur != end_m) {
      cur = cur->next;
      count += LEN;
    }
    return count + pos_m;
  }

  inline iterator begin() { return iterator(beg_m, 0); }
  inline iterator end() { return iterator(end_m, pos_m); }

  inline void* alloc_item() {
    assert(pos_m <= LEN);
    if(pos_m == LEN) {
      Chunk* chunk = (Chunk*)malloc(sizeof(Chunk));
      chunk_num++;
      assert(chunk != NULL);
      chunk->next = NULL;
      end_m->next = chunk;
      end_m = chunk;
      pos_m = 0;
    }
    T* mem = &end_m->items[pos_m];
    ++pos_m;
    return mem;
  }
};
