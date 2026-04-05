#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <vector>

#include <cutil/ref.hpp>

namespace cutil {

// チャンク連結リスト構造のオブジェクトプール。
// - N: 1チャンクあたりのスロット数（テンプレート引数）
// - create() で Ref<T> を返す。Ref が全て解放されるとスロットがプールへ返却される。
// - 削除済みスロットはフリーリストで再利用される。
// - イテレーターは alive なオブジェクトのみ yield する。
//
// 注意: ObjectPool はそこから生成した全 Ref<T> より長生きしなければならない。
template <typename T, size_t N = 64> class ObjectPool {
public:
  // Slot を public にして外部からアクセス可能に
  struct Slot {
    alignas(T) unsigned char storage[sizeof(T)];
    bool alive = false;
  };

private:
  struct Chunk {
    std::array<Slot, N> slots;
    Chunk* next = nullptr;
  };

public:
  // コピー・ムーブ禁止（デリーターが this を捕捉するため）
  ObjectPool()                             = default;
  ObjectPool(const ObjectPool&)            = delete;
  ObjectPool& operator=(const ObjectPool&) = delete;
  ObjectPool(ObjectPool&&)                 = delete;
  ObjectPool& operator=(ObjectPool&&)      = delete;

  ~ObjectPool() {
    assert(alive_count_ == 0 && "ObjectPool が破棄される前に全 Ref<T> を解放してください");
    Chunk* cur = head_;
    while(cur) {
      Chunk* next = cur->next;
      delete cur;
      cur = next;
    }
  }

  // オブジェクトを生成して Ref<T> で返す。
  // Ref が最後に解放された時点でスロットがプールへ返却される。
  template <typename... Args> Ref<T> create(Args&&... args) {
    Slot* slot  = allocate_slot();
    T* obj      = new(slot->storage) T(std::forward<Args>(args)...);
    slot->alive = true;
    ++alive_count_;

    auto deleter = [this, slot](T* p) {
      p->~T();
      slot->alive = false;
      --alive_count_;
      free_list_.push_back(slot);
    };

    return Ref<T>(obj, deleter);
  }

  // alive なオブジェクト数を返す。
  size_t size() const { return alive_count_; }

  // 全 alive オブジェクトを破棄し、スロットをフリーリストへ返却する。
  // チャンクのメモリ自体は保持する。
  void clear() {
    Chunk* cur = head_;
    while(cur) {
      size_t limit = (cur == end_) ? end_pos_ : N;
      for(size_t i = 0; i < limit; ++i) {
        if(cur->slots[i].alive) {
          reinterpret_cast<T*>(cur->slots[i].storage)->~T();
          cur->slots[i].alive = false;
        }
      }
      cur = cur->next;
    }
    free_list_.clear();
    alive_count_ = 0;
    // end_, end_pos_ はそのまま（チャンクは保持）
  }

  // ---------- Iterator ----------

  class iterator {
  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type        = T;
    using difference_type   = std::ptrdiff_t;
    using pointer           = T*;
    using reference         = T&;

    iterator() : chunk_(nullptr), idx_(0), end_chunk_(nullptr), end_pos_(0) {}

    iterator(Chunk* chunk, size_t idx, Chunk* end_chunk, size_t end_pos) : chunk_(chunk), idx_(idx), end_chunk_(end_chunk), end_pos_(end_pos) { advance_to_alive(); }

    reference operator*() const { return *reinterpret_cast<T*>(chunk_->slots[idx_].storage); }

    pointer operator->() const { return reinterpret_cast<T*>(chunk_->slots[idx_].storage); }

    iterator& operator++() {
      ++idx_;
      advance_to_alive();
      return *this;
    }

    iterator operator++(int) {
      iterator tmp(*this);
      ++(*this);
      return tmp;
    }

    bool operator==(const iterator& o) const { return chunk_ == o.chunk_ && idx_ == o.idx_; }

    bool operator!=(const iterator& o) const { return !(*this == o); }

  private:
    Chunk* chunk_;
    size_t idx_;
    Chunk* end_chunk_;
    size_t end_pos_;

    void advance_to_alive() {
      while(chunk_) {
        size_t limit = (chunk_ == end_chunk_) ? end_pos_ : N;
        while(idx_ < limit) {
          if(chunk_->slots[idx_].alive) return;
          ++idx_;
        }
        chunk_ = chunk_->next;
        idx_   = 0;
      }
    }
  };

  iterator begin() {
    if(!head_) return end();
    return iterator(head_, 0, end_, end_pos_);
  }

  iterator end() { return iterator(nullptr, 0, nullptr, 0); }

  // 外部から allocate/deallocate を使用可能に（advanced usage用）
  Slot* allocate_slot_for_external() { return allocate_slot(); }

  void deallocate_slot_for_external(Slot* slot) {
    if(slot) {
      slot->alive = false;
      free_list_.push_back(slot);
    }
  }

private:
  Chunk* head_    = nullptr;
  Chunk* end_     = nullptr;
  size_t end_pos_ = 0;
  std::vector<Slot*> free_list_;
  size_t alive_count_ = 0;

  Slot* allocate_slot() {
    // フリーリストから再利用
    if(!free_list_.empty()) {
      Slot* s = free_list_.back();
      free_list_.pop_back();
      return s;
    }
    // 初回: チャンク確保
    if(!head_) {
      head_    = new Chunk();
      end_     = head_;
      end_pos_ = 0;
    }
    // 末尾チャンクが満杯なら新チャンク追加
    if(end_pos_ == N) {
      Chunk* chunk = new Chunk();
      end_->next   = chunk;
      end_         = chunk;
      end_pos_     = 0;
    }
    return &end_->slots[end_pos_++];
  }
};

} // namespace cutil
