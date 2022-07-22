#pragma once

#include "pool.hpp"
#include <assert.h>
#include <cmath> //for sqrt
#include <iostream>
#include <limits>
#include <stdlib.h>
#include <vector>

namespace Magi {

inline bool isPow2(unsigned i) {
  return ((i - 1) & i) == 0;
}

inline unsigned log2(unsigned n) {
  assert(n != 0);
  assert(isPow2(n));
  unsigned log = 0;
  while(true) {
    n >>= 1;
    if(n == 0) {
      break;
    }
    log++;
  }
  return log;
}

enum class SearchMethod {
  Nearest,
  Sphere,
  Volume,
};

template <typename T, int DEPTH = 10, int LEAF_MAX_N = 5> class Octree2 {
public:
  class Node {};

  struct Branch : public Node {
    Node* children[8];
    Branch() : children() {}
    void* operator new(size_t num_bytes, Pool<Branch>* mem) {
      assert(sizeof(Branch) == num_bytes);
      return mem->alloc_item();
    }
    Node* operator[](unsigned i) const {
      assert(i < 8);
      return children[i];
    }
  };

  struct Leaves : public Node {
    T* values[LEAF_MAX_N];
    int size;
    inline Leaves() : values(), size(0) {}
    inline Leaves(const T* v) {
      values[0] = const_cast<T*>(v);
      size = 1;
    }
    void* operator new(size_t num_bytes, Pool<Leaves>* mem) { /* assert(this->size < LEAF_MAX_N); */
      assert(sizeof(Leaves) == num_bytes);
      return mem->alloc_item();
    }
    inline T* operator[](unsigned i) const {
      assert(i < LEAF_MAX_N);
      return values[i];
    }
    inline bool hasCapacity() { return size < LEAF_MAX_N; }
    inline bool append(const T* v) {
      if(size >= LEAF_MAX_N) {
        return false;
      }
      values[size] = const_cast<T*>(v);
      size++;
    }
    // void set_ptr(T *value){ }
    // T& value(int ids) {return m_value;}
    // void value(T& v) {m_value = v;}
  };

  typedef typename Pool<Leaves>::iterator leaves_iterator;
  typedef typename Pool<Branch>::iterator branches_iterator;

  // private:
  Node* m_root;
  unsigned leaf_count;
  unsigned branch_count;

  Pool<Leaves> leaves_pool;
  Pool<Branch> branch_pool;
  int xmin, xmax, ymin, ymax, zmin, zmax;

  struct _int3 {
    int x, y, z;
    _int3() { x = y = z = 0; }
    _int3(int x, int y, int z) {
      this->x = x;
      this->y = y;
      this->z = z;
    }
    _int3 operator+=(const _int3& other) {
      x += other.x;
      y += other.y;
      z += other.z;
      return *this;
    }
    _int3 operator-=(const _int3& other) {
      x -= other.x;
      y -= other.y;
      z += other.z;
      return *this;
    }
    _int3 operator-(const _int3& other) const { return _int3(x - other.x, y - other.y, z - other.z); }
    _int3 operator+(const _int3& other) const { return _int3(x + other.x, y + other.y, z + other.z); }
    _int3 operator+(const int& v) const { return _int3(x + v, y + v, z + v); }
    _int3 operator-(const int& v) const { return _int3(x - v, y - v, z - v); }
    bool operator==(const _int3& other) const { return x == other.x && y == other.y; }
  };

  struct Search {
    // 探したい場所（ここから最も近い点を探す
    _int3 target_pos;
    std::vector<Leaves*> nn_leaves;
    uint64_t nn_sq_distance;
    _int3 nearest;            // 現在最も近い点
    _int3 bbox_min, bbox_max; // target_pos からnn_sq_distance + 1 離れた領域。これの中に点群が入っていればNearestになるかも
    SearchMethod method;
    uint64_t r_sq; //球体中の一覧を取得するときに、この半径**2いないの点群を取得する

    Search() {}
    Search(const _int3 tar, const SearchMethod meth) {
      method = meth;
      target_pos = tar;
      nn_sq_distance = std::numeric_limits<uint64_t>::max();
    }

    Search(const int x, const int y, const int z, const SearchMethod meth) {
      method = meth;
      target_pos = _int3(x, y, z);
      nn_sq_distance = std::numeric_limits<uint64_t>::max();
    }

    // 指定されたLeaveが目的に合ったものかどうかを調べ、もし条件に適している場合はpush_backする
    inline void update_nearest(Leaves* l, const uint64_t x, const uint64_t y, const uint64_t z) {
      const long dx = target_pos.x - x;
      const long dy = target_pos.y - y;
      const long dz = target_pos.z - z;
      const unsigned sq_distance = (dx * dx) + (dy * dy) + (dz * dz);

      if(sq_distance < nn_sq_distance) {
        if(nn_leaves.size() == 0) nn_leaves.resize(1);
        nn_leaves[0] = l;
        nn_sq_distance = sq_distance;
        nearest = _int3(x, y, z);

        const int r = std::sqrt(sq_distance) + 1.0;
        bbox_min = target_pos - r;
        bbox_max = target_pos + r;
      }
    }

    inline void update_volume(Leaves* l, const uint64_t x, const uint64_t y, const uint64_t z) {
      const long dx = target_pos.x - x;
      const long dy = target_pos.y - y;
      const long dz = target_pos.z - z;
      const unsigned sq_distance = (dx * dx) + (dy * dy) + (dz * dz);
      if(sq_distance < r_sq) {
        nn_leaves.push_back(l);
      }
    }

    // Check if any point of the position is in the search box.
    // bbox_min,  bbox_maxで示された領域ないに入っているか？
    // 一部が含まれているか
    inline bool check_branch(const int x, const int y, const int z, const int w) const {
      // if (bbox_min.x > w + x) {std::cout << "A#" << std::endl; return false;}
      // if (bbox_max.x <     x) {std::cout << "B" << std::endl; return false;}
      // if (bbox_min.y > w + y) {std::cout << "C#" << std::endl; return false;}
      // if (bbox_max.y <     y) {std::cout << "D#" << std::endl; return false;}
      // if (bbox_min.z > w + z) {std::cout << "E#" << std::endl; return false;}
      // if (bbox_max.z <     z) {std::cout << "F#" << std::endl; return false;}
      // return true;   // 衝突！！
      return !(bbox_max.x < x || bbox_min.x > x + w || bbox_max.y < y || bbox_min.y > y + w || bbox_max.z < z || bbox_min.z > z + w);
    }

    void search_nearest(const Branch* b, _int3 t, const unsigned size) {
      assert(b != nullptr);
      assert(isPow2(size));
      const unsigned start_i = !!(t.x & size) * 1 + !!(t.y & size) * 2 + !!(t.z & size) * 4;
      for(unsigned i = start_i; i < (start_i + 8); ++i) {
        Node* n = b->children[i & 7];
        if(n == nullptr) {
          continue;
        }

// set bits in mask to 1 (not 0, no toggle) if i has bit [1,2,3] set
#define set1_if_bit1(value, i, mask) ((i & 1) ? ((value) | (mask)) : (value))
#define set1_if_bit2(value, i, mask) ((i & 2) ? ((value) | (mask)) : (value))
#define set1_if_bit3(value, i, mask) ((i & 4) ? ((value) | (mask)) : (value))
        // const unsigned child_x = set1_if_bit1(t.x, i, size);
        // const unsigned child_y = set1_if_bit2(t.y, i, size);
        // const unsigned child_z = set1_if_bit3(t.z, i, size);

        const unsigned child_x = (i & 1) ? (t.x | size) : t.x;
        const unsigned child_y = (i & 2) ? (t.y | size) : t.y;
        const unsigned child_z = (i & 4) ? (t.z | size) : t.z;

        if(size == 1) {
          update_nearest(reinterpret_cast<Leaves*>(n), child_x, child_y, child_z);
        } else if(check_branch(child_x, child_y, child_z, size)) {
          search_nearest(reinterpret_cast<Branch*>(n), _int3(child_x, child_y, child_z), (size / 2));
        }
      }
    }

    void search_volume(const Branch* b, _int3 t, const unsigned size) {
      assert(b != nullptr);
      assert(isPow2(size));
      const unsigned start_i = !!(t.x & size) * 1 + !!(t.y & size) * 2 + !!(t.z & size) * 4;
      for(unsigned i = start_i; i < (start_i + 8); ++i) {
        Node* n = b->children[i & 7];
        if(n == nullptr) {
          continue;
        }
        const unsigned child_x = (i & 1) ? (t.x | size) : t.x;
        const unsigned child_y = (i & 2) ? (t.y | size) : t.y;
        const unsigned child_z = (i & 4) ? (t.z | size) : t.z;
        if(size == 1) {
          update_volume(reinterpret_cast<Leaves*>(n), child_x, child_y, child_z);
        } else if(check_branch(child_x, child_y, child_z, size)) {
          search_volume(reinterpret_cast<Branch*>(n), _int3(child_x, child_y, child_z), (size / 2));
        }
      }
    }
  };

public:
  Octree2() {}
  ~Octree2() {}

  inline unsigned resolution() const {
    return 1 << DEPTH;
  }
  inline unsigned depth() const {
    return DEPTH;
  }
  inline unsigned capacity() const {
    auto w = resolution();
    return w * w * w;
  }
  inline void setVolume(int xmin, int xmax, int ymin, int ymax, int zmin, int zmax) {
    this->xmin = xmin;
    this->xmax = xmax;
    this->ymin = ymin;
    this->ymax = ymax;
    this->zmin = zmin;
    this->zmax = zmax;
  }
  inline Branch* root() const {
    return reinterpret_cast<Branch*>(m_root);
  }
  inline leaves_iterator leaf_begin() {
    return leaves_pool.begin();
  }
  inline leaves_iterator leaf_end() {
    return leaves_pool.end();
  }
  inline branches_iterator branch_begin() {
    return branch_pool.begin();
  }
  inline branches_iterator branch_end() {
    return branch_pool.end();
  }
  inline unsigned countLeaves() {
    return leaves_pool.size();
  }
  inline unsigned countBranches() {
    return branch_pool.size();
  }

  // 背景削除に用いる
  // 与えられた点(x, y, z)を含むBox(Depth＝MAX)が存在するか
  inline bool hasNode(int x, int y, int z) {
    assert(xmin < x && x < xmax);
    assert(ymin < y && y < ymax);
    assert(zmin < z && z < zmax);
    assert(root() != nullptr);

    _int3 bin((x - xmin) * resolution() / (xmax - xmin), (y - ymin) * resolution() / (ymax - ymin), (z - zmin) * resolution() / (zmax - zmin));
    auto b = root();
    for(int _dw = resolution() / 2; _dw > 1; dw << 1) {
      const unsigned idx = !!(bin.x & _dw) * 1 + !!(bin.y & _dw) * 2 + !!(bin.z & _dw) * 4;
      b = b->children[i & 7];
      if(n == nullptr) {
        return false;
      }
    }
    return true;
  }

  // Leaf* at(const unsigned x, const unsigned y, const unsigned z) const {
  // 	Node* n = m_root;
  // 	unsigned size = width();
  // 	assert(x <= size);
  // 	assert(y <= size);
  // 	assert(z <= size);
  // 	while(size != 1 && n){
  // 		size /= 2;
  // 		n = reinterpret_cast<Branch*>(n)->children[
  // 			!!(x & size) * 1 + !!(y & size) * 2 + !!(z & size) * 4
  // 		];
  // 	}
  // 	return reinterpret_cast<Leaf*>(n);
  // }

  bool insert(const double x, const double y, const double z, const T* value) {
    assert(xmin <= x && x <= xmax);
    assert(ymin <= y && y <= ymax);
    assert(xmax - xmin > 0);
    assert(ymax - ymin > 0);
    assert(zmax - zmin > 0);

    Node** n = &m_root;
    uint16_t depth = DEPTH;

    _int3 bin((x - xmin) * resolution() / (xmax - xmin), (y - ymin) * resolution() / (ymax - ymin), (z - zmin) * resolution() / (zmax - zmin));

    while(depth) {
      if(*n == nullptr) {
        *n = new(&branch_pool) Branch();
        ++branch_count;
      } else {
        --depth;
        const unsigned size = (1 << depth);
        unsigned i = ((bin.x & size) ? 1 : 0) + ((bin.y & size) ? 2 : 0) + ((bin.z & size) ? 4 : 0);
        n = &reinterpret_cast<Branch*>(*n)->children[i];
      }
    }
    bool result = true;
    if(*n == nullptr) {
      assert(depth == 0);
      *n = new(&leaves_pool) Leaves(value);
      ++leaf_count;
    } else {
      result = reinterpret_cast<Leaves*>(*n)->append(value);
    }
    // return reinterpret_cast<Leaf*>(*n);
    return result;
  }

  std::vector<T*> findNearest(double x, double y, double z, SearchMethod method, double r = 0.0) {
    assert(xmin < x && x < xmax);
    assert(ymin < y && y < ymax);
    assert(zmin < z && z < zmax);

    _int3 bin((x - xmin) * resolution() / (xmax - xmin), (y - ymin) * resolution() / (ymax - ymin), (z - zmin) * resolution() / (zmax - zmin));
    int rb = r * resolution() / (xmax - xmin);

    if(root() == nullptr) {
      return std::vector<T*>();
    }

    Search searcher(x, y, z, method);

    switch(method) {
      case SearchMethod::Nearest: {
        // searcher.bbox_min = _int3(xmin, ymin, zmin);
        // searcher.bbox_max = _int3(xmax, ymax, zmax);
        searcher.bbox_min = bin - resolution() / 20;
        searcher.bbox_max = bin + resolution() / 20;
        searcher.search_nearest(root(), bin, resolution() / 2);
        break;
      }
      case SearchMethod::Sphere: {
        searcher.bbox_min = bin - rb;
        searcher.bbox_max = bin + rb;
        searcher.r_sq = rb * rb;
        searcher.search_volume(root(), bin, resolution() / 2);
        break;
      }
    }

    std::vector<T*> res;
    std::cout << "sssss" << searcher.nn_leaves.size() << std::endl;
    for(auto l : searcher.nn_leaves) {
      std::cout << "l->size" << l->size << std::endl;
      for(int i = 0; i < l->size; i++) res.push_back((*l)[i]);
    }
    return res;
  }


#ifdef VKUI_ENGINE_DRAWING_WIDGET
#include <engine.hpp>
  void _drawOctreeBranches(const Branch* b, _int3 t, const unsigned size, vkUI::Engine::uiWIndow* wnd) {
    assert(b != nullptr);
    assert(isPow2(size));
    for(unsigned i = 0; i < 8; i++) {
      auto n = b->children[i & 7];
      if(n == nullptr) {
        continue;
      }

      const unsigned child_x = (i & 1) ? (t.x | size) : t.x;
      const unsigned child_y = (i & 2) ? (t.y | size) : t.y;
      const unsigned child_z = (i & 4) ? (t.z | size) : t.z;

      const int x = child_x * (xmax - xmin) / resolution() + xmin;
      const int y = child_y * (ymax - ymin) / resolution() + ymin;
      const int z = child_z * (zmax - zmin) / resolution() + zmin;
      wnd->DrawCube()


        if(size == 1) {
        update_nearest(reinterpret_cast<Leaves*>(n), child_x, child_y, child_z);
      }
      else if(check_branch(child_x, child_y, child_z, size)) {
        search_nearest(reinterpret_cast<Branch*>(n), _int3(child_x, child_y, child_z), (size / 2));
      }
    }
  }
#endif
};

} // namespace Magi
