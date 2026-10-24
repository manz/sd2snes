// Minimal stand-ins for the bsnes infrastructure used by
// sfc/coprocessor/{spc7110,epsonrtc}/*.cpp, so the reference models build
// standalone.
#pragma once
#include <cstdint>
#include <vector>

using uint = unsigned int;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;
using int16 = int16_t;
using int32 = int32_t;

template<int N> struct Natural {
  uint32_t v = 0;
  Natural() = default;
  Natural(uint32_t x) : v(x & ((1u << N) - 1)) {}
  operator uint32_t() const { return v; }
  Natural& operator=(uint32_t x) { v = x & ((1u << N) - 1); return *this; }
  Natural& operator|=(uint32_t x) { return *this = v | x; }
  Natural& operator&=(uint32_t x) { return *this = v & x; }
  Natural& operator^=(uint32_t x) { return *this = v ^ x; }
  Natural operator++(int) { Natural t = *this; *this = v + 1; return t; }
  Natural& operator++() { return *this = v + 1; }
};
using uint1 = Natural<1>;
using uint3 = Natural<3>;
using uint4 = Natural<4>;
using uint21 = Natural<21>;
using uint2 = Natural<2>;
using uint7 = Natural<7>;
using uint23 = Natural<23>;

struct serializer {
  template<typename T> void integer(T&) {}
  template<typename T> void array(T&) {}
};

struct Thread {
  int64_t clock = 0;
  void create(void (*)(), uint) {}
  void serialize(serializer&) {}
};

struct Scheduler {
  void resume(int) {}
  void synchronize() {}
};

struct CPU {
  int thread = 0;
  uint frequency = 21477272;
  void synchronizeCoprocessors() {}
};

struct Bus {
  // power-of-two sizes only
  static uint mirror(uint addr, uint size) { return size ? addr & (size - 1) : 0; }
};

struct ReadableMemory {
  std::vector<uint8> data;
  uint size() const { return data.size(); }
  uint8 read(uint addr) const { return data[addr]; }
  void reset() { data.clear(); }
};

struct WritableMemory : ReadableMemory {
  void write(uint addr, uint8 v) { data[addr] = v; }
};

struct range {
  uint n;
  explicit range(uint n) : n(n) {}
  struct it {
    uint i;
    uint operator*() const { return i; }
    it& operator++() { ++i; return *this; }
    bool operator!=(const it& o) const { return i != o.i; }
  };
  it begin() const { return {0}; }
  it end() const { return {n}; }
};

namespace SuperFamicom {
  inline Scheduler scheduler;
  inline CPU cpu;
  inline Bus bus;
#define private public
#if __has_include("spc7110.hpp")
#include "spc7110.hpp"
#endif
#if __has_include("epsonrtc.hpp")
#include "epsonrtc.hpp"
#endif
#undef private
}
