// Generates a randomized Epson RTC-4513 test scenario together with the
// responses expected from the bsnes reference implementation.
//
// Output (ops_rtc.txt, for tb_rtc.v):
//   W a dd             write $484a
//   R a dd             read $484a, expect dd
//   T nnnnnnnn         run nnnnnnnn steps of the 2.097152 MHz clock
//   S tttttttttttttt   set time from the MCU (FPGA_CMD_RTCSET payload,
//                      weekday in the thousands-of-year nibble)
//   L ssssssssssssssss load chip state (EpsonRTC::load, bytes 0-7, byte 0 last)
//   G ssssssssssssssss expect chip state (EpsonRTC::save, bytes 0-7)
//
// usage: gen_rtc [seed] [blocks]
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <initializer_list>
#include <random>
using std::min;
// EpsonRTC::load() advances the clock by the time elapsed since the save;
// make "now" its own parsed timestamp so only the state transfer is tested.
// (load() assembles the timestamp with int shifts of up to 56 bits, which
// clang -O2 turns into garbage)
#define time(x) (timestamp)
#include "epsonrtc.cpp"
#undef time

using namespace SuperFamicom;

static std::mt19937 rng;
static FILE* ops;

static uint rnd(uint n) { return rng() % n; }

static void wr(uint a, uint d) {
  epsonrtc.write(0x4840 | a, d);
  fprintf(ops, "W %x %02x\n", a, d);
}

static void rd(uint a) {
  uint d = epsonrtc.read(0x4840 | a, 0x00);
  fprintf(ops, "R %x %02x\n", a, d);
}

static void run(uint n) {
  for(uint i = 0; i < n; i++) epsonrtc.main();
  fprintf(ops, "T %08x\n", n);
}

static uint bcd(uint v) { return (v / 10) << 4 | (v % 10); }

static void sync() {
  // a random date/time in 2000-2099
  tm t = {};
  t.tm_year = 100 + rnd(100);
  t.tm_mon = rnd(12);
  t.tm_mday = 1 + rnd(28);
  t.tm_hour = rnd(24);
  t.tm_min = rnd(60);
  t.tm_sec = rnd(60);
  time_t ts = timegm(&t);
  gmtime_r(&ts, &t);  // fills tm_wday
  epsonrtc.synchronize(ts);
  uint64_t v = (uint64_t)t.tm_wday << 52 | (uint64_t)bcd(t.tm_year % 100) << 40
             | (uint64_t)bcd(t.tm_mon + 1) << 32 | (uint64_t)bcd(t.tm_mday) << 24
             | bcd(t.tm_hour) << 16 | bcd(t.tm_min) << 8 | bcd(t.tm_sec);
  fprintf(ops, "S %014llx\n", (unsigned long long)v);
}

static uint64_t pack(const uint8_t* d) {
  uint64_t v = 0;
  for(int i = 7; i >= 0; i--) v = v << 8 | d[i];
  return v;
}

static void get_state() {
  uint8_t d[16];
  epsonrtc.save(d);
  fprintf(ops, "G %016llx\n", (unsigned long long)pack(d));
}

static void load_state() {
  uint8_t d[16] = {};
  for(int i = 0; i < 8; i++) d[i] = rng();
  epsonrtc.load(d);
  fprintf(ops, "L %016llx\n", (unsigned long long)pack(d));
}

// wait for the ready flag the way a program would
static void settle() { run(8); rd(2); }

static void select() { wr(0, 1); rd(0); }

static void write_regs(uint first, const uint* d, uint n) {
  select();
  wr(1, 0x3); settle();
  wr(1, first); settle();
  for(uint i = 0; i < n; i++) { wr(1, d[i]); settle(); }
  wr(0, 0);
}

static void read_all() {
  select();
  wr(1, 0xc); settle();
  wr(1, 0); settle();
  for(uint i = 0; i < 16; i++) { rd(1); settle(); }
  wr(0, 0);
}

// random (sometimes invalid) time near a rollover, random control state,
// then let one or two seconds pass
static uint pick(std::initializer_list<uint> edge, uint mask) {
  if(rnd(4) == 0) return rnd(16) & mask;
  return edge.begin()[rnd(edge.size())] & mask;
}

static void test_tick() {
  uint d[16];
  // digits: rollover values plus the invalid BCD ones the chip counts oddly
  d[0] = pick({9, 9, 10, 12, 13, 15}, 15);
  d[1] = pick({5, 5, 6, 7}, 15);
  d[2] = pick({9, 9, 10, 12, 13, 15}, 15);
  d[3] = pick({5, 5, 6, 7}, 15);
  d[4] = pick({1, 2, 3, 4, 9, 11, 12, 13, 15}, 15);
  d[5] = pick({0, 1, 2, 3, 4, 5, 6, 7}, 15);
  d[6] = pick({0, 1, 8, 9, 10, 12, 13, 14, 15}, 15);
  d[7] = pick({2, 3, 3, 0}, 15);
  d[8] = pick({1, 2, 9, 0, 10, 12, 13, 15}, 15);
  d[9] = pick({0, 1, 1}, 15);
  d[10] = pick({9, 12, 13, 15, 0, 2, 4}, 15);
  d[11] = pick({9, 12, 13, 15, 0, 1}, 15);
  d[12] = rnd(16);
  d[13] = (rnd(16) & (rnd(4) ? 0xc : 0xf)) | (rnd(5) ? 2 : 0);   // mostly calendar on, not held
  d[14] = rnd(16);
  d[15] = rnd(16) & (rnd(4) ? 0x4 : 0xf);                        // mostly running
  if(rnd(2)) { wr(0, 1); wr(1, 0x3); settle(); wr(1, 15); settle(); wr(1, d[15]); settle(); wr(0, 0); }
  write_regs(0, d, rnd(4) ? 15 : 16);
  run(rnd(4) ? 2097152 + rnd(65536) : rnd(70000));
  read_all();
  if(rnd(3) == 0) {
    // release a hold after a second has passed
    uint h = d[13] | 1;
    write_regs(13, &h, 1);
    run(2097152);
    h &= ~1u;
    write_regs(13, &h, 1);
    run(64);
    read_all();
  }
}

// protocol misuse: wrong chip select, invalid modes, accesses while busy
static void test_protocol() {
  uint n = 4 + rnd(30);
  for(uint i = 0; i < n; i++) {
    switch(rnd(8)) {
    case 0: wr(0, rnd(16)); break;
    case 1: wr(1, rnd(16)); break;
    case 2: wr(1, rnd(2) ? 0x3 : 0xc); break;
    case 3: rd(0); break;
    case 4: rd(1); break;
    case 5: rd(2); break;
    case 6: run(rnd(12)); break;
    case 7: wr(2, rnd(256)); break;
    }
  }
  wr(0, 0);
  read_all();
}

int main(int argc, char** argv) {
  uint seed = argc > 1 ? atoi(argv[1]) : 1;
  uint blocks = argc > 2 ? atoi(argv[2]) : 40;
  rng.seed(seed);
  setenv("TZ", "UTC", 1);
  tzset();

  epsonrtc.initialize();
  epsonrtc.power();
  ops = fopen("ops_rtc.txt", "w");
  get_state();
  read_all();
  for(uint i = 0; i < blocks; i++) {
    switch(rnd(4)) {
    case 0: sync(); read_all(); break;
    case 1: test_protocol(); break;
    default: test_tick(); break;
    }
    get_state();
    if(rnd(3) == 0) { load_state(); get_state(); read_all(); run(rnd(2) ? 2097152 : 1000); get_state(); }
  }
  fprintf(ops, "E\n");
  fclose(ops);
  return 0;
}
