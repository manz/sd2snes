// Generates a randomized SPC7110 register-level test scenario together with
// the responses expected from the bsnes reference implementation.
//
// Output:
//   mem.hex   PSRAM image: 1MB program ROM + 1MB data ROM (one byte per line)
//   ops.txt   operations for tb_spc7110.v:
//               W aa dd      write $48aa
//               R aa dd      read $48aa, expect dd
//               D            wait until $480C.7 is set
//               M aaaaaa dd  SNES ROM read at aaaaaa, expect dd
//               S nnnn       idle nnnn cycles
//
// usage: gen [seed] [blocks] [max $4807 stride + 1]
#include <cstdio>
#include <cstdlib>
#include <random>
#include "spc7110.cpp"

using namespace SuperFamicom;

static std::mt19937 rng;
static FILE* ops;

static uint rnd(uint n) { return rng() % n; }
static uint max_stride = 2;   // $4807 values used with $480B.0; games only use stride 1
                              // (3 cycles/bit on Mk2 cannot sustain DMA at stride >= 2)

static void wr(uint a, uint d) {
  spc7110.write(0x4800 | a, d);
  spc7110.main();
  fprintf(ops, "W %02x %02x\n", a, d);
}

static void rd(uint a) {
  uint d = spc7110.read(0x4800 | a, 0x00);
  spc7110.main();
  fprintf(ops, "R %02x %02x\n", a, d);
}

static void idle(uint n) { fprintf(ops, "S %04x\n", n); }

static const uint TABLE = 0x0f0000;

static void test_mul() {
  for(uint a : {0x20u, 0x21u, 0x24u}) wr(a, rnd(256));
  wr(0x2e, rnd(2));
  wr(0x25, rnd(256));
  idle(8);
  for(uint a = 0x28; a <= 0x2b; a++) rd(a);
  rd(0x2f);
}

static void test_div() {
  for(uint a = 0x20; a <= 0x23; a++) wr(a, rnd(256));
  wr(0x2e, rnd(2));
  wr(0x26, rnd(8) ? rnd(256) : 0);
  wr(0x27, rnd(8) ? rnd(256) : 0);
  idle(48);
  for(uint a = 0x28; a <= 0x2d; a++) rd(a);
  rd(0x2f);
}

static void test_dataport() {
  wr(0x11, rnd(256));
  wr(0x12, rnd(256));
  wr(0x14, rnd(256));
  wr(0x15, rnd(4) ? 0 : rnd(256));
  wr(0x16, rnd(256));
  wr(0x17, rnd(4) ? 0 : rnd(256));
  wr(0x18, rnd(128));
  wr(0x13, rnd(16));
  uint n = 4 + rnd(40);
  for(uint i = 0; i < n; i++) {
    switch(rnd(10)) {
    case 0: rd(0x1a); break;
    case 1: wr(0x14, rnd(256)); break;
    case 2: wr(0x15, rnd(4)); break;
    case 3: for(uint a = 0x11; a <= 0x18; a++) rd(a); break;
    default: rd(0x10); break;
    }
  }
}

static void test_dcu() {
  wr(0x01, TABLE & 0xff);
  wr(0x02, TABLE >> 8 & 0xff);
  wr(0x03, TABLE >> 16);
  wr(0x04, rnd(256));
  wr(0x07, rnd(max_stride));
  uint settings = rnd(4);
  wr(0x0b, settings);
  wr(0x09, rnd(256));
  wr(0x0a, rnd(256));
  wr(0x05, rnd(16));
  wr(0x06, 0);
  idle(0x200);
  if(spc7110.dcuMode != 3) fprintf(ops, "D\n");
  rd(0x0c);
  uint n = 8 + rnd(300);
  for(uint i = 0; i < n; i++) rd(0x00);
  rd(0x09);
  rd(0x0a);
}

static void test_mcu() {
  wr(0x30, rnd(256));
  for(uint a = 0x31; a <= 0x34; a++) wr(a, rnd(8));
  for(uint a = 0x30; a <= 0x34; a++) rd(a);
  for(uint i = 0; i < 32; i++) {
    uint bank = rnd(2) ? (rnd(64) | (rnd(2) << 7)) : (0xc0 + rnd(64));
    uint off = rnd(65536);
    if(bank < 0xc0) off |= 0x8000;
    uint snes = bank << 16 | off;
    uint d = spc7110.mcuromRead(snes & 0x3fffff, 0x00);
    fprintf(ops, "M %06x %02x\n", snes, d);
  }
}

int main(int argc, char** argv) {
  uint seed = argc > 1 ? atoi(argv[1]) : 1;
  uint blocks = argc > 2 ? atoi(argv[2]) : 200;
  if(argc > 3) max_stride = atoi(argv[3]);
  rng.seed(seed);

  spc7110.prom.data.resize(0x100000);
  spc7110.drom.data.resize(0x100000);
  for(auto& b : spc7110.prom.data) b = rng();
  for(auto& b : spc7110.drom.data) b = rng();
  // directory: {mode, address[22:16], address[15:8], address[7:0]}
  for(uint i = 0; i < 256; i++) {
    uint p = TABLE + i * 4;
    uint addr = rnd(0x0e0000);
    spc7110.drom.data[p + 0] = rnd(16) ? rnd(3) : 3;
    spc7110.drom.data[p + 1] = addr >> 16;
    spc7110.drom.data[p + 2] = addr >> 8;
    spc7110.drom.data[p + 3] = addr;
  }

  FILE* mem = fopen("mem.hex", "w");
  for(auto b : spc7110.prom.data) fprintf(mem, "%02x\n", b);
  for(auto b : spc7110.drom.data) fprintf(mem, "%02x\n", b);
  fclose(mem);

  spc7110.power();
  ops = fopen("ops.txt", "w");
  for(uint i = 0; i < blocks; i++) {
    switch(rnd(5)) {
    case 0: test_mul(); break;
    case 1: test_div(); break;
    case 2: test_dataport(); break;
    case 3: test_dcu(); break;
    case 4: test_mcu(); break;
    }
  }
  fprintf(ops, "E\n");
  fclose(ops);
  return 0;
}
