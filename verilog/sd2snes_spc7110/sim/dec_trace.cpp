// Per-decode trace of the bsnes decompressor: "mode origin" -> results
#include <cstdio>
#include <cstdlib>
#include "spc7110.cpp"
using namespace SuperFamicom;
int main(int argc, char** argv) {
  uint mode = atoi(argv[1]), n = atoi(argv[2]);
  spc7110.drom.data.resize(0x100000);
  srand(7);
  for(auto& b : spc7110.drom.data) b = rand();
  FILE* f = fopen("dec_rom.hex", "w");
  for(auto b : spc7110.drom.data) fprintf(f, "%02x\n", b);
  fclose(f);
  spc7110.power();
  auto& d = *spc7110.decompressor;
  d.initialize(mode, 0);
  for(uint i = 0; i < n; i++) { d.decode(); printf("%08x\n", d.result); }
}
