# Simulation of the SPC7110 core against the bsnes reference implementation.
# Requires: Icarus Verilog, a C++17 compiler, curl.
#
#   make             run with default seed
#   make SEED=42 BLOCKS=1000
#   make rtc         Epson RTC-4513 test (make rtc SEED=3 RTC_BLOCKS=40)
#
# The bsnes sources (GPLv3) are fetched at build time and not stored in this repo.

BSNES_REV := 78474e5d39f58872dc21aca18cf9eb583b147fe7
BSNES_URL := https://raw.githubusercontent.com/bsnes-emu/bsnes/$(BSNES_REV)/bsnes/sfc/coprocessor/spc7110
REF_FILES := spc7110.cpp spc7110.hpp dcu.cpp data.cpp alu.cpp decompressor.cpp
REF := $(addprefix ref/,$(REF_FILES))
RTC_URL := https://raw.githubusercontent.com/bsnes-emu/bsnes/$(BSNES_REV)/bsnes/sfc/coprocessor/epsonrtc
RTC_FILES := epsonrtc.cpp epsonrtc.hpp memory.cpp time.cpp serialization.cpp
RTC_REF := $(addprefix ref/epsonrtc/,$(RTC_FILES))
RTC_BLOCKS ?= 40

SEED ?= 1
BLOCKS ?= 200
GAP ?= 28
CGAP ?= 100
WGAP ?= 100

RTL := ../spc7110.v ../spc7110_dec.v

all: run

ref/epsonrtc/%:
	@mkdir -p ref/epsonrtc
	curl -sfL -o $@ $(RTC_URL)/$*

ref/%:
	@mkdir -p ref
	curl -sfL -o $@ $(BSNES_URL)/$*

gen: gen.cpp shim/sfc/sfc.hpp $(REF)
	$(CXX) -std=c++17 -O2 -w -Ishim -Ishim/sfc -Iref -o $@ gen.cpp

tb.vvp: tb_spc7110.v $(RTL) Makefile
	iverilog -g2005 -o $@ -P tb_spc7110.GAP=$(GAP) -P tb_spc7110.WGAP=$(WGAP) -P tb_spc7110.CGAP=$(CGAP) tb_spc7110.v $(RTL)

run: gen tb.vvp
	./gen $(SEED) $(BLOCKS)
	vvp -n tb.vvp

gen_rtc: gen_rtc.cpp shim/sfc/sfc.hpp $(RTC_REF)
	$(CXX) -std=c++17 -O2 -w -Ishim -Ishim/sfc -Iref/epsonrtc -o $@ gen_rtc.cpp

rtc.vvp: tb_rtc.v ../epsonrtc.v
	iverilog -g2005 -o $@ tb_rtc.v ../epsonrtc.v

rtc: gen_rtc rtc.vvp
	./gen_rtc $(SEED) $(RTC_BLOCKS)
	vvp -n rtc.vvp

clean:
	rm -rf ref gen tb.vvp mem.hex ops.txt gen_rtc rtc.vvp ops_rtc.txt

.PHONY: all run rtc clean
