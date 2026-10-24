/* sd2snes - SD card based universal cartridge for the SNES

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

   epsonrtc.c: Epson RTC-4513 (SPC7110) state persistence, after bsnes
*/

#include <string.h>
#include "config.h"
#include "uart.h"
#include "smc.h"
#include "fpga_spi.h"
#include "memory.h"
#include "rtc.h"
#include "epsonrtc.h"

extern snes_romprops_t romprops;

/* days between the time2gtime() epoch and 1970-01-01 */
#define GTIME_UNIX_EPOCH (865565L)

/* chip registers, sized as on the RTC-4513 (bsnes epsonrtc.hpp) */
typedef struct {
  uint8_t secondlo, secondhi, batteryfailure;
  uint8_t minutelo, minutehi, resync;
  uint8_t hourlo, hourhi, meridian;
  uint8_t daylo, dayhi, dayram;
  uint8_t monthlo, monthhi, monthram;
  uint8_t yearlo, yearhi;
  uint8_t weekday;
  uint8_t hold, calendar, irqflag, roundseconds;
  uint8_t irqmask, irqduty, irqperiod;
  uint8_t pause, stop, atime, test;
} epsonrtc_t;

static uint8_t epsonrtc_active(void) {
  return romprops.has_spc7110 && (romprops.fpga_features & FEAT_SRTC);
}

uint32_t epsonrtc_srm_bytes(void) {
  return epsonrtc_active() ? EPSONRTC_TAIL_SIZE : 0;
}

static uint32_t tail_addr(void) {
  return SRAM_SAVE_ADDR + romprops.ramsize_bytes;
}

static uint64_t unix_now(void) {
  struct tm time;
  struct gtm gtime;
  read_rtc(&time);
  time2gtime(&gtime, &time);
  return (uint64_t)(gtime.gtm_days - GTIME_UNIX_EPOCH) * 86400
       + gtime.gtm_hour * 3600 + gtime.gtm_min * 60 + gtime.gtm_sec;
}

/* EpsonRTC::load() bit positions */
static void unpack(epsonrtc_t *r, const uint8_t *d) {
  r->secondlo = d[0] & 15; r->secondhi = (d[0] >> 4) & 7; r->batteryfailure = d[0] >> 7;
  r->minutelo = d[1] & 15; r->minutehi = (d[1] >> 4) & 7; r->resync = d[1] >> 7;
  r->hourlo = d[2] & 15; r->hourhi = (d[2] >> 4) & 3; r->meridian = (d[2] >> 6) & 1;
  r->daylo = d[3] & 15; r->dayhi = (d[3] >> 4) & 3; r->dayram = (d[3] >> 6) & 1;
  r->monthlo = d[4] & 15; r->monthhi = (d[4] >> 4) & 1; r->monthram = (d[4] >> 5) & 3;
  r->yearlo = d[5] & 15; r->yearhi = d[5] >> 4;
  r->weekday = d[6] & 7;
  r->hold = (d[6] >> 4) & 1; r->calendar = (d[6] >> 5) & 1;
  r->irqflag = (d[6] >> 6) & 1; r->roundseconds = d[6] >> 7;
  r->irqmask = d[7] & 1; r->irqduty = (d[7] >> 1) & 1; r->irqperiod = (d[7] >> 2) & 3;
  r->pause = (d[7] >> 4) & 1; r->stop = (d[7] >> 5) & 1;
  r->atime = (d[7] >> 6) & 1; r->test = d[7] >> 7;
}

/* EpsonRTC::save() bit positions */
static void pack(uint8_t *d, const epsonrtc_t *r) {
  d[0] = r->secondlo | r->secondhi << 4 | r->batteryfailure << 7;
  d[1] = r->minutelo | r->minutehi << 4 | r->resync << 7;
  d[2] = r->hourlo | r->hourhi << 4 | r->meridian << 6 | r->resync << 7;
  d[3] = r->daylo | r->dayhi << 4 | r->dayram << 6 | r->resync << 7;
  d[4] = r->monthlo | r->monthhi << 4 | r->monthram << 5 | r->resync << 7;
  d[5] = r->yearlo | r->yearhi << 4;
  d[6] = r->weekday | r->resync << 3 | r->hold << 4 | r->calendar << 5
       | r->irqflag << 6 | r->roundseconds << 7;
  d[7] = r->irqmask | r->irqduty << 1 | r->irqperiod << 2 | r->pause << 4
       | r->stop << 5 | r->atime << 6 | r->test << 7;
}

/* bsnes time.cpp, including its handling of invalid BCD values */
static void tick_year(epsonrtc_t *r) {
  if(r->yearlo <= 8 || r->yearlo == 12) {
    r->yearlo++;
  } else {
    r->yearlo = !(r->yearlo & 1);
    if(r->yearhi <= 8 || r->yearhi == 12) {
      r->yearhi++;
    } else {
      r->yearhi = !(r->yearhi & 1);
    }
  }
}

static void tick_month(epsonrtc_t *r) {
  if(r->monthhi == 0 || !(r->monthlo & 2)) {
    if(r->monthlo <= 8 || r->monthlo == 12) {
      r->monthlo++;
    } else {
      r->monthlo = !(r->monthlo & 1);
      r->monthhi ^= 1;
    }
  } else {
    r->monthlo = !(r->monthlo & 1);
    r->monthhi = 0;
    tick_year(r);
  }
}

static uint8_t month_ends(const epsonrtc_t *r, uint8_t days) {
  switch(days) {
    case 28: return r->dayhi == 3 || (r->dayhi == 2 && r->daylo >= 8);
    case 29: return r->dayhi == 3 || (r->dayhi == 2 && r->daylo > 8 && r->daylo != 12);
    case 30: return r->dayhi == 3 || (r->dayhi == 2 && (r->daylo == 10 || r->daylo == 14));
    default: return r->dayhi == 3 && (r->daylo & 3);
  }
}

static void tick_day(epsonrtc_t *r) {
  /* January - December = 0x01 - 0x09; 0x10 - 0x12 */
  static const uint8_t daysinmonth[32] = {
    30, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 30, 31, 30,
    31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30,
  };
  if(r->calendar == 0) return;
  r->weekday = ((r->weekday + 1) + (r->weekday == 6)) & 7;

  uint8_t days = daysinmonth[r->monthhi << 4 | r->monthlo];
  if(days == 28) {
    /* add one day for leap years */
    if((r->yearhi & 1) == 0 && ((r->yearlo - 0) & 3) == 0) days++;
    if((r->yearhi & 1) == 1 && ((r->yearlo - 2) & 3) == 0) days++;
  }

  if(month_ends(r, days)) {
    r->daylo = 1;
    r->dayhi = 0;
    tick_month(r);
  } else if(r->daylo <= 8 || r->daylo == 12) {
    r->daylo++;
  } else {
    r->daylo = !(r->daylo & 1);
    r->dayhi = (r->dayhi + 1) & 3;
  }
}

static void tick_hour_24(epsonrtc_t *r) {
  if(r->hourhi < 2) {
    if(r->hourlo <= 8 || r->hourlo == 12) {
      r->hourlo++;
    } else {
      r->hourlo = !(r->hourlo & 1);
      r->hourhi++;
    }
  } else if(r->hourlo != 3 && !(r->hourlo & 4)) {
    if(r->hourlo <= 8 || r->hourlo >= 12) {
      r->hourlo = (r->hourlo + 1) & 15;
    } else {
      r->hourlo = !(r->hourlo & 1);
      r->hourhi = (r->hourhi + 1) & 3;
    }
  } else {
    r->hourlo = !(r->hourlo & 1);
    r->hourhi = 0;
    tick_day(r);
  }
}

static void tick_hour_12(epsonrtc_t *r) {
  if(r->hourhi == 0) {
    if(r->hourlo <= 8 || r->hourlo == 12) {
      r->hourlo++;
    } else {
      r->hourlo = !(r->hourlo & 1);
      r->hourhi ^= 1;
    }
    return;
  }
  if(r->hourlo & 1) r->meridian ^= 1;
  if(r->hourlo < 2 || r->hourlo == 4 || r->hourlo == 5 || r->hourlo == 8 || r->hourlo == 12) {
    r->hourlo++;
  } else {
    r->hourlo = !(r->hourlo & 1);
    r->hourhi ^= 1;
  }
  if(r->meridian == 0 && !(r->hourlo & 1)) tick_day(r);
}

static void tick_hour(epsonrtc_t *r) {
  if(r->atime) {
    tick_hour_24(r);
  } else {
    tick_hour_12(r);
  }
}

static void tick_minute(epsonrtc_t *r) {
  if(r->minutelo <= 8 || r->minutelo == 12) {
    r->minutelo++;
  } else {
    r->minutelo = 0;
    if(r->minutehi < 5) {
      r->minutehi++;
    } else {
      r->minutehi = 0;
      tick_hour(r);
    }
  }
}

static void tick_second(epsonrtc_t *r) {
  if(r->secondlo <= 8 || r->secondlo == 12) {
    r->secondlo++;
  } else {
    r->secondlo = 0;
    if(r->secondhi < 5) {
      r->secondhi++;
    } else {
      r->secondhi = 0;
      tick_minute(r);
    }
  }
}

/* advance a saved state by elapsed seconds the way EpsonRTC::load() does */
static void replay(uint8_t *state, uint64_t elapsed) {
  epsonrtc_t r;
  unpack(&r, state);
  while(elapsed >= 86400) { tick_day(&r); elapsed -= 86400; }
  while(elapsed >= 3600) { tick_hour(&r); elapsed -= 3600; }
  while(elapsed >= 60) { tick_minute(&r); elapsed -= 60; }
  while(elapsed--) tick_second(&r);
  pack(state, &r);
}

static uint64_t read_tail(uint8_t *state) {
  uint8_t tail[EPSONRTC_TAIL_SIZE];
  uint64_t timestamp = 0;
  sram_readblock(tail, tail_addr(), EPSONRTC_TAIL_SIZE);
  memcpy(state, tail, 8);
  for(int i = 7; i >= 0; i--) timestamp = timestamp << 8 | tail[8 + i];
  return timestamp;
}

static void write_tail(const uint8_t *state, uint64_t timestamp) {
  uint8_t tail[EPSONRTC_TAIL_SIZE];
  memcpy(tail, state, 8);
  for(int i = 0; i < 8; i++) tail[8 + i] = timestamp >> (8 * i);
  sram_writeblock(tail, tail_addr(), EPSONRTC_TAIL_SIZE);
}

/* state at `now` if the game left the RTC alone since the tail was taken */
static void predict(uint8_t *state, uint64_t timestamp, uint64_t now) {
  if(now > timestamp) replay(state, now - timestamp);
}

void epsonrtc_clear(void) {
  if(!epsonrtc_active()) return;
  sram_memset(tail_addr(), EPSONRTC_TAIL_SIZE, 0);
}

void epsonrtc_load(void) {
  uint8_t state[8];
  if(!epsonrtc_active()) return;
  uint64_t timestamp = read_tail(state);
  if(timestamp) {
    predict(state, timestamp, unix_now());
  } else {
    /* EpsonRTC::initialize(): only the battery failure flag is set */
    memset(state, 0, sizeof(state));
    state[0] = 0x80;
  }
  printf("Epson RTC: ts=%08lx%08lx state=%02x%02x%02x%02x%02x%02x%02x%02x\n",
         (uint32_t)(timestamp >> 32), (uint32_t)timestamp,
         state[7], state[6], state[5], state[4], state[3], state[2], state[1], state[0]);
  fpga_set_epsonrtc_state(state);
}

uint8_t epsonrtc_update(void) {
  uint8_t state[8], saved[8], next[8];
  if(!epsonrtc_active()) return 0;
  if(!fpga_get_epsonrtc_state(state)) return 0;

  uint64_t now = unix_now();
  uint64_t timestamp = read_tail(saved);
  if(timestamp) {
    /* the chip and the MCU RTC tick at different phases: allow one second */
    predict(saved, timestamp, now - 1);
    memcpy(next, saved, sizeof(next));
    replay(next, 1);
    if(!memcmp(state, saved, sizeof(state)) || !memcmp(state, next, sizeof(state))) return 0;
  }
  write_tail(state, now);
  return 1;
}
