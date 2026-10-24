#ifndef _EPSONRTC_H
#define _EPSONRTC_H

#include <stdint.h>

/* The Epson RTC-4513 state is kept bsnes-style behind the SaveRAM in the
   .srm file: 8 bytes of chip state followed by a 64-bit little endian
   Unix timestamp of when it was taken. */
#define EPSONRTC_TAIL_SIZE (16)

/* bytes appended to the .srm for the current game (0 or EPSONRTC_TAIL_SIZE) */
uint32_t epsonrtc_srm_bytes(void);

/* zero the state tail before the .srm is loaded */
void epsonrtc_clear(void);

/* replay the time elapsed since the save and load the chip state into the
   FPGA, or reset the chip if the .srm has no RTC state */
void epsonrtc_load(void);

/* refresh the state tail if the game changed the RTC.
   returns 1 if the tail changed and the .srm should be saved */
uint8_t epsonrtc_update(void);

#endif
