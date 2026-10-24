`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// Company: Rehkopf
// Engineer: Rehkopf
//
// Create Date:    01:13:46 05/09/2009
// Design Name:
// Module Name:    address
// Project Name:
// Target Devices:
// Tool versions:
// Description: Address logic w/ SaveRAM masking
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module address(
  input CLK,
  input [15:0] featurebits,
  input [2:0] MAPPER,       // MCU detected mapper
  input [23:0] SNES_ADDR,   // requested address from SNES
  input [7:0] SNES_PA,      // peripheral address from SNES
  input SNES_ROMSEL,        // ROMSEL from SNES
  input [23:0] SPC_ROM_ADDR,// ROM address as mapped by the SPC7110 MCU unit
  output [23:0] ROM_ADDR,   // Address to request from SRAM0
  output ROM_HIT,           // want to access RAM0
  output IS_SAVERAM,        // address/CS mapped as SRAM?
  output IS_ROM,            // address mapped as ROM?
  output IS_WRITABLE,       // address somehow mapped as writable area?
  input [23:0] SAVERAM_MASK,
  input [23:0] ROM_MASK,
  output msu_enable,
  output spc_enable,
  output [5:0] spc_reg_addr,
  output rtc_enable,
  output r213f_enable,
  output r2100_hit,
  output snescmd_enable,
  output nmicmd_enable,
  output return_vector_enable,
  output branch1_enable,
  output branch2_enable,
  output branch3_enable
);

parameter [2:0]
  FEAT_SRTC = 2,  // here: Epson RTC-4513 present
  FEAT_MSU1 = 3,
  FEAT_213F = 4,
  FEAT_2100 = 6
;

wire [23:0] SRAM_SNES_ADDR;

/* SPC7110 mapper:
   - ROM @ 00-3f,80-bf:8000-ffff and c0-ff:0000-ffff (HiROM style),
     bank switched in 1MB slots by $4830-$4834 (see spc7110.v)
   - MMIO @ 00-3f,80-bf:4800-483f; 50:0000-ffff = $4800; 58:0000-ffff = $4808
   - SRAM @ 00-3f,80-bf:6000-7fff (8KB windows, bank selects window)
 */

assign IS_ROM = ~SNES_ROMSEL & ~(~SNES_ADDR[23] & SNES_ADDR[22]);

assign IS_SAVERAM = |SAVERAM_MASK & ~SNES_ADDR[22] & (SNES_ADDR[15:13] == 3'b011);

assign SRAM_SNES_ADDR = IS_SAVERAM
                        ? (24'hE00000 | ({SNES_ADDR[21:16], SNES_ADDR[12:0]}
                         & SAVERAM_MASK))
                        : (SPC_ROM_ADDR & ROM_MASK);

assign ROM_ADDR = SRAM_SNES_ADDR;

assign IS_WRITABLE = IS_SAVERAM;

assign ROM_HIT = IS_ROM | IS_WRITABLE;

wire msu_enable_w = featurebits[FEAT_MSU1] & (!SNES_ADDR[22] && ((SNES_ADDR[15:0] & 16'hfff8) == 16'h2000));
`ifdef MK2
assign msu_enable = 1'b0;  // no MSU-1 on the Mk2 SPC7110 core
`else
assign msu_enable = msu_enable_w;
`endif

wire spc_bank50 = (SNES_ADDR[23:16] == 8'h50);
wire spc_bank58 = (SNES_ADDR[23:16] == 8'h58);
assign spc_enable = (~SNES_ADDR[22] & (SNES_ADDR[15:6] == 10'b0100_1000_00)) | spc_bank50 | spc_bank58;
assign spc_reg_addr = spc_bank50 ? 6'h00 : spc_bank58 ? 6'h08 : SNES_ADDR[5:0];

// Epson RTC-4513: 00-3f,80-bf:4840-4842
assign rtc_enable = featurebits[FEAT_SRTC] & ~SNES_ADDR[22]
                  & (SNES_ADDR[15:2] == 14'h1210) & (SNES_ADDR[1:0] != 2'd3);

assign r213f_enable = featurebits[FEAT_213F] & (SNES_PA == 8'h3f);
assign r2100_hit = (SNES_PA == 8'h00);

assign snescmd_enable = ({SNES_ADDR[22], SNES_ADDR[15:9]} == 8'b0_0010101);
assign nmicmd_enable = (SNES_ADDR == 24'h002BF2);
assign return_vector_enable = (SNES_ADDR == 24'h002A6C);
assign branch1_enable = (SNES_ADDR == 24'h002A1F);
assign branch2_enable = (SNES_ADDR == 24'h002A59);
assign branch3_enable = (SNES_ADDR == 24'h002A5E);
endmodule
