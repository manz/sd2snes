`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// Module Name:    spc7110
// Description:    SPC7110 (Hudson/Epson) coprocessor:
//                 - decompression unit (DCU)   $4800-$480C, $50:xxxx = $4800
//                 - data port unit             $4810-$481A
//                 - arithmetic logic unit      $4820-$482F
//                 - memory control unit (MCU)  $4830-$4834
//
//                 Behaviour follows bsnes (sfc/coprocessor/spc7110).
//
//                 Cartridge ROM layout in PSRAM (as loaded from file):
//                   $000000-$0FFFFF  program ROM (8 Mbit)
//                   $100000-...      data ROM
//                 DROM_MASK must be (data ROM size - 1), data ROM size being
//                 a power of two.
//
//                 Data ROM reads (data port, directory, compressed stream)
//                 are issued as 16-bit word reads on the BUS_* port; BUS_DI
//                 is {byte @ even address, byte @ odd address}.
//////////////////////////////////////////////////////////////////////////////////
module spc7110(
  input CLK,
  input RST,                  // synchronous reset to power-on state

  // SNES register access ($00-3f,80-bf:4800-483f; $50:xxxx; $58:xxxx)
  input [5:0] reg_addr,
  input [7:0] reg_data_in,
  output [7:0] reg_data_out,
  input reg_we,               // one-cycle strobe at end of SNES write
  input reg_rd_end,           // one-cycle strobe at end of SNES read (side effects)

  // SNES ROM mapping ($00-3f,80-bf:8000-ffff; $c0-ff:0000-ffff)
  input [23:0] SNES_ADDR,
  output [23:0] ROM_ADDR,     // PSRAM byte address
  output ROM_ZERO,            // unmapped data ROM area; reads return $00
  output SRAM_EN,             // $4830.7: SRAM enable

  // data ROM bus
  input [23:0] DROM_MASK,
  output reg [23:0] BUS_ADDR,
  output reg BUS_RRQ,
  input BUS_ACK,              // one-cycle strobe, BUS_DI valid
  input [15:0] BUS_DI,

  // debug
  output reg dcu_underrun
);

// register accesses take effect one cycle after the strobe (timing); the read
// data mux below still uses the live address
reg [5:0] reg_addr_q;
reg [7:0] reg_data_q;
reg       reg_we_q, reg_rd_end_q;
always @(posedge CLK) begin
  reg_addr_q <= reg_addr;
  reg_data_q <= reg_data_in;
  reg_we_q <= reg_we & ~RST;
  reg_rd_end_q <= reg_rd_end & ~RST;
end

/* ===================== registers ===================== */
// decompression unit
reg [7:0] r4801, r4802, r4804, r4805, r4806, r4807, r4809, r480a;
reg [6:0] r4803;
reg [1:0] r480b;
reg       r480c_ready;  // r480c bit 7
// data port unit
reg [7:0] r4810, r4811, r4812, r4814, r4815, r4816, r4817;
reg [6:0] r4813, r4818;
// arithmetic logic unit
reg [7:0] r4820, r4821, r4822, r4823, r4824, r4825, r4826, r4827;
reg [31:0] alu_q;       // r4828-r482b
reg [15:0] alu_r;       // r482c-r482d
reg       r482e;
reg [7:0] r482f;
// memory control unit
reg [2:0] r4830_bank, r4831, r4832, r4833, r4834;
reg       r4830_sram;

assign SRAM_EN = r4830_sram;

/* ===================== data ROM mapping ===================== */
// bsnes dataromRead(): returns {zero, PSRAM address}
function [24:0] drom_map;
  input [23:0] addr;
  input [2:0] cfg;
  input [23:0] dmask;
  reg [23:0] size_mask;
  begin
    case(cfg[1:0])
      2'd0: size_mask = 24'h0fffff;
      2'd1: size_mask = 24'h1fffff;
      2'd2: size_mask = 24'h3fffff;
      default: size_mask = 24'h7fffff;
    endcase
    drom_map = {(cfg[1:0] != 2'd3) & addr[22],
                24'h100000 + (addr & size_mask & dmask)};
  end
endfunction

/* SNES ROM mapping: bsnes mcuromRead() */
wire [1:0]  rom_slot = SNES_ADDR[21:20];
wire [19:0] rom_offset = SNES_ADDR[19:0];
wire [2:0]  rom_bank = (rom_slot == 2'd0) ? r4830_bank
                     : (rom_slot == 2'd1) ? r4831
                     : (rom_slot == 2'd2) ? r4832
                     : r4833;
wire        rom_is_prom = (rom_slot == 2'd0) | ((rom_slot == 2'd1) & r4834[2]);
wire [24:0] rom_drom = drom_map({1'b0, rom_bank, rom_offset}, r4834, DROM_MASK);

assign ROM_ADDR = rom_is_prom ? {4'h0, rom_offset} : rom_drom[23:0];
assign ROM_ZERO = ~rom_is_prom & rom_drom[24];

/* ===================== data ROM bus arbitration ===================== */
localparam [2:0] OWN_NONE = 3'd0, OWN_DP = 3'd1, OWN_DIR = 3'd2, OWN_STREAM = 3'd3, OWN_PF = 3'd4;
reg [2:0] bus_owner = OWN_NONE;
reg       bus_zero;
reg       bus_odd;
reg       bus_epoch;
wire      bus_idle = (bus_owner == OWN_NONE) & ~BUS_RRQ;
wire [7:0] bus_byte = bus_zero ? 8'h00 : (bus_odd ? BUS_DI[7:0] : BUS_DI[15:8]);

/* ===================== data port unit ===================== */
wire [22:0] dp_offset = {r4813, r4812, r4811};
wire [15:0] dp_adjust = {r4815, r4814};
wire [15:0] dp_stride = {r4817, r4816};
wire [23:0] dp_adjust_ext = r4818[3] ? {{8{dp_adjust[15]}}, dp_adjust} : {8'h00, dp_adjust};
wire [23:0] dp_stride_ext = ~r4818[0] ? 24'h1
                          : r4818[2] ? {{8{dp_stride[15]}}, dp_stride} : {8'h00, dp_stride};


reg         dp_pending;
// a $4810 read moves the port to the address that was dp_next_map before it:
// that fetch can start at once instead of after the address pipeline settles
reg         dp_fast;
reg  [24:0] dp_fast_addr;
reg         dp_fast_hit;    // the prefetch buffer held that address at the read
reg         dp_fast_wait;   // a prefetch of that address was in flight at the read

// prefetch of the byte a $4810 read would fetch next; tagged with its mapped address
wire [22:0] dp_next_offset = r4818[4] ? dp_offset : dp_offset + dp_stride_ext[22:0];
wire [15:0] dp_next_adjust = r4818[4] ? dp_adjust + dp_stride_ext[15:0] : dp_adjust;
wire [23:0] dp_next_adjust_ext = r4818[3] ? {{8{dp_next_adjust[15]}}, dp_next_adjust} : {8'h00, dp_next_adjust};

// address computation is pipelined over three stages (offset/adjust, read
// address, mapped address); results are valid once no register access
// happened in the previous four cycles. A fetch issued in the cycle of a
// register access uses the registers from before it, which is when bsnes
// performs the access.
reg  [22:0] dp_offset_r, dp_next_offset_r;
reg  [23:0] dp_adjust_r, dp_next_adjust_r;
reg  [23:0] dp_read_addr_r, dp_next_read_addr_r;
reg  [24:0] dp_map, dp_next_map;
reg  [3:0]  dp_reg_event_d;
reg         pf_valid;
reg         pf_enable;   // set by $4810 reads, cleared by data port setup writes
reg  [24:0] pf_tag;
reg  [24:0] pf_req_tag;
reg  [7:0]  pf_data;
wire        dp_reg_event = reg_we_q | reg_rd_end_q;
wire        dp_stable = ~|dp_reg_event_d;
always @(posedge CLK) begin
  dp_offset_r <= dp_offset;
  dp_next_offset_r <= dp_next_offset;
  dp_adjust_r <= r4818[1] ? dp_adjust_ext : 24'h0;
  dp_next_adjust_r <= r4818[1] ? dp_next_adjust_ext : 24'h0;
  dp_read_addr_r <= {1'b0, dp_offset_r} + dp_adjust_r;
  dp_next_read_addr_r <= {1'b0, dp_next_offset_r} + dp_next_adjust_r;
  dp_map <= drom_map(dp_read_addr_r, r4834, DROM_MASK);
  dp_next_map <= drom_map(dp_next_read_addr_r, r4834, DROM_MASK);
  dp_reg_event_d <= {dp_reg_event_d[2:0], dp_reg_event};
end
wire [24:0] dp_req_addr = dp_fast ? dp_fast_addr : dp_map;
// the prefetch hit compare for other pending fetches is registered; pf_upd
// marks the cycle after a prefetch buffer update, when it is not valid yet
reg         pf_upd;
reg         pf_hit_map;
wire        pf_ack = BUS_ACK && (bus_owner == OWN_PF);
wire        pf_inflight_next = (bus_owner == OWN_PF) && (pf_req_tag == dp_next_map);
wire        dp_slow_ok = dp_stable & ~pf_upd;
wire        pf_wanted = ~(pf_valid && (pf_tag == dp_next_map));

/* ===================== decompression unit ===================== */
reg        dcu_load_pending;
reg        dcu_start_pending;
reg [1:0]  dcu_mode;
reg [1:0]  dcu_cur_mode;       // mode latched at transfer start (bsnes: decompressor->bpp)
reg [22:0] dcu_address;
reg        dcu_active;         // a stream has been started (mode valid)
reg [4:0]  dcu_offset;
reg        dcu_front;          // tile buffer currently read by the SNES
reg        dcu_back_valid;
reg [31:0] dcu_tile [0:15];    // [buffer*8 + row]

wire [4:0] dcu_len_mask = (dcu_cur_mode == 2'd0) ? 5'd7 : (dcu_cur_mode == 2'd1) ? 5'd15 : 5'd31;
wire [2:0] dcu_rd_row = (dcu_cur_mode == 2'd0) ? dcu_offset[2:0] : dcu_offset[3:1];
wire [1:0] dcu_rd_byte = (dcu_cur_mode == 2'd0) ? 2'd0 : {dcu_offset[4] & (dcu_cur_mode == 2'd2), dcu_offset[0]};
wire [31:0] dcu_rd_word = dcu_tile[{dcu_front, dcu_rd_row}];
wire [7:0] dcu_data = ~r480c_ready ? 8'h00 : dcu_rd_word[8*dcu_rd_byte +: 8];

// compressed stream FIFO
reg [7:0]  fifo [0:7];
reg [2:0]  fifo_rd, fifo_wr;
reg [3:0]  fifo_cnt;
reg [23:0] stream_ptr;
reg        stream_on;
wire [24:0] stream_map = drom_map(stream_ptr, r4834, DROM_MASK);

reg  dec_init, dec_start;
reg [16:0] dec_count;
wire dec_busy, dec_take;
wire [31:0] dec_result;

spc7110_dec dec(
  .clk(CLK),
  .init(dec_init),
  .mode(dcu_mode),
  .start(dec_start),
  .count(dec_count),
  .busy(dec_busy),
  .result(dec_result),
  .in_data(fifo[fifo_rd]),
  .in_valid(fifo_cnt != 4'd0),
  .in_take(dec_take)
);

// directory loader / transfer control
localparam [2:0]
  D_IDLE = 3'd0,
  D_DIR  = 3'd1,
  D_INIT = 3'd2,
  D_WAIT = 3'd3,
  D_ROW  = 3'd4;

reg [2:0]  dcu_state = D_IDLE;
reg [1:0]  dir_idx;
reg        dir_req;
reg [2:0]  row;
reg        fill_buf;
reg        fill_first;
wire [23:0] dir_addr = {1'b0, r4803, r4802, r4801} + {14'h0, r4804, 2'b00} + {22'h0, dir_idx};
wire [24:0] dir_map = drom_map(dir_addr, r4834, DROM_MASK);
wire [7:0]  row_seek = r480b[0] ? r4807 : 8'd1;
wire        dcu_abort = dcu_load_pending | dcu_start_pending;

/* ===================== arithmetic logic unit ===================== */
reg        div_busy;
reg [5:0]  div_cnt;
reg [31:0] div_n;     // |dividend|, shifted into quotient
reg [15:0] div_d;     // |divisor|
reg [16:0] div_rem;
reg        div_qneg, div_rneg;
reg        mul_pending;

// one signed 17x17 multiplier serves both modes ($482E.0: signed)
wire signed [16:0] mul_a = {r482e & r4825[7], r4825, r4824};
wire signed [16:0] mul_b = {r482e & r4821[7], r4821, r4820};
wire signed [33:0] mul_p = mul_a * mul_b;
wire [31:0] dividend = {r4823, r4822, r4821, r4820};
wire [15:0] divisor = {r4827, r4826};
wire [16:0] div_trial = {div_rem[15:0], div_n[31]} - {1'b0, div_d};

/* ===================== register read ===================== */
reg [7:0] rdata;
always @* begin
  case(reg_addr)
    6'h00: rdata = dcu_data;
    6'h01: rdata = r4801;
    6'h02: rdata = r4802;
    6'h03: rdata = {1'b0, r4803};
    6'h04: rdata = r4804;
    6'h05: rdata = r4805;
    6'h06: rdata = r4806;
    6'h07: rdata = r4807;
    6'h09: rdata = r4809;
    6'h0a: rdata = r480a;
    6'h0b: rdata = {6'h00, r480b};
    6'h0c: rdata = {r480c_ready, 7'h00};
    6'h10: rdata = r4810;
    6'h11: rdata = r4811;
    6'h12: rdata = r4812;
    6'h13: rdata = {1'b0, r4813};
    6'h14: rdata = r4814;
    6'h15: rdata = r4815;
    6'h16: rdata = r4816;
    6'h17: rdata = r4817;
    6'h18: rdata = {1'b0, r4818};
    6'h20: rdata = r4820;
    6'h21: rdata = r4821;
    6'h22: rdata = r4822;
    6'h23: rdata = r4823;
    6'h24: rdata = r4824;
    6'h25: rdata = r4825;
    6'h26: rdata = r4826;
    6'h27: rdata = r4827;
    6'h28: rdata = alu_q[7:0];
    6'h29: rdata = alu_q[15:8];
    6'h2a: rdata = alu_q[23:16];
    6'h2b: rdata = alu_q[31:24];
    6'h2c: rdata = alu_r[7:0];
    6'h2d: rdata = alu_r[15:8];
    6'h2e: rdata = {7'h00, r482e};
    6'h2f: rdata = r482f;
    6'h30: rdata = {r4830_sram, 4'h0, r4830_bank};
    6'h31: rdata = {5'h00, r4831};
    6'h32: rdata = {5'h00, r4832};
    6'h33: rdata = {5'h00, r4833};
    6'h34: rdata = {5'h00, r4834};
    default: rdata = 8'h00;
  endcase
end
assign reg_data_out = rdata;

reg stream_on_epoch = 0;
// stream epoch: bumped whenever the stream restarts so in-flight words are dropped
wire stream_flush = RST | (dcu_state == D_IDLE && !dcu_load_pending && dcu_start_pending);
always @(posedge CLK) if(stream_flush) stream_on_epoch <= ~stream_on_epoch;

wire stream_ack = BUS_ACK && bus_owner == OWN_STREAM && bus_epoch == stream_on_epoch && !stream_flush;
wire [3:0] fifo_push_n = stream_ack ? (bus_odd ? 4'd1 : 4'd2) : 4'd0;


// tile buffer write port (distributed RAM)
wire tile_we = ~RST & (dcu_state == D_ROW) & ~dcu_abort;
always @(posedge CLK) if(tile_we) dcu_tile[{fill_buf, row}] <= dec_result;

/* ===================== main sequential logic ===================== */
// data port: new offset/adjust after an increment event
reg [22:0] dp_new_offset;
reg [15:0] dp_new_adjust;
reg        dp_inc_offset, dp_inc_adjust;
// adjust used by the $4814/$4815/$481A seek; includes the byte being written for $4814/$4815
wire [15:0] dp_wr_adjust = (reg_addr_q == 6'h14) ? {r4815, reg_data_q} : {reg_data_q, r4814};
wire [23:0] dp_seek_adjust = ~reg_we_q ? dp_adjust_ext
                           : r4818[3] ? {{8{dp_wr_adjust[15]}}, dp_wr_adjust} : {8'h00, dp_wr_adjust};
always @* begin
  dp_inc_offset = 1'b0;
  dp_inc_adjust = 1'b0;
  dp_new_offset = dp_offset;
  dp_new_adjust = dp_adjust;
  if(reg_rd_end_q && reg_addr_q == 6'h10) begin
    if(~r4818[4]) begin dp_inc_offset = 1'b1; dp_new_offset = dp_offset + dp_stride_ext[22:0]; end
    else          begin dp_inc_adjust = 1'b1; dp_new_adjust = dp_adjust + dp_stride_ext[15:0]; end
  end else if((reg_rd_end_q && reg_addr_q == 6'h1a && r4818[6:5] == 2'd3)
            ||(reg_we_q && reg_addr_q == 6'h14 && r4818[6:5] == 2'd1)
            ||(reg_we_q && reg_addr_q == 6'h15 && r4818[6:5] == 2'd2)) begin
    dp_inc_offset = 1'b1;
    // adjust includes the byte being written for $4814/$4815
    dp_new_offset = dp_offset + dp_seek_adjust[22:0];
  end
end

always @(posedge CLK) begin
  pf_hit_map <= pf_valid && (pf_tag == dp_map);
end

always @(posedge CLK) begin
  dec_init <= 1'b0;
  pf_upd <= 1'b0;
  dec_start <= 1'b0;
  BUS_RRQ <= 1'b0;

  if(RST) begin
    r4801 <= 0; r4802 <= 0; r4803 <= 0; r4804 <= 0; r4805 <= 0; r4806 <= 0;
    r4807 <= 0; r4809 <= 0; r480a <= 0; r480b <= 0; r480c_ready <= 0;
    r4810 <= 0; r4811 <= 0; r4812 <= 0; r4813 <= 0; r4814 <= 0; r4815 <= 0;
    r4816 <= 0; r4817 <= 0; r4818 <= 0;
    r4820 <= 0; r4821 <= 0; r4822 <= 0; r4823 <= 0; r4824 <= 0; r4825 <= 0;
    r4826 <= 0; r4827 <= 0; alu_q <= 0; alu_r <= 0; r482e <= 0; r482f <= 0;
    r4830_bank <= 0; r4830_sram <= 0; r4831 <= 0; r4832 <= 3'd1; r4833 <= 3'd2; r4834 <= 0;
    dp_pending <= 0; dp_fast <= 0; dp_fast_hit <= 0; dp_fast_wait <= 0;
    pf_valid <= 0; pf_upd <= 0; pf_enable <= 0;
    dcu_load_pending <= 0; dcu_start_pending <= 0;
    dcu_mode <= 0; dcu_cur_mode <= 0; dcu_address <= 0; dcu_active <= 0;
    dcu_offset <= 0; dcu_front <= 0; dcu_back_valid <= 0;
    dcu_state <= D_IDLE; dir_req <= 0;
    dcu_underrun <= 0;
    stream_on <= 0; fifo_cnt <= 0; fifo_rd <= 0; fifo_wr <= 0;
    bus_owner <= OWN_NONE; bus_epoch <= 0;
    div_busy <= 0; mul_pending <= 0;
  end else begin
    /* ---------- register writes ---------- */
    if(reg_we_q) begin
      case(reg_addr_q)
        6'h01: r4801 <= reg_data_q;
        6'h02: r4802 <= reg_data_q;
        6'h03: r4803 <= reg_data_q[6:0];
        6'h04: begin r4804 <= reg_data_q; dcu_load_pending <= 1'b1; end
        6'h05: r4805 <= reg_data_q;
        6'h06: begin r4806 <= reg_data_q; r480c_ready <= 1'b0; dcu_start_pending <= 1'b1; end
        6'h07: r4807 <= reg_data_q;
        6'h09: r4809 <= reg_data_q;
        6'h0a: r480a <= reg_data_q;
        6'h0b: r480b <= reg_data_q[1:0];
        6'h11: r4811 <= reg_data_q;
        6'h12: r4812 <= reg_data_q;
        6'h13: begin r4813 <= reg_data_q[6:0]; dp_pending <= 1'b1; end
        6'h14: r4814 <= reg_data_q;
        6'h15: begin r4815 <= reg_data_q; if(r4818[1]) dp_pending <= 1'b1; end
        6'h16: r4816 <= reg_data_q;
        6'h17: r4817 <= reg_data_q;
        6'h18: begin r4818 <= reg_data_q[6:0]; dp_pending <= 1'b1; end
        6'h20: r4820 <= reg_data_q;
        6'h21: r4821 <= reg_data_q;
        6'h22: r4822 <= reg_data_q;
        6'h23: r4823 <= reg_data_q;
        6'h24: r4824 <= reg_data_q;
        6'h25: begin r4825 <= reg_data_q; r482f <= r482f | 8'h81; mul_pending <= 1'b1; end
        6'h26: r4826 <= reg_data_q;
        6'h27: begin r4827 <= reg_data_q; r482f <= r482f | 8'h80; div_busy <= 1'b1; div_cnt <= 6'd0; end
        6'h2e: r482e <= reg_data_q[0];
        6'h30: begin r4830_sram <= reg_data_q[7]; r4830_bank <= reg_data_q[2:0]; end
        6'h31: r4831 <= reg_data_q[2:0];
        6'h32: r4832 <= reg_data_q[2:0];
        6'h33: r4833 <= reg_data_q[2:0];
        6'h34: r4834 <= reg_data_q[2:0];
        default: ;
      endcase
    end

    /* ---------- data port increments ---------- */
    if(reg_rd_end_q && reg_addr_q == 6'h10) pf_enable <= 1'b1;
    else if(reg_we_q && reg_addr_q[5:4] == 2'b01) pf_enable <= 1'b0;
    if(dp_inc_offset) begin
      {r4813, r4812, r4811} <= dp_new_offset;
      dp_pending <= 1'b1;
    end
    if(dp_inc_adjust) begin
      {r4815, r4814} <= dp_new_adjust;
      dp_pending <= 1'b1;
    end
    if(dp_reg_event) begin
      dp_fast <= reg_rd_end_q && reg_addr_q == 6'h10 && dp_stable;
      dp_fast_addr <= dp_next_map;
      // (a prefetch completing in this cycle replaces the buffer contents)
      dp_fast_hit <= (~pf_wanted & ~pf_ack) | (pf_ack & pf_inflight_next);
      dp_fast_wait <= pf_inflight_next & ~BUS_ACK;
    end

    /* ---------- $4800 read ---------- */
    if(reg_rd_end_q && reg_addr_q == 6'h00) begin
      {r480a, r4809} <= {r480a, r4809} - 16'd1;
      if(r480c_ready) begin
        if((dcu_offset & dcu_len_mask) == dcu_len_mask) begin
          dcu_offset <= 5'd0;
          dcu_front <= ~dcu_front;
          dcu_back_valid <= 1'b0;
          if(!dcu_back_valid) dcu_underrun <= 1'b1;
        end else begin
          dcu_offset <= dcu_offset + 5'd1;
        end
      end
    end

    /* ---------- ALU ---------- */
    if(mul_pending) begin
      mul_pending <= 1'b0;
      alu_q <= mul_p[31:0];
      r482f <= r482f & 8'h7f;
    end
    if(div_busy && !(reg_we_q && reg_addr_q == 6'h27)) begin
      if(div_cnt == 6'd0) begin
        div_qneg <= r482e & (dividend[31] ^ divisor[15]);
        div_rneg <= r482e & dividend[31];
        div_n <= (r482e & dividend[31]) ? -dividend : dividend;
        div_d <= (r482e & divisor[15]) ? -divisor : divisor;
        div_rem <= 17'h0;
        div_cnt <= 6'd1;
      end else if(div_cnt <= 6'd32) begin
        if(!div_trial[16]) div_rem <= div_trial;
        else div_rem <= {div_rem[15:0], div_n[31]};
        div_n <= {div_n[30:0], ~div_trial[16]};
        div_cnt <= div_cnt + 6'd1;
      end else begin
        div_busy <= 1'b0;
        r482f <= r482f & 8'h7f;
        if(divisor == 16'h0) begin
          alu_q <= 32'h0;
          alu_r <= dividend[15:0];
        end else begin
          alu_q <= div_qneg ? -div_n : div_n;
          alu_r <= div_rneg ? -div_rem[15:0] : div_rem[15:0];
        end
      end
    end

    /* ---------- data ROM bus ---------- */
    if(dp_pending && !dp_reg_event
       && (dp_fast ? dp_fast_hit : (dp_slow_ok && pf_hit_map))) begin
      dp_pending <= 1'b0;
      dp_fast <= 1'b0;
      r4810 <= pf_data;
    end
    if(BUS_ACK) begin
      bus_owner <= OWN_NONE;
      case(bus_owner)
        OWN_DP: if(!dp_pending) begin
          r4810 <= bus_byte;
          // the other byte of the word may be the next one ($4818.0 = 0: stride 1)
          if(dp_stable && dp_next_map[24:1] == {bus_zero, BUS_ADDR[23:1]}) begin
            pf_valid <= 1'b1;
            pf_upd <= 1'b1;
            pf_tag <= dp_next_map;
            pf_data <= bus_zero ? 8'h00 : (dp_next_map[0] ? BUS_DI[7:0] : BUS_DI[15:8]);
          end
        end
        OWN_PF: begin
          if(dp_pending && dp_fast && dp_fast_wait) begin
            // the pending read was waiting for this prefetch
            dp_pending <= 1'b0;
            dp_fast <= 1'b0;
            r4810 <= bus_byte;
          end
          dp_fast_hit <= 1'b0;
          dp_fast_wait <= 1'b0;
          pf_valid <= 1'b1;
          pf_upd <= 1'b1;
          pf_tag <= pf_req_tag;
          pf_data <= bus_byte;
        end
        OWN_DIR: begin
          case(dir_idx)
            2'd0: dcu_mode <= bus_byte[1:0];
            2'd1: dcu_address[22:16] <= bus_byte[6:0];
            2'd2: dcu_address[15:8] <= bus_byte;
            2'd3: dcu_address[7:0] <= bus_byte;
          endcase
          dir_idx <= dir_idx + 2'd1;
          dir_req <= 1'b0;
          if(dir_idx == 2'd3) dcu_state <= D_IDLE;
        end
        OWN_STREAM: if(bus_epoch == stream_on_epoch) begin
          fifo[fifo_wr] <= bus_zero ? 8'h00 : (bus_odd ? BUS_DI[7:0] : BUS_DI[15:8]);
          if(!bus_odd) fifo[fifo_wr + 3'd1] <= bus_zero ? 8'h00 : BUS_DI[7:0];
        end
        default: ;
      endcase
    end else if(bus_idle) begin
      if(dp_pending && !dp_reg_event
         && (dp_fast ? (!dp_fast_hit && !dp_fast_wait) : (dp_slow_ok && !pf_hit_map))) begin
        dp_pending <= 1'b0;
        dp_fast <= 1'b0;
        bus_owner <= OWN_DP;
        {bus_zero, BUS_ADDR} <= dp_req_addr;
        bus_odd <= dp_req_addr[0];
        BUS_RRQ <= 1'b1;
      end else if(dcu_state == D_DIR && !dir_req) begin
        dir_req <= 1'b1;
        bus_owner <= OWN_DIR;
        {bus_zero, BUS_ADDR} <= dir_map;
        bus_odd <= dir_map[0];
        BUS_RRQ <= 1'b1;
      end else if(pf_enable && !dp_pending && dp_stable && pf_wanted) begin
        bus_owner <= OWN_PF;
        pf_req_tag <= dp_next_map;
        {bus_zero, BUS_ADDR} <= dp_next_map;
        bus_odd <= dp_next_map[0];
        BUS_RRQ <= 1'b1;
      end else if(stream_on && fifo_cnt <= 4'd6) begin
        bus_owner <= OWN_STREAM;
        bus_epoch <= stream_on_epoch;
        {bus_zero, BUS_ADDR} <= stream_map;
        bus_odd <= stream_map[0];
        BUS_RRQ <= 1'b1;
      end
    end

    // FIFO bookkeeping (stream fill and decoder take)
    if(stream_flush) begin
      fifo_cnt <= 4'd0;
      fifo_rd <= 3'd0;
      fifo_wr <= 3'd0;
    end else begin
      fifo_cnt <= fifo_cnt + fifo_push_n - {3'b000, dec_take};
      fifo_wr <= fifo_wr + fifo_push_n[2:0];
      stream_ptr <= stream_ptr + {22'h0, fifo_push_n[1:0]};
      if(dec_take) fifo_rd <= fifo_rd + 3'd1;
    end

    /* ---------- DCU control ---------- */
    if(dcu_state != D_DIR && dcu_state != D_IDLE && dcu_abort) begin
      dcu_state <= D_IDLE;
    end else begin
      case(dcu_state)
        D_IDLE: begin
          if(dcu_load_pending) begin
            dcu_load_pending <= 1'b0;
            dir_idx <= 2'd0;
            dir_req <= 1'b0;
            dcu_state <= D_DIR;
          end else if(dcu_start_pending) begin
            dcu_start_pending <= 1'b0;
            dcu_back_valid <= 1'b0;
            if(dcu_mode == 2'd3) begin
              // invalid mode: transfer never becomes ready
              dcu_active <= 1'b0;
              stream_on <= 1'b0;
            end else begin
              dcu_active <= 1'b1;
              dcu_cur_mode <= dcu_mode;
              stream_on <= 1'b1;
              stream_ptr <= {1'b0, dcu_address};
              dec_init <= 1'b1;
              dcu_state <= D_INIT;
            end
          end else if(dcu_active && r480c_ready && !dcu_back_valid) begin
            fill_buf <= ~dcu_front;
            fill_first <= 1'b0;
            row <= 3'd0;
            dcu_state <= D_WAIT;
          end
        end
        D_DIR: ; // advanced by BUS_ACK
        D_INIT: begin
          // initial decode plus optional seek ($480B.1: skip {$4806,$4805} decodes)
          if(!dec_busy && !dec_init) begin
            dec_start <= 1'b1;
            dec_count <= 17'd1 + (r480b[1] ? {1'b0, r4806, r4805} : 17'h0);
            dcu_state <= D_WAIT;
            fill_buf <= dcu_front;
            fill_first <= 1'b1;
            row <= 3'd0;
          end
        end
        D_WAIT: begin
          if(!dec_busy && !dec_start) dcu_state <= D_ROW;
        end
        D_ROW: begin
          // store a row, then advance by the row stride ($480B.0: $4807 decodes, else 1)
          if(row != 3'd7) begin
            dec_start <= 1'b1;
            dec_count <= {9'h0, row_seek};
            row <= row + 3'd1;
            dcu_state <= D_WAIT;
          end else begin
            dec_start <= 1'b1;
            dec_count <= {9'h0, row_seek};
            dcu_state <= D_IDLE;
            if(fill_first) begin
              r480c_ready <= 1'b1;
              dcu_offset <= 5'd0;
            end else begin
              dcu_back_valid <= 1'b1;
            end
          end
        end
        default: dcu_state <= D_IDLE;
      endcase
    end
  end
end

endmodule
