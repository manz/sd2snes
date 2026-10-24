`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// Module Name:    spc7110_dec
// Description:    SPC7110 decompression engine (context-adaptive arithmetic
//                 decoder). Bit-exact port of the bsnes/higan decompressor
//                 (original implementation by neviksti, optimized by talarubi).
//
//                 One "decode" produces 8 pixels: 1, 2 or 4 bytes of output
//                 depending on mode (1bpp, 2bpp, 4bpp), returned in `result`
//                 in the same layout as bsnes' Decompressor::result.
//
//                 Compressed input is consumed from a byte stream
//                 (in_data/in_valid/in_take). The engine stalls while
//                 in_valid is low.
//
//                 Timing: 3 cycles per bit (context lookup, arithmetic,
//                 renormalize + next context index), i.e. 24/48/96 cycles per
//                 decode for 1/2/4bpp (+1 per start, + input stalls); 75 cycles
//                 to clear the contexts on init. Sized for the Mk2 XC3S400.
//////////////////////////////////////////////////////////////////////////////////
module spc7110_dec(
  input clk,
  input init,              // (re)initialize: reset contexts, load 2 stream bytes
  input [1:0] mode,        // latched on init: 0=1bpp, 1=2bpp, 2=4bpp
  input start,             // run `count` decodes back-to-back; ignored while busy
  input [16:0] count,
  output busy,
  output reg [31:0] result,

  input [7:0] in_data,
  input in_valid,
  output in_take
);

localparam [2:0]
  S_IDLE  = 3'd0,
  S_CLEAR = 3'd1,
  S_INIT0 = 3'd2,
  S_INIT1 = 3'd3,
  S_LOOK  = 3'd4,
  S_ARITH = 3'd5,
  S_NORM  = 3'd6;

reg [2:0] state = S_IDLE;
assign busy = (state != S_IDLE);

reg [1:0]  bppm = 0;        // log2(bpp)
reg [2:0]  pixel = 0;
reg [1:0]  plane = 0;
reg [8:0]  range = 9'h100;
reg [15:0] inp = 0;
reg [3:0]  bits = 4'd8;
reg [7:0]  outb = 0;
reg [35:0] pixels = 0;      // bsnes keeps 64 bits; only [35:0] are ever observed
reg [63:0] colormap = 64'hfedcba9876543210;
reg [63:0] map = 0;
reg [2:0]  diff = 0;
reg [16:0] remaining = 0;
reg [6:0]  clr_idx = 0;

// contexts {swap, prediction}: index = set * 15 + (bit + history - 1); set 0..4
(* ram_style = "distributed" *) reg [6:0] ctx_mem [0:127];
reg [6:0] cidx = 0;
wire [6:0] ctx_rd = ctx_mem[cidx];

// model of the current bit (S_LOOK)
reg [5:0] m_pred = 0;
reg [7:0] m_prob = 0;
reg [5:0] m_next0 = 0;
reg [5:0] m_next1 = 0;
reg       c_swap = 0;

// arithmetic result of the current bit (S_ARITH)
reg        a_sym = 0;
reg [7:0]  a_range = 0;
reg [15:0] a_inp = 0;

// move-to-front sequencer, one step per cycle after a pixel starts:
//   step 0: map = mtf(colormap, pc), colormap = mtf(colormap, pa)
//   step 1: map = mtf(map, pb)
//   step 2: map = mtf(map, pa)
reg [1:0] mstep = 2'd3;     // step 0..2, 3 = idle
reg [3:0] pa_r = 0, pb_r = 0, pc_r = 0;
reg [3:0] map_nib = 0;      // nibble of the next map step, registered ahead

wire [3:0] bpp = 4'd1 << bppm;

/* model state evolution table: {probability, next[MPS], next[LPS]} */
function [19:0] evolution;
  input [5:0] s;
  begin
    case(s)
      6'd0:  evolution = {8'h5a, 6'd1,  6'd1 };
      6'd1:  evolution = {8'h25, 6'd2,  6'd6 };
      6'd2:  evolution = {8'h11, 6'd3,  6'd8 };
      6'd3:  evolution = {8'h08, 6'd4,  6'd10};
      6'd4:  evolution = {8'h03, 6'd5,  6'd12};
      6'd5:  evolution = {8'h01, 6'd5,  6'd15};
      6'd6:  evolution = {8'h5a, 6'd7,  6'd7 };
      6'd7:  evolution = {8'h3f, 6'd8,  6'd19};
      6'd8:  evolution = {8'h2c, 6'd9,  6'd21};
      6'd9:  evolution = {8'h20, 6'd10, 6'd22};
      6'd10: evolution = {8'h17, 6'd11, 6'd23};
      6'd11: evolution = {8'h11, 6'd12, 6'd25};
      6'd12: evolution = {8'h0c, 6'd13, 6'd26};
      6'd13: evolution = {8'h09, 6'd14, 6'd28};
      6'd14: evolution = {8'h07, 6'd15, 6'd29};
      6'd15: evolution = {8'h05, 6'd16, 6'd31};
      6'd16: evolution = {8'h04, 6'd17, 6'd32};
      6'd17: evolution = {8'h03, 6'd18, 6'd34};
      6'd18: evolution = {8'h02, 6'd5,  6'd35};
      6'd19: evolution = {8'h5a, 6'd20, 6'd20};
      6'd20: evolution = {8'h48, 6'd21, 6'd39};
      6'd21: evolution = {8'h3a, 6'd22, 6'd40};
      6'd22: evolution = {8'h2e, 6'd23, 6'd42};
      6'd23: evolution = {8'h26, 6'd24, 6'd44};
      6'd24: evolution = {8'h1f, 6'd25, 6'd45};
      6'd25: evolution = {8'h19, 6'd26, 6'd46};
      6'd26: evolution = {8'h15, 6'd27, 6'd25};
      6'd27: evolution = {8'h11, 6'd28, 6'd26};
      6'd28: evolution = {8'h0e, 6'd29, 6'd26};
      6'd29: evolution = {8'h0b, 6'd30, 6'd27};
      6'd30: evolution = {8'h09, 6'd31, 6'd28};
      6'd31: evolution = {8'h08, 6'd32, 6'd29};
      6'd32: evolution = {8'h07, 6'd33, 6'd30};
      6'd33: evolution = {8'h05, 6'd34, 6'd31};
      6'd34: evolution = {8'h04, 6'd35, 6'd33};
      6'd35: evolution = {8'h04, 6'd36, 6'd33};
      6'd36: evolution = {8'h03, 6'd37, 6'd34};
      6'd37: evolution = {8'h02, 6'd38, 6'd35};
      6'd38: evolution = {8'h02, 6'd5,  6'd36};
      6'd39: evolution = {8'h58, 6'd40, 6'd39};
      6'd40: evolution = {8'h4d, 6'd41, 6'd47};
      6'd41: evolution = {8'h43, 6'd42, 6'd48};
      6'd42: evolution = {8'h3b, 6'd43, 6'd49};
      6'd43: evolution = {8'h34, 6'd44, 6'd50};
      6'd44: evolution = {8'h2e, 6'd45, 6'd51};
      6'd45: evolution = {8'h29, 6'd46, 6'd44};
      6'd46: evolution = {8'h25, 6'd24, 6'd45};
      6'd47: evolution = {8'h56, 6'd48, 6'd47};
      6'd48: evolution = {8'h4f, 6'd49, 6'd47};
      6'd49: evolution = {8'h47, 6'd50, 6'd48};
      6'd50: evolution = {8'h41, 6'd51, 6'd49};
      6'd51: evolution = {8'h3c, 6'd52, 6'd50};
      6'd52: evolution = {8'h37, 6'd43, 6'd51};
      default: evolution = {8'h5a, 6'd1, 6'd1};
    endcase
  end
endfunction

/* extract a nibble and move it to the low four bits (move-to-front list)
   The lists are always permutations of 0..15, so the nibble is always found.
   Nibble i takes nibble i-1 when the match lies at or above i; each `before`
   bit is an independent OR reduction (shallow trees, no ripple chain). */
function [63:0] mtf;
  input [63:0] list;
  input [3:0] nibble;
  integer n, k;
  reg [15:0] hit;
  reg [15:0] before;  // before[i]: the match is at an index < i
  begin
    for(n = 0; n < 16; n = n + 1) hit[n] = (list[4*n +: 4] == nibble);
    for(n = 0; n < 16; n = n + 1) begin
      before[n] = 1'b0;
      for(k = 0; k < 16; k = k + 1)
        if(k < n) before[n] = before[n] | hit[k];
    end
    mtf[3:0] = nibble;
    for(n = 1; n < 16; n = n + 1)
      mtf[4*n +: 4] = before[n] ? list[4*n +: 4] : list[4*(n-1) +: 4];
  end
endfunction

/* inverse morton code transform: unpack big-endian packed pixels */
function [31:0] deinterleave;
  input [63:0] data;
  input [6:0] nbits;
  reg [63:0] x;
  begin
    x = data & ((64'd1 << nbits) - 64'd1);
    x = 64'h5555555555555555 & ((x << nbits) | (x >> 1));
    x = 64'h3333333333333333 & (x | (x >> 1));
    x = 64'h0f0f0f0f0f0f0f0f & (x | (x >> 2));
    x = 64'h00ff00ff00ff00ff & (x | (x >> 4));
    x = 64'h0000ffff0000ffff & (x | (x >> 8));
    deinterleave = x[31:0] | x[47:16];
  end
endfunction

/* ---- neighbour pixel classification ---- */
function [2:0] classify;
  input [63:0] px;
  input [1:0] bm;
  reg [3:0] a, b, c, m;
  begin
    a = (bm == 2'd1) ? {2'b00, px[3:2]}   : px[3:0];
    b = (bm == 2'd1) ? {2'b00, px[15:14]} : px[31:28];
    c = (bm == 2'd1) ? {2'b00, px[17:16]} : px[35:32];
    m = a ^ b ^ c;
    classify = ((a == b) && (b == c)) ? 3'd0
             : ((m ^ a) == 4'd0) ? 3'd1
             : ((m ^ b) == 4'd0) ? 3'd2
             : ((m ^ c) == 4'd0) ? 3'd3
             : 3'd4;
  end
endfunction

/* context index for (set, bit, history) */
function [6:0] ctx_index;
  input [2:0] set;
  input [3:0] bt;
  input [3:0] hist;
  begin
    ctx_index = set * 7'd15 + {3'b000, bt} + {3'b000, hist} - 7'd1;
  end
endfunction

/* ---- neighbours of the pixel about to be decoded ---- */
wire [3:0] pa = (bppm == 2'd1) ? {2'b00, pixels[3:2]}   : pixels[3:0];
wire [3:0] pb = (bppm == 2'd1) ? {2'b00, pixels[15:14]} : pixels[31:28];
wire [3:0] pc = (bppm == 2'd1) ? {2'b00, pixels[17:16]} : pixels[35:32];
wire       last_plane = ({2'b00, plane} == bpp - 4'd1);

/* ---- S_LOOK: context model ---- */
wire [19:0] model = evolution(ctx_rd[5:0]);

/* ---- S_ARITH ---- */
wire [7:0] lps_offset = range[7:0] - m_prob;    // range-p fits 8 bits (p >= 1)
wire       symbol = (inp[15:8] >= lps_offset);  // inp >= lps_offset << 8

/* ---- S_NORM: renormalize ---- */
wire [2:0]  nshift = a_range[7] ? 3'd0 : a_range[6] ? 3'd1 : a_range[5] ? 3'd2
                   : a_range[4] ? 3'd3 : a_range[3] ? 3'd4 : a_range[2] ? 3'd5
                   : a_range[1] ? 3'd6 : 3'd7;
wire        need_byte = ({1'b0, nshift} >= bits);
wire [3:0]  byte_shift = {1'b0, nshift} - bits;
wire [15:0] inp_f = (a_inp << nshift) + (need_byte ? ({8'h00, in_data} << byte_shift) : 16'h0);
wire [3:0]  bits_f = need_byte ? (4'd8 - byte_shift) : (bits - {1'b0, nshift});
wire        out_bit = a_sym ^ c_swap;
wire [7:0]  outb_f = {outb[6:0], out_bit};
wire        stall = need_byte & ~in_valid;
wire [5:0]  pred_f = (nshift != 3'd0) ? (a_sym ? m_next1 : m_next0) : m_pred;
wire        swap_f = (a_sym && m_prob > 8'h55) ? ~c_swap : c_swap;

/* ---- S_NORM: pixel finalize (last plane) ---- */
wire [3:0]  index_w = (bppm == 2'd0) ? {3'b000, outb_f[0] ^ pixels[15]}
                    : (bppm == 2'd1) ? {2'b00, outb_f[1:0]}
                    : outb_f[3:0];
wire [63:0] map_cur = (bppm == 2'd0) ? colormap : map;
wire [3:0]  newpix = map_cur[4*index_w +: 4];
wire [35:0] pixels_f = (bppm == 2'd0) ? {pixels[34:0], newpix[0]}
                     : (bppm == 2'd1) ? {pixels[33:0], newpix[1:0]}
                     : {pixels[31:0], newpix};
wire [31:0] deint2 = deinterleave({28'h0, pixels_f}, 7'd16);
wire [31:0] deint4a = deinterleave({28'h0, pixels_f}, 7'd32);
wire [31:0] deint4 = deinterleave({32'h0, deint4a}, 7'd32);
wire [31:0] result_f = (bppm == 2'd0) ? pixels_f[31:0]
                     : (bppm == 2'd1) ? deint2
                     : deint4;

/* ---- S_NORM: context index of the next bit ---- */
wire [2:0] pixel_n = pixel + 3'd1;
wire [1:0] plane_n = plane + 2'd1;
// 2bpp: the neighbours of the next pixel do not include the pixel being finalized
wire [2:0] diff_next = classify({30'h0, pixels[31:0], 2'b00}, 2'd1);
reg [3:0] nb_bit, nb_hist;
reg [2:0] nb_set;
always @* begin
  if(bppm == 2'd0) begin
    nb_bit = 4'd1 << pixel_n[1:0];
    nb_hist = (nb_bit - 4'd1) & outb_f[3:0];
    nb_set = {2'b00, pixel_n[2]};
  end else if(!last_plane) begin
    nb_bit = 4'd1 << plane_n;
    nb_hist = (nb_bit - 4'd1) & outb_f[3:0];
    nb_set = (bppm == 2'd1) ? diff : (plane_n[1] && nb_hist <= 4'd1) ? diff : 3'd0;
  end else begin
    nb_bit = 4'd1;
    nb_hist = 4'd0;
    nb_set = (bppm == 2'd1) ? diff_next : 3'd0;
  end
end
wire [6:0] cidx_next = ctx_index(nb_set, nb_bit, nb_hist);
// first bit of a decode started from idle: pixel 0, plane 0 (bit = 1, history = 0)
wire [6:0] cidx_first = ctx_index((bppm == 2'd1) ? classify({28'h0, pixels}, 2'd1) : 3'd0, 4'd1, 4'd0);

/* ---- move-to-front unit (one step per cycle) ---- */
wire        pixel_start = (state == S_LOOK) && (plane == 2'd0) && (bppm != 2'd0);
wire [63:0] mtf_cm_pc = mtf(colormap, pc_r);
wire [63:0] mtf_cm_pa = mtf(colormap, pa_r);
wire [63:0] mtf_map = mtf(map, map_nib);

/* ---- context RAM write port ---- */
wire       ctx_we = (state == S_CLEAR) | ((state == S_NORM) & ~stall);
wire [6:0] ctx_waddr = (state == S_CLEAR) ? clr_idx : cidx;
wire [6:0] ctx_wdata = (state == S_CLEAR) ? 7'h00 : {swap_f, pred_f};
always @(posedge clk) if(ctx_we) ctx_mem[ctx_waddr] <= ctx_wdata;

assign in_take = in_valid & ((state == S_INIT0) | (state == S_INIT1)
                             | ((state == S_NORM) & need_byte));

always @(posedge clk) begin
  // move-to-front steps run alongside the bit pipeline; they are done before the
  // pixel's last plane is finalized (>= 6 cycles per pixel at 2bpp)
  if(pixel_start) begin
    pa_r <= pa;
    pb_r <= pb;
    pc_r <= pc;
    diff <= classify({28'h0, pixels}, bppm);
    mstep <= 2'd0;
  end else begin
    case(mstep)
      2'd0: begin map <= mtf_cm_pc; colormap <= mtf_cm_pa; map_nib <= pb_r; end
      2'd1: begin map <= mtf_map; map_nib <= pa_r; end
      2'd2: map <= mtf_map;
      default: ;
    endcase
    if(mstep != 2'd3) mstep <= mstep + 2'd1;
  end

  if(init) begin
    bppm <= (mode == 2'd3) ? 2'd2 : mode;
    bits <= 4'd8;
    range <= 9'h100;
    outb <= 8'h00;
    pixels <= 36'h0;
    colormap <= 64'hfedcba9876543210;
    mstep <= 2'd3;
    remaining <= 17'h0;
    clr_idx <= 7'd0;
    state <= S_CLEAR;
  end else begin
    case(state)
      S_IDLE: begin
        if(start && count != 17'h0) begin
          remaining <= count;
          pixel <= 3'd0;
          plane <= 2'd0;
          cidx <= cidx_first;
          state <= S_LOOK;
        end
      end
      S_CLEAR: begin
        clr_idx <= clr_idx + 7'd1;
        if(clr_idx == 7'd74) state <= S_INIT0;
      end
      S_INIT0: begin
        if(in_valid) begin
          inp[15:8] <= in_data;
          state <= S_INIT1;
        end
      end
      S_INIT1: begin
        if(in_valid) begin
          inp[7:0] <= in_data;
          state <= S_IDLE;
        end
      end
      S_LOOK: begin
        m_pred <= ctx_rd[5:0];
        c_swap <= ctx_rd[6];
        {m_prob, m_next0, m_next1} <= model;
        state <= S_ARITH;
      end
      S_ARITH: begin
        a_sym <= symbol;
        a_range <= symbol ? (range[7:0] - lps_offset) : lps_offset;
        a_inp <= symbol ? {inp[15:8] - lps_offset, inp[7:0]} : inp;
        state <= S_NORM;
      end
      S_NORM: begin
        if(!stall) begin
          range <= {1'b0, a_range << nshift};
          inp <= inp_f;
          bits <= bits_f;
          outb <= outb_f;
          cidx <= cidx_next;
          state <= S_LOOK;
          if(!last_plane) begin
            plane <= plane_n;
          end else begin
            plane <= 2'd0;
            pixels <= pixels_f;
            pixel <= pixel_n;
            if(pixel == 3'd7) begin
              result <= result_f;
              remaining <= remaining - 17'd1;
              if(remaining == 17'd1) state <= S_IDLE;
            end
          end
        end
      end
      default: state <= S_IDLE;
    endcase
  end
end

endmodule
