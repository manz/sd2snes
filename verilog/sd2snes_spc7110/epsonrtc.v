`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// Module Name:    epsonrtc
// Description:    Epson RTC-4513 real-time clock as used on the SPC7110 board
//                 ($4840-$4842). Port of the bsnes/higan model, including its
//                 handling of invalid BCD values.
//
//                 `tick` is a 2.097152 MHz (32768 Hz * 64) clock enable, the
//                 rate bsnes steps the chip at. Carries between time fields
//                 are resolved over a few cycles after a tick (`busy`); ticks
//                 arriving meanwhile are held.
//
//                 The chip state is saved and restored by the MCU in the bsnes
//                 format (EpsonRTC::save/load, first 8 bytes): `state_out` /
//                 `state_in`. `dirty` is set whenever the SNES writes a chip
//                 register, so the MCU knows when to save.
//
//                 The time can also be set from the MCU via `time_we`/`time_in`
//                 (FPGA_CMD_RTCSET format, BCD YYMMDDhhmmss, with the weekday
//                 0=Sunday in the thousands-of-year nibble), like bsnes'
//                 synchronize().
//////////////////////////////////////////////////////////////////////////////////
module epsonrtc(
  input CLK,
  input RST,                // power-on initialization
  input tick,

  input [1:0] reg_addr,     // $4840-$4842 (3: open bus, not decoded)
  input [7:0] reg_data_in,
  output reg [7:0] reg_data_out,
  input reg_we,             // one-cycle strobe at end of SNES write
  input reg_rd_end,         // one-cycle strobe at end of SNES read (side effects)

  input [55:0] time_in,
  input time_we,

  input [63:0] state_in,
  input state_we,
  output [63:0] state_out,
  output reg dirty,
  input dirty_clr,

  output busy
);

localparam [1:0] ST_MODE = 2'd0, ST_SEEK = 2'd1, ST_READ = 2'd2, ST_WRITE = 2'd3;

reg [20:0] clocks;
reg [10:0] seconds;       // bsnes counts seconds modulo 1440 for the hour IRQ
reg [5:0]  seconds60;     // seconds % 60

reg [1:0]  chipselect;
reg [1:0]  state;
reg [3:0]  mdr;
reg [3:0]  offset;
reg [3:0]  wait_cnt;
reg        ready;
reg        holdtick;

reg [3:0] secondlo;  reg [2:0] secondhi;  reg batteryfailure;
reg [3:0] minutelo;  reg [2:0] minutehi;  reg resync;
reg [3:0] hourlo;    reg [1:0] hourhi;    reg meridian;
reg [3:0] daylo;     reg [1:0] dayhi;     reg dayram;
reg [3:0] monthlo;   reg       monthhi;   reg [1:0] monthram;
reg [3:0] yearlo;    reg [3:0] yearhi;
reg [2:0] weekday;
reg hold, calendar, irqflag, roundseconds;
reg irqmask, irqduty;     reg [1:0] irqperiod;
reg pause, stop, atime, test;

/* ---- bsnes EpsonRTC::save() layout, byte 0 in [7:0] ---- */
assign state_out = {
  {test, atime, stop, pause, irqperiod, irqduty, irqmask},
  {roundseconds, irqflag, calendar, hold, resync, weekday},
  {yearhi, yearlo},
  {resync, monthram, monthhi, monthlo},
  {resync, dayram, dayhi, daylo},
  {resync, meridian, hourhi, hourlo},
  {resync, minutehi, minutelo},
  {batteryfailure, secondhi, secondlo}
};

/* ---- register read value (bsnes rtcRead, without side effects) ---- */
function [3:0] rtc_read;
  input [3:0] a;
  begin
    case(a)
      4'd0:  rtc_read = secondlo;
      4'd1:  rtc_read = {batteryfailure, secondhi};
      4'd2:  rtc_read = minutelo;
      4'd3:  rtc_read = {resync, minutehi};
      4'd4:  rtc_read = hourlo;
      4'd5:  rtc_read = {resync, meridian, hourhi};
      4'd6:  rtc_read = daylo;
      4'd7:  rtc_read = {resync, dayram, dayhi};
      4'd8:  rtc_read = monthlo;
      4'd9:  rtc_read = {resync, monthram, monthhi};
      4'd10: rtc_read = yearlo;
      4'd11: rtc_read = yearhi;
      4'd12: rtc_read = {resync, weekday};
      4'd13: rtc_read = {roundseconds, irqflag & ~irqmask, calendar, hold};
      4'd14: rtc_read = {irqperiod, irqduty, irqmask};
      default: rtc_read = {test, atime, stop, pause};
    endcase
  end
endfunction

always @* begin
  case(reg_addr)
    2'd0: reg_data_out = {6'h00, chipselect};
    2'd1: reg_data_out = (chipselect != 2'd1 || !ready) ? 8'h00
                       : (state == ST_WRITE) ? {4'h0, mdr}
                       : (state == ST_READ) ? {4'h0, rtc_read(offset)}
                       : 8'h00;
    2'd2: reg_data_out = {ready, 7'h00};
    default: reg_data_out = 8'h00;
  endcase
end

/* ---- BCD digit increment used by every field: values <= 8 and 12 count up ---- */
function digit_inc;   // 1: increment, 0: wrap (to !(v & 1) or 0) and carry
  input [3:0] v;
  digit_inc = (v <= 4'd8) || (v == 4'd12);
endfunction

/* ---- 24-hour BCD hour (00-23) to 12-hour BCD (01-12) ---- */
function [5:0] hour12;   // {hourhi[1:0], hourlo}
  input [5:0] bcd;
  reg [4:0] h;
  begin
    h = bcd[5:4] * 4'd10 + bcd[3:0];
    if(h >= 5'd12) h = h - 5'd12;
    if(h == 5'd0) h = 5'd12;
    hour12 = (h >= 5'd10) ? {2'd1, h[3:0] - 4'd10} : {2'd0, h[3:0]};
  end
endfunction

/* ---- days in the current month (bsnes daysinmonth + leap year rule) ---- */
wire [4:0] month_idx = {monthhi, monthlo};
wire [4:0] days_base = (month_idx == 5'd2) ? 5'd28
                     : (month_idx[4] ? ~month_idx[0] : (month_idx == 5'd0) ? 1'b0
                        : (month_idx <= 5'd7) ? month_idx[0] : ~month_idx[0]) ? 5'd31 : 5'd30;
wire leap = (~yearhi[0] & (yearlo[1:0] == 2'd0)) | (yearhi[0] & (yearlo[1:0] == 2'd2));
wire [4:0] days = (days_base == 5'd28 && leap) ? 5'd29 : days_base;
wire month_end =
     (days == 5'd28 && (dayhi == 2'd3 || (dayhi == 2'd2 && daylo >= 4'd8)))
  || (days == 5'd29 && (dayhi == 2'd3 || (dayhi == 2'd2 && daylo > 4'd8 && daylo != 4'd12)))
  || (days == 5'd30 && (dayhi == 2'd3 || (dayhi == 2'd2 && (daylo == 4'd10 || daylo == 4'd14))))
  || (days == 5'd31 && (dayhi == 2'd3 && daylo[1:0] != 2'd0));

/* ---- carry sequencer: one time field per cycle ---- */
localparam [2:0] C_IDLE = 3'd0, C_SEC = 3'd1, C_MIN = 3'd2, C_HOUR = 3'd3,
                 C_DAY = 3'd4, C_MONTH = 3'd5, C_YEAR = 3'd6;
reg [2:0] cstate;
reg pend_sec, pend_min, pend_tick;
assign busy = (cstate != C_IDLE) | pend_sec | pend_min | pend_tick;

wire [20:0] clocks_n = clocks + 21'd1;
wire [10:0] seconds_n = seconds + 11'd1;
wire        rtc_idle = (cstate == C_IDLE) & ~pend_sec & ~pend_min;

always @(posedge CLK) begin
  if(RST) begin
    clocks <= 0; seconds <= 0; seconds60 <= 0;
    chipselect <= 0; state <= ST_MODE; offset <= 0; mdr <= 0;
    wait_cnt <= 0; ready <= 0; holdtick <= 0;
    secondlo <= 0; secondhi <= 0; batteryfailure <= 1'b1;
    minutelo <= 0; minutehi <= 0; resync <= 0;
    hourlo <= 0; hourhi <= 0; meridian <= 0;
    daylo <= 0; dayhi <= 0; dayram <= 0;
    monthlo <= 0; monthhi <= 0; monthram <= 0;
    yearlo <= 0; yearhi <= 0; weekday <= 0;
    hold <= 0; calendar <= 0; irqflag <= 0; roundseconds <= 0;
    irqmask <= 0; irqduty <= 0; irqperiod <= 0;
    pause <= 0; stop <= 0; atime <= 0; test <= 0;
    cstate <= C_IDLE; pend_sec <= 0; pend_min <= 0; pend_tick <= 0;
    dirty <= 0;
  end else begin
    if(dirty_clr) dirty <= 1'b0;
    /* -------- 2.097152 MHz step (bsnes EpsonRTC::main) -------- */
    if(tick & ~rtc_idle) pend_tick <= 1'b1;
    if((tick | pend_tick) & rtc_idle) begin
      pend_tick <= 1'b0;
      if(wait_cnt != 0) begin
        wait_cnt <= wait_cnt - 4'd1;
        if(wait_cnt == 4'd1) ready <= 1'b1;
      end
      clocks <= clocks_n;
      // roundSeconds (first 125us of each second)
      if(clocks_n[20:8] == 0 && roundseconds) begin
        roundseconds <= 1'b0;
        if(secondhi >= 3'd3) pend_min <= 1'b1;
        secondlo <= 0;
        secondhi <= 0;
      end
      // duty (first 1/128 s): clear, then irq(0) (first 1/64 s): set
      if(clocks_n[20:14] == 0 && irqduty) irqflag <= 1'b0;
      if(clocks_n[20:15] == 0 && !(stop | pause) && irqperiod == 2'd0) irqflag <= 1'b1;
      if(clocks_n == 0) begin
        seconds <= (seconds_n == 11'd1440) ? 11'd0 : seconds_n;
        seconds60 <= (seconds60 == 6'd59) ? 6'd0 : seconds60 + 6'd1;
        if(!(stop | pause)) begin
          if(irqperiod == 2'd1
             || (irqperiod == 2'd2 && seconds60 == 6'd59)
             || (irqperiod == 2'd3 && seconds_n == 11'd1440)) irqflag <= 1'b1;
          // tick
          if(hold) holdtick <= 1'b1;
          else begin
            resync <= 1'b1;
            pend_sec <= 1'b1;
          end
        end
      end
    end

    /* -------- field carries (bsnes tickSecond .. tickYear) -------- */
    case(cstate)
      C_IDLE: begin
        if(pend_min) begin pend_min <= 1'b0; cstate <= C_MIN; end
        else if(pend_sec) begin pend_sec <= 1'b0; cstate <= C_SEC; end
      end
      C_SEC: begin
        cstate <= C_IDLE;
        if(digit_inc(secondlo)) secondlo <= secondlo + 4'd1;
        else begin
          secondlo <= 0;
          if(secondhi < 3'd5) secondhi <= secondhi + 3'd1;
          else begin secondhi <= 0; cstate <= C_MIN; end
        end
      end
      C_MIN: begin
        cstate <= C_IDLE;
        if(digit_inc(minutelo)) minutelo <= minutelo + 4'd1;
        else begin
          minutelo <= 0;
          if(minutehi < 3'd5) minutehi <= minutehi + 3'd1;
          else begin minutehi <= 0; cstate <= C_HOUR; end
        end
      end
      C_HOUR: begin
        cstate <= C_IDLE;
        if(atime) begin
          if(hourhi < 2'd2) begin
            if(digit_inc(hourlo)) hourlo <= hourlo + 4'd1;
            else begin hourlo <= {3'b000, ~hourlo[0]}; hourhi <= hourhi + 2'd1; end
          end else if(hourlo != 4'd3 && !hourlo[2]) begin
            if(hourlo <= 4'd8 || hourlo >= 4'd12) hourlo <= hourlo + 4'd1;
            else begin hourlo <= {3'b000, ~hourlo[0]}; hourhi <= hourhi + 2'd1; end
          end else begin
            hourlo <= {3'b000, ~hourlo[0]};
            hourhi <= 2'd0;
            cstate <= C_DAY;
          end
        end else if(hourhi == 2'd0) begin
          if(digit_inc(hourlo)) hourlo <= hourlo + 4'd1;
          else begin hourlo <= {3'b000, ~hourlo[0]}; hourhi <= hourhi ^ 2'd1; end
        end else begin : h12
          reg m;
          reg [3:0] lo;
          m = meridian ^ hourlo[0];
          if(hourlo < 4'd2 || hourlo == 4'd4 || hourlo == 4'd5 || hourlo == 4'd8 || hourlo == 4'd12)
            lo = hourlo + 4'd1;
          else begin
            lo = {3'b000, ~hourlo[0]};
            hourhi <= hourhi ^ 2'd1;
          end
          meridian <= m;
          hourlo <= lo;
          if(!m && !lo[0]) cstate <= C_DAY;
        end
      end
      C_DAY: begin
        cstate <= C_IDLE;
        if(calendar) begin
          weekday <= (weekday == 3'd6) ? 3'd0 : weekday + 3'd1;
          if(month_end) begin
            daylo <= 4'd1;
            dayhi <= 2'd0;
            cstate <= C_MONTH;
          end else if(digit_inc(daylo)) daylo <= daylo + 4'd1;
          else begin daylo <= {3'b000, ~daylo[0]}; dayhi <= dayhi + 2'd1; end
        end
      end
      C_MONTH: begin
        cstate <= C_IDLE;
        if(!monthhi || !monthlo[1]) begin
          if(digit_inc(monthlo)) monthlo <= monthlo + 4'd1;
          else begin monthlo <= {3'b000, ~monthlo[0]}; monthhi <= ~monthhi; end
        end else begin
          monthlo <= {3'b000, ~monthlo[0]};
          monthhi <= 1'b0;
          cstate <= C_YEAR;
        end
      end
      C_YEAR: begin
        cstate <= C_IDLE;
        if(digit_inc(yearlo)) yearlo <= yearlo + 4'd1;
        else begin
          yearlo <= {3'b000, ~yearlo[0]};
          if(digit_inc(yearhi)) yearhi <= yearhi + 4'd1;
          else yearhi <= {3'b000, ~yearhi[0]};
        end
      end
      default: cstate <= C_IDLE;
    endcase

    /* -------- SNES register access (bsnes read/write) -------- */
    if(reg_rd_end && reg_addr == 2'd1 && chipselect == 2'd1 && ready && state == ST_READ) begin
      ready <= 1'b0;
      wait_cnt <= 4'd8;
      offset <= offset + 4'd1;
      if(offset == 4'd13) irqflag <= 1'b0;
    end
    if(reg_we) begin
      if(reg_addr == 2'd0) begin
        chipselect <= reg_data_in[1:0];
        ready <= 1'b1;
        if(reg_data_in[1:0] != 2'd1) begin   // rtcReset
          state <= ST_MODE;
          offset <= 0;
          resync <= 0;
          pause <= 0;
          test <= 0;
        end
      end else if(reg_addr == 2'd1 && chipselect == 2'd1 && ready) begin
        case(state)
          ST_MODE: if(reg_data_in[3:0] == 4'h3 || reg_data_in[3:0] == 4'hc) begin
            state <= ST_SEEK;
            ready <= 1'b0;
            wait_cnt <= 4'd8;
            mdr <= reg_data_in[3:0];
          end
          ST_SEEK: begin
            if(mdr == 4'h3) state <= ST_WRITE;
            if(mdr == 4'hc) state <= ST_READ;
            offset <= reg_data_in[3:0];
            ready <= 1'b0;
            wait_cnt <= 4'd8;
            mdr <= reg_data_in[3:0];
          end
          ST_WRITE: begin : rtc_write
            reg [3:0] d;
            d = reg_data_in[3:0];
            dirty <= 1'b1;
            offset <= offset + 4'd1;
            ready <= 1'b0;
            wait_cnt <= 4'd8;
            mdr <= d;
            case(offset)
              4'd0:  secondlo <= d;
              4'd1:  begin secondhi <= d[2:0]; batteryfailure <= d[3]; end
              4'd2:  minutelo <= d;
              4'd3:  minutehi <= d[2:0];
              4'd4:  hourlo <= d;
              4'd5:  begin
                       hourhi <= atime ? d[1:0] : {1'b0, d[0]};
                       meridian <= atime ? 1'b0 : d[2];
                     end
              4'd6:  daylo <= d;
              4'd7:  begin dayhi <= d[1:0]; dayram <= d[2]; end
              4'd8:  monthlo <= d;
              4'd9:  begin monthhi <= d[0]; monthram <= d[2:1]; end
              4'd10: yearlo <= d;
              4'd11: yearhi <= d;
              4'd12: weekday <= d[2:0];
              4'd13: begin
                       hold <= d[0];
                       calendar <= d[1];
                       roundseconds <= d[3];
                       // a second passed during hold: count it on release
                       if(hold && !d[0] && holdtick) begin
                         holdtick <= 1'b0;
                         pend_sec <= 1'b1;
                       end
                     end
              4'd14: begin irqmask <= d[0]; irqduty <= d[1]; irqperiod <= d[3:2]; end
              4'd15: begin
                       pause <= d[0];
                       stop <= d[1];
                       atime <= d[2];
                       test <= d[3];
                       if(d[2]) meridian <= 1'b0;
                       else hourhi <= {1'b0, hourhi[0]};
                       if(d[0]) begin secondlo <= 0; secondhi <= 0; end
                     end
            endcase
          end
          default: ;
        endcase
      end
    end

    /* -------- time from the MCU (bsnes synchronize) -------- */
    if(time_we) begin : sync
      reg [5:0] h12;
      secondlo <= time_in[3:0];
      secondhi <= time_in[6:4];
      minutelo <= time_in[11:8];
      minutehi <= time_in[14:12];
      h12 = hour12(time_in[21:16]);
      if(atime) begin
        hourlo <= time_in[19:16];
        hourhi <= time_in[21:20];
      end else begin
        meridian <= (time_in[21:16] >= 6'h12);
        hourlo <= h12[3:0];
        hourhi <= h12[5:4];
      end
      daylo <= time_in[27:24];
      dayhi <= time_in[29:28];
      monthlo <= time_in[35:32];
      monthhi <= time_in[36];
      yearlo <= time_in[43:40];
      yearhi <= time_in[47:44];
      weekday <= time_in[54:52];
      resync <= 1'b1;
    end

    /* -------- saved state from the MCU (bsnes load, first 8 bytes) -------- */
    if(state_we) begin
      secondlo <= state_in[3:0];   secondhi <= state_in[6:4];   batteryfailure <= state_in[7];
      minutelo <= state_in[11:8];  minutehi <= state_in[14:12]; resync <= state_in[15];
      hourlo <= state_in[19:16];   hourhi <= state_in[21:20];   meridian <= state_in[22];
      daylo <= state_in[27:24];    dayhi <= state_in[29:28];    dayram <= state_in[30];
      monthlo <= state_in[35:32];  monthhi <= state_in[36];     monthram <= state_in[38:37];
      yearlo <= state_in[43:40];   yearhi <= state_in[47:44];
      weekday <= state_in[50:48];
      hold <= state_in[52];        calendar <= state_in[53];
      irqflag <= state_in[54];     roundseconds <= state_in[55];
      irqmask <= state_in[56];     irqduty <= state_in[57];     irqperiod <= state_in[59:58];
      pause <= state_in[60];       stop <= state_in[61];
      atime <= state_in[62];       test <= state_in[63];
    end
  end
end

endmodule
