`timescale 1 ns / 1 ns
// Replays ops.txt (see gen.cpp) against spc7110 and compares every read
// with the bsnes reference model.
module tb_spc7110;

parameter GAP = 40;          // cycles between consecutive $4810 reads (DMA: 8 master
                             // clocks/byte = ~30 cycles at 80MHz, ~36 at 96MHz)
parameter CGAP = 100;        // cycles before any other read (CPU: >= ~4 cycles/access)
parameter WGAP = 100;        // cycles after a write (CPU: next access >= ~3 SNES cycles)
parameter LAT_MIN = 4;       // data ROM bus latency (cycles)
parameter LAT_RND = 20;

reg CLK = 0;
always #5 CLK = ~CLK;

reg RST = 1;
reg [5:0] reg_addr = 0;
reg [7:0] reg_data_in = 0;
reg reg_we = 0;
reg reg_rd_end = 0;
reg [23:0] SNES_ADDR = 0;

wire [7:0] reg_data_out;
wire [23:0] ROM_ADDR;
wire ROM_ZERO, SRAM_EN;
wire [23:0] BUS_ADDR;
wire BUS_RRQ;
reg BUS_ACK = 0;
reg [15:0] BUS_DI = 0;
wire dcu_underrun;

spc7110 dut(
  .CLK(CLK),
  .RST(RST),
  .reg_addr(reg_addr),
  .reg_data_in(reg_data_in),
  .reg_data_out(reg_data_out),
  .reg_we(reg_we),
  .reg_rd_end(reg_rd_end),
  .SNES_ADDR(SNES_ADDR),
  .ROM_ADDR(ROM_ADDR),
  .ROM_ZERO(ROM_ZERO),
  .SRAM_EN(SRAM_EN),
  .DROM_MASK(24'h0fffff),
  .BUS_ADDR(BUS_ADDR),
  .BUS_RRQ(BUS_RRQ),
  .BUS_ACK(BUS_ACK),
  .BUS_DI(BUS_DI),
  .dcu_underrun(dcu_underrun)
);

reg [7:0] mem [0:24'h1fffff];

/* data ROM bus model: one outstanding request, random latency */
reg [23:0] bus_a;
integer bus_lat = -1;
always @(posedge CLK) begin
  BUS_ACK <= 1'b0;
  if(BUS_RRQ) begin
    bus_a <= BUS_ADDR;
    bus_lat <= LAT_MIN + ({$random} % (LAT_RND + 1));
  end else if(bus_lat > 0) begin
    bus_lat <= bus_lat - 1;
  end else if(bus_lat == 0) begin
    bus_lat <= -1;
    BUS_ACK <= 1'b1;
    BUS_DI <= {mem[{bus_a[23:1], 1'b0}], mem[{bus_a[23:1], 1'b1}]};
  end
end

integer fd, r, errors = 0, checks = 0, line = 0, n, t;
reg [7:0] op;
reg [31:0] a, d;
reg [7:0] got;

reg [5:0] last_rd = 6'h3f;   // address of the previous read (3f: a write came after it)

task wait_cycles(input integer c);
  integer i;
  for(i = 0; i < c; i = i + 1) @(posedge CLK);
endtask

task snes_write(input [5:0] ad, input [7:0] dd);
  begin
    last_rd = 6'h3f;
    reg_addr <= ad; reg_data_in <= dd; reg_we <= 1;
    @(posedge CLK); reg_we <= 0;
    wait_cycles(WGAP);
  end
endtask

task snes_read(input [5:0] ad, output [7:0] dd);
  begin
    // only a DMA can read back to back, and a DMA reads one address
    if(!(ad == 6'h10 && last_rd == 6'h10)) wait_cycles(CGAP - GAP);
    last_rd = ad;
    reg_addr <= ad;
    @(posedge CLK); #1 dd = reg_data_out;
    reg_rd_end <= 1;
    @(posedge CLK); reg_rd_end <= 0;
    wait_cycles(GAP);
  end
endtask

initial begin
  $readmemh("mem.hex", mem);
  fd = $fopen("ops.txt", "r");
  if(fd == 0) begin $display("cannot open ops.txt"); $finish; end
  wait_cycles(4);
  RST <= 0;
  wait_cycles(4);
  op = 0;
  while(op != "E") begin
    r = $fscanf(fd, " %c", op);
    line = line + 1;
    case(op)
      "W": begin r = $fscanf(fd, "%h %h", a, d); snes_write(a[5:0], d[7:0]); end
      "R": begin
        r = $fscanf(fd, "%h %h", a, d);
        snes_read(a[5:0], got);
        checks = checks + 1;
        if(got !== d[7:0]) begin
          errors = errors + 1;
          if(errors <= 20) $display("line %0d: R $48%02x got %02x expected %02x", line, a[5:0], got, d[7:0]);
        end
      end
      "M": begin
        r = $fscanf(fd, "%h %h", a, d);
        SNES_ADDR <= a[23:0];
        @(posedge CLK); #1;
        got = ROM_ZERO ? 8'h00 : mem[ROM_ADDR[20:0]];
        checks = checks + 1;
        if(got !== d[7:0]) begin
          errors = errors + 1;
          if(errors <= 20) $display("line %0d: M %06x -> %06x%s got %02x expected %02x",
                                    line, a[23:0], ROM_ADDR, ROM_ZERO ? " (zero)" : "", got, d[7:0]);
        end
      end
      "S": begin r = $fscanf(fd, "%h", n); wait_cycles(n); end
      "D": begin
        t = 0;
        reg_addr <= 6'h0c;
        @(posedge CLK); #1;
        while(!reg_data_out[7] && t < 2000000) begin @(posedge CLK); #1; t = t + 1; end
        if(t >= 2000000) begin
          errors = errors + 1;
          $display("line %0d: timeout waiting for DCU ready", line);
        end
      end
      "E": ;
      default: begin $display("line %0d: bad op '%c'", line, op); op = "E"; end
    endcase
  end
  $display("%0d checks, %0d errors, dcu_underrun=%0d", checks, errors, dcu_underrun);
  $finish;
end

endmodule
