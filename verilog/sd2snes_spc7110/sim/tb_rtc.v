`timescale 1 ns / 1 ns
// Replays ops_rtc.txt (see gen_rtc.cpp) against epsonrtc and compares every
// read with the bsnes reference model.
module tb_rtc;

reg CLK = 0;
always #5 CLK = ~CLK;

reg RST = 1;
reg tick = 0;
reg [1:0] reg_addr = 0;
reg [7:0] reg_data_in = 0;
reg reg_we = 0;
reg reg_rd_end = 0;
reg [55:0] time_in = 0;
reg time_we = 0;
reg [63:0] state_in = 0;
reg state_we = 0;
wire [63:0] state_out;
wire dirty;
wire [7:0] reg_data_out;
wire busy;

epsonrtc dut(
  .CLK(CLK),
  .RST(RST),
  .tick(tick),
  .reg_addr(reg_addr),
  .reg_data_in(reg_data_in),
  .reg_data_out(reg_data_out),
  .reg_we(reg_we),
  .reg_rd_end(reg_rd_end),
  .time_in(time_in),
  .time_we(time_we),
  .state_in(state_in),
  .state_we(state_we),
  .state_out(state_out),
  .dirty(dirty),
  .dirty_clr(1'b0),
  .busy(busy)
);

integer fd, r, errors = 0, checks = 0, line = 0, i;
reg [7:0] op;
reg [63:0] a, d;
reg [7:0] got;

// bsnes steps are instantaneous: let carries settle before anything else
task idle;
  begin
    @(posedge CLK);
    while(busy) @(posedge CLK);
  end
endtask

initial begin
  fd = $fopen("ops_rtc.txt", "r");
  if(fd == 0) begin $display("cannot open ops_rtc.txt"); $finish; end
  repeat(4) @(posedge CLK);
  RST <= 0;
  op = 0;
  while(op != "E") begin
    r = $fscanf(fd, " %c", op);
    line = line + 1;
    case(op)
      "W": begin
        r = $fscanf(fd, "%h %h", a, d);
        idle;
        reg_addr <= a[1:0]; reg_data_in <= d[7:0]; reg_we <= 1;
        @(posedge CLK); reg_we <= 0;
      end
      "R": begin
        r = $fscanf(fd, "%h %h", a, d);
        idle;
        reg_addr <= a[1:0];
        @(posedge CLK); #1 got = reg_data_out;
        reg_rd_end <= 1;
        @(posedge CLK); reg_rd_end <= 0;
        checks = checks + 1;
        if(got !== d[7:0]) begin
          errors = errors + 1;
          if(errors <= 20) $display("line %0d: R $484%0d got %02x expected %02x", line, a[1:0], got, d[7:0]);
        end
      end
      "T": begin
        r = $fscanf(fd, "%h", d);
        for(i = 0; i < d; i = i + 1) begin
          idle;
          tick <= 1; @(posedge CLK); tick <= 0;
        end
      end
      "S": begin
        r = $fscanf(fd, "%h", d);
        idle;
        time_in <= d[55:0]; time_we <= 1;
        @(posedge CLK); time_we <= 0;
      end
      "L": begin
        r = $fscanf(fd, "%h", d);
        idle;
        state_in <= d; state_we <= 1;
        @(posedge CLK); state_we <= 0;
      end
      "G": begin
        r = $fscanf(fd, "%h", d);
        idle;
        checks = checks + 1;
        if(state_out !== d) begin
          errors = errors + 1;
          if(errors <= 20) $display("line %0d: state got %016x expected %016x", line, state_out, d);
        end
      end
      "E": ;
      default: begin $display("line %0d: bad op '%c'", line, op); op = "E"; end
    endcase
  end
  $display("%0d checks, %0d errors", checks, errors);
  $finish;
end

endmodule
