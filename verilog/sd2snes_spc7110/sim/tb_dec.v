`timescale 1 ns / 1 ns
module tb_dec;
parameter MODE = 0;
parameter N = 64;
reg clk = 0; always #5 clk = ~clk;
reg init = 0, start = 0; reg [16:0] count = 1;
wire busy, take; wire [31:0] result;
reg [7:0] rom [0:20'hfffff]; reg [19:0] ptr = 0;
spc7110_dec dut(.clk(clk), .init(init), .mode(MODE[1:0]), .start(start), .count(count),
  .busy(busy), .result(result), .in_data(rom[ptr]), .in_valid(1'b1), .in_take(take));
always @(posedge clk) if(take) ptr <= ptr + 1;
integer i;
initial begin
  $readmemh("dec_rom.hex", rom);
  @(posedge clk); init <= 1; @(posedge clk); init <= 0;
  @(posedge clk); while(busy) @(posedge clk);
  for(i = 0; i < N; i = i + 1) begin
    start <= 1; @(posedge clk); start <= 0; @(posedge clk);
    while(busy) @(posedge clk);
    $display("%08x", result);
  end
  $finish;
end
endmodule
