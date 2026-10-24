// Port-compatible stubs of the Quartus IP cores, for lint/elaboration only.
module pll(input areset, input inclk0, output c0, output locked);
  assign c0 = inclk0; assign locked = 1'b1;
endmodule
module snescmd_buf(input [8:0] address_a, input [8:0] address_b, input clock,
  input [7:0] data_a, input [7:0] data_b, input wren_a, input wren_b,
  output [7:0] q_a, output [7:0] q_b);
  assign q_a = 8'h00; assign q_b = 8'h00;
endmodule
module dac_buf(input clock, input [7:0] data, input [8:0] rdaddress,
  input [10:0] wraddress, input wren, output [31:0] q);
  assign q = 32'h0;
endmodule
module msu_databuf(input clock, input [7:0] data, input [13:0] rdaddress,
  input [13:0] wraddress, input wren, output [7:0] q);
  assign q = 8'h00;
endmodule
