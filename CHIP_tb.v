`timescale 1 ns/10 ps
`define	H_CYCLE 5
`define CYCLE 10

module CHIP_tb;

	reg clk;
	reg rst;
	reg IWEN;
	reg sclk;
	
	reg [7:0] gpio_in;

	reg wInst;

	wire [7:0] gpio_out;

	Chip chip(
		.clk(clk),
		.rst(rst),
		.gpio_in(gpio_in),
		.gpio_out(gpio_out),
		.IWEN(IWEN),
		.sclk(sclk),
		.wInst(wInst)
		);

	initial begin
		clk = 0;
		sclk = 1;
		rst = 1'b1;
		IWEN = 1'b0;
		gpio_in = 8'b01011010;
		#(`CYCLE*0.1) rst = 1'b0;
		#(`CYCLE*0.5) rst = 1'b1;

		#(`CYCLE*0.8)

		IWEN = 1'b1;

		send_inst(32'h1304500a);
		send_inst(32'ha32e8006);
		send_inst(32'h1305f00e);
		send_inst(32'ha32fa006);
		send_inst(32'h9304000f);
		send_inst(32'h232f9006);
		send_inst(32'h232e0006);
		send_inst(32'h0326c007);
		send_inst(32'h33068600);

		IWEN = 0;
	end
	always begin
		#`H_CYCLE clk = ~clk;
 	end
 	always begin
 		#(`H_CYCLE*2) sclk = ~sclk;
 	end

 	task send_inst;
 		input [31:0] inst;
 		integer i;
 		begin
 		 	$display("send inst = %x ", inst);
 		 	for (i = 0; i < 32; i = i + 1)
 		 		begin
 		 			#(`H_CYCLE*4)
 		 			if (inst[0]) begin
 		 				wInst = 1;
 		 			end
 		 			else begin
 		 				wInst = 0;
 		 			end
 		 			inst = inst >> 1;
 		 		end
 		 end 
 	endtask
endmodule