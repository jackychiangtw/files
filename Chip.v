`timescale 1 ns/10 ps

`define ALU_OP_WIDTH 4
`define ALU_OP_ADD  `ALU_OP_WIDTH'd0 
`define ALU_OP_SUB  `ALU_OP_WIDTH'd1
`define ALU_OP_XOR  `ALU_OP_WIDTH'd2
`define ALU_OP_OR   `ALU_OP_WIDTH'd3
`define ALU_OP_AND  `ALU_OP_WIDTH'd4
`define ALU_OP_SLL  `ALU_OP_WIDTH'd5
`define ALU_OP_SRL  `ALU_OP_WIDTH'd6
`define ALU_OP_SRA  `ALU_OP_WIDTH'd7
`define ALU_OP_SLT  `ALU_OP_WIDTH'd8
`define ALU_OP_SLTU `ALU_OP_WIDTH'd9
`define ALU_OP_XNOR `ALU_OP_WIDTH'd10
`define ALU_OP_MOD  `ALU_OP_WIDTH'd11

module Chip(
	clk,
	rst,
	gpio_in,
	gpio_out,
	IWEN,
	I_Addr,
	wInst
	);

input clk;
input rst;
input IWEN;

input [6:0]  I_Addr;
input [7:0]  gpio_in;
input [31:0] wInst;

output [7:0] gpio_out;

wire WEN, CEN, OEN, I_ready;
wire Iwen_pos, Iwen_neg, IWen;

wire [7:0] data_in;
wire [6:0] Addr;
wire [6:0] I_addr;

wire [31:0] WriteReg;
wire [31:0] WriteData;
wire [31:0] IR_addr;
wire [31:0] wdata;
wire [31:0] Inst;
wire [31:0] Mem_rdata;
wire [31:0] Mem_wdata;

assign Mem_wdata = (Addr == 7'h7c) ? {{24{1'b0}},data_in} : WriteData;
assign I_addr = IWEN ? I_Addr : IR_addr[8:2];
assign wdata = IWEN ? wInst : Mem_wdata;
assign I_ready = ~IWEN;
assign IWen = IWEN ? Iwen_neg ? 0 : 1 : Iwen_pos ? 1 : 0;

SingleCycle Core(
	.clk(clk),
	.rst(rst),
	.IR_addr(IR_addr),
	.IR(Inst),
	.A(Addr),
	.WriteReg(WriteReg),
	.ReadDataMem(Mem_rdata),
	.WriteData(WriteData),
	.CEN(CEN),
	.WEN(WEN),
	.OEN(OEN),
	.I_ready(I_ready)
	);

GPIO Gpio(
	.clk(clk),
	.rst(rst),
	.Addr(Addr),
	.data_out(Mem_wdata),
	.data_in(data_in),
	.gpio_out(gpio_out),
	.gpio_in(gpio_in)
	);

RAM memory(
	.clk(clk),
	.rst(rst),
	.Addr(Addr),
	.rdata(Mem_rdata),
	.wdata(wdata),
	.WEN(~WEN)
	);


RAM INST(
	.clk(clk),
	.rst(rst),
	.Addr(I_addr),
	.rdata(wdata),
	.wdata(wInst),
	.WEN(IWen)
	);

edge_detect Iwen(
	.clk(clk),
	.rst_n(rst),
	.data_in(IWEN),
	.pos_edge(Iwen_pos),
	.neg_edge(Iwen_neg)
	);

endmodule

module edge_detect (
   clk,
   rst_n,
   data_in,
   pos_edge,
   neg_edge 
   );
input      clk;
input      rst_n;
input      data_in;
output     pos_edge;
output     neg_edge;

reg        data_in_d1;
reg        data_in_d2; 


assign pos_edge =  data_in_d1 & ~data_in_d2;
assign neg_edge =  ~data_in_d1 & data_in_d2;

 
always@(posedge clk or negedge rst_n) 
begin
  if (!rst_n)
  begin
    data_in_d1 <=#1 1'b0;
    data_in_d2 <=#1 1'b0;
  end
  else 
  begin
    data_in_d1 <=#1 data_in;
    data_in_d2 <=#1 data_in_d1;   
  end
end

endmodule 

module SingleCycle(
	clk,
	rst,
	IR_addr,
	IR,
	A,
	WriteReg,
	ReadDataMem,
	WriteData,
	CEN,
	WEN,
	OEN,
	I_ready
	);

input clk, rst, I_ready;
input [31:0] IR;
input [31:0] ReadDataMem;

output [31:0] WriteReg;
output [31:0] WriteData;
output [6:0]  A;
output [31:0] IR_addr;

output CEN, WEN, OEN;

wire [31:0]inst;

wire [4:0]rd 		= inst[11:7];
wire [4:0]rs1 		= inst[19:15];
wire [4:0]rs2 		= inst[24:20];
wire [2:0]func3 	= inst[14:12];
wire [6:0]func7 	= inst[31:25];
wire [6:0]opcode 	= inst[6:0];
wire Rtype 		= opcode == 7'b0110011;
wire Itype 		= opcode == 7'b0010011;
wire Btype 		= opcode == 7'b1100011;
wire store 		= opcode == 7'b0100011;
wire load 		= opcode == 7'b0000011;
wire jal 		= opcode == 7'b1101111;
wire jalr 		= opcode == 7'b1100111;
wire NewRtype 	= opcode == 7'b1101011;
wire NewItype	= opcode == 7'b0011111;
wire lui;
wire auipc;


wire [31:0]I_imm = {{20{inst[31]}},inst[31:20]};
wire [31:0]J_imm = {{12{inst[31]}},
					inst[19:12],
					inst[20],
					inst[30:21],
					1'b0};
wire [31:0]B_imm = {{20{inst[31]}},
					inst[7],
					inst[30:25],
					inst[11:8],
					1'b0};
wire [31:0]S_imm = {{21{inst[31]}},
					inst[31:25],
					inst[11:7]};
wire [31:0]U_imm = {inst[31:12],
					12'b0};
wire [4:0] shamt = {{26{inst[31]}},inst[24:20]};
wire [31:0]imm;

reg [31:0] PC_nxt,PC;
reg [31:0] ir_addr;

wire shift;
wire branchflag;
wire RegWen;
wire [1:0]WBSel;
wire ALUSrc;

wire [31:0]data_out1, data_out2, WriteReg;

reg  [31:0]ALU_IN1, ALU_IN2;
wire [31:0]ALU_OUT;

assign CEN = 0;
assign OEN = 0;

always @(*) begin
	if(jal)begin
		PC_nxt = PC + J_imm;
	end
	else if(jalr) begin
		PC_nxt = ALU_OUT;
	end
	else if(branchflag&&Btype) begin
		PC_nxt = PC + B_imm;
	end
	else begin
		PC_nxt = PC + 4;
	end
end

always @(posedge clk) begin
	if (!rst) begin
		PC = 32'h0;
		ir_addr = 32'h0;
	end
	else if (I_ready) begin
		PC = PC_nxt;
		ir_addr = PC;
	end
	else begin
		PC = 32'h0;
		ir_addr = 32'h0;
	end
end

assign IR_addr = ir_addr;

assign inst = {IR[7:0],IR[15:8],IR[23:16],IR[31:24]};

assign ALUSrc = ~(Rtype | Btype | jal | NewRtype);
assign imm = ({32{(Itype & ~shift) | (NewItype & ~shift)}} & I_imm)
		|	({32{load|jalr}} & I_imm)
		|	({32{shift}} & shamt)
		|	({32{store}} & S_imm)
		|	({32{Btype}} & B_imm)
		|	({32{jal}} & J_imm);

assign WriteReg = WBSel[1] ? PC+4 : WBSel[0] ? {ReadDataMem[7:0],ReadDataMem[15:8],ReadDataMem[23:16],ReadDataMem[31:24]} : ALU_OUT;

register RF(.clk(clk),
	.rst(rst),
	.WriteReg(WriteReg),
	.REGWEN(RegWen),
	.RegRead1(rs1),
	.RegRead2(rs2),
	.RegWrite(rd),
	.data_out1(data_out1),
	.data_out2(data_out2)
	);

wire [3:0]ALU_OP;
wire MemRen, MemWen;

Control CU(
	.rv32_func7(func7),
	.rv32_func3(func3),
	.rv32_Rtype(Rtype),
	.rv32_Itype(Itype),
	.rv32_load(load),
	.rv32_store(store),
	.rv32_jal(jal),
	.rv32_jalr(jalr),
	.rv32_lui(lui),
	.rv32_auipc(auipc),
	.rv32_NewRtype(NewRtype),
	.rv32_NewItype(NewItype),
	.ALU_OP(ALU_OP),
	.RegWen(RegWen),
	.MemWen(WEN),
	.WBSel(WBSel),
	.shift(shift)
	);

always @(*) begin
		ALU_IN1 = data_out1;
end
always @(*) begin
	if (ALUSrc) begin
		ALU_IN2 = imm;
	end
	else begin
		ALU_IN2 = data_out2;
	end
end
assign WriteData = data_out2;
branch Branch(.data1(data_out1), .data2(data_out2), .rv32_func3(func3), .branchflag(branchflag));
ALU alu(.clk(clk),.ALU_OP(ALU_OP),.data1(ALU_IN1),.data2(ALU_IN2),.alu_out(ALU_OUT));

assign A = ALU_OUT[6:0];

endmodule

module register(
	clk, 
	rst,
	WriteReg,
	REGWEN,
	RegRead1,
	RegRead2,
	RegWrite,
	data_out1,
	data_out2
	);
input clk, rst, REGWEN;
input [4:0] RegRead1, RegRead2, RegWrite;
input [31:0] WriteReg;
output  [31:0] data_out1, data_out2;

reg [31:0] reg_r [31:0];
reg [31:0] reg_w [31:0];

integer i;

always @(posedge clk) begin
	if(REGWEN == 1 && rst == 1 && RegWrite != 5'b00000)begin
		reg_r[RegWrite] <= WriteReg;
	end
	else begin
		for(i=1;i<32;i=i+1)begin
			reg_w[i] <= reg_r[i];
		end
	end
end

assign data_out1 = reg_r[RegRead1];
assign data_out2 = reg_r[RegRead2];

always @(posedge clk) begin
	if(!rst) begin
		for(i=0;i<32;i=i+1)begin
			reg_r[i] <= 0;
		end
	end
	else begin
		reg_r[0] <= 0;
	end
end

endmodule

module Control(
	rv32_func7,
	rv32_func3,
	rv32_Rtype,
	rv32_Itype,
	rv32_load,
	rv32_store,
	rv32_jal ,
	rv32_jalr,
	rv32_lui,
	rv32_auipc,
	rv32_NewRtype,
	rv32_NewItype,
	ALU_OP,
	RegWen,
	MemWen,
	WBSel,
	shift
	);
	input [6:0]rv32_func7;
	input [2:0]rv32_func3;
	input rv32_Rtype;
	input rv32_Itype;
	input rv32_load;
	input rv32_store;
	input rv32_jal;
	input rv32_jalr;
	input rv32_lui;
	input rv32_auipc;
	input rv32_NewRtype;
	input rv32_NewItype;
	output reg [3:0]ALU_OP;
	output RegWen;
	output MemWen;
	output [1:0]WBSel;
	output reg shift;

	assign RegWen 	= (rv32_Rtype | rv32_Itype | rv32_NewRtype| rv32_NewItype | rv32_load | rv32_jal | rv32_jalr | rv32_lui | rv32_auipc);
	assign MemWen 	= ~rv32_store;
	assign WBSel[1] = rv32_jal | rv32_jalr;
	assign WBSel[0] = rv32_load;

	always @(*) begin
		shift = 0;
		if (~(rv32_Rtype | rv32_Itype | rv32_NewRtype | rv32_NewItype)) begin
			ALU_OP = `ALU_OP_ADD;
		end
		else if(rv32_NewRtype)begin
				case(rv32_func3)
					3'b000 	: ALU_OP = `ALU_OP_MOD;
					default : ALU_OP = `ALU_OP_ADD;
				endcase
		end
		else if (rv32_NewItype) begin
			case(rv32_func3)
					3'b000 	: ALU_OP = `ALU_OP_XNOR;
					default : ALU_OP = `ALU_OP_ADD;
				endcase
		end
		else begin
				case(rv32_func3)
					3'b000 : begin
							 	if(rv32_func7[5] & rv32_Rtype) ALU_OP = `ALU_OP_SUB;
							 	else ALU_OP = `ALU_OP_ADD;
							 end
					3'b001 : begin
								shift = 1;
								ALU_OP = `ALU_OP_SLL;
							 end
					3'b010 : ALU_OP = `ALU_OP_SLT;
					3'b011 : ALU_OP = `ALU_OP_SLTU;
					3'b100 : begin 
								if(rv32_func7[5])ALU_OP = `ALU_OP_XNOR;
								else ALU_OP = `ALU_OP_XOR;
							 end
					3'b101 : begin  
								shift = 1;
								if(rv32_func7[5]) ALU_OP = `ALU_OP_SRA;
							 	else ALU_OP = `ALU_OP_SRL;
							 end
					3'b110 : ALU_OP = `ALU_OP_OR;
					3'b111 : ALU_OP = `ALU_OP_AND;
				endcase
			 end
	end
endmodule

module ALU(clk,ALU_OP, data1, data2, alu_out);

input clk;
input [3:0]ALU_OP;
input [31:0]data1, data2;
output reg [31:0]alu_out;
wire [4:0]shift;
wire signed [31:0]sdata1, sdata2;
assign shift = data2[4:0];
assign sdata1 = data1;
assign sdata2 = data2;

always @(*)begin
	case(ALU_OP)
		`ALU_OP_ADD 	: alu_out = data1 + data2;
		`ALU_OP_SUB 	: alu_out = data1 - data2;
		`ALU_OP_XOR 	: alu_out = data1 ^ data2;
		`ALU_OP_OR  	: alu_out = data1 | data2;
		`ALU_OP_AND 	: alu_out = data1 & data2;
		`ALU_OP_SLL 	: alu_out = data1 << shift;
		`ALU_OP_SRL 	: alu_out = data1 >> shift;
		`ALU_OP_SRA 	: alu_out = sdata1 >>> shift;
		`ALU_OP_SLT 	: alu_out = {31'b0, sdata1<sdata2};
		`ALU_OP_SLTU 	: alu_out = {31'b0, data1 < data2};
		`ALU_OP_XNOR	: alu_out = ~(data1 ^ data2);
		`ALU_OP_MOD		: alu_out = data1 % data2;
		default 		: alu_out = 32'd0;
	endcase
end

endmodule

module branch(
	data1,
	data2,
	rv32_func3,
	branchflag
	);

input [31:0]data1, data2;
input [2:0]rv32_func3;
output reg branchflag;
wire signed [31:0]sdata1,sdata2;
assign sdata1 = data2;
assign sdata2 = data2;

always @(*) begin
	case(rv32_func3)
		3'b000 : branchflag = (data1 == data2);
		3'b001 : branchflag = (data1 != data2);
		3'b100 : branchflag = (sdata1 < sdata2);
		3'b101 : branchflag = (sdata1 >= sdata2);
		3'b110 : branchflag = (data1 < data2);
		3'b111 : branchflag = (data1 >= data2);
		default : branchflag = 0;
	endcase
end
endmodule

module dff(	
			clk,
		 	rst,
		 	d,
		 	q
		 );

parameter DW = 32;

input clk,rst;
input [DW-1:0] d;
output reg [DW-1:0] q;

	always @(posedge clk) begin
		if (!rst) begin
			q <= {DW{1'b0}};
		end
		else begin
			q <= d;
		end
	end

endmodule

module GPIO(
	clk,
	rst,
	Addr,
	data_out,
	data_in,
	gpio_out,
	gpio_in
);

parameter PIN = 8;

input clk;
input rst;
input [6:0] Addr;
input [31:0] data_out;
input [PIN - 1:0] gpio_in;

output [PIN - 1:0] data_in;
output [PIN - 1:0] gpio_out;

reg [PIN - 1:0] pin_mode;
reg [PIN - 1:0] g_out;
reg [PIN - 1:0] g_out_temp;
reg [PIN - 1:0] gpio_mask;



always @(posedge clk) begin
	if (!rst) begin
		gpio_mask = 0;
		g_out = 0;
		pin_mode = 0;
	end
	else begin
		case (Addr)
			7'h7f : pin_mode = data_out[7:0];
			7'h7e : g_out = (data_out[7:0] & gpio_mask & pin_mode);
			7'h7d : gpio_mask = data_out[7:0];
			default : g_out_temp = g_out;
		endcase
	end
end

assign data_in = gpio_in;
assign gpio_out = g_out;

endmodule

module RAM(
	clk,
	rst,
	Addr,
	rdata,
	wdata,
	WEN
	);

input clk;
input rst;
input WEN;
input [6:0] Addr;
input [31:0] wdata;

output [31:0] rdata;

reg [31:0] data [127:0];
reg [31:0] data_t [127:0];

integer i;

always @(posedge clk) begin
	if (WEN) begin
		data[Addr] <= wdata;
	end
	else begin
		for(i = 0; i < 128; i = i + 1)begin
			data_t[i] <= data[i];
		end
	end
end

assign rdata = data[Addr];

always @(posedge clk) begin
	if (!rst) begin
		for(i = 0; i < 128; i = i + 1)begin
			data[i] <= 0;
		end
	end
	else begin
		for(i = 0; i < 128; i = i + 1)begin
			data_t[i] <= data[i];
		end
	end
end

endmodule