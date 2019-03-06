`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/04/02 16:00:03
// Design Name: 
// Module Name: test
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
parameter wid = 32;

module alu(
	input logic [2:0] aluCtrl,
	input logic [wid - 1:0] a, [wid - 1:0] b,
	
	output logic [wid - 1:0] result
    );

	always_comb
	begin
		case(aluCtrl[1:0])
			3'b000: result = a & b;
			3'b001: result = a | b;
			2'b010: result = a + b;
			3'b110: result = a - b; 
			3'b111: result = a < b ? 1 : 0;
			default: result = 32'bx; 
		endcase
	end

endmodule

module equalCmp(
	input logic [wid - 1:0] in1, in2,

	output logic eq
	);

	assign eq = (in1 == in2);

endmodule

module dataMem(
	input logic clk,
	input logic wEn,
	input logic [wid - 1:0] addr, writeData,

	output logic [wid - 1:0] readData
	);

	logic [wid - 1:0] ram [63:0];

	always_ff @(posedge clk)
	begin
		if (wEn) ram[addr[31:2]] <= writeData;
	end

	assign readData = ram[addr[31:2]];

endmodule

module insMem(
	input logic [5:0] pc,

	output logic [wid - 1:0] ins
	);

	logic [wid - 1:0] ram [63:0];

	initial
	begin
		$readmemh("C:/Users/10441/Desktop/project_1/project_1.srcs/sources_1/new/iMemFile2.dat", ram);
		//$readmemh("C:/Users/10441/Desktop/ccj.data", ram);
	end

	assign ins = ram[pc];

endmodule

module regFile(
	input logic clk,
	input logic rst,
	input logic regWriteEn,

	input logic [4:0] regWriteAddr,
	input logic [wid - 1:0] regWriteData,

	input logic [4:0] rsAddr,
	input logic [4:0] rtAddr,

	output logic [wid - 1:0] rsData,
	output logic [wid - 1:0] rtData
	);

	logic [wid - 1:0] rf [wid - 1:0];

	integer i;
	
	always_ff @(negedge clk)
	begin
		if (rst)
		begin
			for (i = 0; i < 32; i = i + 1)
				rf[i] <= 32'b0;
		end
		else if (regWriteEn) rf[regWriteAddr] <= regWriteData;
	end

	assign rsData = (rsAddr != 0) ? rf[rsAddr] : 0; // #
	assign rtData = (rtAddr != 0) ? rf[rtAddr] : 0; // #

endmodule

module signExt(
	input logic [wid / 2 - 1:0] in,

	output logic [wid - 1:0] out
	);

	assign out = {{(wid / 2){in[wid / 2 - 1]}}, in}; // #

endmodule

module mux2 #(parameter w = 32)
	(
	input logic sw,
	input logic [w - 1:0] in1, in2,

	output logic [w - 1:0] out
	);

	assign out = sw ? in2 : in1; // #

endmodule

module mux2En #(parameter w = 32)
	(
	input logic clk, sw, en,
	input logic [w - 1:0] in1, in2,

	output logic [w - 1:0] out
	);

	always_ff @(posedge clk)
	begin
		if (en) 
		begin
			if (sw) out = in2;
			else out = in1;
		end
		else out = out;
	end

endmodule

module mux3 #(parameter w = 32)
	(
	input logic [1:0] sw,
	input logic [w - 1:0] in1, in2, in3,

	output logic [w - 1:0] out
	);

	assign out = sw[1] ? in3 : (sw[0] ? in2 : in1); // #

endmodule

module flopr #(parameter w = 32)
	(
		input logic clk, rst,
		input logic [w - 1:0] in,

		output logic [w - 1:0] out
	);
	always_ff @(posedge clk)
	begin
		if (rst) out <= 0; // #
		else out <= in; // #
	end

endmodule

module floprc #(parameter w = 32)
	(
		input logic clk, rst, clr,
		input logic [w - 1:0] in,

		output logic [w - 1:0] out
	);

	always_ff @(posedge clk)
	begin
		if (rst) out <= 0; // #
		else if (clr) out <= 0; // #
		else out <= in; // #
	end

endmodule

module floprEn #(parameter w = 32)
	(
		input logic clk, rst,
		input logic en,
		input logic [w - 1:0] in,

		output logic [w - 1:0] out
	);

	always_ff @(posedge clk)
	begin
		if (rst) out <= 0; // #
		else if (en) out <= in; // #
	end

endmodule

module floprcEn #(parameter w = 32)
	(
		input logic clk, rst, en,
		input logic clr,
		input logic [w - 1:0] in,

		output logic [w - 1:0] out
	);

	always_ff @(posedge clk)
	begin
		if (rst) out <= 0;
		else if (en)
		begin
			if (clr) out <= 0;
			else out <= in;
		end
	end

endmodule

	/***************************
	 *						   *
	 * if (rst) out <= 0;	   *
	 * else if (en) out <= in; *
	 * else if (clr) out <= 0; *
	 *						   *
	 ***************************/

module adder(
	input logic [wid - 1:0] in1, in2,

	output logic [wid - 1:0] out
	);

	assign out = in1 + in2; // #

endmodule

module shift2(
	input logic [wid - 1:0] in,

	output logic [wid - 1:0] out
	);

	assign out = {in[wid - 3:0], 2'b00}; // #

endmodule

module mainDec(
	input logic [5:0] op,

	output logic memToReg, memWrite,
	output logic branch0, branch1, aluSrc,
	output logic regDst, regWrite,
	output logic jump,
	output logic [2:0]aluOp // 000 +, 001 -, 010 &, 011 |, 100 Rtype, 101 slt
	);

	logic [10:0] controls;

	assign {regWrite, regDst, aluSrc, branch0, branch1, 
			memWrite, memToReg, jump, aluOp} = controls;

	always_comb
	begin
		case(op)
			6'b000000: controls = 11'b11000000100; // Rtype
			6'b100011: controls = 11'b10100010000; // LW read mem
			6'b101011: controls = 11'b00100100000; // SW write mem
			6'b000100: controls = 11'b00010000001; // BEQ
			6'b000101: controls = 11'b00001000001; // BNE
			6'b001000: controls = 11'b10100000000; // ADDI
			6'b001100: controls = 11'b10100000010; // ANDI
			6'b001101: controls = 11'b10100000011; // ORI
			6'b001010: controls = 11'b10100000101; // SLTI
			6'b000010: controls = 11'b00000001000; // J
			default:   controls = 11'bxxxxxxxxxxx;
		endcase
	end

endmodule

module aluDec(
	input logic [5:0] func,
	input logic [2:0] aluOp,

	output logic [2:0] aluCtrl
	);

	always_comb
	begin
		case(aluOp)
			3'b000: aluCtrl = 3'b010;
			3'b001: aluCtrl = 3'b110;
			3'b010: aluCtrl = 3'b000;
			3'b011: aluCtrl = 3'b001;
			3'b101: aluCtrl = 3'b111;
			default: 
				case(func)
					6'b000000: aluCtrl = 3'b010; // NOP
					6'b100000: aluCtrl = 3'b010; // ADD
					6'b100010: aluCtrl = 3'b110; // SUB
					6'b100100: aluCtrl = 3'b000; // AND
					6'b100101: aluCtrl = 3'b001; // OR
					6'b101010: aluCtrl = 3'b111; // SLT
					default:   aluCtrl = 3'bxxx;
				endcase
		endcase
	end

endmodule

module controller(
	input logic clk, rst,
	input logic [5:0] opD, funcD,
	input logic flushE, equalD,

	output logic memToRegE, memToRegM, memToRegW, memWriteM,
	output logic pcSrcD, branchD0, branchD1,
	output logic aluSrcE, regDstE, 
	output logic regWriteE, regWriteM, regWriteW,
	output logic jumpD,
	output logic [2:0] aluCtrlE
	);

	logic [2:0] aluOpD;

	logic memToRegD, memWriteD, aluSrcD, regDstD, regWriteD;

	logic [2:0] aluCtrlD;

	logic memWriteE;

	mainDec mDec(
					opD,
					memToRegD, memWriteD,
					branchD0, branchD1, aluSrcD,
					regDstD, regWriteD, jumpD,
					aluOpD
				);

	aluDec aDec(funcD, aluOpD, aluCtrlD);

	assign pcSrcD = (branchD0 & equalD) | (branchD1 & ~equalD);

	floprc #(8) regE(
						clk, rst, flushE,
						{memToRegD, memWriteD, aluSrcD, regDstD, regWriteD, aluCtrlD},
						{memToRegE, memWriteE, aluSrcE, regDstE, regWriteE, aluCtrlE}
					);
	flopr #(3) regM(
						clk, rst,
						{memToRegE, memWriteE, regWriteE},
						{memToRegM, memWriteM, regWriteM}
					);
	flopr #(2) regW(
						clk, rst,
						{memToRegM, regWriteM},
						{memToRegW, regWriteW}
					);

endmodule

module hazard(
	input logic [4:0] rsD, rtD, rsE, rtE,
	input logic [4:0] writeRegE, writeRegM, writeRegW,
	input logic regWriteE, regWriteM, regWriteW,
	input logic memToRegE, memToRegM,
	input logic branchD0, branchD1,

	output logic forwardRsD, forwardRtD,
	output logic [1:0] forwardRsE, forwardRtE,
	output logic stallF, stallD, flushE
	);

	logic lwStallD, branchStallD;

	assign forwardRsD = (rsD != 0 & rsD == writeRegM & regWriteM);
	assign forwardRtD = (rtD != 0 & rtD == writeRegM & regWriteM);

	always_comb
	begin
		forwardRsE = 2'b00; forwardRtE = 2'b00;
		if (rsE != 0)
		begin
			if (rsE == writeRegM & regWriteM) forwardRsE = 2'b10;
			else if (rsE == writeRegW & regWriteW) forwardRsE = 2'b01;
		end

		if (rtE != 0)
		begin
			if (rtE == writeRegM & regWriteM) forwardRtE = 2'b10;
			else if (rtE == writeRegW & regWriteW) forwardRtE = 2'b01;
		end
	end


	assign lwStallD = memToRegE & (rtE == rsD | rtE == rtD);
	assign branchStallD = (branchD0 | branchD1) & 
							(
								(regWriteE & (writeRegE == rsD | writeRegE == rtD)) |
								(memToRegM & (writeRegM == rsD | writeRegM == rtD))
							);
	assign stallD = lwStallD | branchStallD;
	assign stallF = stallD;
	assign flushE = stallD;

endmodule

module dataPath(
	input logic clk, rst,

	input logic memToRegE, memToRegM, memToRegW,
	input logic pcSrcD, branchD0, branchD1,
	input logic aluSrcE, regDstE, 
	input logic regWriteE, regWriteM, regWriteW,
	input logic jumpD,
	input logic [2:0] aluCtrlE,

	input logic [wid - 1:0] insF,
	input logic [wid - 1:0] readDataM,

	output logic equalD,
	output logic [wid - 1:0] pcF,
	output logic [wid - 1:0] aluOutM,
	output logic [wid - 1:0] writeDataM,
	output logic [5:0] opD, funcD,
	output logic flushE
	);

	logic forwardRsD, forwardRtD;
	logic [1:0] forwardRsE, forwardRtE;
	logic stallF, stallD;
	logic [4:0] rsD, rtD, rdD, rsE, rtE, rdE;
	logic [4:0] writeRegE, writeRegM, writeRegW;
	logic flushD;
	logic [wid - 1:0] pcNxtFD, pcNxtBrFD, pcPlus4F, pcBranchD;
	logic [wid - 1:0] sigImmD, sigImmE, sigImmShD;
	logic [wid - 1:0] srcAD, srcA2D, srcAE, srcA2E;
	logic [wid - 1:0] srcBD, srcB2D, srcBE, srcB2E, srcB3E;
	logic [wid - 1:0] pcPlus4D, insD;
	logic [wid - 1:0] aluOutE, aluOutW;
	logic [wid - 1:0] readDataW, resultW;

	hazard haz(
				rsD, rtD, rsE, rtE,
				writeRegE, writeRegM, writeRegW,
				regWriteE, regWriteM, regWriteW,
				memToRegE, memToRegM,
				branchD0, branchD1,

				forwardRsD, forwardRtD,
				forwardRsE, forwardRtE,
				stallF, stallD, flushE
				);

	mux2 #(32) pcBrMux(pcSrcD, pcPlus4F, pcBranchD, pcNxtBrFD);
	mux2 #(32) pcMux(jumpD, pcNxtBrFD, {pcPlus4D[31:28], insD[25:0], 2'b00}, pcNxtFD);

	regFile rf(clk, rst, regWriteW, writeRegW, resultW, rsD, rtD, srcAD, srcBD);

	floprEn #(32) pcReg(clk, rst, ~stallF, pcNxtFD, pcF);
	adder pcAdd1(pcF, 32'b100, pcPlus4F);

	floprEn #(32) r1D(clk, rst, ~stallD, pcPlus4F, pcPlus4D);
	floprcEn #(32) r2D(clk, rst, ~stallD, flushD, insF, insD);
	signExt sigEx(insD[15:0], sigImmD);
	shift2 immSh(sigImmD, sigImmShD);
	adder pcAdd2(pcPlus4D, sigImmShD, pcBranchD);
	mux2 #(32) forwardRsDMux(forwardRsD, srcAD, aluOutM, srcA2D);
	mux2 #(32) forwardRtDMux(forwardRtD, srcBD, aluOutM, srcB2D);
	equalCmp comp(srcA2D, srcB2D, equalD);

	assign opD = insD[31:26];
	assign funcD = insD[5:0];
	assign rsD = insD[25:21];
	assign rtD = insD[20:16];
	assign rdD = insD[15:11];

	assign flushD = pcSrcD | jumpD;

	floprc #(32) r1E(clk, rst, flushE, srcAD, srcAE);
	floprc #(32) r2E(clk, rst, flushE, srcBD, srcBE);
	floprc #(32) r3E(clk, rst, flushE, sigImmD, sigImmE);
	floprc #(5) r4E(clk, rst, flushE, rsD, rsE);
	floprc #(5) r5E(clk, rst, flushE, rtD, rtE);
	floprc #(5) r6E(clk, rst, flushE, rdD, rdE);

	mux3 #(32) forwardRsEMux(forwardRsE, srcAE, resultW, aluOutM, srcA2E);
	mux3 #(32) forwardRtEMux(forwardRtE, srcBE, resultW, aluOutM, srcB2E);
	mux2 #(32) srcBMux(aluSrcE, srcB2E, sigImmE, srcB3E);
	alu alu(aluCtrlE, srcA2E, srcB3E, aluOutE);
	mux2 #(5) wrMux(regDstE, rtE, rdE, writeRegE);

	flopr #(32) r1M(clk, rst, srcB2E, writeDataM);
	flopr #(32) r2M(clk, rst, aluOutE, aluOutM);
	flopr #(5) r3M(clk, rst, writeRegE, writeRegM);

	flopr #(32) r1W(clk, rst, aluOutM, aluOutW);
	flopr #(32) r2W(clk, rst, readDataM, readDataW);
	flopr #(5) r3W(clk, rst, writeRegM, writeRegW);
	mux2 #(32) resMux(memToRegW, aluOutW, readDataW, resultW);

endmodule	

module mips(
	input logic clk, rst,
	input logic [wid - 1:0] insF,
	input logic [wid - 1:0] readDataM,

	output logic [wid - 1:0] pcF,
	output logic memWriteM,
	output logic [wid - 1:0] aluOutM, writeDataM
	);
	
	logic [5:0] opD, funcD;
	logic regDstE, aluSrcE, pcSrcD;
	logic branchD0, branchD1;
	logic jumpD;
	logic memToRegE, memToRegM, memToRegW;
	logic regWriteE, regWriteM, regWriteW;
	logic [2:0] aluCtrlE;
	logic flushE, equalD;

	controller ctrl(
					clk, rst,
					opD, funcD,
					flushE, equalD,

					memToRegE, memToRegM, memToRegW, memWriteM,
					pcSrcD, branchD0, branchD1,
					aluSrcE, regDstE, 
					regWriteE, regWriteM, regWriteW,
					jumpD,
					aluCtrlE
					);

	dataPath datPath(
					clk, rst,

					memToRegE, memToRegM, memToRegW,
					pcSrcD, branchD0, branchD1,
					aluSrcE, regDstE, 
					regWriteE, regWriteM, regWriteW,
					jumpD,
					aluCtrlE,

					insF,
					readDataM,

					equalD,
					pcF,
					aluOutM,
					writeDataM,
					opD, funcD,
					flushE
					);

endmodule

module top(
	input logic clk, rst,

	output logic [wid - 1:0] writeMemAddr, writeMemData,
	output logic memWrite
	);

	logic [wid - 1:0] pc, instr, readData;

	mips mips(clk, rst, instr, readData, pc, memWrite, writeMemAddr, writeMemData);
	insMem imem(pc[7:2], instr);
	dataMem dmem(clk, memWrite, writeMemAddr, writeMemData, readData);
	
endmodule
/*
module top(
	input logic clk, rst,

	output logic [wid - 1:0] writeMemAddr, writeMemData,
	output logic memWrite,

	output logic [7:0] pc_out, rD_out,
	output logic [7:0] wMA_out, wMD_out,
	output logic mW_out
	);

	logic [wid - 1:0] pc, instr, readData;

	mips mips(clk, rst, instr, readData, pc, memWrite, writeMemAddr, writeMemData);
	insMem imem(pc[7:2], instr);
	dataMem dmem(clk, memWrite, writeMemAddr, writeMemData, readData);

	assign wMA_out = writeMemAddr[7:0];
	assign wMD_out = writeMemData[7:0];
	assign pc_out = pc[7:0];
	assign rD_out = readData[7:0];
	assign mW_out = memWrite;
	
endmodule

// test
module clkDiv(
	input logic clk,

	output logic clk190,
	output logic clk48,
	output logic clk1_4
	);

	logic [27:0] q;

	initial
	begin
		q = 0;
	end

	always_ff @(posedge clk)
	begin
		q <= q + 1;
	end

	assign clk190 = q[18];
	assign clk48 = q[20];
	assign clk1_4 = q[26];

endmodule

module forShow(
	input logic clk, rst,
	input logic k,

	output logic [6:0] a2g,
	output logic [7:0] enA2g,
	output logic dp
	);

	logic clk190, clk48, clk1_4;

	clkDiv div(clk, clk190, clk48, clk1_4);

	logic [31:0] sw; 

	logic [wid - 1:0] writeMemAddr, writeMemData;
	logic memWrite;

	assign clk1_4_k = clk1_4 & k;

	top top(clk1_4_k, rst, writeMemAddr, writeMemData, memWrite,
			sw[31:24], sw[23:16], sw[15:8], sw[7:0]);

	assign dp = ~memWrite;

	logic [2:0] i;
	logic [20:0] fre;

	initial
	begin
		i = 0;
		fre = 0;
	end

	always_ff @(posedge clk)
	begin
		fre <= fre + 1;
	end

	assign i = fre[19:17];

	always_comb
	begin
		if (i == 3'b000)
		begin
			enA2g = 8'b1111_1110;
			case (sw[3:0])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b001)
		begin
			enA2g = 8'b1111_1101;
			case (sw[7:4])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b010)
		begin
			enA2g = 8'b1111_1011;
			case (sw[11:8])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b011)
		begin
			enA2g = 8'b1111_0111;
			case (sw[15:12])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b100)
		begin
			enA2g = 8'b1110_1111;
			case (sw[19:16])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b101)
		begin
			enA2g = 8'b1101_1111;
			case (sw[23:20])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b110)
		begin
			enA2g = 8'b1011_1111;
			case (sw[27:24])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b111)
		begin
			enA2g = 8'b0111_1111;
			case (sw[31:28])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
	end

endmodule

module forShowTop(
	input logic CLK100MHZ, SW[1:0],

	output logic [6:0] A2G,
	output logic [7:0] AN,
	output logic DP
	);

	forShow show(CLK100MHZ, SW[0], SW[1], A2G[6:0], AN[7:0], DP);

endmodule
*/