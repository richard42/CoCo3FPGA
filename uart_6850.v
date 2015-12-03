////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		uart_6850.v
//
// CoCo3 in an FPGA
//
// Revision: 3.0 08/15/15
////////////////////////////////////////////////////////////////////////////////
//
// CPU section copyrighted by John Kent
// The FDC co-processor copyrighted Daniel Wallner.
//
////////////////////////////////////////////////////////////////////////////////
//
// Color Computer 3 compatible system on a chip
//
// Version : 3.0
//
// Copyright (c) 2008 Gary Becker (gary_l_becker@yahoo.com)
//
// All rights reserved
//
// Redistribution and use in source and synthezised forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// Redistributions in synthesized form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// Neither the name of the author nor the names of other contributors may
// be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please report bugs to the author, but before you do so, please
// make sure that this is not a derivative work and that
// you have the latest version of this file.
//
// The latest version of this file can be found at:
//      http://groups.yahoo.com/group/CoCo3FPGA
//
// File history :
//
//  1.0		Full Release
//  2.0		Partial Release
//  3.0		Full Release
////////////////////////////////////////////////////////////////////////////////
// Gary Becker
// gary_L_becker@yahoo.com
////////////////////////////////////////////////////////////////////////////////

module glb6850(
RESET_N,
RX_CLK,
TX_CLK,
E,
DI,
DO,
IRQ,
CS,
RW_N,
RS,
TXDATA,
RXDATA,
RTS,
CTS,
DCD
);

input					RESET_N;
input					RX_CLK;
input					TX_CLK;
input					E;
input		[7:0]		DI;
output	[7:0]		DO;
output				IRQ;
input					CS;
input					RS;
input					RW_N;
output				TXDATA;
input					RXDATA;
output				RTS;
input					CTS;
input					DCD;

reg		[7:0]		TX_BUFFER;
reg		[7:0]		TX_REG;
wire		[7:0]		RX_BUFFER;
reg		[7:0]		RX_REG;
wire		[7:0]		STATUS_REG;
reg		[7:0]		CTL_REG;
wire					TX_DONE;
reg					TX_DONE0;
reg					TX_DONE1;
reg					TX_START;
reg					TDRE;
reg					RDRF;
reg		[1:0]		READ_STATE;
wire					GOT_DATA;
reg					READY0;
reg					READY1;
reg		[1:0]		TX_CLK_DIV;
reg		[1:0]		RX_CLK_DIV;
wire					TX_CLK_X;
wire					RX_CLK_X;
reg					FRAME;
wire					FRAME_BUF;
reg					OVERRUN;
reg					PARITY;
wire		[1:0]		COUNTER_DIVIDE;
wire					WORD_SELECT;
wire		[1:0]		TX_CTL;
wire					RESET_X;
wire					STOP;
wire					PARITY_ERR;
wire					PAR_DIS;

always @ (negedge TX_CLK)
	TX_CLK_DIV <= TX_CLK_DIV +1'b1;

always @ (posedge RX_CLK)
	RX_CLK_DIV <= RX_CLK_DIV +1'b1;

assign TX_CLK_X = (COUNTER_DIVIDE == 2'b10) ?	TX_CLK_DIV[1]:
																TX_CLK;
assign RX_CLK_X = (COUNTER_DIVIDE == 2'b10) ?	RX_CLK_DIV[1]:
																RX_CLK;

assign RESET_X = (COUNTER_DIVIDE == 2'b11) ?	1'b0:
															RESET_N;
//							IRQ   PE
assign STATUS_REG = {!IRQ, PARITY, OVERRUN, FRAME, CTS, DCD, TDRE, RDRF};

assign DO =	RS		?	RX_REG[7:0]:
							STATUS_REG;

assign IRQ =	({CTL_REG[7], RDRF} == 2'b11)		?	1'b0:
					({CTL_REG[6:5], TDRE} == 3'b011)	?	1'b0:	1'b1;

assign COUNTER_DIVIDE = CTL_REG[1:0];
assign WORD_SELECT =	CTL_REG[4];
assign TX_CTL = CTL_REG[6:5];
assign RTS = (TX_CTL == 2'b10);
assign STOP =	(CTL_REG[4:2] == 3'b000) ? 1'b1:
					(CTL_REG[4:2] == 3'b001) ? 1'b1:
					(CTL_REG[4:2] == 3'b100) ? 1'b1: 1'b0;

assign PAR_DIS =(CTL_REG[4:2] == 3'b100) ? 1'b1:
					 (CTL_REG[4:2] == 3'b101) ? 1'b1: 1'b0;


always @ (negedge E or negedge RESET_N)
begin
	if(!RESET_N)
		CTL_REG <= 8'h00;
	else
		if({RW_N, CS, RS} == 3'b010)						// Write CTL register
			CTL_REG <= DI;
		else
			if(COUNTER_DIVIDE == 2'b11)
				CTL_REG <= 8'h03;
end

always @ (negedge E or negedge RESET_X)
begin
	if(!RESET_X)
	begin
		RDRF <= 1'b0;
		READ_STATE <= 2'b00;
		RX_REG <= 8'h00;
		TX_BUFFER <= 8'h00;
		TDRE <= 1'b1;
		TX_START <= 1'b0;
		OVERRUN <= 1'b0;
		FRAME <= 1'b0;
		PARITY <= 1'b0;
		TX_DONE1 <= 1'b1;
		TX_DONE0 <= 1'b1;
		READY1 <= 1'b0;
		READY0 <= 1'b0;
	end
	else
	begin
		TX_DONE1 <= TX_DONE0;			// sync TX_DONE with E clock for metastability?
		TX_DONE0 <= TX_DONE;
		READY1 <= READY0;
		READY0 <= GOT_DATA;

		case (READ_STATE)
		2'b00:
		begin
			if(READY1)										// Grab data from buffer
			begin
				RDRF <= 1'b1;
				READ_STATE <= 2'b01;
				PARITY <= (PARITY_ERR & !PAR_DIS);
				OVERRUN <= 1'b0;
				FRAME <= FRAME_BUF;
				RX_REG <= RX_BUFFER;
			end
		end
		2'b01:												// Data read by cpu, before new data in buffer
		begin
			if({RW_N, CS, RS} == 3'b111)
			begin
				RDRF <= 1'b0;
				READ_STATE <= 2'b10;
//				PARITY <= 1'b0;
//				OVERRUN <= 1'b0;
//				FRAME <= 1'b0;
			end
			else												// Data not read yet, but getting new data in buffer
			begin
				if(~READY1)
					READ_STATE <= 2'b11;
			end
		end
		2'b10:												// wait until buffer filling
		begin
			if(~READY1)
				READ_STATE <= 2'b00;
		end
		2'b11:												// Data read by cpu while buffer filling
		begin
			if({RW_N, CS, RS} == 3'b111)
			begin
				RDRF <= 1'b0;
				READ_STATE <= 2'b00;
//				PARITY <= 1'b0;
//				OVERRUN <= 1'b0;
//				FRAME <= 1'b0;
			end
			else
			begin
				if(READY1)									// buffer full but data not read by cpu
				begin
					RDRF <= 1'b1;
					READ_STATE <= 2'b01;
					OVERRUN <= 1'b1;
					PARITY <= (PARITY_ERR & !PAR_DIS);
					FRAME <= FRAME_BUF;
					RX_REG <= RX_BUFFER;
				end
			end
		end
		endcase

		if(~TDRE & TX_DONE1 & ~TX_START & ~CS)
		begin
			TX_BUFFER <= TX_REG;
			TDRE <= 1'b1;
			TX_START <= 1'b1;
		end
		else
		begin
			if({RW_N, CS, RS} == 3'b011)				// Write TX data register
			begin
				TDRE <= 1'b0;
				TX_REG <= DI;
			end
			if(~TX_DONE1)
			begin
				TX_START <= 1'b0;
			end
		end
	end
end

UART_TX TX(
.BAUD_CLK(TX_CLK_X),
.RESET_N(RESET_X),
.TX_DATA(TXDATA),
.TX_START(TX_START),
.TX_DONE(TX_DONE),
.TX_STOP(STOP),
.TX_WORD(WORD_SELECT),
.TX_PAR_DIS(PAR_DIS),
.TX_PARITY(CTL_REG[2]),
.TX_BUFFER(TX_BUFFER)
);

UART_RX RX(
.RESET_N(RESET_X),
.BAUD_CLK(RX_CLK_X),
.RX_DATA(RXDATA),
.RX_BUFFER(RX_BUFFER),
.RX_WORD(WORD_SELECT),
.RX_PAR_DIS(PAR_DIS),
.RX_PARITY(CTL_REG[2]),
.PARITY_ERR(PARITY_ERR),
.FRAME(FRAME_BUF),
.READY(GOT_DATA)
);

endmodule
