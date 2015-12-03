////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		ps2_keyboard.v
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

module ps2_keyboard (
CLK,
RESET_N,
PS2_CLK,
PS2_DATA,
RX_PRESSED,
RX_EXTENDED,
RX_SCAN
);

input					CLK;
input					RESET_N;
input					PS2_CLK;
input					PS2_DATA;
output				RX_PRESSED;
reg					RX_PRESSED;
output				RX_EXTENDED;
reg					RX_EXTENDED;
output	[7:0]		RX_SCAN;
reg		[7:0]		RX_SCAN;

reg					KB_CLK;
reg					KB_DATA;
reg					KB_CLK_B;
reg					KB_DATA_B;
reg					PRESSED_N;
reg					EXTENDED;
reg		[2:0]		BIT;
reg		[3:0]		STATE;
reg		[7:0]		SCAN;
wire					PARITY;
reg		[10:0]	TIMER;
reg					KILLER;
wire					RESET_X;



// Double buffer
always @ (posedge CLK)
begin
	KB_CLK_B		<=	PS2_CLK;
	KB_DATA_B	<= PS2_DATA;
	KB_CLK		<= KB_CLK_B;
	KB_DATA		<= KB_DATA_B;
end
assign PARITY =	~(((SCAN[0]^SCAN[1])
						  ^(SCAN[2]^SCAN[3]))
						 ^((SCAN[4]^SCAN[5])
						  ^(SCAN[6]^SCAN[7])));

assign RESET_X = RESET_N & KILLER;
always @ (negedge CLK or negedge RESET_N)
	if(!RESET_N)
	begin
		KILLER <= 1'b1;
		TIMER <= 11'h000;
	end
	else
		case(TIMER)
		11'h000:
		begin
			KILLER <= 1'b1;
			if(STATE != 4'h0)
				TIMER <= 11'h001;
		end
		11'h7FD:
		begin
			KILLER <= 1'b0;
			TIMER <= 11'h7FE;
		end
		default:
			if(STATE == 4'h0)
				TIMER <= 11'h000;
			else
				TIMER <= TIMER + 1'b1;
		endcase

always @ (posedge CLK or negedge RESET_X)
begin
	if(!RESET_X)
	begin
		STATE				<= 4'h0;
		SCAN				<= 8'h00;
		BIT				<= 3'b000;
		RX_SCAN			<=	8'h00;
		RX_PRESSED		<= 1'b0;
		RX_EXTENDED		<= 1'b0;
		PRESSED_N		<= 1'b0;
		EXTENDED			<= 1'b0;
	end
	else
	begin
		
		case (STATE)
		4'h0:						// Hunt for start bit
		begin
			BIT				<= 3'b000;
			RX_SCAN			<= 8'h00;
			RX_PRESSED		<= 1'b0;
			RX_EXTENDED		<= 1'b0;
			if(~KB_DATA & ~KB_CLK)				//look for start bit
					STATE			<= 4'h1;
		end
		4'h1:						// next bit
		begin
			if(KB_CLK)
				STATE				<= 4'h2;
		end
		4'h2:						// Hunt for Bit
		begin
			if(~KB_CLK)
			begin
				SCAN[BIT]		<= KB_DATA;
				BIT				<= BIT + 1'b1;
				if(BIT == 3'b111)
					STATE				<= 4'h3;
				else
					STATE				<= 4'h1;
			end
		end
		4'h3:						// Hunt for Bit
		begin
			if(KB_CLK)
				STATE	<= 4'h4;
		end
		4'h4:						// Test parity
		begin
			if(~KB_CLK)
			begin
				if(KB_DATA == PARITY)
					STATE				<= 4'h5;
				else
				begin
					PRESSED_N	<= 1'b0;
					EXTENDED		<= 1'b0;
					SCAN			<= 8'h00;
					STATE			<= 4'hF;
				end
			end
		end
		4'h5:						// Look for Stop bit
		begin
			if(KB_CLK)
				STATE	<= 4'h6;
		end
		4'h6:						// Stop bit
		begin
			if(~KB_CLK)
				STATE	<= 4'h7;
		end
		4'h7:
		begin
			if(SCAN ==8'hE0)
			begin
				EXTENDED <= 1'b1;
				STATE		<= 4'hF;
			end
			else
				if(SCAN == 8'hF0)
				begin
					PRESSED_N	<= 1'b1;
					STATE			<= 4'hF;
				end
				else
				begin
					RX_SCAN		<= SCAN;
					RX_PRESSED	<= ~PRESSED_N;
					RX_EXTENDED	<= EXTENDED;
					PRESSED_N	<= 1'b0;
					EXTENDED		<= 1'b0;
					SCAN			<= 8'h00;
					STATE			<= 4'hF;
				end
			end
		4'h8:
		begin
			STATE				<= 4'h9;
		end
		4'h9:
		begin
			STATE				<= 4'hA;
		end
		4'hA:
		begin
			STATE				<= 4'hB;
		end
		4'hB:
		begin
			STATE				<= 4'hC;
		end
		4'hC:
		begin
			STATE				<= 4'hD;
		end
		4'hD:
		begin
			STATE				<= 4'hE;
		end
		4'hE:
		begin
			STATE				<= 4'hF;
		end
		4'hF:
		begin
			if(KB_CLK)
				STATE			<= 4'h0;
		end
		endcase
	end
end

endmodule
