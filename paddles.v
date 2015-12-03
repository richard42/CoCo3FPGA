////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		paddles.v
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

/*****************************************************************************
* Joystick to CoCo compatable
******************************************************************************/
assign PADDLE_MCLK = MCLOCK[10];
always @(negedge MCLOCK[10] or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		JOY_CLK <= 13'h000;
		JOY_TRIGGER <= 1'b0;
	end
	else
		case(JOY_CLK)
		13'd0000:
		begin
			JOY_CLK <= 13'd0001;
			JOY_TRIGGER <= 1'b0;
		end
		13'd5883:
		begin
			JOY_CLK <= 13'd5884;
			JOY_TRIGGER <= 1'b1;
		end
		13'd8191:
		begin
			JOY_CLK <= 13'd0000;
			JOY_TRIGGER <= 1'b0;
		end
		default:
			JOY_CLK <= JOY_CLK + 1'b1;
		endcase
end

always @(negedge PADDLE_CLK[0] or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		PADDLE_ZERO_0 <= 10'd0000;
		PADDLE_VAL_0 <= 12'd0000;
		PADDLE_STATE_0 <= 2'b00;
		JOY1_COUNT <= 6'h00;
	end
	else
	begin
		case(PADDLE_STATE_0)
		2'b00:
		begin
			PADDLE_ZERO_0 <= PADDLE_ZERO_0 + 1'b1;
			PADDLE_VAL_0 <= 12'd0000;
			if(PADDLE_ZERO_0 == 10'd611)
				PADDLE_STATE_0 <= 2'b01;
		end
		2'b01:
		begin
			PADDLE_ZERO_0 <= 10'd000;
			PADDLE_VAL_0 <= PADDLE_VAL_0 + 1'b1;
			if(PADDLE_VAL_0 == 12'd4094)
				PADDLE_STATE_0 <= 2'b10;
			else
			begin
				if(JOY_TRIGGER)
					PADDLE_STATE_0 <= 2'b10;
				else
					PADDLE_STATE_0 <= 2'b01;
			end
		end
		2'b10:
		begin
			JOY1_COUNT <= PADDLE_VAL_0[11:6];
			PADDLE_LATCH_0 <= PADDLE_VAL_0;
			if(JOY_TRIGGER)
					PADDLE_STATE_0 <= 2'b11;
		end
		2'b11:
		begin
			if(!JOY_TRIGGER)
				PADDLE_STATE_0 <= 2'b00;
		end
		endcase
	end
end

always @(negedge PADDLE_CLK[1] or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		PADDLE_ZERO_1 <= 10'd0000;
		PADDLE_VAL_1 <= 12'd0000;
		PADDLE_STATE_1 <= 2'b00;
		JOY2_COUNT <= 6'h00;
	end
	else
	begin
		case(PADDLE_STATE_1)
		2'b00:
		begin
			PADDLE_ZERO_1 <= PADDLE_ZERO_1 + 1'b1;
			PADDLE_VAL_1 <= 12'd0000;
			if(PADDLE_ZERO_1 == 10'd611)
				PADDLE_STATE_1 <= 2'b01;
		end
		2'b01:
		begin
			PADDLE_ZERO_1 <= 10'd000;
			PADDLE_VAL_1 <= PADDLE_VAL_1 + 1'b1;
			if(PADDLE_VAL_1 == 12'd4094)
				PADDLE_STATE_1 <= 2'b10;
			else
			begin
				if(JOY_TRIGGER)
					PADDLE_STATE_1 <= 2'b10;
				else
					PADDLE_STATE_1 <= 2'b01;
			end
		end
		2'b10:
		begin
			JOY2_COUNT <= PADDLE_VAL_1[11:6];
			PADDLE_LATCH_1 <= PADDLE_VAL_1;
			if(JOY_TRIGGER)
					PADDLE_STATE_1 <= 2'b11;
		end
		2'b11:
		begin
			if(!JOY_TRIGGER)
				PADDLE_STATE_1 <= 2'b00;
		end
		endcase
	end
end

always @(negedge PADDLE_CLK[2] or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		PADDLE_ZERO_2 <= 10'd0000;
		PADDLE_VAL_2 <= 12'd0000;
		PADDLE_STATE_2 <= 2'b00;
		JOY3_COUNT <= 6'h00;
	end
	else
	begin
		case(PADDLE_STATE_2)
		2'b00:
		begin
			PADDLE_ZERO_2 <= PADDLE_ZERO_2 + 1'b1;
			PADDLE_VAL_2 <= 12'd0000;
			if(PADDLE_ZERO_2 == 10'd611)
				PADDLE_STATE_2 <= 2'b01;
		end
		2'b01:
		begin
			PADDLE_ZERO_2 <= 10'd000;
			PADDLE_VAL_2 <= PADDLE_VAL_2 + 1'b1;
			if(PADDLE_VAL_2 == 12'd4094)
				PADDLE_STATE_2 <= 2'b10;
			else
			begin
				if(JOY_TRIGGER)
					PADDLE_STATE_2 <= 2'b10;
				else
					PADDLE_STATE_2 <= 2'b01;
			end
		end
		2'b10:
		begin
			JOY3_COUNT <= PADDLE_VAL_2[11:6];
			PADDLE_LATCH_2 <= PADDLE_VAL_2;
			if(JOY_TRIGGER)
					PADDLE_STATE_2 <= 2'b11;
		end
		2'b11:
		begin
			if(!JOY_TRIGGER)
				PADDLE_STATE_2 <= 2'b00;
		end
		endcase
	end
end

always @(negedge PADDLE_CLK[3] or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		PADDLE_ZERO_3 <= 10'd0000;
		PADDLE_VAL_3 <= 12'd0000;
		PADDLE_STATE_3 <= 2'b00;
		JOY4_COUNT <= 6'h00;
	end
	else
	begin
		case(PADDLE_STATE_3)
		2'b00:
		begin
			PADDLE_ZERO_3 <= PADDLE_ZERO_3 + 1'b1;
			PADDLE_VAL_3 <= 12'd0000;
			if(PADDLE_ZERO_3 == 10'd611)
				PADDLE_STATE_3 <= 2'b01;
		end
		2'b01:
		begin
			PADDLE_ZERO_3 <= 10'd000;
			PADDLE_VAL_3 <= PADDLE_VAL_3 + 1'b1;
			if(PADDLE_VAL_3 == 12'd4094)
				PADDLE_STATE_3 <= 2'b10;
			else
			begin
				if(JOY_TRIGGER)
					PADDLE_STATE_3 <= 2'b10;
				else
					PADDLE_STATE_3 <= 2'b01;
			end
		end
		2'b10:
		begin
			JOY4_COUNT <= PADDLE_VAL_3[11:6];
			PADDLE_LATCH_3 <= PADDLE_VAL_3;
			if(JOY_TRIGGER)
					PADDLE_STATE_3 <= 2'b11;
		end
		2'b11:
		begin
			if(!JOY_TRIGGER)
				PADDLE_STATE_3 <= 2'b00;
		end
		endcase
	end
end

assign JSTICK =	(SEL == 2'b11)		?	JOY3:			// Left Y
						(SEL == 2'b10)		?	JOY4:			// Left X
						(SEL == 2'b01)		?	JOY1:			// Right Y
//						(SEL == 2'b000)		?	JOY2:			// Right X
//						(SEL == 2'b111)		?	JOY1:			// Right Y
//						(SEL == 2'b110)		?	JOY2:			// Right X
//						(SEL == 2'b101)		?	JOY3:			// Left Y
//																		JOY4;			// Left X
																		JOY2;			// Right X

assign JOY1 = (JOY1_COUNT >= DTOA_CODE)	?	1'b1:
															1'b0;

assign JOY2 = (JOY2_COUNT >= DTOA_CODE)	?	1'b1:
															1'b0;

assign JOY3 = (JOY3_COUNT >= DTOA_CODE)	?	1'b1:
															1'b0;

assign JOY4 = (JOY4_COUNT >= DTOA_CODE)	?	1'b1:
															1'b0;

