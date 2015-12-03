////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		i2c.v
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

module I2C(
CLOCK,
RESET_N,
I2C_CLK,
I2C_CLK_EN,
I2C_DAT,
I2C_DAT_EN,
DEVICE,
REGISTER,
DATA_IN,
DATA_OUT,
DONE,
FAIL,
RW_N,
START
);

input 			CLOCK;
input				RESET_N;
input				I2C_CLK;
output			I2C_CLK_EN;
reg				I2C_CLK_EN;
input				I2C_DAT;
output			I2C_DAT_EN;
reg				I2C_DAT_EN;
input		[6:0]	DEVICE;
input		[7:0]	REGISTER;
output	[7:0]	DATA_IN;
reg		[7:0]	DATA_IN;
input		[7:0]	DATA_OUT;
output			DONE;
reg				DONE;
output			FAIL;
reg				FAIL;
input				RW_N;
input				START;

reg		[5:0]	STATE;
reg		[2:0]	BIT;
wire		[7:0]	ADD_READ;
wire		[7:0]	ADD_WRITE;
reg		[1:0]	START_BUF;

assign ADD_READ = {DEVICE, 1'b1};
assign ADD_WRITE = {DEVICE, 1'b0};

always @(negedge CLOCK or negedge RESET_N)
begin
	if(!RESET_N)
	begin
		I2C_CLK_EN	<= 1'b1;
		I2C_DAT_EN	<= 1'b1;
		DONE			<= 1'b1;
		FAIL			<= 1'b0;
		DATA_IN		<= 8'h00;
		STATE			<= 6'h00;
		BIT			<= 3'h7;
		START_BUF	<= 2'b00;
	end
	else
	begin
	START_BUF[1] <= START_BUF[0];
	START_BUF[0] <= START;
	case (STATE)
// Start Sequence
		6'h00:
		begin
			if(START_BUF[1])
			begin
				I2C_DAT_EN <= 1'b0;
				BIT <= 3'h7;
				DONE <= 1'b0;
				FAIL <= 1'b0;
				STATE <= 6'h01;
			end
		end
		6'h01:
		begin
				STATE <= 6'h02;
		end
		6'h02:
		begin
				I2C_CLK_EN <= 1'b0;
				STATE <= 6'h03;
		end
// Device Address
		6'h03:
		begin
				I2C_DAT_EN <= ADD_WRITE[BIT];
				STATE <= 6'h04;
		end
		6'h04:
		begin
				I2C_CLK_EN <= 1'b1;
				STATE <= 6'h05;
		end
		6'h05:
		begin
			if(I2C_CLK)
			begin
				STATE <= 6'h06;
			end
		end
		6'h06:
		begin
				I2C_CLK_EN <= 1'b0;
				if(BIT == 3'h0)
					STATE <= 6'h07;
				else
				begin
					BIT <= BIT - 1'h1;
					STATE <= 6'h03;
				end
		end
// Acknowledge
		6'h07:
		begin
				I2C_DAT_EN <= 1'b1;
				STATE <= 6'h08;
		end
		6'h08:
		begin
				I2C_CLK_EN <= 1'b1;
				STATE <= 6'h09;
		end
		6'h09:
		begin
			if(I2C_CLK)
			begin
				STATE <= 6'h0A;
			end
		end
		6'h0A:
		begin
			I2C_CLK_EN <= 1'b0;
			I2C_DAT_EN <= 1'b0;
			BIT <= 3'h7;
			if(I2C_DAT)
			begin
				FAIL <= 1'b1;
				STATE <= 6'h3A; // terminate
			end
			else
				STATE <= 6'h0B;
		end
// Register Address
		6'h0B:
		begin
				I2C_DAT_EN <= REGISTER[BIT];
				STATE <= 6'h0C;
		end
		6'h0C:
		begin
				I2C_CLK_EN <= 1'b1;
				STATE <= 6'h0D;
		end
		6'h0D:
		begin
			if(I2C_CLK)
			begin
				STATE <= 6'h0E;
			end
		end
		6'h0E:
		begin
				I2C_CLK_EN <= 1'b0;
				if(BIT == 3'h0)
					STATE <= 6'h0F;
				else
				begin
					BIT <= BIT - 1'h1;
					STATE <= 6'h0B;
				end
		end
// Acknowledge
		6'h0F:
		begin
				I2C_DAT_EN <= 1'b1;
				STATE <= 6'h10;
		end
		6'h10:
		begin
				I2C_CLK_EN <= 1'b1;
				STATE <= 6'h11;
		end
		6'h11:
		begin
			if(I2C_CLK)
			begin
				STATE <= 6'h12;
			end
		end
		6'h12:
		begin
			I2C_CLK_EN <= 1'b0;
			I2C_DAT_EN <= 1'b0;
			BIT <= 3'h7;
			if(I2C_DAT)
			begin
				FAIL <= 1'b1;
				STATE <= 6'h3A; // terminate
			end
			else
				if(!RW_N)
					STATE <= 6'h13;	//Goto write data
				else
					STATE <= 6'h20;	//Goto read data
		end
// Write data
		6'h13:
		begin
				I2C_DAT_EN <= DATA_OUT[BIT];
				STATE <= 6'h14;
		end
		6'h14:
		begin
				I2C_CLK_EN <= 1'b1;
				STATE <= 6'h15;
		end
		6'h15:
		begin
			if(I2C_CLK)
			begin
				STATE <= 6'h16;
			end
		end
		6'h16:
		begin
				I2C_CLK_EN <= 1'b0;
				if(BIT == 3'h0)
					STATE <= 6'h17;
				else
				begin
					BIT <= BIT - 1'h1;
					STATE <= 6'h13;
				end
		end
// Acknowledge
		6'h17:
		begin
				I2C_DAT_EN <= 1'b1;
				STATE <= 6'h18;
		end
		6'h18:
		begin
				I2C_CLK_EN <= 1'b1;
				STATE <= 6'h19;
		end
		6'h19:
		begin
			if(I2C_CLK)
			begin
				STATE <= 6'h1A;
			end
		end
		6'h1A:
		begin
			I2C_CLK_EN <= 1'b0;
			I2C_DAT_EN <= 1'b0;
			BIT <= 3'h7;
			STATE <= 6'h3A;
			if(I2C_DAT)
				FAIL <= 1'b1;
		end
// Resend Start sequence for read data
// I2C_CLK_EN is 0
		6'h20:
		begin
			I2C_DAT_EN <= 1'b1;
			if(I2C_DAT)
				STATE <= 6'h21;
		end
		6'h21:
		begin
			I2C_CLK_EN <= 1'b1;
			if(I2C_CLK)
				STATE <= 6'h22;
		end
		6'h22:
		begin
				STATE <= 6'h23;
		end
		6'h23:
		begin
				STATE <= 6'h24;
		end
		6'h24:
		begin
				I2C_DAT_EN <= 1'b0;
				BIT <= 3'h7;
				STATE <= 6'h25;
		end
		6'h25:
		begin
				STATE <= 6'h26;
		end
		6'h26:
		begin
				I2C_CLK_EN <= 1'b0;
				STATE <= 6'h27;
		end
// Device Address (Write)
		6'h27:
		begin
				I2C_DAT_EN <= ADD_READ[BIT];
				STATE <= 6'h28;
		end
		6'h28:
		begin
				I2C_CLK_EN <= 1'b1;
				STATE <= 6'h29;
		end
		6'h29:
		begin
			if(I2C_CLK)
			begin
				STATE <= 6'h2A;
			end
		end
		6'h2A:
		begin
				I2C_CLK_EN <= 1'b0;
				if(BIT == 3'h0)
					STATE <= 6'h2B;
				else
				begin
					BIT <= BIT - 1'h1;
					STATE <= 6'h27;
				end
		end
// Acknowledge
		6'h2B:
		begin
				I2C_DAT_EN <= 1'b1;
				STATE <= 6'h2C;
		end
		6'h2C:
		begin
				I2C_CLK_EN <= 1'b1;
				STATE <= 6'h2D;
		end
		6'h2D:
		begin
			if(I2C_CLK)
			begin
				STATE <= 6'h2E;
			end
		end
		6'h2E:
		begin
			I2C_CLK_EN <= 1'b0;
			I2C_DAT_EN <= 1'b0;
			BIT <= 3'h7;
			if(I2C_DAT)
			begin
//				DONE <= 1'b1;
				FAIL <= 1'b1;
				STATE <= 6'h3A; // terminate
			end
			else
				STATE <= 6'h2F;
		end
// Read data
		6'h2F:
		begin
				STATE <= 6'h30;
		end
		6'h30:
		begin
				I2C_CLK_EN <= 1'b1;
				STATE <= 6'h31;
		end
		6'h31:
		begin
			if(I2C_CLK)
			begin
				STATE <= 6'h32;
			end
		end
		6'h32:
		begin
				I2C_CLK_EN <= 1'b0;
				DATA_IN[BIT] <= I2C_DAT;
				if(BIT == 3'h0)
					STATE <= 6'h33;
				else
				begin
					BIT <= BIT - 1'h1;
					STATE <= 6'h2F;
				end
		end
// Acknowledge
		6'h33:
		begin
				I2C_DAT_EN <= 1'b1;
				STATE <= 6'h34;
		end
		6'h34:
		begin
				I2C_CLK_EN <= 1'b1;
				STATE <= 6'h35;
		end
		6'h35:
		begin
			if(I2C_CLK)
			begin
				STATE <= 6'h36;
			end
		end
		6'h36:
		begin
			I2C_CLK_EN <= 1'b0;
			I2C_DAT_EN <= 1'b0;
			BIT <= 3'h7;
			STATE <= 6'h3A;
			if(I2C_DAT)
				FAIL <= 1'b1;
		end
// Stop Sequence
		6'h3A:
		begin
			I2C_DAT_EN <= 1'b0;
			STATE <= 6'h3B;
		end
		6'h3B:
		begin
			I2C_CLK_EN <= 1'b1;
			if(I2C_CLK)
				STATE <= 6'h3C;
		end
		6'h3C:
		begin
				STATE <= 6'h3D;
		end
		6'h3D:
		begin
			I2C_DAT_EN <= 1'b1;
			if(I2C_DAT)
				STATE <= 6'h3E;
		end
		6'h3E:
		begin
				STATE <= 6'h3F;
		end
		6'h3F:
		begin
			if(!START_BUF[1])
			begin
				DONE <= 1'b1;
				STATE <= 6'h00;
			end
		end
		default:
			STATE <= 6'h00;
		endcase
		end
	end
endmodule
