////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		sound.v
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

// Internal Sound generation
assign SOUND		=	{1'b0, SBS, SOUND_DTOA};

assign DAC_LEFT	=	{2'b00, ORCH_LEFT,  ORCH_LEFT_EXT, 1'b0}	+ {2'b00, SOUND, 9'h000};
assign DAC_RIGHT	=	{2'b00, ORCH_RIGHT, ORCH_RIGHT_EXT, 1'b0}	+ {2'b00, SOUND, 9'h000};

assign AUD_XCK = CLK24MHZ_2;

//Delay LRCLK half cycle
always @(posedge AUD_BCLK)
begin
	DACLRCLK <= AUD_DACLRCK;
	ADCLRCLK <= AUD_ADCLRCK;
end
always @(negedge AUD_DACLRCK)
begin
	LEFT <= DAC_LEFT;
	RIGHT <= DAC_RIGHT;
end
always @(negedge AUD_BCLK or negedge RESET_N)
begin
	if(!RESET_N)
		DAC_STATE <= 6'h00;
	else
		case (DAC_STATE)
		6'h00:
		begin
			if(!DACLRCLK)
			begin
				AUD_DACDAT <= LEFT[18];
				DAC_STATE <= 6'h01;
			end
		end
		6'h01:
		begin
				AUD_DACDAT <= LEFT[17];
				DAC_STATE <= 6'h02;
		end
		6'h02:
		begin
				AUD_DACDAT <= LEFT[16];
				DAC_STATE <= 6'h03;
		end
		6'h03:
		begin
				AUD_DACDAT <= LEFT[15];
				DAC_STATE <= 6'h04;
		end
		6'h04:
		begin
				AUD_DACDAT <= LEFT[14];
				DAC_STATE <= 6'h05;
		end
		6'h05:
		begin
				AUD_DACDAT <= LEFT[13];
				DAC_STATE <= 6'h06;
		end
		6'h06:
		begin
				AUD_DACDAT <= LEFT[12];
				DAC_STATE <= 6'h07;
		end
		6'h07:
		begin
				AUD_DACDAT <= LEFT[11];
				DAC_STATE <= 6'h08;
		end
		6'h08:
		begin
				AUD_DACDAT <= LEFT[10];
				DAC_STATE <= 6'h09;
		end
		6'h09:
		begin
				AUD_DACDAT <= LEFT[9];
				DAC_STATE <= 6'h0A;
		end
		6'h0A:
		begin
				AUD_DACDAT <= LEFT[8];
				DAC_STATE <= 6'h0B;
		end
		6'h0B:
		begin
				AUD_DACDAT <= LEFT[7];
				DAC_STATE <= 6'h0C;
		end
		6'h0C:
		begin
				AUD_DACDAT <= LEFT[6];
				DAC_STATE <= 6'h0D;
		end
		6'h0D:
		begin
				AUD_DACDAT <= LEFT[5];
				DAC_STATE <= 6'h0E;
		end
		6'h0E:
		begin
				AUD_DACDAT <= LEFT[4];
				DAC_STATE <= 6'h0F;
		end
		6'h0F:
		begin
				AUD_DACDAT <= LEFT[3];
				DAC_STATE <= 6'h10;
		end
		6'h10:
		begin
				AUD_DACDAT <= LEFT[2];
				DAC_STATE <= 6'h11;
		end
		6'h11:
		begin
				AUD_DACDAT <= LEFT[1];
				DAC_STATE <= 6'h12;
		end
		6'h12:
		begin
				AUD_DACDAT <= LEFT[0];
				DAC_STATE <= 6'h13;
		end
		6'h13:
		begin
				AUD_DACDAT <= 1'b0;
				DAC_STATE <= 6'h14;
		end
		6'h14:
		begin
			if(DACLRCLK)
			begin
				AUD_DACDAT <= RIGHT[18];
				DAC_STATE <= 6'h15;
			end
		end
		6'h15:
		begin
				AUD_DACDAT <= RIGHT[17];
				DAC_STATE <= 6'h16;
		end
		6'h16:
		begin
				AUD_DACDAT <= RIGHT[16];
				DAC_STATE <= 6'h17;
		end
		6'h17:
		begin
				AUD_DACDAT <= RIGHT[15];
				DAC_STATE <= 6'h18;
		end
		6'h18:
		begin
				AUD_DACDAT <= RIGHT[14];
				DAC_STATE <= 6'h19;
		end
		6'h19:
		begin
				AUD_DACDAT <= RIGHT[13];
				DAC_STATE <= 6'h1A;
		end
		6'h1A:
		begin
				AUD_DACDAT <= RIGHT[12];
				DAC_STATE <= 6'h1B;
		end
		6'h1B:
		begin
				AUD_DACDAT <= RIGHT[11];
				DAC_STATE <= 6'h1C;
		end
		6'h1C:
		begin
				AUD_DACDAT <= RIGHT[10];
				DAC_STATE <= 6'h1D;
		end
		6'h1D:
		begin
				AUD_DACDAT <= RIGHT[9];
				DAC_STATE <= 6'h1E;
		end
		6'h1E:
		begin
				AUD_DACDAT <= RIGHT[8];
				DAC_STATE <= 6'h1F;
		end
		6'h1F:
		begin
				AUD_DACDAT <= RIGHT[7];
				DAC_STATE <= 6'h20;
		end
		6'h20:
		begin
				AUD_DACDAT <= RIGHT[6];
				DAC_STATE <= 6'h21;
		end
		6'h21:
		begin
				AUD_DACDAT <= RIGHT[5];
				DAC_STATE <= 6'h22;
		end
		6'h22:
		begin
				AUD_DACDAT <= RIGHT[4];
				DAC_STATE <= 6'h23;
		end
		6'h23:
		begin
				AUD_DACDAT <= RIGHT[3];
				DAC_STATE <= 6'h24;
		end
		6'h24:
		begin
				AUD_DACDAT <= RIGHT[2];
				DAC_STATE <= 6'h25;
		end
		6'h25:
		begin
				AUD_DACDAT <= RIGHT[1];
				DAC_STATE <= 6'h26;
		end
		6'h26:
		begin
				AUD_DACDAT <= RIGHT[0];
				DAC_STATE <= 6'h27;
		end
		6'h27:
		begin
				AUD_DACDAT <= 1'b0;
				DAC_STATE <= 6'h00;
		end
		default:
		begin
				AUD_DACDAT <= 1'b0;
				DAC_STATE <= 6'h00;
		end
		endcase
end
