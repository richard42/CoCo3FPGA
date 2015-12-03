////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		cocokey.v
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

module COCOKEY(
		RESET_N,
		CLK50MHZ,				//Not really 50 MHz
		SLO_CLK,
		PS2_CLK,
		PS2_DATA,
		KEY,
		SHIFT,
		SHIFT_OVERRIDE,
		RESET
);

input					RESET_N;
input 				CLK50MHZ;
input					SLO_CLK;
input 				PS2_CLK;
input 				PS2_DATA;
output	[55:0]	KEY;
reg		[55:0]	KEY;
output				SHIFT;
reg					SHIFT;
output				SHIFT_OVERRIDE;
reg					SHIFT_OVERRIDE;
output				RESET;
reg					RESET;

reg		[5:0]		SLO_RESET;
wire					SLO_RESET_N;
reg		[4:0]		KB_CLK;
wire		[7:0]		SCAN;
wire					PRESS;
wire					EXTENDED;

/*			Norm					Shift						CTRL
00			@
01			a						A							^a
02			b						B							^b
03			c						C							^c
04			d						D							^d
05			e						E							^e
06			f						F							^f
07			g						G							^g

08			h						H							^h
09			i						I							^i
10			j						J							^j
11			k						K							^k
12			l						L							^l
13			m						M							^m
14			n						N							^n
15			o						O							^o

16			p						P							^p
17			q						Q							^q
18			r						R							^r
19			s						S							^s
20			t						T							^t
21			u						U							^u
22			v						V							^v
23			w						W							^w

24			x						X							^x
25			y						Y							^y
26			z						Z							^z
27			up						UP							^up
28			dn						DN							^dn
29			lft BS				LFT						^lft
30			rgt					RGT						^rgt
31			sp						SP							^sp

32			0						CL							^0
33			1						!							 |
34			2						"							^2
35			3						#							 ~
36			4						$							^4
37			5						%							^5
38			6						&							^6
39			7						'							 ^

40			8						(							 [ 
41			9						)							 ]
42			:						*							
43			;						+
44			,						<							 {
45			-						=							 _
46			.						>							 }
47			/						?							 \

48			cr						CR							^cr
49			tab					TAB						^tab
50			esc					ESC						^esc
51			alt					ALT						^alt
52			ctrl					CTRL						^
53			f1						F1							^f1
54			f2						F2							^f2
55			lsh rsh				LSH RSH					^lsh ^rsh
*/

always @ (negedge SLO_CLK or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		SLO_RESET <= 6'h00;
	end
	else
	begin
		if(SLO_RESET != 6'h3F)
			SLO_RESET <= SLO_RESET + 1'b1;
	end
end
assign SLO_RESET_N = (SLO_RESET == 6'h3F);

always @(posedge KB_CLK[4] or negedge SLO_RESET_N)
begin
	if(~SLO_RESET_N)
	begin
		KEY <= 56'h00000000000000;
		SHIFT_OVERRIDE <= 1'b0;
		SHIFT <= 1'b0;
		RESET <= 1'b0;
	end
	else
	begin
		case(SCAN)
		8'h76:		KEY[50] <= PRESS;					// ESC
		8'h05:		KEY[53] <= PRESS;					// F1
		8'h06:		KEY[54] <= PRESS;					// F2
		8'h07:		KEY[0] <= PRESS;					// @ (must be used when there is a shift or ctrl)
		8'h0E:
		begin
			if(PRESS)
			begin
						KEY[35] <= 1'b1;					// ~ is CTRL - 3
						KEY[52] <= 1'b1;
						SHIFT_OVERRIDE <= 1'b1;
			end
			else
			begin
						KEY[35] <= 1'b0;					// ~ is CTRL - 3
						KEY[52] <= 1'b0;
						SHIFT_OVERRIDE <= 1'b0;
			end						
		end
		8'h16:		KEY[33] <= PRESS;					// 1 !
		8'h1E:												// 2 @
		begin
			if(PRESS)										// Pressed
			begin
				if(!KEY[55])								// not shifted
						KEY[34] <= 1'b1;					// 2
				else											// shifted
				begin
						KEY[34] <= 1'b0;					// Not 2 = @
						KEY[0] <= 1'b1;					// @
						SHIFT_OVERRIDE <= 1'b1;			// Override the shift
//						SHIFT <= 1'b0;						// Not Right Shifted
				end
			end
			else												// Released
			begin
						KEY[34] <= 1'b0;
						KEY[0] <= 1'b0;
						SHIFT_OVERRIDE <= 1'b0;
			end
		end
		8'h26:		KEY[35] <= PRESS;					// 3 #
		8'h25:		KEY[36] <= PRESS;					// 4 $
		8'h2E:		KEY[37] <= PRESS;					// 5 %
		8'h36:
		begin
			if(PRESS)
			begin
				if(!KEY[55])
						KEY[38] <= PRESS;					// 6
				else
				begin
						KEY[39] <= 1'b1;					// CTRL 7 = ^
						KEY[52] <= 1'b1;					// CTRL
						SHIFT_OVERRIDE <= 1'b1;			// No shift
				end
			end
			else
			begin
						KEY[38] <= 1'b0;
						KEY[39] <= 1'b0;
						KEY[52] <= 1'b0;
						SHIFT_OVERRIDE <= 1'b0;
			end
		end
		8'h3D:
		begin
			if(PRESS)
			begin
				if(!KEY[55])
						KEY[39] <= 1'b1;					// 7
				else
						KEY[38] <= 1'b1;					// Shifted 6 = &
			end
			else
			begin
						KEY[38] <= 1'b0;
						KEY[39] <= 1'b0;
			end
		end
		8'h3E:
		begin
			if(PRESS)										// Pressed
			begin
				if(!KEY[55])								// not shifted
						KEY[40] <= 1'b1;					// 8
				else
						KEY[42] <= 1'b1;					// Shift : = *
			end
			else
			begin
						KEY[40] <= 1'b0;
						KEY[42] <= 1'b0;
			end
		end
		8'h46:
		begin
			if(PRESS)										// Pressed
			begin
				if(!KEY[55])								// not shifted
						KEY[41] <= 1'b1;					// 9
				else
						KEY[40] <= 1'b1;					// Shifted 8 = (
			end
			else
			begin
						KEY[40] <= 1'b0;
						KEY[41] <= 1'b0;
			end
		end
		8'h45:
		begin
			if(PRESS)										// Pressed
			begin
				if(!KEY[55])								// not shifted
						KEY[32] <= 1'b1;					// 0
				else
						KEY[41] <= 1'b1;					// shifted 9 = )
			end
			else
			begin
						KEY[32] <= 1'b0;
						KEY[41] <= 1'b0;
			end
		end
		8'h4E:
		begin
			if(PRESS)
			begin
				if(!KEY[55])
						KEY[45] <= 1'b1;					// -
				else
				begin
						KEY[45] <= 1'b1;					// CTRL - = _
						SHIFT_OVERRIDE <= 1'b1;			// not shifted
						KEY[52] <= 1'b1;					// CTRL
				end
			end
			else
			begin
						KEY[45] <= 1'b0;
						SHIFT_OVERRIDE <= 1'b0;
						KEY[52] <= 1'b0;
			end
		end
		8'h55:
		begin
			if(PRESS)										// Pressed
			begin
				if(!KEY[55])								// not shifted
				begin
						KEY[45] <= 1'b1;					// =
//						SHIFT_OVERRIDE <= 1'b1;			// Override the shift
						SHIFT <= 1'b1;						// Shifted
				end
				else											// shifted
				begin
						KEY[43] <= 1'b1;					// +
				end
			end
			else
			begin
						KEY[43] <= 1'b0;
						KEY[45] <= 1'b0;
//						SHIFT_OVERRIDE <= 1'b0;
						SHIFT <= 1'b0;
			end
		end
		8'h5D:
		begin
			if(PRESS)										// Pressed
			begin
				if(!KEY[55])								// not shifted
				begin
						KEY[47] <= 1'b1;					// CTRL - / = \
						KEY[52] <= 1'b1;					// CTRL
				end
				else
				begin
						KEY[33] <= 1'b1;					// CTRL - 1 = |
						KEY[52] <= 1'b1;					// CTRL
						SHIFT_OVERRIDE <= 1'b1;
				end
			end
			else
			begin
						KEY[33] <= 1'b0;
						KEY[47] <= 1'b0;
						KEY[52] <= 1'b0;
						SHIFT_OVERRIDE <= 1'b0;
			end
		end
		8'h66:		KEY[29] <= PRESS;					// backspace
		8'h0D:		KEY[49] <= PRESS;					// TAB
		8'h15:		KEY[17] <= PRESS;					// Q
		8'h1D:		KEY[23] <= PRESS;					// W
		8'h24:		KEY[5] <= PRESS;					// E
		8'h2D:		KEY[18] <= PRESS;					// R
		8'h2C:		KEY[20] <= PRESS;					// T
		8'h35:		KEY[25] <= PRESS;					// Y
		8'h3C:		KEY[21] <= PRESS;					// U
		8'h43:		KEY[9] <= PRESS;					// I
		8'h44:		KEY[15] <= PRESS;					// O
		8'h4D:		KEY[16] <= PRESS;					// P
		8'h54:
		begin
			if(PRESS)										// Pressed
			begin
				if(!KEY[55])								// not shifted
				begin
						KEY[40] <= 1'b1;					// CTRL - 8 = [
						KEY[52] <= 1'b1;					// CTRL
				end
				else
				begin
						KEY[44] <= 1'b1;					// CTRL - , = {
						KEY[52] <= 1'b1;					// CTRL
						SHIFT_OVERRIDE <= 1'b1;
				end
			end
			else
			begin
						KEY[40] <= 1'b0;
						KEY[44] <= 1'b0;
						KEY[52] <= 1'b0;
						SHIFT_OVERRIDE <= 1'b0;
			end
		end
		8'h5B:
		begin
			if(PRESS)										// Pressed
			begin
				if(!KEY[55])								// not shifted
				begin
						KEY[41] <= 1'b1;					// CTRL - 9 = ]
						KEY[52] <= 1'b1;					// CTRL
				end
				else
				begin
						KEY[46] <= 1'b1;					// CTRL - . = }
						KEY[52] <= 1'b1;					// CTRL
						SHIFT_OVERRIDE <= 1'b1;
				end
			end
			else
			begin
						KEY[41] <= 1'b0;
						KEY[46] <= 1'b0;
						KEY[52] <= 1'b0;
						SHIFT_OVERRIDE <= 1'b0;
			end
		end
		8'h58:
		begin
						KEY[32] <= PRESS;					// Caps Lock = Shift 0
						KEY[55] <= PRESS;
//						SHIFT_OVERRIDE <= PRESS;
		end
		8'h1C:		KEY[1] <= PRESS;					// A
		8'h1B:		KEY[19] <= PRESS;					// S
		8'h23:		KEY[4] <= PRESS;					// D
		8'h2B:		KEY[6] <= PRESS;					// F
		8'h34:		KEY[7] <= PRESS;					// G
		8'h33:		KEY[8] <= PRESS;					// H
		8'h3B:		KEY[10] <= PRESS;					// J
		8'h42:		KEY[11] <= PRESS;					// K
		8'h4B:		KEY[12] <= PRESS;					// L
		8'h4C:
		begin
			if(PRESS)										// Pressed
			begin
				if(!KEY[55])								// not shifted
						KEY[43] <= 1'b1;					// ;
				else
				begin
						KEY[42] <= 1'b1;					// :
						SHIFT_OVERRIDE <= 1'b1;			// Override the shift
				end
			end
			else												// Released
			begin
						KEY[42] <= 1'b0;
						KEY[43] <= 1'b0;
						SHIFT_OVERRIDE <= 1'b0;
			end
		end
		8'h52:
		begin
			if(PRESS)
			begin
				if(!KEY[55])
				begin
						KEY[39] <= 1'b1;					// Shift 7 = '
						SHIFT <= 1'b1;
				end
				else
				begin
						KEY[34] <= 1'b1;					// Shift 2 = "
				end
			end
			else
			begin
						KEY[34] <= 1'b0;
						KEY[39] <= 1'b0;
						SHIFT <= 1'b0;
			end
		end
		8'h5A:		KEY[48] <= PRESS;					// CR
		8'h12:		KEY[55] <= PRESS;					// L-Shift
		8'h1A:		KEY[26] <= PRESS;					// Z
		8'h22:		KEY[24] <= PRESS;					// X
		8'h21:		KEY[3] <= PRESS;					// C
		8'h2a:		KEY[22] <= PRESS;					// V
		8'h32:		KEY[2] <= PRESS;					// B
		8'h31:		KEY[14] <= PRESS;					// N
		8'h3A:		KEY[13] <= PRESS;					// M
		8'h41:		KEY[44] <= PRESS;					// , <
		8'h49:		KEY[46] <= PRESS;					// . >
		8'h4A:		KEY[47] <= PRESS;					// / ?
		8'h59:		KEY[55] <= PRESS;					// R-Shift
		8'h14:		KEY[52] <= PRESS;					// Ctrl either left or right
		8'h11:		KEY[51] <= PRESS;					// ALT either left or right
		8'h29:		KEY[31] <= PRESS;					// Space
		8'h75:		if(EXTENDED)
							KEY[27] <= PRESS;					// up
		8'h74:		if(EXTENDED)
							KEY[30] <= PRESS;					// right
		8'h6B:		if(EXTENDED)
							KEY[29] <= PRESS;					// left
		8'h72:		if(EXTENDED)
							KEY[28] <= PRESS;					// down
		8'h71:
		begin
			if(KEY[51] & KEY[52])
						RESET <= PRESS;
		end
		8'h7E:
		begin
						KEY[23] <= PRESS;					// Scroll Lock = CTRL w
						KEY[52] <= PRESS;
						SHIFT_OVERRIDE <= PRESS;
		end
		endcase
	end
end

//	KB_CLK[0] = 50/2	= 25 MHz
//	KB_CLK[1] = 50/4	= 12.5 MHz
//	KB_CLK[2] = 50/8	= 6.25 MHz
//	KB_CLK[3] = 50/16	= 3.125 MHz
//	KB_CLK[4] = 50/32	= 1.5625 MHz
//	KB_CLK[5] = 50/64	= 0.78125 MHz
always @ (posedge CLK50MHZ)				//50 MHz
	KB_CLK <= KB_CLK + 1'b1;

ps2_keyboard KEYBOARD(
		.RESET_N(RESET_N),
		.CLK(KB_CLK[4]),
		.PS2_CLK(PS2_CLK),
		.PS2_DATA(PS2_DATA),
		.RX_SCAN(SCAN),
		.RX_PRESSED(PRESS),
		.RX_EXTENDED(EXTENDED)
);

endmodule
