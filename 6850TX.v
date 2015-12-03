////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		6850TX.v
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

module UART_TX(
BAUD_CLK,
RESET_N,
TX_DATA,
TX_START,
TX_DONE,
TX_STOP,
TX_WORD,
TX_PAR_DIS,
TX_PARITY,
TX_BUFFER
);

input					BAUD_CLK;
input					RESET_N;
output				TX_DATA;
reg					TX_DATA;
input					TX_START;
output				TX_DONE;
reg					TX_DONE;
input					TX_STOP;
input					TX_WORD;
input					TX_PAR_DIS;
input					TX_PARITY;

input		[7:0]		TX_BUFFER;

reg		[6:0]		STATE;
reg		[2:0]		BIT;
wire					PARITY;
reg					TX_START0;
reg					TX_START1;

assign PARITY =	((TX_BUFFER[0] ^ TX_BUFFER[1])
					^	 (TX_BUFFER[2] ^ TX_BUFFER[3]))
					^	((TX_BUFFER[4] ^ TX_BUFFER[5])
					^	 (TX_BUFFER[6] ^ (TX_BUFFER[7] & TX_WORD))) // clear bit #8 if only 7 bits
					^	  TX_PARITY;

always @ (negedge BAUD_CLK or negedge RESET_N)
begin
	if(!RESET_N)
	begin
		STATE <= 7'b0000000;
		TX_DATA <= 1'b1;
		TX_DONE <= 1'b1;
		BIT <= 3'b000;
		TX_START0 <= 1'b0;
		TX_START1 <= 1'b0;
	end
	else
	begin
		TX_START0 <= TX_START;
		TX_START1 <= TX_START0;
		case (STATE)
		7'b0000000:
		begin
			BIT <= 3'b000;
			TX_DATA <= 1'b1;
			if(TX_START1)
			begin
				TX_DONE <= 1'b0;
				STATE <= 7'b0000001;
			end
		end
		7'b0000001:
		begin
			TX_DATA <= 1'b0;
			STATE <= 7'b0000010;
		end
		7'b0010001:
		begin
			TX_DATA <= TX_BUFFER[BIT];
			STATE <= 7'b0010010;
		end
		7'b0100000:
		begin
			BIT <= BIT + 1'b1;
			if(BIT != {2'b11, TX_WORD})
				STATE <= 7'b0010001;
			else
				if(!TX_PAR_DIS)
					STATE <= 7'b0100001;				// do parity
				else
					STATE <= 7'b0110001;				// do stop
		end
// Start parity bit
		7'b0100001:
		begin
			TX_DATA <= PARITY;
			STATE <= 7'b0100010;
		end
// start stop
		7'b0110001:
		begin
			TX_DONE <= 1'b1;
			TX_DATA <= 1'b1;
			STATE <= 7'b0110010;
		end
// end of first stop bit
		7'b0111111:
		begin
			if(!TX_STOP)
				STATE <= 7'b1001111;
			else
				STATE <= 7'b1000000;
		end
		7'b1001111:
		begin
			STATE <= 7'b0000000;
		end
		default: STATE <= STATE + 1'b1;
		endcase
	end
end
endmodule
