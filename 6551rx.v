////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		6551rx.v
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

module uart51_rx(
RESET_N,
BAUD_CLK,
RX_DATA,
RX_BUFFER,
RX_WORD,
RX_PAR_DIS,
RX_PARITY,
PARITY_ERR,
FRAME,
READY
);
input					RESET_N;
input					BAUD_CLK;
input					RX_DATA;
output	[7:0]		RX_BUFFER;
reg		[7:0]		RX_BUFFER;
input		[1:0]		RX_WORD;
input					RX_PAR_DIS;
input		[1:0]		RX_PARITY;
output				PARITY_ERR;
reg					PARITY_ERR;
output				FRAME;
reg					FRAME;
output				READY;
reg					READY;
reg		[5:0]		STATE;
reg		[2:0]		BIT;
reg					RX_DATA0;
reg					RX_DATA1;

always @ (posedge BAUD_CLK or negedge RESET_N)
begin
	if(!RESET_N)
	begin
		RX_BUFFER <= 8'h00;
		STATE <= 6'b000000;
		FRAME <= 1'b0;
		BIT <= 3'b000;
		RX_DATA0 <= 1'b1;
		RX_DATA1 <= 1'b1;
		READY <= 1'b0;
	end
	else
	begin
		RX_DATA0 <= RX_DATA;
		RX_DATA1 <= RX_DATA0;
		case (STATE)
		6'b000000:										// States 0-15 will be start bit
		begin
			BIT <= 3'b000;
			if(~RX_DATA1)
				STATE <= 6'b000001;
		end
		6'b001111:								// End of start bit, flag data not ready
		begin										// If data is not retrieved before this, then overrun
			READY <= 1'b0;
			STATE <= 6'b010000;
		end
		6'b010111:										// Each data bit is states 16-31, the middle is 23
		begin
			RX_BUFFER[BIT] <= RX_DATA1;
			STATE <= 6'b011000;
		end
		6'b011111:										// End of the data bits
		begin
			if(BIT == 3'b111)
			begin
				STATE <= 6'b100000;
			end
			else
			begin
				if((RX_WORD == 2'b01) && (BIT == 3'b110))
				begin
					STATE <= 6'b100000;
				end
				else
				begin
					if((RX_WORD == 2'b10) && (BIT == 3'b101)) 
					begin
						STATE <= 6'b100000;
					end
					else
					begin
						if((RX_WORD == 2'b11) && (BIT == 3'b100))
						begin
							STATE <= 6'b100000;
						end
						else
						begin
							BIT <= BIT + 1'b1;
							STATE <= 6'b010000;
						end
					end
				end
			end
		end
		6'b100000:										// First tick of Stop or Parity, Parity is 32 - 47
		begin
			if(RX_PAR_DIS)
				STATE <= 6'b110001;		// get stop
			else
				STATE <= 6'b100001;		// get parity
		end
		6'b100111:										// Middle of Parity is 39
		begin
			PARITY_ERR <= ~RX_PARITY[1] &											// Get but do not check Parity if 1 is set
							 (((RX_BUFFER[0] ^ RX_BUFFER[1])
							 ^ (RX_BUFFER[2] ^ RX_BUFFER[3]))

							 ^((RX_BUFFER[4] ^ RX_BUFFER[5])
							 ^ (RX_BUFFER[6] ^ RX_BUFFER[7]))	// clear bit #8 if only 7 bits

							 ^ (~RX_PARITY[0] ^ RX_DATA1));
			STATE <= 6'b101000;
// 1 bit early for timing reasons
		end
		6'b110111:										// first stop bit is 32 or 48 then 49 - 63
		begin
			FRAME <= !RX_DATA1;			// if data != 1 then not stop bit
			READY <= 1'b1;
			STATE <= 6'b111000;
		end
// In case of a framing error, wait until data is 1 then start over
// We skipped this check for 6 clock cycles so CPU speed is not a factor
// in the RX_READY state machine above
		6'b111000:
		begin
			if(RX_DATA1)
				STATE <= 6'b000000;
		end
		default: STATE <= STATE + 1'b1;
		endcase
	end
end
endmodule
