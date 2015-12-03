////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		SDCard.v
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

module SDCard(
// CPU interface
	input		wire			clk_i,			// Clock asynchronous 2x sclk
	input		wire			cpuclk_n_i,		// CPU Clock
	input		wire			reset_n_i,		// reset (asynchronous active low)
	input		wire			cs_i,				// Chip Select (active high)
	input		wire			adr_i,         // Address 0
	input		wire			rw_n_i,			// Read (active high) Write_N (active low)
	output	reg			halt_o,				// Halt to CPU for sync / testing
	input		wire	[7:0]	dat_i,			// data input
	output	wire	[7:0]	dat_o,
	output	wire			irq_n_o,			// irq output (low active)

// SPI port
	output	wire			act_led_n_o,
	input		wire			card_detect_n_i,	// 0=card inserted 1=slot empty
	input		wire			wp_locked_i,		// 0=not locked 1=locked
	output	reg			spi_ss_n_o,		// SPI Slot Select (low active)
	output	reg			sclk_o,			// serial clock output
	output	reg			mosi_o,			// MasterOut SlaveIN
	input		wire			miso_i			// MasterIn SlaveOut
);

reg							en_spi;
reg					[2:0]	state;
reg					[3:0]	bcnt;
reg							wffull;
reg					[1:0]	wffull_buf;
reg					[7:0]	rreg1;
reg					[7:0]	buffer1;
reg							wffull_reset;
reg					[2:0]	cd_buff0;
reg							irq_n;
reg							en_irq;
wire							irq_reset_n;
reg							halt_buf0;
reg							halt_buf1;
reg					[1:0]	halt_state;

assign dat_o = (!adr_i)	?	{!irq_n, 5'b00000, wp_locked_i, !card_detect_n_i}:	// Address=0
									rreg1;																// Address=1

assign irq_reset_n =	~reset_n_i							?	1'b0:							// system reset
							~en_spi								?	1'b0:							// SPI disabled
							~en_irq								?	1'b0:
																		1'b1;

assign irq_n_o = 			irq_n;

assign act_led_n_o = state[0];

always @(negedge cpuclk_n_i or posedge wffull_reset)
begin
	if(wffull_reset)
		wffull <= 1'b0;
	else
		if({cs_i, adr_i} == 2'b11)										// Read / Write data register
		begin
			wffull <= 1'b1;
		end
end

always @(negedge cpuclk_n_i or negedge reset_n_i)
begin
	if(!reset_n_i)
	begin
		halt_buf0 <= 1'b0; 
		halt_buf1 <= 1'b0;
		halt_state <= 2'b00;
		halt_o <= 1'b0;
	end
	else
	begin
		halt_buf0 <= !state[0];			// double buffer since different clock domains 
		halt_buf1 <= halt_buf0;
		case(halt_state)
		2'b00:
		begin
			if({cs_i, adr_i, en_spi}== 3'b111)		// Read or write to data buffer while SPI is enabled
			begin
				halt_o <= 1'b1;				// Send out halt
				halt_state <= 2'b01;
			end
		end
		2'b01:
		begin
			if(halt_buf1)						// The TX is started
				halt_state <= 2'b10;
		end
		2'b10:
		begin
			if(!halt_buf1)						// The TX is finished
			begin
				halt_state <= 2'b00;
				halt_o <= 1'b0;				// Turn off halt
			end
		end
		2'b11:									// JUST IN CASE
		begin
			halt_state <= 2'b00;
		end
		endcase
	end
end

always @(negedge cpuclk_n_i or negedge reset_n_i)
begin
	if(!reset_n_i)
	begin
		spi_ss_n_o <= 1'b1;
		en_irq <= 1'b0;
		en_spi <= 1'b0;
		buffer1 <= 8'hFF;
	end
	else
	begin
		case ({cs_i, rw_n_i, adr_i})
		3'b100:																		// Write to control register
		begin
			spi_ss_n_o <= !dat_i[0]|!dat_i[7];								// Slot Select = 0 if SPI Enable = 1
			en_irq <= dat_i[6];
			en_spi <= dat_i[7];
		end
		3'b101:																		// Write to data register
		begin
			buffer1 <= dat_i;
		end
		3'b111:																		// Read from data register 0
		begin																			// Which also writes an FF to the data register
			buffer1 <= 8'hFF;
		end
		endcase
	end
end

always @(negedge cpuclk_n_i)
begin
	if(!irq_reset_n)																// Syncronous RESET
	begin
		irq_n <= 1'b1;
		cd_buff0 <= 3'b000;
	end
	else
	begin
		cd_buff0 <= ({cd_buff0[1], cd_buff0[0], card_detect_n_i});			// Edge Detect insert card
		if(cd_buff0 == 3'b100)															// Assert irq on insert card
			irq_n <= 1'b0;
	end
end

always @(posedge clk_i)
begin
	if (~en_spi)
	begin
		state <= 3'b001; 				// idle
		bcnt  <= 4'h0;
		sclk_o <= 1'b0;
		wffull_buf <= 2'b00;
		wffull_reset <= 1'b1;		// if SPI is not enabled, then keep write buffer empty
		rreg1 <= 8'h00;
	end
	else
	begin
		wffull_buf <= {wffull_buf[0], wffull};
		case (state)
		3'b001:								// idle state
		begin
			sclk_o <= 1'b0;				// set sck
			wffull_reset <= 1'b0;
			if (wffull_buf[1])
			begin
				bcnt  <= 4'h0;				// set transfer counter
				rreg1 <= buffer1;
				state <= 3'b010;
			end
		end
		3'b010:								// clock-phase2, next data
		begin
			sclk_o   <= 1'b0;
			wffull_reset <= 1'b1;
			state   <= 3'b100;
			if (bcnt[3])
			begin
				state <= 3'b001;
				wffull_reset <= 1'b0;
				mosi_o <= 1'b1;
			end
			else
			begin
				state <= 3'b100;
				wffull_reset <= 1'b1;
				mosi_o <= rreg1[7];
			end
		end

		3'b100:								// clock phase1
		begin
			state <= 3'b010;
			sclk_o <= 1'b1;
			rreg1 <= {rreg1[6:0], miso_i};
			bcnt <= bcnt + 4'h1;
		end
		default:
		begin
			state <= 3'b001;
		end
		endcase
	end
end

endmodule
