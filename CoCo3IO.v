////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		CoCo3IO.v
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
* Floppy
******************************************************************************/
assign PH2_02 = MCLOCK[1];			//12.5 MHz
assign CPU_RESET_N = !CPU_RESET;

T65 GLB6502(
  .Clk(~PH2_02),
  .Abort_n(1'b1),
  .NMI_n(1'b1),
  .Rdy(1'b1),
  .Enable(1'b1),
  .Res_n(CPU_RESET_N),
  .SO_n(1'b1),
  .IRQ_n(IRQ_02_BUF1_N),
  .EF(EF),
  .R_W_n(RW_02_N),
  .VDA(VDA),
  .MF(MF),
  .VPA(VPA),
  .ML_n(ML_N),
  .XF(XF),
  .Sync(SYNC),
  .VP_n(VP_N),
  .DI(DATA_IN_02),
  .Mode(2'b00),
  .DO(DATA_OUT_02),
  .A({CPU_BANK, ADDRESS_02})
);
/******************************************************************************
* Memory Map for 6502
*
* 0000-07FF		RAM/ROM
* 0800-0FFF		Secondary RAM/ROM and Vectors
* 1000-EFFF    NA
* F000-F1FF		NA (Buffer?)
* F400			Day of Week
* F401			Seconds
* F402			Minutes
* F403			Hours
* F404			Day
* F405			Month
* F406			Year
* F407			Century
* F408			Extended Drive Select from 6809
* F409			Sector from 6809			(LSN 0-7)
* F40A			Track Regular from 6809		(LSN 8-15)
* F40B			Track Regular from 6502
* F40C			Track Extended from 6809	(LSN 16-23)
* F40D			Track Extended from 6502
* F40E			Disk Command from 6809
* F40F			Data Extended from 6809
* F410			DISK Status from 6502
* F411			Control Bits from 6502
*						Bit 7			IMM_HALT_09        Halt from 6502 to 6809
*						Bit 6			HALT_09_EN         Halt Enable when 256 read / write is finished
*						Bit 5			NMI_09_EN          Immediate NMI
*						Bit 4			IRQ_09_EN          IRQ to 6809 for No-Halt drivers
*						Bit 3			ADDR_RESET_N	    Reset memory pointer to beginning of buffer
*						Bit 2			WAIT_HALT
*						Bit 1			CMD_RESET           6502 RESET for the Command Register
*                 Bit 0       IRQ_RESET           Used to reset the Interrupt going to 6502
* F412			Controller Status
*						Bit 7			HALT                6809 Halted
*						Bit 6			ADD_100             6809 Read 256 bytes
*						Bit 2			RD_FIFO FULL
*						Bit 1			WR_FIFO Not EMPTY
*						Bit 0			UPDATE	    	    Pseudo-minute
* F413			Interrupt Source Byte 0
*						Bit 7			IRQ                 Floppy Command Write
* F414			Interrupt Source Byte 1
* F415			Drive Select from 6809, modified order
*						Bit 7			6809 Halt Enable
*						Bit 6			Density
*						Bit 5			Write Precompensation
*						Bit 4			Motor On
*						Bit 3			Drive Select 3 (Side Select)
*						Bit 2			Drive Select 2
*						Bit 1			Drive Select 1
*						Bit 0			Drive Select 0
* F416			Trace
* F417			Track 1
* F418			Track 2
* F419			Heads
* F41A			FIFO Read / Write
* F41B			I2C Device
* F41C			I2C Register
* F41D			I2C Data
* F41E			I2C Status
* F500-F57F		6850 UART
* F580-F5FF		SPI Control / Status
* F600-F7FF    6502 DISK BUFFER
* F800-FFFF		Mirror of Secondary RAM/ROM and Vectors
*******************************************************************************/

assign	DATA_IN_02 =								RAM02_00_EN 		?	DATAO_02_HDD:			// 0000-07FF and F800-FFFF
															COM1_EN				?	DATA_COM1:				// F500-F5FF
															DISKBUF_02			?	DISK_BUF_Q:				// F600-F7FF
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010000000)	?	DWK:						// F4+00
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010000001)	?	SEC:						// F4+01
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010000010)	?	MIN:						// F4+02
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010000011)	?	HOUR:						// F4+03
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010000100)	?	DMTH:						// F4+04
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010000101)	?	MNTH:						// F4+05
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010000110)	?	YEAR:						// F4+06
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010000111)	?	CENT:						// F4+07
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010001000)	?	DRIVE_SEL_EXT:			// F4+08
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010001001)	?	SECTOR:					// F4+09
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010001010)	?	TRACK_REG_W:			// F4+0A
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010001011)	?	TRACK_REG_R:			// F4+0B
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010001100)	?	TRACK_EXT_W:			// F4+0C
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010001101)	?	TRACK_EXT_R:			// F4+0D
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010001110)	?	COMMAND:					// F4+0E
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010001111)	?	DATA_EXT:				// F4+0F
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010010000)	?	STATUS:					// F4+10
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010010001)	?	{IMM_HALT_09,			// F4+11	Immediate halt 6809
																						HALT_100_09,			// Halt 6809 when data address is 0x100
																						NMI_09_EN,				// Used to generate NMI on 6809
																						IRQ_09_EN,				// Used to generate IRQ on 6809
																						ADDR_RESET_N,			// Reset data register address
																						WAIT_HALT,
																						CMD_RST,
																						IRQ_RESET}:				// Used to reset the Interrupt going to 6502
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010010010)	?	{HALT_SIG_BUF1,		// F4+12	Used to inform 6502 that 6809 is halted
																						ADDR_100_BUF1,			// Used to inform 6502 data address is 0x100
																						3'h0,
																						RDFIFO_WRFULL,
																						!WRFIFO_RDEMPTY,
																						SEC[5]}:					//Update
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010010011)	?	{!IRQ_02_BUF1_N,		// F4+13
																						7'h00}:
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010010100)	?	8'h00:					// F4+14
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010010101)	?	{HALT_EN,				// F4+15 Slightly rearranged dskreg
																						DENSITY,
																						WRT_PREC,
																						MOTOR,
																						DRIVE_SEL_EXT[3:0]}:
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010010110)	?	TRACE:					// F4+16
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010010111)	?	TRACK1:					// F4+17
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010011000)	?	TRACK2:					// F4+18
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010011001)	?	HEADS:					// F4+19
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010011010)	?	WRFIFO_DATA:			// F4+1A
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010011011)	?	I2C_DEVICE:				// F4+1B
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010011100)	?	I2C_REG:					// F4+1C
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010011101)	?	I2C_DATA_IN:			// F4+1D
	({ADDRESS_02[15:8], ADDRESS_02[4:0]} == 13'b1111010011110)	?	{I2C_DONE_BUF[1],		// F4+1E
																						I2C_FAIL,
																						I2C_START,
																						5'b00000}:
																						8'hAA;

assign	RAM02_00_EN =		(ADDRESS_02[15:11] == 5'b00000)										// Zero Page 	(0000h to 07FFh)
								|	(ADDRESS_02[15:11] == 5'b11111);										// Mirror/Vectors (F800-FFFF)
assign	COM1_EN = 			(ADDRESS_02[15:8]  == 8'b11110101);									// UART (F500h to F5FFh)
assign	DISKBUF_02 = 		(ADDRESS_02[15:9]  == 7'b1111011);									// F600-F7FF	Buffers 	(F600h to F7FFh)

assign	WRFIFO_RDREQ =		({ADDRESS_02[15:8], ADDRESS_02[4:0],
									RW_02_N, WRFIFO_RDEMPTY}
													== 15'b111101001101010)	?	1'b1:						// F4+1A (Write)
																						1'b0;

assign	RDFIFO_WRREQ =		({ADDRESS_02[15:8], ADDRESS_02[4:0],
									RW_02_N, RDFIFO_WRFULL}
													== 15'b111101001101000)	?	1'b1:						// F4+1A (Read)
																						1'b0;

always @(negedge PH2_02 or negedge RESET_N)
begin
	if(!RESET_N)
	begin
		TRACE <= 8'h00;
		TRACK_REG_R <= 8'h00;
		TRACK_EXT_R <= 8'h00;
		STATUS <= 8'h00;
		IMM_HALT_09 <= 1'b0;
		HALT_100_09 <= 1'b0;
		NMI_09_EN <= 1'b0;
		IRQ_09_EN <= 1'b0;
		CMD_RST <= 1'b0;
		WAIT_HALT <= 1'b0;
		ADDR_RESET_N <= 1'b0;
		IRQ_RESET <= 1'b0;
		IRQ_02_BUF0_N <= 1'b1;
		IRQ_02_BUF1_N <= 1'b1;
		ADDR_100_BUF0 <= 1'b0;
		ADDR_100_BUF1 <= 1'b0;
		HALT_SIG_BUF0 <= 1'b0;
		HALT_SIG_BUF1 <= 1'b0;
		I2C_DONE_BUF <= 2'b00;
	end
	else
	begin
		IRQ_02_BUF0_N <= IRQ_02_N;				// Double buffer 6502 IRQ generated by command write
		IRQ_02_BUF1_N <= IRQ_02_BUF0_N;
		ADDR_100_BUF0 <= BUFF_ADD[8];			// Double buffer Buffer Address Flag
		ADDR_100_BUF1 <= ADDR_100_BUF0;
		HALT_SIG_BUF0 <= HALT_BUF2;			// Double buffer the halt signal
		HALT_SIG_BUF1 <= HALT_SIG_BUF0;
		I2C_DONE_BUF[1] <= I2C_DONE_BUF[0];
		I2C_DONE_BUF[0] <= I2C_DONE;
		case ({RW_02_N, ADDRESS_02[15:8],ADDRESS_02[4:0]})
		14'b01111010001011:						// F4+0B Track Register
			TRACK_REG_R <= DATA_OUT_02;
		14'b01111010001101:						// F4+0D Extended Track Register
			TRACK_EXT_R <= DATA_OUT_02;
		14'b01111010010000:						// F4+10 Status to 6809
			STATUS <= DATA_OUT_02;
		14'b01111010010001:						// F4+11 Control
		begin
			IMM_HALT_09 <= DATA_OUT_02[7];
			HALT_100_09 <= DATA_OUT_02[6];
			NMI_09_EN <= DATA_OUT_02[5];
			IRQ_09_EN <= DATA_OUT_02[4];
			ADDR_RESET_N <= DATA_OUT_02[3];
			WAIT_HALT <= DATA_OUT_02[2];
			CMD_RST <= DATA_OUT_02[1];
			IRQ_RESET <= DATA_OUT_02[0];
		end
		14'b01111010010110:						// F4+16 Trace
			TRACE <= DATA_OUT_02;
		14'b01111010011011:						// F4+1B I2C Device
			I2C_DEVICE <= DATA_OUT_02;
		14'b01111010011100:						// F4+1C I2C Register
			I2C_REG <= DATA_OUT_02;
		14'b01111010011101:						// F4+1D I2C Data Out
			I2C_DATA_OUT <= DATA_OUT_02;
		14'b01111010011110:						// F4+1E I2C Start
			I2C_START <= DATA_OUT_02[5];
		endcase
	end
end

//Capture TRACK1, TRACK2, and HEADS from Sector 0 data Since the 6502 cannot read from the sector buffer
//Used to detect if the NitrOS-9 Disk is double sided
always @(negedge PH2_02 or negedge RESET_N)
begin
	if(!RESET_N)
	begin
		TRACK1 <= 8'h00;
		TRACK2 <= 8'h00;
		HEADS <= 8'h00;
	end
	else
	begin
		case ({RW_02_N,ADDRESS_02})
		17'h0F603:
			TRACK1 <= DATA_OUT_02;
		17'h0F610:
			HEADS <= DATA_OUT_02;
		17'h0F612:
			TRACK2 <= DATA_OUT_02;
		endcase
	end
end

assign HALT_CODE =			(COMMAND==8'h00)			?	1'b0:		// No halt system if command is NULL
									(COMMAND[7:4]==4'h9)		?	1'b0:		// No halt system if command is read sector NOH
									(COMMAND[7:4]==4'hb)		?	1'b0:		// No halt system if command is write sector NOH
					({WAIT_HALT, COMMAND[7:4]}==5'h00)	?	1'b0:		// No halt system if command is Restore
					({WAIT_HALT, COMMAND[7:4]}==5'h01)	?	1'b0:		// No halt system if command is Seek
					({WAIT_HALT, COMMAND[7:4]}==5'h02)	?	1'b0:		// No halt system if command is Step
					({WAIT_HALT, COMMAND[7:4]}==5'h03)	?	1'b0:		// No halt system if command is Step
					({WAIT_HALT, COMMAND[7:4]}==5'h04)	?	1'b0:		// No halt system if command is Step In
					({WAIT_HALT, COMMAND[7:4]}==5'h05)	?	1'b0:		// No halt system if command is Step In
					({WAIT_HALT, COMMAND[7:4]}==5'h06)	?	1'b0:		// No halt system if command is Step Out
					({WAIT_HALT, COMMAND[7:4]}==5'h07)	?	1'b0:		// No halt system if command is Step Out
					({WAIT_HALT, COMMAND[7:4]}==5'h0C)	?	1'b0:		// No halt system if command is Read Address
					({WAIT_HALT, COMMAND[7:4]}==5'h0D)	?	1'b0:		// No halt system if command is Force Interrupt
																		1'b1;

// Delay for some short amount of time to allow BOOT_1773 to write the side select
always @(negedge PH_2 or negedge RESET_N)
begin
	if(!RESET_N)
	begin
		HALT_STATE <= 7'h00;
	end
	else
	begin
		if(HALT_CODE)
		begin
			if(!HALT_BUF2)
				HALT_STATE <= HALT_STATE + 1'b1;
		end
		else
			HALT_STATE <= 7'h00;
	end
end

assign HALT =	  IMM_HALT_09												// Immediate halt
					| WRFIFO_WRFULL											// Test for FIFO FULL error
					| HALT_STATE[6]											// Halt if A HALT command code is issued
					| (HALT_100_09 & BUFF_ADD[8]) 						// Halt after reading / writing buffer
					| SPI_HALT;													// Activity on the SPI bus

assign NMI_09	=	DENSITY & FORCE_NMI_09_BUF1;				// Send NMI if Double Density (Halt Mode)

assign	IRQ_09 = 	(DENSITY &	IRQ_09_BUF2)						// Send IRQ if Double Density (No Halt)
				|	(!RDFIFO_RDEMPTY & BI_IRQ_EN);

always @(negedge PH_2 or negedge RESET_N)
begin
	if(!RESET_N)
	begin
		BUFF_ADD <= 9'h000;
		ADDR_RST_BUFF0_N <= 1'b0;
		ADDR_RST_BUFF1_N <= 1'b0;
		FORCE_NMI_09_BUF0 <= 1'b0;
		FORCE_NMI_09_BUF1 <= 1'b0;
		HALT_BUF0 <= 1'b0;
		HALT_BUF1 <= 1'b0;
		HALT_BUF2 <= 1'b0;
	end
	else
	begin
		ADDR_RST_BUFF0_N <= ADDR_RESET_N;						// double buffer Buffer Address Reset
		ADDR_RST_BUFF1_N <= ADDR_RST_BUFF0_N;
		FORCE_NMI_09_BUF0 <= NMI_09_EN;							// Double buffer NMI
		FORCE_NMI_09_BUF1 <= FORCE_NMI_09_BUF0;
		HALT_BUF0 <= HALT | !act_led_n;										// Double buffer Halt, Haltis generated by FDD and SD Card
		HALT_BUF1 <= HALT_BUF0;
		HALT_BUF2 <= HALT_BUF1;
		if(!ADDR_RST_BUFF1_N)
		begin
			BUFF_ADD <= 9'h000;
		end
		else
		begin
			if({HDD_EN, ADDRESS[3:0]}== 5'h1b)
			begin
				BUFF_ADD <= BUFF_ADD + 1'b1;
			end
		end
	end
end
// 2K 6502 firmware block
disk02	disk02_inst (
	.address (ADDRESS_02[10:0]),
	.clock (PH2_02),
	.data (DATA_OUT_02),
	.wren (!RW_02_N & RAM02_00_EN),
	.q (DATAO_02_HDD)
	);
// 512 byte 6502 to 6809 buffer (Read Sector)
buffer_dp	buffer_dp_read (
	.data (DATA_OUT_02),
	.wraddress (ADDRESS_02[8:0]),
	.wren (!RW_02_N & DISKBUF_02),
	.wrclock (PH2_02),
	.rdaddress (BUFF_ADD[8:0]),
	.rdclock (PH_2),
	.q (DATAO_09_HDD)
	);
// 512 byte 6809 to 6502 buffer (Write Sector)
buffer_dp	buffer_dp_write (
	.data (DATA_OUT),
	.wraddress (BUFF_ADD[8:0]),
	.wren (!RW_N & HDD_EN_DATA),
	.wrclock (PH_2),
	.rdaddress (ADDRESS_02[8:0]),
	.rdclock (PH2_02),
	.q (DISK_BUF_Q)
	);

FIFO_READ	FIFO_READ_inst (
	.aclr ( !RESET_N ),
	.data ( DATA_OUT_02 ),
	.rdclk ( PH_2 ),
	.rdreq ( RDFIFO_RDREQ ),
	.wrclk ( PH2_02 ),
	.wrreq ( RDFIFO_WRREQ ),
	.q ( RDFIFO_DATA ),
	.rdempty ( RDFIFO_RDEMPTY ),
	.wrfull ( RDFIFO_WRFULL )
	);

FIFO_WRITE	FIFO_WRITE_inst (
	.aclr ( !RESET_N ),
	.data ( DATA_OUT ),
	.rdclk ( PH2_02 ),
	.rdreq ( WRFIFO_RDREQ ),
	.wrclk ( PH_2 ),
	.wrreq ( WRFIFO_WRREQ ),
	.q ( WRFIFO_DATA ),
	.rdempty ( WRFIFO_RDEMPTY ),
	.wrfull ( WRFIFO_WRFULL )
	);

glb6850 COM1(
.RESET_N(RESET_N),
.RX_CLK(UART1_CLK),
.TX_CLK(UART1_CLK),
.E(PH2_02),
.DI(DATA_OUT_02),
.DO(DATA_COM1),
.CS(COM1_EN),
.RW_N(RW_02_N),
.IRQ(IRQ_02_UART),
.RS(ADDRESS_02[0]),
.TXDATA(UART50_TXD),
.RXDATA(UART50_RXD),
.RTS(UART50_RTS),
.CTS(UART50_RTS),
.DCD(UART50_RTS)
);

/******************************************************************************
*
* State machine to accept writes to each of the registers
*
* Memory Map
*
* CS+0
* Halt Enable, Side, Density, Write Precompensation, Motor, Drive Select (3 bits)
*
* CS+4
* Drive Select Extended(8 bits)
*
* CS+8
* Command (8 bit command)
*
* CS+9
* Track (Lower 8 bits of track address)
*
* CS+10
* Sector (8 bit sector address)
*
* CS+11
* Data Register from RAM Block
*
* CS+13
* Track (Upper 8 bits of track address)
*
******************************************************************************/
always @(negedge PH_2 or negedge RESET_N)
begin
	if(!RESET_N)
	begin
		DRIVE_SEL_EXT <= 8'h00;
		MOTOR <= 1'b0;
		WRT_PREC <= 1'b0;
		DENSITY <= 1'b0;
		HALT_EN <= 1'b0;
		COMMAND <= 8'h00;
		TRACK_REG_W <= 8'h00;
		TRACK_EXT_W <= 8'h00;
		SECTOR <= 8'h00;
		DATA_REG <= 8'h00;
		DATA_EXT <= 8'h00;
		BUSY0 <= 1'b0;
		BUSY1 <= 1'b0;
		IRQ_02_N <= 1'b1;
		IRQ_09_BUF0 <= 1'b0;
		IRQ_09_BUF1 <= 1'b0;
		IRQ_09_BUF2 <= 1'b0;
		CMD_RST_BUF0 <= 1'b0;
		CMD_RST_BUF1 <= 1'b0;
	end
	else
	begin
// Double buffer BUSY to reset IRQ out
		BUSY0 <= IRQ_RESET;
		BUSY1 <= BUSY0;
		CMD_RST_BUF0 <= CMD_RST;
		CMD_RST_BUF1 <= CMD_RST_BUF0;
		case ({RW_N, HDD_EN, ADDRESS[3:0]})
		6'b010000:
		begin
			DRIVE_SEL_EXT <= {4'b0000,
									DATA_OUT[6],		// Drive Select [3] / Side Select
									DATA_OUT[2:0]};	// Drive Select [2:0]
			MOTOR <= DATA_OUT[3];					// Turn on motor, not used here just checked, 0=MotorOff 1=MotorOn
			WRT_PREC <= DATA_OUT[4];				// Write Precompensation, not used here
			DENSITY <= DATA_OUT[5];					// Density, not used here just checked
			HALT_EN <= DATA_OUT[7];					// Normal Halt enable, 0=Disabled 1=Enabled
		end
		6'b010001:
		begin
			IRQ_09_BUF2 <= 1'b0;
			BI_IRQ_EN <= DATA_OUT[0];
		end
		6'b010100:										// Extended Drive Select
		begin
			DRIVE_SEL_EXT <= DATA_OUT;				// Extended Drive Select
		end
		6'b011000:
		begin
			COMMAND <= DATA_OUT;						// Command
			IRQ_02_N <= BUSY1;							// Signal 6502 unless it is busy
		end
		6'b011001:
		begin
			TRACK_REG_W <= DATA_OUT;				// Lower 8 bits of track
			TRACK_EXT_W <= 8'h00;
		end
		6'b011010:
			SECTOR <= DATA_OUT;					// 0-17 (0-255 Capable)
		6'b011011:
		begin
			DATA_REG <= DATA_OUT;					// Data
			DATA_EXT <= 8'h00;						// Clear extended Data on DATA write
		end
		6'b011101:
			TRACK_EXT_W <= DATA_OUT;				// Extended Track
		6'b011111:
			DATA_EXT <= DATA_OUT;					// Extended Data
		default:
		begin
			IRQ_09_BUF0 <= IRQ_09_EN;
			IRQ_09_BUF1 <= IRQ_09_BUF0;
			if(IRQ_09_BUF1)							// Set but not clear
			begin
				IRQ_09_BUF2 <= 1'b1;
			end
			if(CMD_RST_BUF1)
			begin
				COMMAND <= 8'h00;
			end
			if(FORCE_NMI_09_BUF1)							// If IRQ (end of command) clear Halt enables
			begin
				HALT_EN <= 1'b0;
			end
			if(BUSY1)									// If BUSY is set then IRQ must have worked
			begin
				IRQ_02_N <= 1'b1;
			end
		end
		endcase
	end
end

assign	DATA_HDD =		({HDD_EN, ADDRESS[3:0]} == 5'h10)	?	{HALT_EN, 
																DRIVE_SEL_EXT[3],
																DENSITY, 
																WRT_PREC, 
																MOTOR, 
																DRIVE_SEL_EXT[2:0]}:
						({HDD_EN, ADDRESS[3:0]} == 5'h11)	?	{IRQ_09,
																ADDR_RST_BUFF1_N,
																4'h0,
																!RDFIFO_RDEMPTY,
																BI_IRQ_EN}:
						({HDD_EN, ADDRESS[3:0]} == 5'h12)	?	RDFIFO_DATA:
						({HDD_EN, ADDRESS[3:0]} == 5'h13)	?	8'h03:
						({HDD_EN, ADDRESS[3:0]} == 5'h14)	?	DRIVE_SEL_EXT:
						({HDD_EN, ADDRESS[3:0]} == 5'h18)	?	{STATUS[7:1], IRQ_RESET}:
						({HDD_EN, ADDRESS[3:0]} == 5'h19)	?	TRACK_REG_R:
						({HDD_EN, ADDRESS[3:0]} == 5'h1A)	?	SECTOR:
		({ADDR_RST_BUFF1_N, HDD_EN, ADDRESS[3:0]} == 6'h1B)	?	DATA_REG:								// Dual port issue for Altera
		({ADDR_RST_BUFF1_N, HDD_EN, ADDRESS[3:0]} == 6'h3B)	?	DATAO_09_HDD:
						({HDD_EN, ADDRESS[3:0]} == 5'h1D)	?	TRACK_EXT_R:
						({HDD_EN, ADDRESS[3:0]} == 5'h1F)	?	DATA_EXT:
																8'h00;

assign	HDD_EN_DATA =	({HDD_EN, ADDRESS[3:0]} == 5'h1B)		?	1'b1:	//FF4B      with MPI switch = 4
																	1'b0;
assign RDFIFO_RDREQ =	({HDD_EN, ADDRESS[3:0], RW_N, RDFIFO_RDEMPTY} == 7'b1001010)	?	1'b1:
																							1'b0;
assign WRFIFO_WRREQ =	({HDD_EN, ADDRESS[3:0], RW_N, WRFIFO_WRFULL}  == 7'b1001000)	?	1'b1:
																							1'b0;
I2C GLB_I2C(
.CLOCK(MCLOCK[6]),
.RESET_N(RESET_N),
.I2C_CLK(I2C_SCL),
.I2C_CLK_EN(I2C_SCL_EN),
.I2C_DAT(I2C_DAT),
.I2C_DAT_EN(I2C_DAT_EN),
.DEVICE(I2C_DEVICE[7:1]),
.REGISTER(I2C_REG),
.DATA_IN(I2C_DATA_IN),
.DATA_OUT(I2C_DATA_OUT),
.DONE(I2C_DONE),
.FAIL(I2C_FAIL),
.RW_N(I2C_DEVICE[0]),
.START(I2C_START)
);

assign I2C_SCL = (I2C_SCL_EN == 1'b0) ?	1'b0:
														1'b1;

assign I2C_DAT = (I2C_DAT_EN == 1'b0) ?	1'b0:
														1'bZ;

/*****************************************************************************
* Hardware based clock
******************************************************************************/
//	year-1900 (0-255)
// month (1-12)
// day (1-31)
// hour (0-23)
// minute (0-59)
// second (0-59)
// DofW (0-6) 0=sunday

always @(negedge PH2_02)
begin
	case({RW_02_N, ADDRESS_02})
		17'h0F407:
		begin
			CENT <= DATA_OUT_02[4:0];
		end
		17'h0F406:
		begin
			YEAR <= DATA_OUT_02[6:0];
		end
		17'h0F405:
		begin
			MNTH <= DATA_OUT_02[3:0];
		end
		17'h0F404:
		begin
			DMTH <= DATA_OUT_02[4:0];
		end
		17'h0F403:
		begin
			HOUR <= DATA_OUT_02[4:0];
		end
		17'h0F402:
		begin
			MIN <= DATA_OUT_02[5:0];
		end
		17'h0F401:
		begin
			SEC <= DATA_OUT_02[5:0];
		end
		17'h0F400:
		begin
			DWK <= DATA_OUT_02[2:0];
		end
		default
		begin
			TICK0 <= V_SYNC;
			TICK1 <= TICK0;
			TICK2 <= TICK1;
			if(TICK2 & ~TICK1)
			begin
// 1/60 timer
				if(CLICK == 6'd60)					// Make it run a bit slow
				begin
					CLICK <= 6'd0;
					if(SEC == 6'd59)
					begin
						SEC <= 6'd0;
					end
					else
						SEC <= SEC + 1'b1;
				end
				else
					CLICK <= CLICK + 1'b1;
			end
		end
		endcase
end
