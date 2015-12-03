////////////////////////////////////////////////////////////////////////////////
// Project Name:	CoCo3FPGA Version 3.0
// File Name:		coco3fpga.v
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

// RS232 PAK Hardware included
`define RS232PAK
// New vs Old SRAM
//`define NEW_SRAM

// SPI Bus Analyzer
//`define BUSA

// Only one of the next three
// Floppy Debug
// `define FLPY_DEBUG

// SD Card Degug 7 Segment LEDs and Green LEDs
//`define SD_DEBUG

// No Debug
`define NO_DEBUG

module coco3fpga_dw(
// Input Clocks
CLK50MHZ,
CLK24MHZ,
CLK24MHZ_2,
CLK27MHZ,
CLK27MHZ_2,
CLK3_57MHZ,
// RAM and ROM
RAM0_DATA,				// 16 bit data bus to RAM 0
RAM0_ADDRESS,
RAM0_RW_N,
RAM0_CS_N,				// Chip Select for RAM 0
RAM0_BE0_N,				// Byte Enable for RAM 0
RAM0_BE1_N,				// Byte Enable for RAM 0
RAM0_OE_N,
RAM1_ADDRESS,
RAM1_DATA,
RAM1_BE0_N,
RAM1_BE1_N,
RAM1_CS_N,
RAM1_RW_N,
FLASH_ADDRESS,
FLASH_DATA,
FLASH_WE_N,
FLASH_RESET_N,
FLASH_CE_N,
FLASH_OE_N,
// VGA
RED3,
GREEN3,
BLUE3,
RED2,
GREEN2,
BLUE2,
RED1,
GREEN1,
BLUE1,
RED0,
GREEN0,
BLUE0,
H_SYNC,
V_SYNC,
// PS/2
ps2_clk,
ps2_data,
ms_clk,
ms_data,
//Serial Ports
DE1TXD,
DE1RXD,
OPTTXD,
OPTRXD,
// I2C
I2C_SCL,
I2C_DAT,
//Codec
AUD_XCK,
AUD_BCLK,
AUD_DACDAT,
AUD_DACLRCK,
AUD_ADCDAT,
AUD_ADCLRCK,
// 7 Segment Display
SEGMENT0_N,
SEGMENT1_N,
SEGMENT2_N,
SEGMENT3_N,
// LEDs
LEDG,
LEDR,
// CoCo Joystick
PADDLE_MCLK,
PADDLE_CLK,
P_SWITCH,
//SPI for SD Card
MOSI,
MISO,
SPI_CLK,
SPI_SS_N,
// Debug Test Points
TEST_1,
TEST_2,
TEST_3,
TEST_4,
// Buttons and Switches
SWITCH,
BUTTON_N
);

input				CLK50MHZ;
input				CLK24MHZ;
input				CLK24MHZ_2;
input				CLK27MHZ;
input				CLK27MHZ_2;
output			CLK3_57MHZ;
// DE1 RAM Common
output [17:0]	RAM0_ADDRESS;
reg	 [17:0]	RAM0_ADDRESS;
output			RAM0_RW_N;
reg				RAM0_RW_N;
// DE1 RAM bank 0
inout		[15:0]	RAM0_DATA;
reg		[15:0]	RAM0_DATA;
output				RAM0_CS_N;
wire					RAM0_CS;						// DATA_IN Mux select
output				RAM0_BE0_N;
reg					RAM0_BE0_N;
output				RAM0_BE1_N;
reg					RAM0_BE1_N;
output				RAM0_OE_N;
wire					RAM0_BE0;
wire					RAM0_BE1;

// Analog Board RAM Common
output [17:0]	RAM1_ADDRESS;
output			RAM1_RW_N;
// Ananlog SRAM bank 1
inout  [15:0]	RAM1_DATA;
output			RAM1_BE0_N;
output			RAM1_BE1_N;
output			RAM1_CS_N;

//Flash ROM
output	[21:0]	FLASH_ADDRESS;
input		[7:0]		FLASH_DATA;
output				FLASH_WE_N;
output				FLASH_RESET_N;
output				FLASH_CE_N;
output				FLASH_OE_N;

// VGA
output				RED3;
reg					RED3;
output				GREEN3;
reg					GREEN3;
output				BLUE3;
reg					BLUE3;
output				RED2;
reg					RED2;
output				GREEN2;
reg					GREEN2;
output				BLUE2;
reg					BLUE2;
output				RED1;
reg					RED1;
output				GREEN1;
reg					GREEN1;
output				BLUE1;
reg					BLUE1;
output				RED0;
reg					RED0;
output				GREEN0;
reg					GREEN0;
output				BLUE0;
reg					BLUE0;
output				H_SYNC;
output				V_SYNC;
wire					HBLANK;
wire					VBLANK;
			
// PS/2
input 				ps2_clk;
input					ps2_data;
input 				ms_clk;
input					ms_data;

// Serial Ports
output				DE1TXD;
input					DE1RXD;
output				OPTTXD;
input					OPTRXD;
// I2C
output				I2C_SCL;
inout					I2C_DAT;
//Codec
output				AUD_XCK;
input					AUD_BCLK;
output				AUD_DACDAT;
reg					AUD_DACDAT;
input					AUD_DACLRCK;
input					AUD_ADCDAT;
input					AUD_ADCLRCK;
// Display

output	[6:0]		SEGMENT0_N;
output	[6:0]		SEGMENT1_N;
output	[6:0]		SEGMENT2_N;
output	[6:0]		SEGMENT3_N;
`ifdef SD_DEBUG
reg 		[6:0]		SEGMENT0_N;
reg 		[6:0]		SEGMENT1_N;
reg 		[6:0]		SEGMENT2_N;
reg 		[6:0]		SEGMENT3_N;
wire		[6:0]		SEGMENT_N;
reg		[3:0]		DIGIT_N;
`endif

// LEDs
output	[7:0]		LEDG;
output	[9:0]		LEDR;

// CoCo Perpherial
output				PADDLE_MCLK;
input		[3:0]		PADDLE_CLK;
input		[3:0]		P_SWITCH;
//SPI
output				MOSI;
input					MISO;
output				SPI_CLK;
output				SPI_SS_N;
// Extra Buttons and Switches
input		[9:0]		SWITCH;			//  9 UART / DriveWire
											//		Off - DE1 Port is DriveWire and Analog Board is RS232 PAK
											//		on  - DE1 Port is RS232 PAK and Analog Board is DriveWire
											//  9 SG4 / SG6 ????????
											//  8 Serial Port Speed[1]
											//  7 Serial Port Speed[0]
											//    [1] [0]
											//		OFF OFF - 115200	// Swap UART / DriveWire
											//		OFF ON  - 230400
											//		ON  OFF - 460800	// Fastest for the DE1 Port
											//		ON  ON  - 921600
											//  6 SD Card Presence / Write Protect
											//		Off - Use card signals
											//		On  - Ignore Signals
											//  5 SG4 / SG6 mode select
											//		Off - SG4
											//		On  - SG6
											//  4 Cartridge Interrupt disabled except Disk
											//  3 Video Odd line black
											//		Off - Normal video
											//		On  - Odd lines black
											//  2 MPI [1]
											//  1 MPI [0]
											//    [1] [0]
											//		OFF OFF - Slot 1
											//		OFF ON  - Slot 2
											//		ON  OFF - Slot 3
											//		ON  ON  - Slot 4
											//  0 CPU Turbo Speed
											//		Off - Normal 1.78 MHz
											//		On  - 25 MHz or 8.33 MHz (Old or New SRAM)

input [3:0]			BUTTON_N;		//  3 RESET
											//  2 SD Card Inserted (0=Inserted) wired to switche on the SD card
											//  1 SD Write Protect (1=Protected) wired to switche on the SD card
											//  0 Easter Egg

output				TEST_1;			// Debug Test Points
output				TEST_2;
output				TEST_3;
output				TEST_4;
wire					PH_2;
reg 					PH_2_RAW;
reg					RESET_N;
reg		[13:0]	RESET_SM;
reg					CPU_RESET;
wire					RESET;
wire					RESET_P;
wire		[15:0]	ADDRESS;
wire		[5:0]		BLOCK_ADDRESS;
wire					RW_N;
wire		[7:0]		DATA_IN;
wire		[7:0]		DATA_OUT;
wire					VMA;
reg		[5:0]		CLK;

// Gime Regs
reg		[1:0]		ROM;
reg					RAM;
reg					ST_SCS;
reg					VEC_PAG_RAM;
reg					GIME_FIRQ;
reg					GIME_IRQ;
reg					MMU_EN;
reg					COCO1;
reg		[2:0]		V;
reg		[6:0]		VERT;
reg					RATE;
reg					TIMER_INS;
reg					MMU_TR;
reg					IRQ_TMR;
reg					IRQ_HBORD;
reg					IRQ_VBORD;
reg					IRQ_KEY;
reg					IRQ_CART;
reg					FIRQ_TMR;
reg					FIRQ_HBORD;
reg					FIRQ_VBORD;
reg					FIRQ_KEY;
reg					FIRQ_CART;
reg		[3:0]		TMR_MSB;
reg		[7:0]		TMR_LSB;
reg					TMR_ENABLE;
reg					TIMER_N;
reg		[1:0]		TIMER_STATE;
wire					TIMER_R;
reg		[15:0]	VIDEO_BUFFER;
reg					GRMODE;
reg					DESCEN;
reg					BLINK;
reg					MONO;
reg					HLPR;
reg		[2:0]		LPR;
reg		[1:0]		LPF;
reg		[3:0]		HRES;
reg		[1:0]		CRES;
reg		[3:0]		VERT_FIN_SCRL;
reg		[7:0]		SCRN_START_MSB;
reg		[7:0]		SCRN_START_LSB;
reg		[6:0]		HOR_OFFSET;
reg					HVEN;
reg		[11:0]	PALETTE [16:0];
wire		[8:0]		COLOR;
reg					HSYNC_INT;
reg					HSYNC_POL;
reg		[1:0]		SEL;
reg		[7:0]		KEY_COLUMN;
reg					VSYNC_INT;
reg					VSYNC_POL;
reg		[3:0]		VDG_CONTROL;
reg					CSS;
reg					CART_INT;
reg					CART_POL;
reg					CD_INT;
reg					CD_POL;
reg					CAS_MTR;
reg					SOUND_EN;
wire		[17:0]	VIDEO_ADDRESS;
wire					ROM_RW;
wire					FLASH_CE_S;

wire					ENA_DSK;
wire					ENA_ORCC;
wire					ENA_LOAD;
wire					ENA_PAK;

wire					HDD_EN;
wire					HDD_EN_DATA;

reg		[1:0]		MPI_SCS;				// IO select
reg		[1:0]		MPI_CTS;				// ROM select
reg		[1:0]		W_PROT;
reg					SBS;
reg		[5:0]		SAM00;
reg		[5:0]		SAM01;
reg		[5:0]		SAM02;
reg		[5:0]		SAM03;
reg		[5:0]		SAM04;
reg		[5:0]		SAM05;
reg		[5:0]		SAM06;
reg		[5:0]		SAM07;
reg		[5:0]		SAM10;
reg		[5:0]		SAM11;
reg		[5:0]		SAM12;
reg		[5:0]		SAM13;
reg		[5:0]		SAM14;
reg		[5:0]		SAM15;
reg		[5:0]		SAM16;
reg		[5:0]		SAM17;
wire		[55:0]	KEY;
wire					SHIFT_OVERRIDE;
wire					SHIFT;
wire		[7:0]		KEYBOARD_IN;
reg					DDR1;
reg					DDR2;
reg					DDR3;
reg					DDR4;
wire		[7:0]		DATA_REG1;
wire		[7:0]		DATA_REG2;
wire		[7:0]		DATA_REG3;
wire		[7:0]		DATA_REG4;
reg		[7:0]		DD_REG1;
reg		[7:0]		DD_REG2;
reg		[7:0]		DD_REG3;
reg		[7:0]		DD_REG4;
wire					ROM_SEL;
reg		[5:0]		DTOA_CODE;
reg		[5:0]		SOUND_DTOA;
wire		[7:0]		SOUND;
wire		[18:0]	DAC_LEFT;
wire		[18:0]	DAC_RIGHT;
wire		[7:0]		VU;
wire		[7:0]		VUM;
reg		[18:0]	LEFT;
reg		[18:0]	RIGHT;
reg		[7:0]		ORCH_LEFT;
reg		[7:0]		ORCH_RIGHT;
reg		[7:0]		ORCH_LEFT_EXT;
reg		[7:0]		ORCH_RIGHT_EXT;
reg					DACLRCLK;
reg					ADCLRCLK;
reg		[5:0]		DAC_STATE;
wire 					H_FLAG;

reg		[1:0]		SWITCH_L;
wire					KEY_INT_RAW;
reg					HS_INT;
reg					H_SYNC_IRQ_N;
reg		[1:0]		HS_INT_SM;
reg					VS_INT;
reg					V_SYNC_IRQ_N;
reg		[1:0]		VS_INT_SM;
reg					CART1_INT;
reg		[1:0]		CART1_INT_SM;

reg					TMR_INT;
reg		[1:0]		TMR_INT_SM;
reg					HBORD_INT;
reg		[1:0]		HBORD_INT_SM;
reg					VBORD_INT;
reg		[1:0]		VBORD_INT_SM;
reg					KEY_INT;
reg		[1:0]		KEY_INT_SM;
reg					CAR_INT;
reg		[1:0]		CAR_INT_SM;

reg					TMR_FINT;
reg		[1:0]		TMR_FINT_SM;
reg					HBORD_FINT;
reg		[1:0]		HBORD_FINT_SM;
reg					VBORD_FINT;
reg		[1:0]		VBORD_FINT_SM;
reg					KEY_FINT;
reg		[1:0]		KEY_FINT_SM;
reg					CAR_FINT;
reg		[1:0]		CAR_FINT_SM;

wire					CPU_IRQ;
wire					CPU_FIRQ;
reg		[2:0]		DIV_7;
reg					DIV_14;
reg		[12:0]	TIMER;
wire					TMR_CLK;
wire					SER_IRQ;
reg					CART_IRQ;
reg		[4:0]		COM1_STATE;
reg		[12:0]	COM2_STATE;
reg					COM1_CLOCK_X;
reg					COM1_CLOCK;
reg		[2:0]		COM1_CLK;
wire		[7:0]		DATA_HDD;
wire					RS232_EN;
wire					RX_CLK2;
wire		[7:0]		DATA_RS232;
reg		[2:0]		ROM_BANK;
reg		[1:0]		BANK_SIZE;
reg		[6:0]		BANK0;
reg		[6:0]		BANK1;
reg		[6:0]		BANK2;
reg		[6:0]		BANK3;
reg		[6:0]		BANK4;
reg		[6:0]		BANK5;
reg		[6:0]		BANK6;
reg		[6:0]		BANK7;
wire					SLOT3_HW;
wire					UART51_TXD;
wire					UART51_RXD;
wire					UART51_RTS;
wire					UART51_DTR;
wire					UART50_TXD;
wire					UART50_RXD;
wire					UART50_RTS;
// Clock
reg		[4:0]		CENT;
reg		[6:0]		YEAR;
reg		[3:0]		MNTH;
reg		[4:0]		DMTH;
reg		[2:0]		DWK;
reg		[4:0]		HOUR;
reg		[5:0]		MIN;
reg		[5:0]		SEC;
reg		[5:0]		CLICK;
reg					TICK0;
reg					TICK1;
reg					TICK2;
// Joystick
reg		[12:0]	JOY_CLK;
reg		[9:0]		PADDLE_ZERO_0;
reg		[9:0]		PADDLE_ZERO_1;
reg		[9:0]		PADDLE_ZERO_2;
reg		[9:0]		PADDLE_ZERO_3;
reg		[11:0]	PADDLE_VAL_0;
reg		[11:0]	PADDLE_VAL_1;
reg		[11:0]	PADDLE_VAL_2;
reg		[11:0]	PADDLE_VAL_3;
reg		[11:0]	PADDLE_LATCH_0;
reg		[11:0]	PADDLE_LATCH_1;
reg		[11:0]	PADDLE_LATCH_2;
reg		[11:0]	PADDLE_LATCH_3;
reg		[1:0]		PADDLE_STATE_0;
reg		[1:0]		PADDLE_STATE_1;
reg		[1:0]		PADDLE_STATE_2;
reg		[1:0]		PADDLE_STATE_3;
reg		[5:0]		JOY1_COUNT;
reg		[5:0]		JOY2_COUNT;
reg		[5:0]		JOY3_COUNT;
reg		[5:0]		JOY4_COUNT;
reg					JOY_TRIGGER;
wire					JSTICK;
wire					JOY1;
wire					JOY2;
wire					JOY3;
wire					JOY4;
reg					MOTOR;
reg					WRT_PREC;
reg					DENSITY;
reg					HALT_EN;
reg		[7:0]		COMMAND;
reg		[7:0]		SECTOR;
reg		[7:0]		DATA_EXT;
reg		[7:0]		STATUS;
reg					IRQ_02_N;
reg					IRQ_02_BUF0_N;
reg					IRQ_02_BUF1_N;
wire					IRQ_02_UART;
wire					NMI_09;
reg					HALT_BUF0;
reg					HALT_BUF1;
reg					HALT_BUF2;
reg					HALT_SIG_BUF0;
reg					HALT_SIG_BUF1;
reg		[6:0]		HALT_STATE;
wire					PH2_02;
wire		[15:0]	ADDRESS_02;
wire		[7:0]		CPU_BANK;
wire		[7:0]		DATA_OUT_02;
wire		[7:0]		DATA_IN_02;
wire		[7:0]		DATA_COM1;
reg		[8:0]		BUFF_ADD;
reg					ADDR_RESET_N;
reg					IMM_HALT_09;
wire					COM1_EN;
reg		[7:0]		TRACK_REG_R;
reg		[7:0]		TRACK_REG_W;
reg		[7:0]		TRACK_EXT_R;
reg		[7:0]		TRACK_EXT_W;
reg					NMI_09_EN;
wire					IRQ_09;
reg					IRQ_RESET;
reg					BUSY0;
reg					BUSY1;
reg		[7:0]		DRIVE_SEL_EXT;
wire		[3:0]		HEXX;
wire					HALT;
reg					FORCE_NMI_09_BUF0;
reg					FORCE_NMI_09_BUF1;
reg					ADDR_RST_BUFF0_N;
reg					ADDR_RST_BUFF1_N;
reg		[7:0]		TRACE;
reg					HALT_100_09;
reg					IRQ_09_EN;
reg					ADDR_100_BUF0;
reg					ADDR_100_BUF1;
reg					IRQ_09_BUF0;
reg					IRQ_09_BUF1;
reg					IRQ_09_BUF2;
reg					CMD_RST;
reg					WAIT_HALT;
reg					CMD_RST_BUF0;
reg					CMD_RST_BUF1;
wire					CPU_RESET_N;
wire					RW_02_N;
wire					DISKBUF_02;
wire		[7:0]		DISK_BUF_Q;
reg		[7:0]		DATA_REG;
wire					HALT_CODE;
wire					RAM02_00_EN;
wire		[7:0]		DATAO_02_HDD;
wire		[7:0]		DATAO_09_HDD;
wire					FFF0_EN;
wire		[7:0]		DATAO_FFF0;
reg		[7:0]		TRACK1;
reg		[7:0]		TRACK2;
reg		[7:0]		HEADS;
wire					RDFIFO_RDREQ;
wire					RDFIFO_WRREQ;
wire					WRFIFO_RDREQ;
wire					WRFIFO_WRREQ;
wire		[7:0]		RDFIFO_DATA;
wire		[7:0]		WRFIFO_DATA;
wire					RDFIFO_RDEMPTY;
wire					RDFIFO_WRFULL;
wire					WRFIFO_RDEMPTY;
wire					WRFIFO_WRFULL;
reg					BI_IRQ_EN;
wire					UART1_CLK;
reg 		[11:0]	MCLOCK;
wire					I2C_SCL_EN;
wire					I2C_DAT_EN;
reg		[7:0]		I2C_DEVICE;
reg		[7:0]		I2C_REG;
wire		[7:0]		I2C_DATA_IN;
reg		[7:0]		I2C_DATA_OUT;
wire					I2C_DONE;
reg		[1:0]		I2C_DONE_BUF;
wire					I2C_FAIL;
reg					I2C_START;
wire		[7:0]		SPI_DATA;
wire					SPI_EN;
wire					act_led_n;
wire					IRQ_SPI_N;
`ifdef SD_DEBUG
wire					SPI_TRACE;
reg		[7:0]		SPI_T;
reg		[7:0]		SPI_IN;
reg		[7:0]		SPI_OUT;
`endif
wire					EF;
wire					VDA;
wire					MF;
wire					VPA;
wire					ML_N;
wire					XF;
wire					SYNC;
wire					VP_N;
reg					ODD_LINE;
wire					SPI_HALT;
reg	[18:0]		GART_WRITE;
reg	[18:0]		GART_READ;
reg	[1:0]			GART_INC;

`ifdef FLPY_DEBUG
assign LEDG = TRACE;														// Floppy Trace
`endif

`ifdef SD_DEBUG
assign LEDG = SPI_T;
`endif														// SD Trace

`ifdef NO_DEBUG
//assign LEDG = {COCO1, V, VDG_CONTROL};
assign LEDG[0]= (RAM0_CS & RAM0_BE0);
assign LEDG[1]= (RAM0_CS & RAM0_BE1);
assign LEDG[2]= FLASH_CE_S;
assign LEDG[3]= HDD_EN;
assign LEDG[4]= HALT_BUF2;
assign LEDG[5]= RS232_EN;
assign LEDG[6]= SPI_EN;
assign LEDG[7]= KEY[55];
`endif

assign LEDR[0] = DRIVE_SEL_EXT[0] & MOTOR;
assign LEDR[1] = DRIVE_SEL_EXT[1] & MOTOR;
assign LEDR[2] = DRIVE_SEL_EXT[2] & MOTOR;
assign LEDR[3] = DRIVE_SEL_EXT[3] & MOTOR;
assign LEDR[4] = ~UART50_TXD;
assign LEDR[5] = ~UART50_RXD;
assign LEDR[6] = !RESET_N;
assign LEDR[7] = !BUTTON_N[2] & (BUTTON_N[1] | SWITCH[6]);		// SD Card Write Protected when SD Card Inserted
assign LEDR[8] = !BUTTON_N[2];											// SD Card inserted
assign LEDR[9] = !act_led_n;

`ifdef BUSA
assign TEST_1 = MOSI;
assign TEST_2 = MISO;
assign TEST_3 = SPI_CLK;
assign TEST_4 = SPI_SS_N;
`endif

//Master clock divider chain
//	MCLOCK[0] = 50/2		= 25 MHz
//	MCLOCK[1] = 50/4		= 12.5 MHz
//	MCLOCK[2] = 50/8		= 6.25 MHz
//	MCLOCK[3] = 50/16		= 3.125 MHz
//	MCLOCK[4] = 50/32		= 1.5625 MHz
//	MCLOCK[5] = 50/64		= 781.25 KHz
//	MCLOCK[6] = 50/128		= 390.625 KHz
//	MCLOCK[7] = 50/256		= 195.125 KHz
//	MCLOCK[8] = 50/512		= 97.65625 KHz
//	MCLOCK[9] = 50/1024		= 48.828125 KHz
//	MCLOCK[10] = 50/2048	= 24.4140625 KHz
//	MCLOCK[11] = 50/4096	= 12.20703125 KHz

always @ (negedge CLK50MHZ)				//50 MHz
	MCLOCK <= MCLOCK + 1'b1;

`ifndef SD_DEBUG
assign SEGMENT0_N =	{7'b0100011};					//o
assign SEGMENT1_N =	{7'b1000110};					//C
assign SEGMENT2_N =	{7'b0100011};					//o
assign SEGMENT3_N =	{7'b1000110};					//C
`else
always @ (negedge V_SYNC)					// Anything > 200 HZ
case(DIGIT_N)
  4'b1110:	DIGIT_N <= 4'b1101;
  4'b1101:	DIGIT_N <= 4'b1011;
  4'b1011:	DIGIT_N <= 4'b0111;
  default:  DIGIT_N <= 4'b1110;
 endcase

always @ (negedge V_SYNC)
begin
	case (DIGIT_N)
	4'b1110:
		SEGMENT0_N <= SEGMENT_N;
	4'b1101:
		SEGMENT1_N <= SEGMENT_N;
	4'b1011:
		SEGMENT2_N <= SEGMENT_N;
	default:
		SEGMENT3_N <= SEGMENT_N;
	endcase
end

assign SEGMENT_N = 	(HEXX == 4'h0)	?	{7'b1000000}:				//0
							(HEXX == 4'h1)	?	{7'b1111001}:					//1
							(HEXX == 4'h2)	?	{7'b0100100}:					//2
							(HEXX == 4'h3)	?	{7'b0110000}:					//3
							(HEXX == 4'h4)	?	{7'b0011001}:					//4
							(HEXX == 4'h5)	?	{7'b0010010}:					//5
							(HEXX == 4'h6)	?	{7'b0000010}:					//6
							(HEXX == 4'h7)	?	{7'b1111000}:					//7
							(HEXX == 4'h8)	?	{7'b0000000}:					//8
							(HEXX == 4'h9)	?	{7'b0011000}:					//9
							(HEXX == 4'hA)	?	{7'b0001000}:					//A
							(HEXX == 4'hB)	?	{7'b0000011}:					//B
							(HEXX == 4'hC)	?	{7'b1000110}:					//C
							(HEXX == 4'hD)	?	{7'b0100001}:					//D
							(HEXX == 4'hE)	?	{7'b0000110}:					//E
													{7'b0001110};					//F

assign HEXX	=	({DIGIT_N} == 4'b1110)	?				SPI_IN[3:0]:
					({DIGIT_N} == 4'b1101)	?				SPI_IN[7:4]:
					({DIGIT_N} == 4'b1011)	?				SPI_OUT[3:0]:
																	SPI_OUT[7:4];

`endif
/*****************************************************************************
* RAM signals
******************************************************************************/
assign	RAM0_BE0 = (ADDRESS == 16'hFF73)	?	!GART_READ[0]:
															!ADDRESS[0];								// Not SRAM signls
assign	RAM0_BE1 = (ADDRESS == 16'hFF73)	?	 GART_READ[0]:
															 ADDRESS[0];

assign	BLOCK_ADDRESS =	({MMU_EN, MMU_TR, ADDRESS[15:13]} ==  5'b10000)					?	SAM00:		// 10 000X	0000-1FFF
									({MMU_EN, MMU_TR, ADDRESS[15:13]} ==  5'b10001)					?	SAM01:		// 10 001X	2000-3FFF
									({MMU_EN, MMU_TR, ADDRESS[15:13]} ==  5'b10010)					?	SAM02:		// 10 010X	4000-5FFF
									({MMU_EN, MMU_TR, ADDRESS[15:13]} ==  5'b10011)					?	SAM03:		// 10 011X	6000-7FFF
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:13]} == 6'b010100)		?	SAM04:		//010 100X	8000-9FFF
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:13]} == 6'b010101)		?	SAM05:		//010 101X	A000-BFFF
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:13]} == 6'b010110)		?	SAM06:		//010 110X	C000-DFFF
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:12]} == 7'b0101110)	?	SAM07:		//010 1110 X		E000-EFFF
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:11]} == 8'b01011110)	?	SAM07:		//010 1111 0X		F000-F7FF
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:10]} == 9'b010111110)	?	SAM07:		//010 1111 10X		F800-FBFF
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:9]} == 10'b0101111110)?	SAM07:		//010 1111 110X	FC00-FDFF
							({VEC_PAG_RAM, MMU_EN, MMU_TR, ADDRESS[15:8]} == 11'b01011111110)	?	SAM07:		//010 1111 1110 X	FE00-FEFF Vector page as RAM
									({MMU_EN, MMU_TR, ADDRESS[15:13]} ==  5'b11000)					?	SAM10:		// 11 000X
									({MMU_EN, MMU_TR, ADDRESS[15:13]} ==  5'b11001)					?	SAM11:		// 11 001X
									({MMU_EN, MMU_TR, ADDRESS[15:13]} ==  5'b11010)					?	SAM12:		// 11 010X
									({MMU_EN, MMU_TR, ADDRESS[15:13]} ==  5'b11011)					?	SAM13:		//011 011X
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:13]} == 6'b011100)		?	SAM14:		//011 100X
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:13]} == 6'b011101)		?	SAM15:		//011 101X
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:13]} == 6'b011110)		?	SAM16:		//011 110X
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:12]} == 7'b0111110)	?	SAM17:		//011 1110 X		E000-EFFF
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:11]} == 8'b01111110)	?	SAM17:		//011 1111 0X		F000-F7FF
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:10]} == 9'b011111110)	?	SAM17:		//011 1111 10X		F800-FBFF
									({ROM_SEL, MMU_EN, MMU_TR, ADDRESS[15:9]} == 10'b0111111110)?	SAM17:		//011 1111 110X	FC00-FDFF
							({VEC_PAG_RAM, MMU_EN, MMU_TR, ADDRESS[15:8]} == 11'b01111111110)	?	SAM17:		//011 1111 1110 X	FE00-FEFF Vector page as RAM
																														{3'b111,ADDRESS[15:13]};
//CS and OE hardcoded low for WRITE CYCLE #3 in EDBLL datasheet
// Same for both SRAM chips
assign RAM0_CS_N = 1'b0;																															// Actual RAM CS is always enabled
assign RAM0_OE_N = 1'b0;
assign RAM0_CS =			(ROM_SEL)															?	1'b0:		// Any slot
								({RAM, ADDRESS[15:14]} == 3'b010)							?	1'b0:		// ROM (8000-BFFF)
								({RAM, ADDRESS[15:13]} == 4'b0110)							?	1'b0:		// ROM (C000-DFFF)
								({RAM, ADDRESS[15:12]} == 5'b01110)							?	1'b0:		// ROM (E000-EFFF)
								({RAM, ADDRESS[15:11]} == 6'b011110)						?	1'b0:		// ROM (F000-F8FF)
								({RAM, ADDRESS[15:10]} == 7'b0111110)						?	1'b0:		// ROM (F800-FBFF)
								({RAM, ADDRESS[15:9]}  == 8'b01111110)						?	1'b0:		// ROM (FC00-FDFF)
// FE00-FEFF enabled unless turned off by BLOCK_ADDRESS[6]=1
								({ADDRESS[15:0]}== 16'hFF73)									?	1'b1:		// GART
								({ADDRESS[15:8]}== 8'hFF)										?	1'b0:		// Hardware (FF00-FFFF)
																											1'b1;		// 0K - 512K

/*****************************************************************************
* ROM signals
******************************************************************************/
// ROM_SEL is 1 when the system is accessing any cartridge "ROM" meaning the
// 4 slots of the MPI, this is:
//		Slot 1 	RS232 ROM
//		Slot 2	Cart loader ROM
//		Slot 3	Cart slot
//		Slot 4	Disk Controller ROM
assign	ROM_SEL =( RAM								== 1'b1)		?	1'b0:	// All RAM Mode
						( ROM 							== 2'b10)	?	1'b0:	// All Internal
						({ROM[1], ADDRESS[15:14]}	== 3'b010)	?	1'b0: // Lower (Internal) 16 Internal+16 external
						(			 ADDRESS[15]		== 1'b0)		?	1'b0:	// Lower 32K
						(			 ADDRESS[15:8]		== 8'hFE)	?	1'b0:	// Vector space
						(			 ADDRESS[15:8]		== 8'hFF)	?	1'b0:	// Hardware space
																				1'b1;

//ROM
//00		16 Internal + 16 External
//01		16 Internal + 16 External
//10		32 Internal
//11		32 External

assign	FLASH_ADDRESS =	ENA_DSK			?	{9'b000000100, ADDRESS[12:0]}:	//8K Disk BASIC 8K Slot 4
									ENA_LOAD			?	{8'b00000011, ADDRESS[13:0]}:		//ROM Loader 16K Slot 2 or maybe SDCard Boot
									ENA_ORCC			?	{9'b000000101, ADDRESS[12:0]}:	//8K Orchestra 8K 90CC Slot 1
// Slot 3 ROMPak
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b100000)	?	{BANK0,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b110000)	?	{BANK0,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b101000)	?	{BANK0,1'b0,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b111000)	?	{BANK0,1'b1,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b100001)	?	{BANK1,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b110001)	?	{BANK1,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b101001)	?	{BANK1,1'b0,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b111001)	?	{BANK1,1'b1,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b100010)	?	{BANK2,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b110010)	?	{BANK2,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b101010)	?	{BANK2,1'b0,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b111010)	?	{BANK2,1'b1,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b100011)	?	{BANK3,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b110011)	?	{BANK3,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b101011)	?	{BANK3,1'b0,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b111011)	?	{BANK3,1'b1,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b100100)	?	{BANK4,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b110100)	?	{BANK4,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b101100)	?	{BANK4,1'b0,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b111100)	?	{BANK4,1'b1,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b100101)	?	{BANK5,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b110101)	?	{BANK5,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b101101)	?	{BANK5,1'b0,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b111101)	?	{BANK5,1'b1,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b100110)	?	{BANK6,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b110110)	?	{BANK6,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b101110)	?	{BANK6,1'b0,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b111110)	?	{BANK6,1'b1,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b100111)	?	{BANK7,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b110111)	?	{BANK7,     ADDRESS[14:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b101111)	?	{BANK7,1'b0,ADDRESS[13:0]}:
({ENA_PAK,BANK_SIZE,ROM_BANK}== 6'b111111)	?	{BANK7,1'b1,ADDRESS[13:0]}:
																{7'b0000000, ADDRESS[14:0]};

assign FLASH_WE_N = 1'b1;
assign FLASH_CE_N = 1'b0;
assign FLASH_OE_N = 1'b0;

assign FLASH_CE_S =	({RAM, ROM[1], ADDRESS[15:14]} ==  4'b0010)				?	1'b1:		// Internal 32K ROM 8000-BFFF
							({RAM, ROM,    ADDRESS[15:13]} ==  6'b010110)			?	1'b1:		// Internal 32K ROM C000-DFFF
							({RAM, ROM,    ADDRESS[15:12]} ==  7'b0101110)			?	1'b1:		// Internal 32K ROM E000-EFFF
							({RAM, ROM,    ADDRESS[15:11]} ==  8'b01011110)			?	1'b1:		// Internal 32K ROM F000-F7FF
							({RAM, ROM,    ADDRESS[15:14]} ==  5'b01010)				?	1'b1:		// Internal 16K ROM 8000-B7FF
							({RAM, ROM,    ADDRESS[15:10]} ==  9'b010111110)		?	1'b1:		// Internal ROM F800-F8FF
							({RAM, ROM,    ADDRESS[15:8]}  == 11'b01011111100)		?	1'b1:		// Internal ROM FC00-FCFF
							({RAM, ROM,    ADDRESS[15:8]}  == 11'b01011111101)		?	1'b1:		// Internal ROM FD00-FDFF
							ENA_DSK																?	1'b1:
							ENA_PAK																?	1'b1:
							ENA_LOAD																?	1'b1:
							ENA_ORCC																?	1'b1:
																										1'b0;

assign FFF0_EN =	({ADDRESS[15:4]} == 12'b111111111111)					?	1'b1:				// Vectors
																								1'b0;

assign	FLASH_RESET_N = RESET_N;

assign	ENA_DSK =	({ROM_SEL, MPI_CTS, ADDRESS[15:13]} == 6'b111110)		?	1'b1:	// Disk C000-DFFF Slot 4
																										1'b0;
assign	ENA_ORCC =	({ROM_SEL, MPI_CTS, ADDRESS[15:13]} == 6'b100110)		?	1'b1:	// Orchestra-90CC C000-DFFF Slot 1
																										1'b0;
assign	ENA_LOAD =	({ROM_SEL, MPI_CTS, ADDRESS[15:13]} == 6'b001110)		?	1'b1:	// ROM Loader C000-DFFF Slot 2
																										1'b0;
assign	ENA_PAK =	({ROM_SEL, MPI_CTS, ADDRESS[15]} == 4'b1101)				?	1'b1:	// ROM SLOT 3
																										1'b0;
assign	HDD_EN = ({MPI_SCS, ADDRESS[15:4]}	 == 14'b11111111110100)			?	1'b1:	//FF40-FF4F with MPI switch = 4
																										1'b0;

`ifdef RS232PAK
assign	RS232_EN = (ADDRESS[15:2] == 14'b11111111011010)						?	1'b1:	//FF68-FF6B
`else
assign	RS232_EN =
`endif
																										1'b0;

assign	SPI_EN = 			(ADDRESS[15:1]  == 15'b111111110110010);							// SPI FF64-FF65

`ifdef SD_DEBUG
assign	SPI_TRACE = 		(ADDRESS[15:1]  == 15'b111111110110011);							// SPI FF66-FF67
`endif

assign	SLOT3_HW = ({MPI_SCS, ADDRESS[15:5]} == 13'b1011111111010)			?	1'b1:		 // FF40-FF5F
																										1'b0;

always @(negedge PH_2 or negedge RESET_N)
begin
	if(!RESET_N)
	begin
		ROM_BANK <= 3'b000;
	end
	else
	begin
		if({SLOT3_HW, RW_N} == 2'b10)
			case (ADDRESS[4:0])
			5'h00:
			begin
				ROM_BANK <= DATA_OUT[2:0];
			end
			5'h02:
			begin
				BANK0 <= DATA_OUT[6:0];
			end
			5'h03:
			begin
				BANK_SIZE <= DATA_OUT[1:0];
			end
			5'h04:
			begin
				BANK1 <= DATA_OUT[6:0];
			end
			5'h05:
			begin
				BANK2 <= DATA_OUT[6:0];
			end
			5'h06:
			begin
				BANK3 <= DATA_OUT[6:0];
			end
			5'h07:
			begin
				BANK4 <= DATA_OUT[6:0];
			end
			5'h08:
			begin
				BANK5 <= DATA_OUT[6:0];
			end
			5'h09:
			begin
				BANK6 <= DATA_OUT[6:0];
			end
			5'h0A:
			begin
				BANK7 <= DATA_OUT[6:0];
			end
			endcase
	end
end
/*
$FF40 - This is the bank latch. The same latch that is used by the Super Program Paks to bank 16K of the Pak ROM
at a time. The initial design simply used 32K banks - this would allow the Super Program Paks to function, but
wastes 16K per Bank. This was done so the banks could house any of the 32K CoCo 3 Program PAKs as well. This latch
is set to $00 on reset or power up (same as the super program paks).

$FF41 - the CTS* WRITE data latch. This was incorporated because the CTS* line is read only. I could have just
derived a new CTS* that is active on both reads and writes, however, some PAKs utilize a copy protection scheme
whereas they write to the PAK area. As long as the PAK was in ROM, nothing happened, but if trying to run from a
R/W* RAMPAK, or from disk (wheras the CoCo is placed in the allram mode and the PAK transferred there and executed),
then the PAK code would be corrupted and a crash would occur. This behavior could be patched out of the PAK but I
wanted to be able to execute the PAK code verbatim. Thus this latch at $FF41. A byte of data is written to $FF41.
It is latched and a flip-flop is triggered (this flip-flop starts up un-triggered - either at power on or reset).
Once the flip-flop is triggered, it indicates a valid byte has been stored at $FF41 and then any READ of the CTS*
area will WRITE the byte from $FF41 into the SRAM at the memory location that was READ, the flip-flop is also
reset by this action. This allows writing to the CTS* area while still providing the Read Only protection offered
by the CTS* signal.
$FF42 - BANK 0 latch - this was incorporated because Aaron wanted to be able to start up with his operating code in
bank $00, however, the super program PAKs must start at bank $00. So, whatever is written into this latch will
be the bank that is accessed as BANK 0. This is reset to $00 only at power on (Not reset). So, whatever is written
here will be the bank accessed as bank 00 from that point forward, until it is changed again. Reset will not change it.
$FF43 - Bank Size latch. Only two bits used.:
             Bit 0 = 0 = 32K BANK SIZE, =1=16K Banks Size
             Bit 1 = 0 = Use lower 16K of each 32K bank
             Bit 1 = 1 = Use upper 16K of each 32K bank
Bit 1 is only effective if bank size is set to 16K by bit 0. This register is set to $00 at power on or reset,
and was added to reduce wasted memory. Under proper program control this allows two 16K or less program paks to
exist in each 32K bank.

$FF44-$FF4A = bank 1 through bank 7 latches. The largest super program pak that I am aware of was RoboCop,
consuming 8 banks of 16K for 128K total. These work just like the latch at $FF42 EXCEPT they affect banks 1-7.
They are also set to $01-$07 (respectively) on power up (but not reset). This allows a Super Program Pak to reside
in any banks in any order, by simply writing the proper data into these latches.
*/

// If W_PROT[1] = 1 then ROM_RW is 0, else ROM_RW = ~RW_N
assign	ROM_RW = ~(W_PROT[1] | RW_N);

assign	DATA_IN =				(RAM0_CS & RAM0_BE0)		?	RAM0_DATA[7:0]:
										(RAM0_CS & RAM0_BE1)		?	RAM0_DATA[15:8]:
														FFF0_EN		?	DATAO_FFF0:
														FLASH_CE_S	?	FLASH_DATA:
														HDD_EN		?	DATA_HDD:
`ifdef RS232PAK
														RS232_EN		?	DATA_RS232:
`endif
														SLOT3_HW		?	{5'b00000, ROM_BANK}:
														SPI_EN		?	SPI_DATA:
// FF00, FF04, FF08, FF0C, FF10, FF14, FF18, FF1C
({ADDRESS[15:5], ADDRESS[1:0]} == 13'b1111111100000)	?	DATA_REG1:
// FF01, FF05, FF09, FF0D, FF11, FF15, FF19, FF1D
({ADDRESS[15:5], ADDRESS[1:0]} == 13'b1111111100001)	?	{~HS_INT, 3'b011, SEL[0], DDR1, HSYNC_POL, HSYNC_INT}:
// FF02, FF06, FF0A, FF0E, FF12, FF16, FF1A, FF1E
({ADDRESS[15:5], ADDRESS[1:0]} == 13'b1111111100010)	?	DATA_REG2:
// FF03, FF07, FF0B, FF0F, FF13, FF17, FF1B, FF1F
({ADDRESS[15:5], ADDRESS[1:0]} == 16'b1111111100011)	?	{~VS_INT, 3'b011, SEL[1], DDR2, VSYNC_POL, VSYNC_INT}:
// FF20, FF24, FF28, FF2C, FF30, FF34, FF38, FF3C
({ADDRESS[15:5], ADDRESS[1:0]} == 16'b1111111100100)	?	DATA_REG3:
// FF21, FF25, FF29, FF2D, FF31, FF35, FF39, FF3D
({ADDRESS[15:5], ADDRESS[1:0]} == 16'b1111111100101)	?	{4'b0011, CAS_MTR, DDR3, CD_POL, CD_INT}:
// FF22, FF26, FF2A, FF2E, FF32, FF36, FF3A, FF3E
({ADDRESS[15:5], ADDRESS[1:0]} == 16'b1111111100110)	?	DATA_REG4:
// FF23, FF27, FF2B, FF2F, FF33, FF37, FF3B, FF3F
({ADDRESS[15:5], ADDRESS[1:0]} == 16'b1111111100111)	?	{~CART1_INT, 3'b011, SOUND_EN, DDR4, CART_POL, CART_INT}:
									(ADDRESS == 16'hFF70)		?	{5'h00, GART_WRITE[18:16]}:
									(ADDRESS == 16'hFF71)		?	{       GART_WRITE[15:8]}:
									(ADDRESS == 16'hFF72)		?	{       GART_WRITE[7:0]}:
									(ADDRESS == 16'hFF74)		?	{5'h00, GART_READ[18:16]}:
									(ADDRESS == 16'hFF75)		?	{       GART_READ[15:8]}:
									(ADDRESS == 16'hFF76)		?	{       GART_READ[7:0]}:
									(ADDRESS == 16'hFF76)		?	{6'b000000, GART_INC[1:0]}:
									(ADDRESS == 16'hFF7F)		?	{2'b11, MPI_CTS, W_PROT, MPI_SCS}:
									(ADDRESS == 16'hFF90)		?	{COCO1, MMU_EN, GIME_IRQ, GIME_FIRQ, VEC_PAG_RAM, ST_SCS, ROM}:
									(ADDRESS == 16'hFF91)		?	{2'b00, TIMER_INS, 4'b0000, MMU_TR}:
									(ADDRESS == 16'hFF92)		?	{2'b00, ~TMR_INT,  ~HBORD_INT,  ~VBORD_INT,  1'b0, ~KEY_INT,  ~CAR_INT}:
									(ADDRESS == 16'hFF93)		?	{2'b00, ~TMR_FINT, ~HBORD_FINT, ~VBORD_FINT, 1'b0, ~KEY_FINT, ~CAR_FINT}:
// HiRes Joystick
									(ADDRESS == 16'hFF60)		?	{4'h0, PADDLE_LATCH_0[11:8]}:
									(ADDRESS == 16'hFF61)		?	PADDLE_LATCH_0[7:0]:
									(ADDRESS == 16'hFF62)		?	{4'h0, PADDLE_LATCH_1[11:8]}:
									(ADDRESS == 16'hFF63)		?	PADDLE_LATCH_1[7:0]:
									(ADDRESS == 16'hFF64)		?	{4'h0, PADDLE_LATCH_2[11:8]}:
									(ADDRESS == 16'hFF65)		?	PADDLE_LATCH_2[7:0]:
									(ADDRESS == 16'hFF66)		?	{4'h0, PADDLE_LATCH_3[11:8]}:
									(ADDRESS == 16'hFF67)		?	PADDLE_LATCH_3[7:0]:

									(ADDRESS == 16'hFF94)		?	{4'h0,TMR_MSB}:
									(ADDRESS == 16'hFF95)		?	TMR_LSB:
									(ADDRESS == 16'hFF98)		?	{GRMODE, HRES[3], DESCEN, MONO, 1'b0, LPR}:
									(ADDRESS == 16'hFF99)		?	{HLPR, LPF, HRES[2:0], CRES}:
									(ADDRESS == 16'hFF9A)		?	{2'b00, PALETTE[16][5:0]}:
									(ADDRESS == 16'hFF9C)		?	{4'h0,VERT_FIN_SCRL}:
									(ADDRESS == 16'hFF9D)		?	SCRN_START_MSB:
									(ADDRESS == 16'hFF9E)		?	SCRN_START_LSB:
									(ADDRESS == 16'hFF9F)		?	{HVEN,HOR_OFFSET}:
									(ADDRESS == 16'hFFA0)		?	SAM00:
									(ADDRESS == 16'hFFA1)		?	SAM01:
									(ADDRESS == 16'hFFA2)		?	SAM02:
									(ADDRESS == 16'hFFA3)		?	SAM03:
									(ADDRESS == 16'hFFA4)		?	SAM04:
									(ADDRESS == 16'hFFA5)		?	SAM05:
									(ADDRESS == 16'hFFA6)		?	SAM06:
									(ADDRESS == 16'hFFA7)		?	SAM07:
									(ADDRESS == 16'hFFA8)		?	SAM10:
									(ADDRESS == 16'hFFA9)		?	SAM11:
									(ADDRESS == 16'hFFAA)		?	SAM12:
									(ADDRESS == 16'hFFAB)		?	SAM13:
									(ADDRESS == 16'hFFAC)		?	SAM14:
									(ADDRESS == 16'hFFAD)		?	SAM15:
									(ADDRESS == 16'hFFAE)		?	SAM16:
									(ADDRESS == 16'hFFAF)		?	SAM17:
									(ADDRESS == 16'hFFB0)		?	{2'b00, PALETTE[0][5:0]}:
									(ADDRESS == 16'hFFB1)		?	{2'b00, PALETTE[1][5:0]}:
									(ADDRESS == 16'hFFB2)		?	{2'b00, PALETTE[2][5:0]}:
									(ADDRESS == 16'hFFB3)		?	{2'b00, PALETTE[3][5:0]}:
									(ADDRESS == 16'hFFB4)		?	{2'b00, PALETTE[4][5:0]}:
									(ADDRESS == 16'hFFB5)		?	{2'b00, PALETTE[5][5:0]}:
									(ADDRESS == 16'hFFB6)		?	{2'b00, PALETTE[6][5:0]}:
									(ADDRESS == 16'hFFB7)		?	{2'b00, PALETTE[7][5:0]}:
									(ADDRESS == 16'hFFB8)		?	{2'b00, PALETTE[8][5:0]}:
									(ADDRESS == 16'hFFB9)		?	{2'b00, PALETTE[9][5:0]}:
									(ADDRESS == 16'hFFBA)		?	{2'b00, PALETTE[10][5:0]}:
									(ADDRESS == 16'hFFBB)		?	{2'b00, PALETTE[11][5:0]}:
									(ADDRESS == 16'hFFBC)		?	{2'b00, PALETTE[12][5:0]}:
									(ADDRESS == 16'hFFBD)		?	{2'b00, PALETTE[13][5:0]}:
									(ADDRESS == 16'hFFBE)		?	{2'b00, PALETTE[14][5:0]}:
									(ADDRESS == 16'hFFBF)		?	{2'b00, PALETTE[15][5:0]}:
									(ADDRESS == 16'hFFC0)		?	{3'b000, CENT}:
									(ADDRESS == 16'hFFC1)		?	{1'b0, YEAR}:
									(ADDRESS == 16'hFFC2)		?	{4'h0, MNTH}:
									(ADDRESS == 16'hFFC3)		?	{3'b000, DMTH}:
									(ADDRESS == 16'hFFC4)		?	{5'b00000, DWK}:
									(ADDRESS == 16'hFFC5)		?	{3'b000, HOUR}:
									(ADDRESS == 16'hFFC6)		?	{2'b00, MIN}:
									(ADDRESS == 16'hFFC7)		?	{2'b00, SEC}:
									(ADDRESS == 16'hFFCC)		?	{7'h00,HBLANK}:
									(ADDRESS == 16'hFFCD)		?	{7'h00,VBLANK}:
									(ADDRESS == 16'hFFCE)		?	{7'h00,H_SYNC}:
									(ADDRESS == 16'hFFCF)		?	{7'h00,V_SYNC}:
																			8'h55;

assign	DATA_REG1	= ~DDR1	?	DD_REG1:
											KEYBOARD_IN;

assign	DATA_REG2	= ~DDR2	?	DD_REG2:
											KEY_COLUMN;

assign	DATA_REG3	= ~DDR3	?	DD_REG3:
											{DTOA_CODE, 1'b1, 1'b1};

assign	DATA_REG4	= ~DDR4	?	DD_REG4:
											{VDG_CONTROL, 1'b0, KEY_COLUMN[6], SBS, 1'b1};

// Clock for Drivewire UART on the slave processor(6850)
// 8 cycles in 50 MHz / 27 = 8*50/27 = 14.815 MHz
always @(negedge CLK50MHZ or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		COM1_STATE <= 5'h00;
		COM1_CLOCK_X <= 1'b0;
	end
	else
	begin
		case (COM1_STATE)
		5'h00:
		begin
			COM1_STATE <= 5'h01;
			COM1_CLOCK_X <= 1'b0;
		end
		5'h01:
		begin
			COM1_STATE <= 5'h02;
			COM1_CLOCK_X <= 1'b1;
		end
		5'h02:
		begin
			COM1_STATE <= 5'h03;
			COM1_CLOCK_X <= 1'b0;
		end
		5'h04:
		begin
			COM1_STATE <= 5'h05;
			COM1_CLOCK_X <= 1'b1;
		end
		5'h05:
		begin
			COM1_STATE <= 5'h06;
			COM1_CLOCK_X <= 1'b0;
		end
		5'h07:
		begin
			COM1_STATE <= 5'h08;
			COM1_CLOCK_X <= 1'b1;
		end
		5'h09:
		begin
			COM1_STATE <= 5'h0A;
			COM1_CLOCK_X <= 1'b0;
		end
		5'h0B:
		begin
			COM1_STATE <= 5'h0C;
			COM1_CLOCK_X <= 1'b1;
		end
		5'h0C:
		begin
			COM1_STATE <= 5'h0D;
			COM1_CLOCK_X <= 1'b0;
		end
		5'h0E:
		begin
			COM1_STATE <= 5'h0F;
			COM1_CLOCK_X <= 1'b1;
		end
		5'h0F:
		begin
			COM1_STATE <= 5'h10;
			COM1_CLOCK_X <= 1'b0;
		end
		5'h11:
		begin
			COM1_STATE <= 5'h12;
			COM1_CLOCK_X <= 1'b1;
		end
		5'h13:
		begin
			COM1_STATE <= 5'h14;
			COM1_CLOCK_X <= 1'b0;
		end
		5'h15:
		begin
			COM1_STATE <= 5'h16;
			COM1_CLOCK_X <= 1'b1;
		end
		5'h16:
		begin
			COM1_STATE <= 5'h17;
			COM1_CLOCK_X <= 1'b0;
		end
		5'h18:
		begin
			COM1_STATE <= 5'h19;
			COM1_CLOCK_X <= 1'b1;
		end
		5'h1A:
		begin
			COM1_STATE <= 5'h00;
			COM1_CLOCK_X <= 1'b0;
		end
		5'h1F:
		begin
			COM1_STATE <= 5'h00;
			COM1_CLOCK_X <= 1'b0;
		end
		default:
		begin
			COM1_STATE <= COM1_STATE + 1'b1;
		end
		endcase
	end
end

//Switch selectable baud rate
always @(negedge COM1_CLOCK_X or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		COM1_CLK <= 3'b000;
		COM1_CLOCK <= 1'b0;
	end
	else
	begin
		case (COM1_CLK)
		3'b000:
		begin
			COM1_CLOCK <= 1'b1;
			COM1_CLK <= 3'b001;
		end
		3'b001:
		begin
			COM1_CLOCK <= 1'b0;
			if(SWITCH[8:7]==2'b10)				// divide by 2 460800  = 14.8148 / 2 /16 = 462962.963 = +0.2084335%
				COM1_CLK <= 3'b000;
			else
				COM1_CLK <= 3'b010;
		end
		3'b011:
		begin
			COM1_CLOCK <= 1'b0;
			if(SWITCH[8:7]==2'b01)				// divide by 4 230400
				COM1_CLK <= 3'b000;
			else
				COM1_CLK <= 3'b100;
		end
		3'b111:									// divide by 8 115200
		begin
			COM1_CLK <= 3'b000;
		end
		default:
		begin
			COM1_CLK <= COM1_CLK + 1'b1;
		end
		endcase
	end
end
// Combinatorial clock :(
assign UART1_CLK = (SWITCH[8:7] == 2'b11)	?	COM1_CLOCK_X:	// 921600
															COM1_CLOCK;

`ifdef RS232PAK
// Clock for RS232 PAK (6551)
// 24 MHz / 13 = 1.846 MHz
always @(negedge CLK24MHZ or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		COM2_STATE <= 13'b0000000000001;
	end
	else
	begin
//		case (COM2_STATE)
//		13'b0000001000000:
//		begin
//			COM2_STATE <= {COM2_STATE[11:0],COM2_STATE[12]};
//		end
//		default:
//		begin
			COM2_STATE <= {COM2_STATE[11:0],COM2_STATE[12]};
//		end
//		endcase
	end
end
`endif

`ifndef NEW_SRAM
// CPU clock / SRAM Signals for old SRAM
always @(negedge CLK50MHZ or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		CLK <= 6'h00;
		SWITCH_L <= 2'b00;
		PH_2_RAW <= 1'b0;
		RAM0_RW_N <= 1'b1;
		RAM0_DATA[15:0] <= 16'bZZZZZZZZZZZZZZZZ;
		RAM0_BE0_N <=  1'b0;
		RAM0_BE1_N <= 1'b0;
	end
	else
	begin
		case (CLK)
		6'h00:
		begin
			SWITCH_L <= {SWITCH[0], RATE};					// Normal speed
			PH_2_RAW <= 1'b1;
			VIDEO_BUFFER <= RAM0_DATA;							// Grab video one more time
			CLK <= 6'h01;
			RAM0_RW_N <= RW_N;
			if(ADDRESS[15:0]==16'hFF73)
			begin
				if(!RW_N)
				begin
					RAM0_ADDRESS <= GART_WRITE[18:1];
					RAM0_BE0_N <=  GART_WRITE[0];
					RAM0_BE1_N <= ~GART_WRITE[0];
				end
				else
				begin
					RAM0_ADDRESS <= GART_READ[18:1];
					RAM0_BE0_N <=  GART_READ[0];
					RAM0_BE1_N <= ~GART_READ[0];
				end
			end
			else
			begin
				RAM0_ADDRESS <= {BLOCK_ADDRESS, ADDRESS[12:1]};
				RAM0_BE0_N <=  ADDRESS[0] | !RAM0_CS;
				RAM0_BE1_N <= ~ADDRESS[0] | !RAM0_CS;
			end
			if (!RW_N)
				RAM0_DATA[15:0] <= {DATA_OUT, DATA_OUT};
			else
				RAM0_DATA[15:0] <= 16'bZZZZZZZZZZZZZZZZ;
		end
		6'h01:
		begin
			PH_2_RAW <= 1'b0;
			RAM0_ADDRESS <= VIDEO_ADDRESS;
			RAM0_BE0_N <= 1'b0;
			RAM0_BE1_N <= 1'b0;
			RAM0_DATA[15:0] <= 16'bZZZZZZZZZZZZZZZZ;
			RAM0_RW_N <= 1'b1;
			if({SWITCH_L} == 2'b11)		//50/2 = 25 
				CLK <= 6'h00;
			else
				CLK <= 6'h02;
		end
		6'h1B:								//	50/28 = 1.7857
		begin
			RAM0_ADDRESS <= VIDEO_ADDRESS;
			VIDEO_BUFFER <= RAM0_DATA;
			if(SWITCH_L[0])				//Rate = 1?
				CLK <= 6'h00;
			else
				CLK <= 6'h1C;
		end
		6'h37:								// 50/56 = 0.89286
		begin
			RAM0_ADDRESS <= VIDEO_ADDRESS;
			VIDEO_BUFFER <= RAM0_DATA;
			CLK <= 6'h00;
		end
		6'h3F:								// Just in case
		begin
			CLK <= 6'h00;
		end
		default:
		begin
			CLK <= CLK + 1'b1;
			RAM0_ADDRESS <= VIDEO_ADDRESS;
			VIDEO_BUFFER <= RAM0_DATA;
		end
		endcase
	end
end
`else
// Some SRAM Signals for new SRAM offset by 1/2 clock (10 nS)
always @(posedge CLK50MHZ or negedge RESET_N)
begin
	if(~RESET_N)
	begin
			RAM0_RW_N <= 1'b1;
			RAM0_BE0_N <= 1'b0;
			RAM0_BE1_N <= 1'b0;
	end
	else
	begin
		case (CLK)
		6'h01:
		begin
			RAM0_RW_N <= RW_N;
			if(ADDRESS[15:0]==16'hFF73)
			begin
				if(!RW_N)
				begin
					RAM0_BE0_N <=  GART_WRITE[0];
					RAM0_BE1_N <= ~GART_WRITE[0];
				end
				else
				begin
					RAM0_BE0_N <=  GART_READ[0];
					RAM0_BE1_N <= ~GART_READ[0];
				end
			end
			else
			begin
				RAM0_BE0_N <=  ADDRESS[0] | !RAM0_CS;
				RAM0_BE1_N <= ~ADDRESS[0] | !RAM0_CS;
			end
		end
		6'h02:
		begin
			RAM0_RW_N <= RW_N;
			if(ADDRESS[15:0]==16'hFF73)
			begin
				if(!RW_N)
				begin
					RAM0_BE0_N <=  GART_WRITE[0];
					RAM0_BE1_N <= ~GART_WRITE[0];
				end
				else
				begin
					RAM0_BE0_N <=  GART_READ[0];
					RAM0_BE1_N <= ~GART_READ[0];
				end
			end
			else
			begin
				RAM0_BE0_N <=  ADDRESS[0] | !RAM0_CS;
				RAM0_BE1_N <= ~ADDRESS[0] | !RAM0_CS;
			end
		end
		default:
		begin
			RAM0_RW_N <= 1'b1;
			RAM0_BE0_N <= 1'b0;
			RAM0_BE1_N <= 1'b0;
		end
		endcase
	end
end

// CPU clock / SRAM Signals for new SRAM
always @(negedge CLK50MHZ or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		CLK <= 6'h00;
		SWITCH_L <= 2'b00;
		PH_2_RAW <= 1'b0;
		RAM0_DATA[15:0] <= 16'bZZZZZZZZZZZZZZZZ;
	end
	else
	begin
		case (CLK)
		6'h00:
		begin
			SWITCH_L <= {SWITCH[0], RATE};					// Normal speed
			VIDEO_BUFFER <= RAM0_DATA;							// Grab video one more time
			PH_2_RAW <= 1'b1;
			if(ADDRESS[15:0]==16'hFF73)
			begin
				if(!RW_N)
				begin
					RAM0_ADDRESS <= GART_WRITE[18:1];
				end
				else
				begin
					RAM0_ADDRESS <= GART_READ[18:1];
				end
			end
			else
			begin
				RAM0_ADDRESS <= {BLOCK_ADDRESS, ADDRESS[12:1]};
			end
			CLK <= 6'h01;
			if (!RW_N)
				RAM0_DATA[15:0] <= {DATA_OUT, DATA_OUT};
			else
				RAM0_DATA[15:0] <= 16'bZZZZZZZZZZZZZZZZ;
		end
		6'h01:
		begin
			PH_2_RAW <= 1'b1;
			if(ADDRESS[15:0]==16'hFF73)
			begin
				if(!RW_N)
				begin
					RAM0_ADDRESS <= GART_WRITE[18:1];
				end
				else
				begin
					RAM0_ADDRESS <= GART_READ[18:1];
				end
			end
			else
			begin
				RAM0_ADDRESS <= {BLOCK_ADDRESS, ADDRESS[12:1]};
			end
			CLK <= 6'h02;
			if (!RW_N)
				RAM0_DATA[15:0] <= {DATA_OUT, DATA_OUT};
			else
				RAM0_DATA[15:0] <= 16'bZZZZZZZZZZZZZZZZ;
		end
		6'h02:
		begin
			PH_2_RAW <= 1'b1;
			if(ADDRESS[15:0]==16'hFF73)
			begin
				if(!RW_N)
				begin
					RAM0_ADDRESS <= GART_WRITE[18:1];
				end
				else
				begin
					RAM0_ADDRESS <= GART_READ[18:1];
				end
			end
			else
			begin
				RAM0_ADDRESS <= {BLOCK_ADDRESS, ADDRESS[12:1]};
			end
			CLK <= 6'h03;
			if (!RW_N)
				RAM0_DATA[15:0] <= {DATA_OUT, DATA_OUT};
			else
				RAM0_DATA[15:0] <= 16'bZZZZZZZZZZZZZZZZ;
		end
		6'h03:
		begin
			PH_2_RAW <= 1'b0;
			RAM0_ADDRESS <= VIDEO_ADDRESS;
			CLK <= 6'h04;
			RAM0_DATA[15:0] <= 16'bZZZZZZZZZZZZZZZZ;
		end
		6'h04:
		begin
			VIDEO_BUFFER <= RAM0_DATA;							// Grab video one more time
			PH_2_RAW <= 1'b0;
			RAM0_ADDRESS <= VIDEO_ADDRESS;
			CLK <= 6'h05;
			RAM0_DATA[15:0] <= 16'bZZZZZZZZZZZZZZZZ;
		end
		6'h05:
		begin
			VIDEO_BUFFER <= RAM0_DATA;							// Grab video one more time
			PH_2_RAW <= 1'b0;
			RAM0_ADDRESS <= VIDEO_ADDRESS;
			RAM0_DATA[15:0] <= 16'bZZZZZZZZZZZZZZZZ;
			if(SWITCH_L == 2'b11)		//	50/6 = 8.33
				CLK <= 6'h00;
			else
				CLK <= 6'h06;
		end
		6'h1B:								//	50/28 = 1.7857
		begin
			RAM0_ADDRESS <= VIDEO_ADDRESS;
			VIDEO_BUFFER <= RAM0_DATA;
			if(SWITCH_L[0])				//Rate = 1?
				CLK <= 6'h00;
			else
				CLK <= 6'h1C;
		end
		6'h37:								// 50/56 = 0.89286
		begin
			RAM0_ADDRESS <= VIDEO_ADDRESS;
			VIDEO_BUFFER <= RAM0_DATA;
			CLK <= 6'h00;
		end
		6'h3F:								// Just in case
		begin
			CLK <= 6'h00;
		end
		default:
		begin
			CLK <= CLK + 1'b1;
			RAM0_ADDRESS <= VIDEO_ADDRESS;
			VIDEO_BUFFER <= RAM0_DATA;
		end
		endcase
	end
end
`endif
// Make sure PH2 is a Global Clock
PH2_CLK	PH2_CLK_inst (
	.inclk ( PH_2_RAW ),
	.outclk ( PH_2 )
	);

assign RESET_P =	!BUTTON_N[3]					// Button
						| RESET;							// CTRL-ALT-DEL

// Make sure all resets are enabled for a long enough time to allow voltages to settle
always @ (negedge MCLOCK[8] or posedge RESET_P)		// 50 MHz / 64
begin
	if(RESET_P)
	begin
		RESET_SM <= 14'h0000;
		CPU_RESET <= 1'b1;
		RESET_N <= 1'b0;
	end
	else
		case (RESET_SM)
		14'h3800:									// time = 1.28 uS * 14336 = 18350.08 uS
		begin
			RESET_N <= 1'b1;
			CPU_RESET <= 1'b1;
			RESET_SM <= 14'h3801;
		end
		14'h3FFF:									// time = 1.28 uS * 16383 = 20970.24 uS
		begin
			RESET_N <= 1'b1;
			CPU_RESET <= 1'b0;
			RESET_SM <= 14'h3FFF;
		end
		default:
			RESET_SM <= RESET_SM + 1'b1;
		endcase
end

// CPU section copyrighted by John Kent
CPU09 GLBCPU09(
	.clk(PH_2),
	.rst(CPU_RESET),
	.vma(VMA),
	.addr(ADDRESS),
	.rw(RW_N),
	.data_in(DATA_IN),
	.data_out(DATA_OUT),
	.halt(HALT_BUF2),
	.hold(1'b0),
	.irq(~CPU_IRQ),
	.firq(~CPU_FIRQ),
	.nmi(NMI_09)
);

//FFF0-FFFF needs fast RAM / ROM
//Flash is too slow
FFF0 FFF0(
.address(ADDRESS[10:0]),
.clock(PH_2),
.data(DATA_OUT),
.wren(ROM_RW & FFF0_EN),
.q(DATAO_FFF0)
);

// Disk Drive Controller / Slave processor
`include "CoCo3IO.v"

// Interrupt source for CART signal
always @(negedge PH_2 or negedge RESET_N)
begin
	if(~RESET_N)
	begin
		CART_IRQ <= 1'b1;
	end
	else
	begin
		case (MPI_SCS)
		2'b00:
			CART_IRQ <= ~CART_IRQ | SWITCH[4];
		2'b01:
			CART_IRQ <= ~CART_IRQ | SWITCH[4];
		2'b10:
			CART_IRQ <= ~CART_IRQ | SWITCH[4];
		2'b11:
//			CART_IRQ <= (~IRQ_09 & SER_IRQ & IRQ_SPI_N) | SWITCH[4];
`ifdef RS232PAK
			CART_IRQ <= (~IRQ_09 & IRQ_SPI_N & SER_IRQ) | SWITCH[4];
`else
			CART_IRQ <= (~IRQ_09 & IRQ_SPI_N ) | SWITCH[4];
`endif
		endcase
	end
end
//***********************************************************************
// Interrupt latches
//***********************************************************************
// INT for COCO1
always @ (negedge PH_2)
begin
// H_SYNC int for COCO1
// output	HS_INT
// State		HS_INT_SM
// input		H_SYNC / H_FLAG
// switch	HSYNC_INT @ FF01
// pol		HSYNC_POL
// clear    FF00

		if(!RESET_N)
		begin
			HS_INT <= 1'b1;				// No flag
			H_SYNC_IRQ_N <= 1'b1;			// no int
// Start SM at last step to makse sure you see a deasserted trigger
// before triggering the first time
			HS_INT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (HS_INT_SM)
			2'b00:
			begin
				if((~H_SYNC ^ HSYNC_POL) & H_FLAG)		// 1 = int
				begin
					HS_INT <= 1'b0;
					H_SYNC_IRQ_N <= !HSYNC_INT;
					HS_INT_SM <= 2'b01;
				end
//				else not needed
//				begin
//					HS_INT <= 1'b1;			// no int
//					H_SYNC_IRQ_N <= 1'b1;
//					HS_INT_SM <= 2'b00;
//				end
			end
			2'b01:
			begin
				if({ADDRESS[15:5], ADDRESS[1:0]} == 13'b1111111100000)		// FF00, FF04, FF08, FF0C, FF10, FF14, FF18, FF1C
				begin
					HS_INT <= 1'b1;
					H_SYNC_IRQ_N <= 1'b1;
					HS_INT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(~((~H_SYNC ^ HSYNC_POL) & H_FLAG))
				begin
					HS_INT <= 1'b1;
					H_SYNC_IRQ_N <= 1'b1;
					HS_INT_SM <= 2'b00;
				end
			end
			endcase
		end
end

always @ (negedge PH_2) //  or negedge RESET_N)
begin
// V_SYNC int for COCO1
// Start of Sync is a 1 to 0
// output	VS_INT
// State		VS_INT_SM
// input		V_SYNC
// switch	VSYNC_INT @ FF03
// pol		VSYNC_POL
// clear    FF02

		if(!RESET_N)		// disabled
		begin
			VS_INT <= 1'b1;			// no int
			V_SYNC_IRQ_N <= 1'b1;
// Start SM at last step to makse sure you see a deasserted trigger
// before triggering the first time
			VS_INT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (VS_INT_SM)
			2'b00:
			begin
				if(~V_SYNC ^ VSYNC_POL)		// 1 = int
				begin
					VS_INT <= 1'b0;
					V_SYNC_IRQ_N <= !VSYNC_INT;
					VS_INT_SM <= 2'b01;
				end
//				else
//				begin
//					VS_INT <= 1'b1;			// no int
//					V_SYNC_IRQ_N <= 1'b1;
//					VS_INT_SM <= 2'b00;
//				end
			end
			2'b01:
			begin
				if({ADDRESS[15:5], ADDRESS[1:0]} == 13'b1111111100010)		// FF02, FF06, FF0A, FF0E, FF12, FF16, FF1A, FF1E
				begin
					VS_INT <= 1'b1;
					V_SYNC_IRQ_N <= 1'b1;
					VS_INT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(~(~V_SYNC ^ VSYNC_POL))
				begin
					VS_INT <= 1'b1;
					V_SYNC_IRQ_N <= 1'b1;
					VS_INT_SM <= 2'b00;
				end
			end
			endcase
		end
end

always @ (negedge PH_2) //  or negedge RESET_N)
begin
// Cart int for COCO1
// output	CART1_INT
// State		CART1_INT_SM
// input		CART_IRQ
// switch	CART_INT @ FF23
// pol		CART_POL
// clear    FF22

		if(CART_INT == 1'b0)		// disabled
		begin
			CART1_INT <= 1'b1;			// no int
			CART1_INT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (CART1_INT_SM)
			2'b00:
			begin
				if(~CART_IRQ)
				begin
					CART1_INT <= 1'b0;
					CART1_INT_SM <= 2'b01;
				end
				else
				begin
					CART1_INT <= 1'b1;			// no int
					CART1_INT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:5], ADDRESS[1:0]} == 13'b1111111100110)		// FF22, FF26, FF2A, FF2E, FF32, FF36, FF3A, FF3E
				begin
					CART1_INT <= 1'b1;
					CART1_INT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(CART_IRQ)
				begin
					CART1_INT <= 1'b1;
					CART1_INT_SM <= 2'b00;
				end
			end
			endcase
		end
end


// INT for COCO3
always @ (negedge PH_2) //  or negedge RESET_N)
begin
// TIMER int for COCO3
// output	TMR_INT
// State		TMR_INT_SM
// input		TIMER_N
// switch	IRQ_TMR

		if(IRQ_TMR == 1'b0)		// disabled
		begin
			TMR_INT <= 1'b1;			// no int
			TMR_INT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (TMR_INT_SM)
			2'b00:
			begin
				if(!TIMER_N)
				begin
					TMR_INT <= 1'b0;
					TMR_INT_SM <= 2'b01;
				end
				else
				begin
					TMR_INT <= 1'b1;			// no int
					TMR_INT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:0]} == 16'hFF92)
				begin
					TMR_INT <= 1'b1;
					TMR_INT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(TIMER_N)
				begin
					TMR_INT <= 1'b1;
					TMR_INT_SM <= 2'b00;
				end
			end
			endcase
		end
end

always @ (negedge PH_2) //  or negedge RESET_N)
begin
// H_SYNC int for COCO3
// output	HBORD_INT
// State		HBORD_INT_SM
// input		H_SYNC / H_FLAG
// switch	IRQ_HBORD

		if(IRQ_HBORD == 1'b0)		// disabled
		begin
			HBORD_INT <= 1'b1;			// no int
			HBORD_INT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (HBORD_INT_SM)
			2'b00:
			begin
				if(~H_SYNC & H_FLAG)		// 1 = int
				begin
					HBORD_INT <= 1'b0;
					HBORD_INT_SM <= 2'b01;
				end
				else
				begin
					HBORD_INT <= 1'b1;			// no int
					HBORD_INT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:0]} == 16'HFF92)
				begin
					HBORD_INT <= 1'b1;
					HBORD_INT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(~(~H_SYNC & H_FLAG))
				begin
					HBORD_INT <= 1'b1;
					HBORD_INT_SM <= 2'b00;
				end
			end
			endcase
		end
end
always @ (negedge PH_2) //  or negedge RESET_N)
begin
// V_SYNC int for COCO3
// Start of Sync is a 1 to 0
// output	VBORD_INT
// State		VBORD_INT_SM
// input		V_SYNC
// switch	IRQ_VBORD

		if(IRQ_VBORD == 1'b0)		// disabled
		begin
			VBORD_INT <= 1'b1;			// no int
			VBORD_INT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (VBORD_INT_SM)
			2'b00:
			begin
				if(~V_SYNC)
				begin
					VBORD_INT <= 1'b0;
					VBORD_INT_SM <= 2'b01;
				end
				else
				begin
					VBORD_INT <= 1'b1;			// no int
					VBORD_INT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:0]} == 16'HFF92)
				begin
					VBORD_INT <= 1'b1;
					VBORD_INT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(V_SYNC)
				begin
					VBORD_INT <= 1'b1;
					VBORD_INT_SM <= 2'b00;
				end
			end
			endcase
		end
end

always @ (negedge PH_2) //  or negedge RESET_N)
begin
// Keyboard int for COCO3
// output	KEY_INT
// State		KEY_INT_SM
// input		KEY_INT_RAW
// switch	IRQ_KEY

		if(IRQ_KEY == 1'b0)		// disabled
		begin
			KEY_INT <= 1'b1;			// no int
			KEY_INT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (KEY_INT_SM)
			2'b00:
			begin
				if(~KEY_INT_RAW)
				begin
					KEY_INT <= 1'b0;
					KEY_INT_SM <= 2'b01;
				end
				else
				begin
					KEY_INT <= 1'b1;			// no int
					KEY_INT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:0]} == 16'HFF92)
				begin
					KEY_INT <= 1'b1;
					KEY_INT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(KEY_INT_RAW)
				begin
					KEY_INT <= 1'b1;
					KEY_INT_SM <= 2'b00;
				end
			end
			endcase
		end
end

always @ (negedge PH_2) //  or negedge RESET_N)
begin
// CART (Serial HDD) int for COCO3
// output	CAR_INT
// State		CAR_INT_SM
// input		CART_IRQ
// switch	IRQ_CART
		if(IRQ_CART == 1'b0)		// disabled
		begin
			CAR_INT <= 1'b1;			// no int
			CAR_INT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (CAR_INT_SM)
			2'b00:							// Wait for int
			begin
				if(~CART_IRQ)				// Int in?
				begin							// Yes
					CAR_INT <= 1'b0;
					CAR_INT_SM <= 2'b01;
				end
				else
				begin
					CAR_INT <= 1'b1;			// No int
					CAR_INT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:0]} == 16'HFF92)
				begin
					CAR_INT <= 1'b1;
					CAR_INT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(CART_IRQ)
				begin
					CAR_INT <= 1'b1;
					CAR_INT_SM <= 2'b00;
				end
			end
			endcase
		end
end

// FINT for COCO3
always @ (negedge PH_2) //  or negedge RESET_N)
begin
// TIMER fint for COCO3
// output	TMR_FINT
// State		TMR_FINT_SM
// input		TIMER_N
// switch	FIRQ_TMR

		if(FIRQ_TMR == 1'b0)		// disabled
		begin
			TMR_FINT <= 1'b1;			// no int
			TMR_FINT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (TMR_FINT_SM)
			2'b00:
			begin
				if(!TIMER_N)
				begin
					TMR_FINT <= 1'b0;
					TMR_FINT_SM <= 2'b01;
				end
				else
				begin
					TMR_FINT <= 1'b1;			// no int
					TMR_FINT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:0]} == 16'hFF93)
				begin
					TMR_FINT <= 1'b1;
					TMR_FINT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(TIMER_N)
				begin
					TMR_FINT <= 1'b1;
					TMR_FINT_SM <= 2'b00;
				end
			end
			endcase
		end
end

always @ (negedge PH_2) //  or negedge RESET_N)
begin
// H_SYNC fint for COCO3
// output	HBORD_FINT
// State		HBORD_FINT_SM
// input		H_SYNC / H_FLAG
// switch	FIRQ_HBORD

		if(FIRQ_HBORD == 1'b0)		// disabled
		begin
			HBORD_FINT <= 1'b1;			// no int
			HBORD_FINT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (HBORD_FINT_SM)
			2'b00:
			begin
				if(~H_SYNC & H_FLAG)		// 1 = int
				begin
					HBORD_FINT <= 1'b0;
					HBORD_FINT_SM <= 2'b01;
				end
				else
				begin
					HBORD_FINT <= 1'b1;			// no int
					HBORD_FINT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:0]} == 16'HFF93)
				begin
					HBORD_FINT <= 1'b1;
					HBORD_FINT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if((H_SYNC & H_FLAG))
				begin
					HBORD_FINT <= 1'b1;
					HBORD_FINT_SM <= 2'b00;
				end
			end
			endcase
		end
end

always @ (negedge PH_2) //  or negedge RESET_N)
begin
// V_SYNC int for COCO3
// Start of Sync is a 1 to 0
// output	VBORD_FINT
// State		VBORD_FINT_SM
// input		V_SYNC
// switch	FIRQ_VBORD

		if(FIRQ_VBORD == 1'b0)		// disabled
		begin
			VBORD_FINT <= 1'b1;			// no int
			VBORD_FINT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (VBORD_FINT_SM)
			2'b00:
			begin
				if(~V_SYNC)
				begin
					VBORD_FINT <= 1'b0;
					VBORD_FINT_SM <= 2'b01;
				end
				else
				begin
					VBORD_FINT <= 1'b1;			// no int
					VBORD_FINT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:0]} == 16'HFF93)
				begin
					VBORD_FINT <= 1'b1;
					VBORD_FINT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(V_SYNC)
				begin
					VBORD_FINT <= 1'b1;
					VBORD_FINT_SM <= 2'b00;
				end
			end
			endcase
		end
end

always @ (negedge PH_2) //  or negedge RESET_N)
begin
// Keyboard int for COCO3
// output	KEY_FINT
// State		KEY_FINT_SM
// input		KEY_INT_RAW
// switch	FIRQ_KEY

		if(FIRQ_KEY == 1'b0)		// disabled
		begin
			KEY_FINT <= 1'b1;			// no int
			KEY_FINT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (KEY_FINT_SM)
			2'b00:
			begin
				if(~KEY_INT_RAW)
				begin
					KEY_FINT <= 1'b0;
					KEY_FINT_SM <= 2'b01;
				end
				else
				begin
					KEY_FINT <= 1'b1;			// no int
					KEY_FINT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:0]} == 16'HFF93)
				begin
					KEY_FINT <= 1'b1;
					KEY_FINT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(KEY_INT_RAW)
				begin
					KEY_FINT <= 1'b1;
					KEY_FINT_SM <= 2'b00;
				end
			end
			endcase
		end
end

always @ (negedge PH_2) //  or negedge RESET_N)
begin
// CART (Serial HDD) int for COCO3
// output	CAR_FINT
// State		CAR_FINT_SM
// input		CART_FINT
// switch	FIRQ_CART

		if(FIRQ_CART == 1'b0)		// disabled
		begin
			CAR_FINT <= 1'b1;			// no int
			CAR_FINT_SM <= 2'b10;
		end
		else								// enabled
		begin
			case (CAR_FINT_SM)
			2'b00:
			begin
				if(~CART_IRQ)
				begin
					CAR_FINT <= 1'b0;
					CAR_FINT_SM <= 2'b01;
				end
				else
				begin
					CAR_FINT <= 1'b1;			// no int
					CAR_FINT_SM <= 2'b00;
				end
			end
			2'b01:
			begin
				if({ADDRESS[15:0]} == 16'HFF93)
				begin
					CAR_FINT <= 1'b1;
					CAR_FINT_SM <= 2'b10;
				end
			end
			2'b10:
			begin
				if(CART_IRQ)
				begin
					CAR_FINT <= 1'b1;
					CAR_FINT_SM <= 2'b00;
				end
			end
			endcase
		end
end

// Keyboard Interrupt
assign KEY_INT_RAW = (KEYBOARD_IN == 8'hFF)			?	1'b1:
																		1'b0;

// The Cart interrupts had to be modified because they are not self clearing
assign CPU_IRQ =  (H_SYNC_IRQ_N & V_SYNC_IRQ_N)
					&	(~GIME_IRQ  | (TMR_INT  & HBORD_INT  & VBORD_INT  & KEY_INT  & (!IRQ_CART | CART_IRQ)));
assign CPU_FIRQ = (CART1_INT)
					&	(~GIME_FIRQ | (TMR_FINT & HBORD_FINT & VBORD_FINT & KEY_FINT & (!FIRQ_CART | CART_IRQ)));

//Swap the DW and RS232 ports on connectors
`ifdef RS232PAK
assign UART51_RXD =	(~SWITCH[9])	?	OPTRXD:						// Switch 9 off
													DE1RXD;						// Switch 9 on
assign UART50_RXD =	(~SWITCH[9])	?	DE1RXD:						// Switch 9 off
													OPTRXD;						// Switch 9 on
assign DE1TXD =		(~SWITCH[9])	?	UART50_TXD:					// Switch 9 off
													UART51_TXD;					// Switch 8 on
assign OPTTXD =		(~SWITCH[9])	?	UART51_TXD:					// Switch 9 off
													UART50_TXD;					// Switch 8 on

`else
assign UART50_RXD =	(~SWITCH[9])	?	DE1RXD:						// Switch 9 off
													OPTRXD;						// Switch 9 on
assign DE1TXD =		(~SWITCH[9])	?	UART50_TXD:					// Switch 9 off
													1'b1;							// Switch 8 on
assign OPTTXD =		(~SWITCH[9])	?	1'b1:							// Switch 9 off
													UART50_TXD;					// Switch 8 on
`endif

// CoCo3 Programmable timer interrupt
// Strech Timer interrupt for at least 4 CPU clock cycles
assign TIMER_R = !(TIMER == 13'h0000);
always @(negedge PH_2 or negedge TIMER_R)
begin
	if(!TIMER_R)
	begin
		TIMER_STATE <= 2'b00;
		TIMER_N <= 1'b1;
	end
	else
	begin
		case (TIMER_STATE)
		2'b11:
		begin
			TIMER_N <= 1'b1;
		end
		default:
		begin
			TIMER_STATE <= TIMER_STATE + 1'b1;
			TIMER_N <= 1'b0;
		end
		endcase;
	end
end

assign TMR_CLK = ~TIMER_INS	?	(H_SYNC | H_FLAG):
											DIV_14;					// 50 MHz / 14 = 3.57 MHz
assign CLK3_57MHZ = DIV_14;
always @ (negedge CLK50MHZ or negedge RESET_N)
begin
	if(~RESET_N)
		DIV_7 <= 3'b000;
	else
	case (DIV_7)
	3'b110:
	begin
		DIV_7 <= 3'b000;
		DIV_14 <= !DIV_14;
	end
	default:
		DIV_7 <= DIV_7 + 1'b1;
	endcase
end

always @(negedge TMR_CLK)
begin
	if(~TMR_ENABLE)
	begin
		BLINK <= 1'b1;
		TIMER <= 13'h1FFF;
	end
	else
		case (TIMER)
		13'h0000:
		begin
			BLINK <= ~BLINK;
			TIMER <= 13'h1FFF;
		end
		13'h1FFF: 												//Maybe this should be 1XXX
		begin
// This turns out being TIMER + 1 as in Sockmaster's GIME Reference 1987 GIME
// 0 to TIMER-1 (0 to TIMER-1 is really TIMER counts) + 1 (this is clock going from 0 to 1FFF) = 0 to TIMER+1
				TIMER <= {1'b0,TMR_MSB,TMR_LSB} - 1'b1;
		end
		default:
			TIMER <= TIMER - 1'b1;
		endcase
end

// Most of the latches for settings
always @ (negedge PH_2 or negedge RESET_N)
begin
	if(~RESET_N)
	begin
// FF00
		DD_REG1 <= 8'h00;
// FF01
		HSYNC_INT <= 1'b0;
		HSYNC_POL <= 1'b0;
		DDR1 <= 1'b0;
		SEL[0] <= 1'b0;
// FF02
		DD_REG2 <= 8'h00;
		KEY_COLUMN <= 8'h00;
// FF03
		VSYNC_INT <= 1'b0;
		VSYNC_POL <= 1'b0;
		DDR2 <= 1'b0;
		SEL[1] <= 1'b0;
// FF20
		DD_REG3 <= 8'h00;
		DTOA_CODE <= 6'b000000;
		SOUND_DTOA <= 6'b000000;
//		BBTXD <= 1'b0;
// FF21
		CD_INT <= 1'b0;
		CD_POL <= 1'b0;
		DDR3 <= 1'b0;
		CAS_MTR <= 1'b0;
// FF22
		DD_REG4 <= 8'h00;
		SBS <= 1'b0;
		CSS <= 1'b0;
		VDG_CONTROL <= 4'b0000;
// FF23
		CART_INT <= 1'b0;
		CART_POL <= 1'b0;
		DDR4 <= 1'b0;
		SOUND_EN <= 1'b0;
// FF7A
		ORCH_LEFT <= 8'b10000000;
// FF7B
		ORCH_RIGHT <= 8'b10000000;
// FF7C
		ORCH_LEFT_EXT <= 8'b10000000;
// FF7D
		ORCH_RIGHT_EXT <= 8'b10000000;
// FF70-FF72
		GART_WRITE <= 19'h00000;
// FF74-FF76
		GART_READ <= 19'h00000;
// FF77
		GART_INC <= 2'b1;
// FF7F
		W_PROT <= 2'b11;
		MPI_SCS <= SWITCH[2:1];
		MPI_CTS <= SWITCH[2:1];
// FF90
		ROM <= 2'b00;
		ST_SCS <= 1'b0;
		VEC_PAG_RAM <= 1'b0;
		GIME_FIRQ <= 1'b0;
		GIME_IRQ <= 1'b0;
		MMU_EN <= 1'b0;
		COCO1 <= 1'b0;
// FF91
		TIMER_INS <= 1'b0;
		MMU_TR <= 1'b0;
// FF92
		IRQ_TMR <= 1'b0;
		IRQ_HBORD <= 1'b0;
		IRQ_VBORD <= 1'b0;
//		IRQ_SERIAL <= 1'b0;
		IRQ_KEY <= 1'b0;
		IRQ_CART <= 1'b0;
// FF93
		FIRQ_TMR <= 1'b0;
		FIRQ_HBORD <= 1'b0;
		FIRQ_VBORD <= 1'b0;
//		FIRQ_SERIAL <= 1'b0;
		FIRQ_KEY <= 1'b0;
		FIRQ_CART <= 1'b0;
// FF94
		TMR_MSB <= 4'h0;
		TMR_ENABLE <= 1'b0;
// FF95
		TMR_LSB <= 8'h00;
// FF98
		GRMODE <= 1'b0;
		DESCEN <= 1'b0;
		MONO <= 1'b0;
		LPR <= 3'b000;
// FF99
		HLPR <= 1'b0;
		LPF <= 2'b00;
		HRES <= 4'b0000;
		CRES <= 2'b00;
// FF9A
//		BDR_PAL <= 12'h000;
// FF9C
		VERT_FIN_SCRL <= 4'h0;
// FF9D
		SCRN_START_MSB <= 8'h00;
// FF9E
		SCRN_START_LSB <= 8'h00;
// FF9F
		HVEN <= 1'b0;
		HOR_OFFSET <= 7'h00;
// FFA0
		SAM00 <= 6'h00;
// FFA1
		SAM01 <= 6'h00;
// FFA2
		SAM02 <= 6'h00;
// FFA3
		SAM03 <= 6'h00;
// FFA4
		SAM04 <= 6'h00;
// FFA5
		SAM05 <= 6'h00;
// FFA6
		SAM06 <= 6'h00;
// FFA7
		SAM07 <= 6'h00;
// FFA8
		SAM10 <= 6'h00;
// FFA9
		SAM11 <= 6'h00;
// FFAA
		SAM12 <= 6'h00;
// FFAB
		SAM13 <= 6'h00;
// FFAC
		SAM14 <= 6'h00;
// FFAD
		SAM15 <= 6'h00;
// FFAE
		SAM16 <= 6'h00;
// FFAF
		SAM17 <= 6'h00;
// FFB0
		PALETTE[0] <= 12'h0000;
// FFB1
		PALETTE[1] <= 12'h0000;
// FFB2
		PALETTE[2] <= 12'h000;
// FFB3
		PALETTE[3] <= 12'h000;
// FFB4
		PALETTE[4] <= 12'h000;
// FFB5
		PALETTE[5] <= 12'h000;
// FFB6
		PALETTE[6] <= 12'h000;
// FFB7
		PALETTE[7] <= 12'h000;
// FFB8
		PALETTE[8] <= 12'h000;
// FFB9
		PALETTE[9] <= 12'h000;
// FFBA
		PALETTE[10] <= 12'h000;
// FFBB
		PALETTE[11] <= 12'h000;
// FFBC
		PALETTE[12] <= 12'h000;
// FFBD
		PALETTE[13] <= 12'h000;
// FFBE
		PALETTE[14] <= 12'h000;
// FFBF
		PALETTE[15] <= 12'h000;
// FFC0 / FFC1
		V[0] <= 1'b0;
// FFC2 / FFC3
		V[1] <= 1'b0;
// FFC4 / FFC5
		V[2] <= 1'b0;
// FFC6 / FFC7
		VERT[0] <= 1'b0;
// FFC8 / FFC9
		VERT[1] <= 1'b0;
// FFCA / FFCB
		VERT[2] <= 1'b0;
// FFCC / FFCD
		VERT[3] <= 1'b0;
// FFCE / FFCF
		VERT[4] <= 1'b0;
// FFD0 / FFD1
		VERT[5] <= 1'b0;
// FFD2 / FFD3
		VERT[6] <= 1'b0;
// FFD4
//		PALETTE_BANK <= 4'h0;
// FFD8 / FFD9
		RATE <= 1'b0;
// FFDE / FFDF
		RAM <= 1'b0;
	end
	else
	begin
		if(~RW_N)
		begin
			case (ADDRESS)
			16'hFF00:
			begin
				if(~DDR1)
					DD_REG1 <= DATA_OUT;
			end
			16'hFF01:
			begin
					HSYNC_INT <= DATA_OUT[0];
					HSYNC_POL <= DATA_OUT[1];
					DDR1 <= DATA_OUT[2];
					SEL[0] <= DATA_OUT[3];
			end
			16'hFF02:
			begin
				if(~DDR2)
					DD_REG2 <= DATA_OUT;
				else
					KEY_COLUMN <= DATA_OUT;
			end
			16'hFF03:
			begin
				VSYNC_INT <= DATA_OUT[0];
				VSYNC_POL <= DATA_OUT[1];
				DDR2 <= DATA_OUT[2];
				SEL[1] <= DATA_OUT[3];
			end
			16'hFF04:
			begin
				if(~DDR1)
					DD_REG1 <= DATA_OUT;
			end
			16'hFF05:
			begin
					HSYNC_INT <= DATA_OUT[0];
					HSYNC_POL <= DATA_OUT[1];
					DDR1 <= DATA_OUT[2];
					SEL[0] <= DATA_OUT[3];
			end
			16'hFF06:
			begin
				if(~DDR2)
					DD_REG2 <= DATA_OUT;
				else
					KEY_COLUMN <= DATA_OUT;
			end
			16'hFF07:
			begin
				VSYNC_INT <= DATA_OUT[0];
				VSYNC_POL <= DATA_OUT[1];
				DDR2 <= DATA_OUT[2];
				SEL[1] <= DATA_OUT[3];
			end
			16'hFF08:
			begin
				if(~DDR1)
					DD_REG1 <= DATA_OUT;
			end
			16'hFF09:
			begin
					HSYNC_INT <= DATA_OUT[0];
					HSYNC_POL <= DATA_OUT[1];
					DDR1 <= DATA_OUT[2];
					SEL[0] <= DATA_OUT[3];
			end
			16'hFF0A:
			begin
				if(~DDR2)
					DD_REG2 <= DATA_OUT;
				else
					KEY_COLUMN <= DATA_OUT;
			end
			16'hFF0B:
			begin
				VSYNC_INT <= DATA_OUT[0];
				VSYNC_POL <= DATA_OUT[1];
				DDR2 <= DATA_OUT[2];
				SEL[1] <= DATA_OUT[3];
			end
			16'hFF0C:
			begin
				if(~DDR1)
					DD_REG1 <= DATA_OUT;
			end
			16'hFF0D:
			begin
					HSYNC_INT <= DATA_OUT[0];
					HSYNC_POL <= DATA_OUT[1];
					DDR1 <= DATA_OUT[2];
					SEL[0] <= DATA_OUT[3];
			end
			16'hFF0E:
			begin
				if(~DDR2)
					DD_REG2 <= DATA_OUT;
				else
					KEY_COLUMN <= DATA_OUT;
			end
			16'hFF0F:
			begin
				VSYNC_INT <= DATA_OUT[0];
				VSYNC_POL <= DATA_OUT[1];
				DDR2 <= DATA_OUT[2];
				SEL[1] <= DATA_OUT[3];
			end
			16'hFF10:
			begin
				if(~DDR1)
					DD_REG1 <= DATA_OUT;
			end
			16'hFF11:
			begin
					HSYNC_INT <= DATA_OUT[0];
					HSYNC_POL <= DATA_OUT[1];
					DDR1 <= DATA_OUT[2];
					SEL[0] <= DATA_OUT[3];
			end
			16'hFF12:
			begin
				if(~DDR2)
					DD_REG2 <= DATA_OUT;
				else
					KEY_COLUMN <= DATA_OUT;
			end
			16'hFF13:
			begin
				VSYNC_INT <= DATA_OUT[0];
				VSYNC_POL <= DATA_OUT[1];
				DDR2 <= DATA_OUT[2];
				SEL[1] <= DATA_OUT[3];
			end
			16'hFF14:
			begin
				if(~DDR1)
					DD_REG1 <= DATA_OUT;
			end
			16'hFF15:
			begin
					HSYNC_INT <= DATA_OUT[0];
					HSYNC_POL <= DATA_OUT[1];
					DDR1 <= DATA_OUT[2];
					SEL[0] <= DATA_OUT[3];
			end
			16'hFF16:
			begin
				if(~DDR2)
					DD_REG2 <= DATA_OUT;
				else
					KEY_COLUMN <= DATA_OUT;
			end
			16'hFF17:
			begin
				VSYNC_INT <= DATA_OUT[0];
				VSYNC_POL <= DATA_OUT[1];
				DDR2 <= DATA_OUT[2];
				SEL[1] <= DATA_OUT[3];
			end
			16'hFF18:
			begin
				if(~DDR1)
					DD_REG1 <= DATA_OUT;
			end
			16'hFF19:
			begin
					HSYNC_INT <= DATA_OUT[0];
					HSYNC_POL <= DATA_OUT[1];
					DDR1 <= DATA_OUT[2];
					SEL[0] <= DATA_OUT[3];
			end
			16'hFF1A:
			begin
				if(~DDR2)
					DD_REG2 <= DATA_OUT;
				else
					KEY_COLUMN <= DATA_OUT;
			end
			16'hFF1B:
			begin
				VSYNC_INT <= DATA_OUT[0];
				VSYNC_POL <= DATA_OUT[1];
				DDR2 <= DATA_OUT[2];
				SEL[1] <= DATA_OUT[3];
			end
			16'hFF1C:
			begin
				if(~DDR1)
					DD_REG1 <= DATA_OUT;
			end
			16'hFF1D:
			begin
					HSYNC_INT <= DATA_OUT[0];
					HSYNC_POL <= DATA_OUT[1];
					DDR1 <= DATA_OUT[2];
					SEL[0] <= DATA_OUT[3];
			end
			16'hFF1E:
			begin
				if(~DDR2)
					DD_REG2 <= DATA_OUT;
				else
					KEY_COLUMN <= DATA_OUT;
			end
			16'hFF1F:
			begin
				VSYNC_INT <= DATA_OUT[0];
				VSYNC_POL <= DATA_OUT[1];
				DDR2 <= DATA_OUT[2];
				SEL[1] <= DATA_OUT[3];
			end
			16'hFF20:
			begin
				if(~DDR3)
					DD_REG3 <= DATA_OUT;
				else
				begin
					DTOA_CODE <= DATA_OUT[7:2];
					if({SOUND_EN,SEL} == 3'b100)
						SOUND_DTOA <= DATA_OUT[7:2];
	//				BBTXD <= DATA_OUT[1];
				end
			end
			16'hFF21:
			begin
				CD_INT <= DATA_OUT[0];
				CD_POL <= DATA_OUT[1];
				DDR3 <= DATA_OUT[2];
				CAS_MTR <= DATA_OUT[3];
			end
			16'hFF22:
			begin
				if(~DDR4)
					DD_REG4 <= DATA_OUT;
				else
					SBS <= DATA_OUT[1];
					CSS <= DATA_OUT[3];
					VDG_CONTROL <= DATA_OUT[7:4];
			end
			16'hFF23:
			begin
				CART_INT <= DATA_OUT[0];
				CART_POL <= DATA_OUT[1];
				DDR4 <= DATA_OUT[2];
				SOUND_EN <= DATA_OUT[3];
			end
			16'hFF24:
			begin
				if(~DDR3)
					DD_REG3 <= DATA_OUT;
				else
				begin
					DTOA_CODE <= DATA_OUT[7:2];
					if({SOUND_EN,SEL} == 3'b100)
						SOUND_DTOA <= DATA_OUT[7:2];
	//				BBTXD <= DATA_OUT[1];
				end
			end
			16'hFF25:
			begin
				CD_INT <= DATA_OUT[0];
				CD_POL <= DATA_OUT[1];
				DDR3 <= DATA_OUT[2];
				CAS_MTR <= DATA_OUT[3];
			end
			16'hFF26:
			begin
				if(~DDR4)
					DD_REG4 <= DATA_OUT;
				else
					SBS <= DATA_OUT[1];
					CSS <= DATA_OUT[3];
					VDG_CONTROL <= DATA_OUT[7:4];
			end
			16'hFF27:
			begin
				CART_INT <= DATA_OUT[0];
				CART_POL <= DATA_OUT[1];
				DDR4 <= DATA_OUT[2];
				SOUND_EN <= DATA_OUT[3];
			end
			16'hFF28:
			begin
				if(~DDR3)
					DD_REG3 <= DATA_OUT;
				else
				begin
					DTOA_CODE <= DATA_OUT[7:2];
					if({SOUND_EN,SEL} == 3'b100)
						SOUND_DTOA <= DATA_OUT[7:2];
	//				BBTXD <= DATA_OUT[1];
				end
			end
			16'hFF29:
			begin
				CD_INT <= DATA_OUT[0];
				CD_POL <= DATA_OUT[1];
				DDR3 <= DATA_OUT[2];
				CAS_MTR <= DATA_OUT[3];
			end
			16'hFF2A:
			begin
				if(~DDR4)
					DD_REG4 <= DATA_OUT;
				else
					SBS <= DATA_OUT[1];
					CSS <= DATA_OUT[3];
					VDG_CONTROL <= DATA_OUT[7:4];
			end
			16'hFF2B:
			begin
				CART_INT <= DATA_OUT[0];
				CART_POL <= DATA_OUT[1];
				DDR4 <= DATA_OUT[2];
				SOUND_EN <= DATA_OUT[3];
			end
			16'hFF2C:
			begin
				if(~DDR3)
					DD_REG3 <= DATA_OUT;
				else
				begin
					DTOA_CODE <= DATA_OUT[7:2];
					if({SOUND_EN,SEL} == 3'b100)
						SOUND_DTOA <= DATA_OUT[7:2];
	//				BBTXD <= DATA_OUT[1];
				end
			end
			16'hFF2D:
			begin
				CD_INT <= DATA_OUT[0];
				CD_POL <= DATA_OUT[1];
				DDR3 <= DATA_OUT[2];
				CAS_MTR <= DATA_OUT[3];
			end
			16'hFF2E:
			begin
				if(~DDR4)
					DD_REG4 <= DATA_OUT;
				else
					SBS <= DATA_OUT[1];
					CSS <= DATA_OUT[3];
					VDG_CONTROL <= DATA_OUT[7:4];
			end
			16'hFF2F:
			begin
				CART_INT <= DATA_OUT[0];
				CART_POL <= DATA_OUT[1];
				DDR4 <= DATA_OUT[2];
				SOUND_EN <= DATA_OUT[3];
			end
			16'hFF30:
			begin
				if(~DDR3)
					DD_REG3 <= DATA_OUT;
				else
				begin
					DTOA_CODE <= DATA_OUT[7:2];
					if({SOUND_EN,SEL} == 3'b100)
						SOUND_DTOA <= DATA_OUT[7:2];
	//				BBTXD <= DATA_OUT[1];
				end
			end
			16'hFF31:
			begin
				CD_INT <= DATA_OUT[0];
				CD_POL <= DATA_OUT[1];
				DDR3 <= DATA_OUT[2];
				CAS_MTR <= DATA_OUT[3];
			end
			16'hFF32:
			begin
				if(~DDR4)
					DD_REG4 <= DATA_OUT;
				else
					SBS <= DATA_OUT[1];
					CSS <= DATA_OUT[3];
					VDG_CONTROL <= DATA_OUT[7:4];
			end
			16'hFF33:
			begin
				CART_INT <= DATA_OUT[0];
				CART_POL <= DATA_OUT[1];
				DDR4 <= DATA_OUT[2];
				SOUND_EN <= DATA_OUT[3];
			end
			16'hFF34:
			begin
				if(~DDR3)
					DD_REG3 <= DATA_OUT;
				else
				begin
					DTOA_CODE <= DATA_OUT[7:2];
					if({SOUND_EN,SEL} == 3'b100)
						SOUND_DTOA <= DATA_OUT[7:2];
	//				BBTXD <= DATA_OUT[1];
				end
			end
			16'hFF35:
			begin
				CD_INT <= DATA_OUT[0];
				CD_POL <= DATA_OUT[1];
				DDR3 <= DATA_OUT[2];
				CAS_MTR <= DATA_OUT[3];
			end
			16'hFF36:
			begin
				if(~DDR4)
					DD_REG4 <= DATA_OUT;
				else
					SBS <= DATA_OUT[1];
					CSS <= DATA_OUT[3];
					VDG_CONTROL <= DATA_OUT[7:4];
			end
			16'hFF37:
			begin
				CART_INT <= DATA_OUT[0];
				CART_POL <= DATA_OUT[1];
				DDR4 <= DATA_OUT[2];
				SOUND_EN <= DATA_OUT[3];
			end
			16'hFF38:
			begin
				if(~DDR3)
					DD_REG3 <= DATA_OUT;
				else
				begin
					DTOA_CODE <= DATA_OUT[7:2];
					if({SOUND_EN,SEL} == 3'b100)
						SOUND_DTOA <= DATA_OUT[7:2];
	//				BBTXD <= DATA_OUT[1];
				end
			end
			16'hFF39:
			begin
				CD_INT <= DATA_OUT[0];
				CD_POL <= DATA_OUT[1];
				DDR3 <= DATA_OUT[2];
				CAS_MTR <= DATA_OUT[3];
			end
			16'hFF3A:
			begin
				if(~DDR4)
					DD_REG4 <= DATA_OUT;
				else
					SBS <= DATA_OUT[1];
					CSS <= DATA_OUT[3];
					VDG_CONTROL <= DATA_OUT[7:4];
			end
			16'hFF3B:
			begin
				CART_INT <= DATA_OUT[0];
				CART_POL <= DATA_OUT[1];
				DDR4 <= DATA_OUT[2];
				SOUND_EN <= DATA_OUT[3];
			end
			16'hFF3C:
			begin
				if(~DDR3)
					DD_REG3 <= DATA_OUT;
				else
				begin
					DTOA_CODE <= DATA_OUT[7:2];
					if({SOUND_EN,SEL} == 3'b100)
						SOUND_DTOA <= DATA_OUT[7:2];
	//				BBTXD <= DATA_OUT[1];
				end
			end
			16'hFF3D:
			begin
				CD_INT <= DATA_OUT[0];
				CD_POL <= DATA_OUT[1];
				DDR3 <= DATA_OUT[2];
				CAS_MTR <= DATA_OUT[3];
			end
			16'hFF3E:
			begin
				if(~DDR4)
					DD_REG4 <= DATA_OUT;
				else
					SBS <= DATA_OUT[1];
					CSS <= DATA_OUT[3];
					VDG_CONTROL <= DATA_OUT[7:4];
			end
			16'hFF3F:
			begin
				CART_INT <= DATA_OUT[0];
				CART_POL <= DATA_OUT[1];
				DDR4 <= DATA_OUT[2];
				SOUND_EN <= DATA_OUT[3];
			end
			16'hFF70:
			begin
				GART_WRITE[18:16] <= DATA_OUT[2:0];
			end
			16'hFF71:
			begin
				GART_WRITE[15:8] <= DATA_OUT;
			end
			16'hFF72:
			begin
				GART_WRITE[7:0] <= DATA_OUT;
			end
			16'hFF73:
			begin
				if(GART_INC[0])
					GART_WRITE <= GART_WRITE + 1'b1;
			end
			16'hFF74:
			begin
				GART_READ[18:16] <= DATA_OUT[2:0];
			end
			16'hFF75:
			begin
				GART_READ[15:8] <= DATA_OUT;
			end
			16'hFF76:
			begin
				GART_READ[7:0] <= DATA_OUT;
			end
			16'hFF77:
			begin
				GART_INC <= DATA_OUT[1:0];
			end
			16'hFF7A:
				ORCH_LEFT <= DATA_OUT;
			16'hFF7B:
				ORCH_RIGHT <= DATA_OUT;
			16'hFF7C:
				ORCH_LEFT_EXT <= DATA_OUT;
			16'hFF7D:
				ORCH_RIGHT_EXT <= DATA_OUT;
			16'hFF7F:
			begin
				W_PROT[0] <=  DATA_OUT[2] | ~DATA_OUT[3];
				W_PROT[1] <= ~DATA_OUT[2] |  DATA_OUT[3] | W_PROT[0];
				MPI_SCS <= DATA_OUT[1:0];
				MPI_CTS <= DATA_OUT[5:4];
			end
			16'hFF90:
			begin
				ROM <= DATA_OUT[1:0];
				ST_SCS <= DATA_OUT[2];
				VEC_PAG_RAM <= DATA_OUT[3];
				GIME_FIRQ <= DATA_OUT[4];
				GIME_IRQ <= DATA_OUT[5];
				MMU_EN <= DATA_OUT[6];
				COCO1 <= DATA_OUT[7];
			end
			16'hFF91:
			begin
				TIMER_INS <= DATA_OUT[5];
				MMU_TR <= DATA_OUT[0];
			end
			16'hFF92:
			begin
				IRQ_TMR <= DATA_OUT[5];
				IRQ_HBORD <= DATA_OUT[4];
				IRQ_VBORD <= DATA_OUT[3];
//				IRQ_SERIAL <= DATA_OUT[2];
				IRQ_KEY <= DATA_OUT[1];
				IRQ_CART <= DATA_OUT[0];
			end
			16'hFF93:
			begin
				FIRQ_TMR <= DATA_OUT[5];
				FIRQ_HBORD <= DATA_OUT[4];
				FIRQ_VBORD <= DATA_OUT[3];
//				FIRQ_SERIAL <= DATA_OUT[2];
				FIRQ_KEY <= DATA_OUT[1];
				FIRQ_CART <= DATA_OUT[0];
			end
			16'hFF94:
			begin
				TMR_MSB <= DATA_OUT[3:0];
				TMR_ENABLE <= 1'b1;
			end
			16'hFF95:
			begin
				TMR_LSB <= DATA_OUT;
			end
			16'hFF98:
			begin
				GRMODE <= DATA_OUT[7];
				HRES[3] <= DATA_OUT[6];	// Extended resolutions
				DESCEN <= DATA_OUT[5];
				MONO <= DATA_OUT[4];
				LPR <= DATA_OUT[2:0];
			end
			16'hFF99:
			begin
				HLPR <= DATA_OUT[7];
				LPF <= DATA_OUT[6:5];
				HRES[2:0] <= DATA_OUT[4:2];
				CRES <= DATA_OUT[1:0];
			end
			16'hFF9A:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[16][5:0] <= DATA_OUT[5:0];
					PALETTE[16][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[16][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFF9C:
			begin
				VERT_FIN_SCRL <= DATA_OUT[3:0];
			end
			16'hFF9D:
			begin
				SCRN_START_MSB <= DATA_OUT;
			end
			16'hFF9E:
			begin
				SCRN_START_LSB <= DATA_OUT;
			end
			16'hFF9F:
			begin
				HVEN <= DATA_OUT[7];
				HOR_OFFSET <= DATA_OUT[6:0];
			end
			16'hFFA0:
			begin
				SAM00 <= DATA_OUT[5:0];
			end
			16'hFFA1:
			begin
				SAM01 <= DATA_OUT[5:0];
			end
			16'hFFA2:
			begin
				SAM02 <= DATA_OUT[5:0];
			end
			16'hFFA3:
			begin
				SAM03 <= DATA_OUT[5:0];
			end
			16'hFFA4:
			begin
				SAM04 <= DATA_OUT[5:0];
			end
			16'hFFA5:
			begin
				SAM05 <= DATA_OUT[5:0];
			end
			16'hFFA6:
			begin
				SAM06 <= DATA_OUT[5:0];
			end
			16'hFFA7:
			begin
				SAM07 <= DATA_OUT[5:0];
			end
			16'hFFA8:
			begin
				SAM10 <= DATA_OUT[5:0];
			end
			16'hFFA9:
			begin
				SAM11 <= DATA_OUT[5:0];
			end
			16'hFFAA:
			begin
				SAM12 <= DATA_OUT[5:0];
			end
			16'hFFAB:
			begin
				SAM13 <= DATA_OUT[5:0];
			end
			16'hFFAC:
			begin
				SAM14 <= DATA_OUT[5:0];
			end
			16'hFFAD:
			begin
				SAM15 <= DATA_OUT[5:0];
			end
			16'hFFAE:
			begin
				SAM16 <= DATA_OUT[5:0];
			end
			16'hFFAF:
			begin
				SAM17 <= DATA_OUT[5:0];
			end
			16'hFFB0:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[0][5:0] <= DATA_OUT[5:0];
					PALETTE[0][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[0][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFB1:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[1][5:0] <= DATA_OUT[5:0];
					PALETTE[1][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[1][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFB2:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[2][5:0] <= DATA_OUT[5:0];
					PALETTE[2][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[2][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFB3:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[3][5:0] <= DATA_OUT[5:0];
					PALETTE[3][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[3][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFB4:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[4][5:0] <= DATA_OUT[5:0];
					PALETTE[4][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[4][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFB5:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[5][5:0] <= DATA_OUT[5:0];
					PALETTE[5][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[5][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFB6:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[6][5:0] <= DATA_OUT[5:0];
					PALETTE[6][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[6][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFB7:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[7][5:0] <= DATA_OUT[5:0];
					PALETTE[7][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[7][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFB8:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[8][5:0] <= DATA_OUT[5:0];
					PALETTE[8][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[8][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFB9:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[9][5:0] <= DATA_OUT[5:0];
					PALETTE[9][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[9][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFBA:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[10][5:0] <= DATA_OUT[5:0];
					PALETTE[10][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[10][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFBB:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[11][5:0] <= DATA_OUT[5:0];
					PALETTE[11][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[11][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFBC:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[12][5:0] <= DATA_OUT[5:0];
					PALETTE[12][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[12][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFBD:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[13][5:0] <= DATA_OUT[5:0];
					PALETTE[13][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[13][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFBE:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[14][5:0] <= DATA_OUT[5:0];
					PALETTE[14][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[14][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFBF:
			begin
				if(!DATA_OUT[7])
				begin
					PALETTE[15][5:0] <= DATA_OUT[5:0];
					PALETTE[15][11:6] <= DATA_OUT[5:0];
				end
				else
				begin
					PALETTE[15][5:0] <= DATA_OUT[5:0];
				end
			end
			16'hFFC0:
			begin
				V[0] <= 1'b0;
			end
			16'hFFC1:
			begin
				V[0] <= 1'b1;
			end
			16'hFFC2:
			begin
				V[1] <= 1'b0;
			end
			16'hFFC3:
			begin
				V[1] <= 1'b1;
			end
			16'hFFC4:
			begin
				V[2] <= 1'b0;
			end
			16'hFFC5:
			begin
				V[2] <= 1'b1;
			end
			16'hFFC6:
			begin
				VERT[0] <= 1'b0;
			end
			16'hFFC7:
			begin
				VERT[0] <= 1'b1;
			end
			16'hFFC8:
			begin
				VERT[1] <= 1'b0;
			end
			16'hFFC9:
			begin
				VERT[1] <= 1'b1;
			end
			16'hFFCA:
			begin
				VERT[2] <= 1'b0;
			end
			16'hFFCB:
			begin
				VERT[2] <= 1'b1;
			end
			16'hFFCC:
			begin
				VERT[3] <= 1'b0;
			end
			16'hFFCD:
			begin
				VERT[3] <= 1'b1;
			end
			16'hFFCE:
			begin
				VERT[4] <= 1'b0;
			end
			16'hFFCF:
			begin
				VERT[4] <= 1'b1;
			end
			16'hFFD0:
			begin
				VERT[5] <= 1'b0;
			end
			16'hFFD1:
			begin
				VERT[5] <= 1'b1;
			end
			16'hFFD2:
			begin
				VERT[6] <= 1'b0;
			end
			16'hFFD3:
			begin
				VERT[6] <= 1'b1;
			end
			16'hFFD8:
			begin
				RATE <= 1'b0;
			end
			16'hFFD9:
			begin
				RATE <= 1'b1;
			end
			16'hFFDE:
			begin
				RAM <= 1'b0;
			end
			16'hFFDF:
			begin
				RAM <= 1'b1;
			end
			endcase
		end
		else
		begin
			case (ADDRESS)
					16'hFF73:
			begin
				if(GART_INC[1])
					GART_READ <= GART_READ + 1'b1;
			end
			endcase
		end
	end
end

// The code for the internal and Orchestra sound
`include "sound.v"
// The code for the paddles
`include "paddles.v"

/*****************************************************************************
* Convert PS/2 keyboard to CoCo keyboard
* Buttons
* 0 left 1
* 1 left 2
* 2 right 2
* 3 right 1
******************************************************************************/
assign KEYBOARD_IN[0] =  ~((~KEY_COLUMN[0] & KEY[0])				// @
								 | (~KEY_COLUMN[1] & KEY[1])				// A
								 | (~KEY_COLUMN[2] & KEY[2])				// B
								 | (~KEY_COLUMN[3] & KEY[3])				// C
								 | (~KEY_COLUMN[4] & KEY[4])				// D
								 | (~KEY_COLUMN[5] & KEY[5])				// E
								 | (~KEY_COLUMN[6] & KEY[6])				// F
								 | (~KEY_COLUMN[7] & KEY[7])				// G
								 | ~P_SWITCH[3]);								// Right Joystick Switch 1
//								 | (~SWITCH[5]     & ~P_SWITCH[3])		// Right Joystick Switch 1
//								 | ( SWITCH[5]     & ~P_SWITCH[0]));	// Left Joystick Switch 1

assign KEYBOARD_IN[1] =	 ~((~KEY_COLUMN[0] & KEY[8])				// H
								 | (~KEY_COLUMN[1] & KEY[9])				// I
								 | (~KEY_COLUMN[2] & KEY[10])				// J
								 | (~KEY_COLUMN[3] & KEY[11])				// K
								 | (~KEY_COLUMN[4] & KEY[12])				// L
								 | (~KEY_COLUMN[5] & KEY[13])				// M
								 | (~KEY_COLUMN[6] & KEY[14])				// N
								 | (~KEY_COLUMN[7] & KEY[15])				// O
								 | ~P_SWITCH[0]);								// Left Joystick Switch 1
//								 | (~SWITCH[5]     & ~P_SWITCH[0])		// Left Joystick Switch 1
//								 | ( SWITCH[5]     & ~P_SWITCH[3]));	// Right Joystick Switch 1

assign KEYBOARD_IN[2] =	 ~((~KEY_COLUMN[0] & KEY[16])				// P
								 | (~KEY_COLUMN[1] & KEY[17])				// Q
								 | (~KEY_COLUMN[2] & KEY[18])				// R
								 | (~KEY_COLUMN[3] & KEY[19])				// S
								 | (~KEY_COLUMN[4] & KEY[20])				// T
								 | (~KEY_COLUMN[5] & KEY[21])				// U
								 | (~KEY_COLUMN[6] & KEY[22])				// V
								 | (~KEY_COLUMN[7] & KEY[23])				// W
								 | ~P_SWITCH[2]);								// Left Joystick Switch 2
//								 | (~SWITCH[5]     & ~P_SWITCH[2])		// Left Joystick Switch 2
//								 | ( SWITCH[5]     & ~P_SWITCH[1]));	// Right Joystick Switch 2

assign KEYBOARD_IN[3] =	 ~((~KEY_COLUMN[0] & KEY[24])				// X
								 | (~KEY_COLUMN[1] & KEY[25])				// Y
								 | (~KEY_COLUMN[2] & KEY[26])				// Z
								 | (~KEY_COLUMN[3] & KEY[27])				// up
								 | (~KEY_COLUMN[4] & KEY[28])				// down
								 | (~KEY_COLUMN[5] & KEY[29])				// Backspace & left
								 | (~KEY_COLUMN[6] & KEY[30])				// right
								 | (~KEY_COLUMN[7] & KEY[31])				// space
								 | ~P_SWITCH[1]);								// Right Joystick Switch 2
//								 | (~SWITCH[5]     & ~P_SWITCH[1])		// Right Joystick Switch 2
//								 | ( SWITCH[5]     & ~P_SWITCH[2]));	// Left Joystick Switch 2

assign KEYBOARD_IN[4] =	 ~((~KEY_COLUMN[0] & KEY[32])				// 0
								 | (~KEY_COLUMN[1] & KEY[33])				// 1
								 | (~KEY_COLUMN[2] & KEY[34])				// 2
								 | (~KEY_COLUMN[3] & KEY[35])				// 3
								 | (~KEY_COLUMN[4] & KEY[36])				// 4
								 | (~KEY_COLUMN[5] & KEY[37])				// 5
								 | (~KEY_COLUMN[6] & KEY[38])				// 6
								 | (~KEY_COLUMN[7] & KEY[39]));			// 7

assign KEYBOARD_IN[5] =	 ~((~KEY_COLUMN[0] & KEY[40])				// 8
								 | (~KEY_COLUMN[1] & KEY[41])				// 9
								 | (~KEY_COLUMN[2] & KEY[42])				// :
								 | (~KEY_COLUMN[3] & KEY[43])				// ;
								 | (~KEY_COLUMN[4] & KEY[44])				// ,
								 | (~KEY_COLUMN[5] & KEY[45])				// -
								 | (~KEY_COLUMN[6] & KEY[46])				// .
								 | (~KEY_COLUMN[7] & KEY[47]));			// /

assign KEYBOARD_IN[6] =	 ~((~KEY_COLUMN[0] & KEY[48])				// CR
								 | (~KEY_COLUMN[1] & KEY[49])				// TAB
								 | (~KEY_COLUMN[2] & KEY[50])				// ESC
								 | (~KEY_COLUMN[3] & KEY[51])				// ALT
								 | (~KEY_COLUMN[3] & !BUTTON_N[0])		// ALT (Easter Egg)
								 | (~KEY_COLUMN[4] & KEY[52])				// CTRL
								 | (~KEY_COLUMN[4] & !BUTTON_N[0])		// CTRL (Easter Egg)
								 | (~KEY_COLUMN[5] & KEY[53])				// F1
								 | (~KEY_COLUMN[6] & KEY[54])				// F2
								 | (~KEY_COLUMN[7] & KEY[55] & !SHIFT_OVERRIDE)	// shift
								 |	(~KEY_COLUMN[7] & SHIFT));				// Forced Shift

assign KEYBOARD_IN[7] =	 JSTICK;											// Joystick input

// PS2 Keyboard interface
COCOKEY coco_keyboard(
		.RESET_N(RESET_N),
		.CLK50MHZ(CLK50MHZ),
		.SLO_CLK(V_SYNC),
		.PS2_CLK(ps2_clk),
		.PS2_DATA(ps2_data),
		.KEY(KEY),
		.SHIFT(SHIFT),
		.SHIFT_OVERRIDE(SHIFT_OVERRIDE),
		.RESET(RESET)
);
/*****************************************************************************
* Video
******************************************************************************/
// Request for every other line to be black
// Looks more like the original video
always @ (negedge H_SYNC or negedge V_SYNC)
begin
	if(~V_SYNC)
		ODD_LINE <= 1'b0;
	else
		if(!ODD_LINE & SWITCH[3])
			ODD_LINE <= 1'b1;
		else
			ODD_LINE <= 1'b0;
end

// Video DAC
always @ (negedge MCLOCK[0])
begin
	if(ODD_LINE)									// Odd lines Black
	begin
		{RED3, GREEN3, BLUE3, RED2, GREEN2, BLUE2, RED1, GREEN1, BLUE1, RED0, GREEN0, BLUE0} <= 12'h000;
	end
	else
	begin
		if(COLOR[8])
		begin
			case(COLOR[7:6])
			2'b00:
			begin
				{RED3, GREEN3, BLUE3, RED2, GREEN2, BLUE2, RED1, GREEN1, BLUE1, RED0, GREEN0, BLUE0} <= {3'b000, COLOR[5:0], 3'b000};
			end
			2'b01:
			begin
				{RED3, RED2, RED1, RED0}			<= {1'b0, COLOR[5], COLOR[2], 1'b0}	+ {2'b00, COLOR[5], COLOR[2]};
				{GREEN3, GREEN2, GREEN1, GREEN0}	<= {1'b0, COLOR[4], COLOR[1], 1'b0}	+ {2'b00, COLOR[4], COLOR[1]};
				{BLUE3, BLUE2, BLUE1, BLUE0}		<= {1'b0, COLOR[3], COLOR[0], 1'b0}	+ {2'b00, COLOR[3], COLOR[0]};
			end
			2'b10:
			begin
				{RED3, GREEN3, BLUE3, RED2, GREEN2, BLUE2, RED1, GREEN1, BLUE1, RED0, GREEN0, BLUE0} <= {COLOR[5:0], 6'b000000};
			end
			default:
			begin
				{RED3, GREEN3, BLUE3, RED2, GREEN2, BLUE2, RED1, GREEN1, BLUE1, RED0, GREEN0, BLUE0} <= {COLOR[5:0], COLOR[5:0]};
			end
			endcase
		end
		else
		begin
			RED3 <= PALETTE[COLOR[4:0]][11];
			RED2 <= PALETTE[COLOR[4:0]][8];
			RED1 <= PALETTE[COLOR[4:0]][5];
			RED0 <= PALETTE[COLOR[4:0]][2];
			GREEN3 <= PALETTE[COLOR[4:0]][10];
			GREEN2 <= PALETTE[COLOR[4:0]][7];
			GREEN1 <= PALETTE[COLOR[4:0]][4];
			GREEN0 <= PALETTE[COLOR[4:0]][1];
			BLUE3 <=	PALETTE[COLOR[4:0]][9];
			BLUE2 <=	PALETTE[COLOR[4:0]][6];
			BLUE1 <=	PALETTE[COLOR[4:0]][3];
			BLUE0 <=	PALETTE[COLOR[4:0]][0];
		end
	end
end

// Video timing and modes
COCO3VIDEO COCOVID(
	.PIX_CLK(MCLOCK[0]),		//25 MHz = 40 nS
	.RESET_N(RESET_N),
	.COLOR(COLOR),
	.HSYNC(H_SYNC),
	.SYNC_FLAG(H_FLAG),
	.VSYNC(V_SYNC),
	.HBLANKING(HBLANK),
	.VBLANKING(VBLANK),
	.RAM_ADDRESS(VIDEO_ADDRESS),
	.RAM_DATA(VIDEO_BUFFER),
	.COCO(COCO1),
	.V(V),
	.BP(GRMODE),
	.VERT(VERT),
	.VID_CONT(VDG_CONTROL),
	.HVEN(HVEN),
	.HOR_OFFSET(HOR_OFFSET),
	.SCRN_START_MSB(SCRN_START_MSB),
	.SCRN_START_LSB(SCRN_START_LSB),
 	.CSS(CSS),
	.LPF(LPF),
	.VERT_FIN_SCRL(VERT_FIN_SCRL),
	.HLPR(HLPR & !SWITCH[3]),
	.LPR(LPR),
	.HRES(HRES),
	.CRES(CRES),
	.BLINK(BLINK),
	.SWITCH5(SWITCH[5])
);

// RS232PAK UART
`ifdef RS232PAK
glb6551 RS232(
.RESET_N(RESET_N),
.RX_CLK(RX_CLK2),
.RX_CLK_IN(COM2_STATE[0]),
.XTAL_CLK_IN(COM2_STATE[0]),
.PH_2(PH_2),
.DI(DATA_OUT),
.DO(DATA_RS232),
.IRQ(SER_IRQ),
.CS({1'b0, RS232_EN}),
.RW_N(RW_N),
.RS(ADDRESS[1:0]),
.TXDATA_OUT(UART51_TXD),
.RXDATA_IN(UART51_RXD),
.RTS(UART51_RTS),
.CTS(UART51_RTS),
.DCD(UART51_DTR),
.DTR(UART51_DTR),
.DSR(UART51_DTR)
);
`endif

/*****************************************************************************
* SD Card interface
******************************************************************************/
SDCard SPI_09(
	.clk_i(MCLOCK[0]),		// 25 MHz Clock means 12.5 MHz SDCard transfers
	.cpuclk_n_i(PH_2),		// CPU Clock
	.reset_n_i(RESET_N),		// reset (asynchronous active low)
	.cs_i(SPI_EN),				// chip select
	.adr_i(ADDRESS[0]),		// address[0]
	.rw_n_i(RW_N),				// write enable
	.dat_i(DATA_OUT),			// data input
	.dat_o(SPI_DATA),			// data output
	.irq_n_o(IRQ_SPI_N),		// irq output (low active)
	.halt_o(SPI_HALT),
  // SPI port
	.act_led_n_o(act_led_n),
	.card_detect_n_i(BUTTON_N[2] & !SWITCH[6]),
	.wp_locked_i(BUTTON_N[1] & !SWITCH[6]),
	.spi_ss_n_o(SPI_SS_N),	// SPI Chip Select
	.sclk_o(SPI_CLK),			// serial clock output
	.mosi_o(MOSI),				// MasterOut SlaveIN
	.miso_i(MISO));			// MasterIn SlaveOut

`ifdef SD_DEBUG
//circuitry for SD card software debug
always @(negedge PH_2 or negedge RESET_N)
begin
	if(!RESET_N)
	begin
		SPI_OUT <= 8'h00;
		SPI_IN <= 8'h00;
	end
	else
	begin
		if(SPI_EN)
		begin
			case (RW_N)
			1'b0:
				SPI_OUT <= DATA_OUT;
			1'b1:
				SPI_IN <= SPI_DATA;
			endcase;
		end
		if(SPI_TRACE)
		begin
		   SPI_T <= DATA_OUT;
		end
	end
end
`endif
endmodule
