//============================================================================
//
//  JR100_MiSTer: MiSTer framework wrapper (emu) for the JR-100 core.
//
//  Thin adapter around rtl/jr100/jr100_top.sv, which is verified on Mac
//  by instruction-boundary lockstep against pyjr100emu (docs/).
//
//  Copyright (C) 2026 Zabaglione
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_OSD + per-pin push-pull mask, USER_IO widened to 8 bits
	output        USER_OSD,
	output  [7:0] USER_PP,
	input   [7:0] USER_IN,
	output  [7:0] USER_OUT,
	// [MiSTer-DB9 END]

	input         OSD_STATUS
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_PP driven by wrapper; USER_OUT driven by joydb (USER_OUT_DRIVE) below
assign USER_PP = USER_PP_DRIVE;
// [MiSTer-DB9 END]
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 0;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[122:121];

assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

`include "build_id.v"
localparam CONF_STR = {
	"JR100;;",
	"-;",
	"F0,rom,Load BASIC ROM;",
	"F1,prg,Load PRG;",
	"F2,bas,Load BAS;",
	"-;",
	"S0,prg,Mount Save File;",
	"T[3],Save BASIC to file;",
	"-;",
	"S1,cmt,Mount Tape;",
	"T[4],Tape Play;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[8:6],Display color,White,Green,Amber,Cyan,Orange,Blue,Paper,Mint;",
	"O[2],Extended RAM (reset),Off,On;",
	"O[5],Autostart loaded program,No,Yes;",
	"-;",
	// [MiSTer-DB9-Pro BEGIN] - Saturn-first joy_type (canonical bit notation)
	"O[127:126],UserIO Joystick,Off,Saturn,DB9MD,DB15;",
	// [MiSTer-DB9-Pro END]
	"R[0],Reset;",
	"J,Fire;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire  [10:0] ps2_key;
wire  [31:0] joystick_0;

wire        ioctl_download;
wire  [7:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper
wire         CLK_JOY = CLK_50M;                 // Assign clock between 40-50Mhz
wire   [1:0] joy_type_raw    = status[127:126]; // 0=Off, 1=Saturn, 2=DB9MD, 3=DB15
wire         joy_2p          = 1'b0;          // 1P-only: joy_2p unused
// SNAC cores: replace 1'b0 with the core's SNAC enable expression so SNAC
// preempts the joydb wrapper on shared USER_IO pins. Default 1'b0 is no-op.
wire         snac_active     = 1'b0;
// MT32-pi cores on primary USER_IO: replace 1'b0 with the core's MT32-active
// expression (e.g. `mt32_use` under `ifndef SECOND_MT32`, `~mt32_disable` for
// TRS-80's inverted polarity). Suppresses the OSD-open autodetect probe so it
// doesn't read the RPi's I2C master traffic as a ghost Saturn signature.
wire         mt32_primary_active = 1'b0;
wire   [1:0] joy_type        = snac_active ? 2'd0 : joy_type_raw;
wire         joy_db9md_en    = (joy_type == 2'd2);
wire         joy_db15_en     = (joy_type == 2'd3);
wire         joy_any_en      = |joy_type;
// [MiSTer-DB9 END]

// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
wire         saturn_unlocked;                   // driven by hps_io UIO_DB9_KEY (0xFE)
// [MiSTer-DB9-Pro END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper wires + instance
wire   [7:0] USER_OUT_DRIVE;
wire   [7:0] USER_PP_DRIVE;
wire  [15:0] joydb_1, joydb_2;
wire         joydb_1ena, joydb_2ena;
wire  [15:0] joy_raw_payload;
// Programmable-remap matrix (joydb_remap inside joydb): clk_sys carries the
// 0xFD selector load (HPS-bus domain). joydb_*_mapped are the MiSTer-standard
// joystick words the core consumes at its joy0_USB merge point. db9_remap_*
// are driven by the hps_io instance (see the .db9_remap_* bindings there).
// Until a core consumes joydb_*_mapped (Layer B), the matrix stays at identity
// and synthesis prunes it -- binding these is dormant + fleet-safe.
wire  [15:0] joydb_1_mapped, joydb_2_mapped;
wire         db9_remap_cmd;
wire   [5:0] db9_remap_byte_cnt;
wire  [15:0] db9_remap_din;

// [MiSTer-DB9 BEGIN] - DB9 remap factory default (used until Main_MiSTer streams UIO 0xFD)
// Derived from CONF_STR J1, same rule as db9_map.cpp; lets the core work on a stock MiSTer binary.
// DB15:  Fire=A
// DB9MD: Fire=B
wire  [35:0] db9_remap_default_db15  = 36'hFFFFFFFF4;
wire  [35:0] db9_remap_default_db9md = 36'hFFFFFFFF5;
// [MiSTer-DB9 END]
joydb joydb (
  .clk             ( CLK_JOY         ),
  .clk_sys         ( clk_sys            ),
  .USER_IN         ( USER_IN         ),
  .OSD_STATUS          ( OSD_STATUS          ),
  .snac_active         ( snac_active         ),
  .mt32_primary_active ( mt32_primary_active ),
  .joy_type        ( joy_type        ),
  .joy_2p          ( joy_2p          ),
  .saturn_unlocked ( saturn_unlocked ),
  .USER_OUT_DRIVE  ( USER_OUT_DRIVE  ),
  .USER_PP_DRIVE   ( USER_PP_DRIVE   ),
  .USER_OSD        ( USER_OSD        ),
  .joydb_1         ( joydb_1         ),
  .joydb_2         ( joydb_2         ),
  .joydb_1ena      ( joydb_1ena      ),
  .joydb_2ena      ( joydb_2ena      ),
  .remap_cmd       ( db9_remap_cmd      ),
  .remap_byte_cnt  ( db9_remap_byte_cnt ),
  .remap_din       ( db9_remap_din      ),
  .remap_default_db15  ( db9_remap_default_db15  ),
  .remap_default_db9md ( db9_remap_default_db9md ),
  .joydb_1_mapped  ( joydb_1_mapped     ),
  .joydb_2_mapped  ( joydb_2_mapped     ),
  .joy_raw         ( joy_raw_payload )
);

assign USER_OUT = USER_OUT_DRIVE;
// [MiSTer-DB9 END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: rename USB joystick wires
wire [31:0] joystick_0_USB;
// [MiSTer-DB9 END]

hps_io #(.CONF_STR(CONF_STR), .VDNUM(2)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask(0),

	.ps2_key(ps2_key),
	.joystick_0(joystick_0_USB),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(prg_wait | bas_wait),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.sd_lba('{sd_lba, sd1_lba}),
	.sd_blk_cnt('{6'd0, 6'd0}),
	.sd_rd({sd1_rd, 1'b0}),
	.sd_wr({sd1_wr, sd_wr}),
	.sd_ack({sd1_ack, sd_ack}),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_wr(sd_buff_wr),
	.sd_buff_din('{sd_buff_din, sd1_buff_din}),
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joy_raw
	.joy_raw(joy_raw_payload),
	// programmable remap matrix selector load (UIO_DB9_MAP 0xFD)
	.db9_remap_cmd(db9_remap_cmd),
	.db9_remap_byte_cnt(db9_remap_byte_cnt),
	.db9_remap_din(db9_remap_din),
	// [MiSTer-DB9 END]
	// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
	.saturn_unlocked(saturn_unlocked)
	// [MiSTer-DB9-Pro END]
);

// [MiSTer-DB9-Pro BEGIN] - DB controllers muted while OSD is open
// joydb_*_mapped is already in MiSTer-standard J1 order, identical to the USB
// joystick word, so it replaces the USB input as-is.
assign joystick_0 = joydb_1ena ? (OSD_STATUS ? 16'b0 : joydb_1_mapped) : joystick_0_USB;
// [MiSTer-DB9-Pro END]

wire  [1:0] img_mounted;
wire        img_readonly;
wire [63:0] img_size;
wire [31:0] sd_lba;
wire        sd_wr;
wire  [1:0] sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire        sd_buff_wr;
wire  [7:0] sd_buff_din;
wire [31:0] sd1_lba;
wire        sd1_rd;
wire        sd1_wr;
wire  [7:0] sd1_buff_din;

// OSD momentary status bits -> one-clk pulses
reg  save_req, tape_play;
always @(posedge clk_sys) begin
	reg old_save, old_play;
	old_save <= status[3];
	save_req <= ~old_save & status[3];
	old_play <= status[4];
	tape_play <= ~old_play & status[4];
end

///////////////////////   CLOCKS   ///////////////////////////////

// 57.272727 MHz = 4x NTSC colorburst: /8 = 7.159 MHz pixel,
// /64 = 894.886 kHz CPU (AGENTS.md §3.1)
wire clk_sys;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys)
);

wire reset = RESET | status[0] | buttons[1];

///////////////////////   CORE   /////////////////////////////////

// boot.rom (games/JR100/) and the OSD "F0" slot both arrive as
// ioctl index 0: an 8 KiB image, char ROM first (AGENTS.md §7).
wire rom_download = ioctl_download && (ioctl_index[5:0] == 0);
wire loader_we = ioctl_wr && rom_download && (ioctl_addr[26:13] == 0);

wire prg_download = ioctl_download && (ioctl_index[5:0] == 1);
wire prg_wait;

wire bas_download = ioctl_download && (ioctl_index[5:0] == 2);
wire bas_wait;

// MiSTer joystick -> JR-100 CC02 (AGENTS.md §4): bit0=right, bit1=left,
// bit2=up, bit3=down, bit4=fire, all active high, idle = 00.
wire [7:0] joy_status = {3'b000,
                         joystick_0[4],   // fire
                         joystick_0[2],   // down
                         joystick_0[3],   // up
                         joystick_0[1],   // left
                         joystick_0[0]};  // right

wire [44:0] key_matrix;
jr100_keyboard keyboard
(
	.clk(clk_sys),
	.rst(reset),
	.ps2_key(ps2_key),
	.key_matrix(key_matrix)
);

wire pb7, audio;
wire vid_pixel, vid_de, vid_hs, vid_vs;
wire cen_pix;

jr100_top core
(
	.clk         (clk_sys),
	.rst         (reset),
	.downloading (rom_download),
	.cpu_hold    (1'b0),

	.loader_we   (loader_we),
	.loader_addr (ioctl_addr[12:0]),
	.loader_data (ioctl_dout),

	.prg_download (prg_download),
	.prg_wr       (ioctl_wr && prg_download),
	.prg_data     (ioctl_dout),
	.prg_wait     (prg_wait),

	.bas_download (bas_download),
	.bas_wr       (ioctl_wr && bas_download),
	.bas_data     (ioctl_dout),
	.bas_wait     (bas_wait),

	.save_req     (save_req),
	.img_mounted  (img_mounted[0]),
	.img_readonly (img_readonly),
	.img_size     (img_size),
	.sd_lba       (sd_lba),
	.sd_wr        (sd_wr),
	.sd_ack       (sd_ack[0]),
	.sd_buff_addr (sd_buff_addr),
	.sd_buff_din  (sd_buff_din),

	.tape_play    (tape_play),
	.tape_mounted (img_mounted[1]),
	.tape_readonly (img_readonly),
	.tape_size    (img_size),
	.tape_playing (),
	.tape_recording (),
	.sd1_lba      (sd1_lba),
	.sd1_rd       (sd1_rd),
	.sd1_wr       (sd1_wr),
	.sd1_ack      (sd_ack[1]),
	.sd1_buff_din (sd1_buff_din),
	.sd_buff_dout (sd_buff_dout),
	.sd_buff_wr   (sd_buff_wr),

	.autostart_en (status[5]),

	.key_matrix  (key_matrix),
	.joy_status  (joy_status),
	.ext_ram_en  (status[2]),

	.pb7         (pb7),
	.audio       (audio),

	.vid_pixel   (vid_pixel),
	.vid_de      (vid_de),
	.vid_hs      (vid_hs),
	.vid_vs      (vid_vs),
	.vid_hcnt    (),
	.vid_vcnt    (),

	.cen_pix_out (cen_pix),
	.cen_cpu_out (),
	.boundary    (),
	.dbg_pc      (),
	.dbg_sp      (),
	.dbg_ix      (),
	.dbg_a       (),
	.dbg_b       (),
	.dbg_cc      (),
	.dbg_ora     (),
	.dbg_orb     (),
	.dbg_ddra    (),
	.dbg_ddrb    (),
	.dbg_acr     (),
	.dbg_pcr     (),
	.dbg_ifr     (),
	.dbg_ier     (),
	.dbg_sr      (),
	.dbg_t1      (),
	.dbg_t1l     (),
	.dbg_t2      (),
	.dbg_t2l     ()
);

///////////////////////   VIDEO / AUDIO   ////////////////////////

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = cen_pix;

assign VGA_DE = vid_de;
assign VGA_HS = vid_hs;
assign VGA_VS = vid_vs;
// Display colour (OSD): classic monochrome-monitor phosphors. The
// JR-100's optional dedicated monitor (TR-120MIC) was a green display.
reg [23:0] fg_color;
always @(*) begin
	case (status[8:6])
		3'd0: fg_color = 24'hFFFFFF;   // White
		3'd1: fg_color = 24'h33FF33;   // Green (P1)
		3'd2: fg_color = 24'hFFB000;   // Amber (P3)
		3'd3: fg_color = 24'h66FFFF;   // Cyan
		3'd4: fg_color = 24'hFF8020;   // Orange
		3'd5: fg_color = 24'h99BBFF;   // Blue
		3'd6: fg_color = 24'hFFE8C8;   // Paper
		default: fg_color = 24'hCCFFCC; // Mint
	endcase
end

assign VGA_R  = vid_pixel ? fg_color[23:16] : 8'h00;
assign VGA_G  = vid_pixel ? fg_color[15:8]  : 8'h00;
assign VGA_B  = vid_pixel ? fg_color[7:0]   : 8'h00;

// Band-limited Timer 1 square wave (AGENTS.md §3.4)
assign AUDIO_L = {1'b0, audio, 14'd0};
assign AUDIO_R = {1'b0, audio, 14'd0};

assign LED_USER = ioctl_download;

endmodule
