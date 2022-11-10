
`timescale 10ns/1ns

// ----------------------------------------------------------------------------
// --- top --------------------------------------------------------------------
// ----------------------------------------------------------------------------
module top (

	input 		hw_clk,
	input 		hw_reset,
	// input [3:0]	hw_KEY,
	// input 		hw_DIP_Switch3,
	// input 		hw_KEY0,
	// input 		hw_KEY1,
	// input 		hw_KEY2,
	// input 		hw_KEY3,
	
	input 		hw_DIG_in,	
	output 		hw_DIG_out2,

	// ram
	output			hw_sdClk_o,
	output			hw_sdCke_o,
	output	[1:0]	hw_sdBs_o,
	output 	[11:0]	hw_sdAddr_o,
	output			hw_sdCe_bo,
	output			hw_sdRas_bo,
	output			hw_sdCas_bo,
	output			hw_sdWe_bo,
	output			hw_sdDqmh_o,
	output			hw_sdDqml_o,
	inout	[15:0]	hw_sdData_io,
	
	// output [6:0] hw_SEG_out,
	// output [2:0] hw_DIG_out,
	output hw_DIG_out0,
	// output hw_DIG_out1,
	
	output			hw_serial_out_ds1302, 		// SCLK
	inout			hw_serial_io_ds1302, 		// DATA
	output			hw_chip_enable_out_ds1302, 	// CE
	
	output [7:0] hw_LEDs
);
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

	// ds 1302
	wire	ds1302_SCLK;
	// wire	ds1302_DATA;
	wire	ds1302_CE;

	wire	ds_trigger;
	wire	ds_clk_out;
	
	assign	hw_serial_out_ds1302		= ds1302_SCLK;
	// assign	hw_serial_io_ds1302			= ds1302_DATA;
	assign	hw_chip_enable_out_ds1302	= ds1302_CE;
	assign 	ds1302_CE 					= ds_trigger;
	assign 	ds1302_SCLK 				= ds_clk_out;
	
	// ram
	wire 		ram_clk	; // ,	// Clock
	wire 		cke		; // ,	// Clock Enable
	// wire 		BA0		; // ,	// Bank select
	// wire 		BA1		; // ,	// Bank select
	wire [11:0]	addr	; // ,	// Address
	wire 		n_cs	; // ,	// Chip Select
	wire 		n_RAS	; // ,
	wire 		n_CAS	; // ,
	wire 		n_WE	; // ,	// Write Enable
	// wire 		UDQM	; // ,
	// wire 		LDQM	; // ,
	// wire [15:0]	DQ		; // ,


	// assign hw_sdBs_o[0] = BA0;
	// assign hw_sdBs_o[1] = BA1;
	assign hw_sdBs_o[0] = 1'b0;
	assign hw_sdBs_o[1] = 1'b0;
	assign hw_sdAddr_o	= addr;
	
	assign hw_sdClk_o = ram_clk;
	assign hw_sdCke_o = cke;
	assign hw_sdCe_bo = n_cs;
	assign hw_sdRas_bo = n_RAS;
	assign hw_sdCas_bo = n_CAS;
	assign hw_sdWe_bo = n_WE;
	
	// assign hw_sdDqmh_o = UDQM;
	// assign hw_sdDqml_o = LDQM;
	assign hw_sdDqmh_o = 1'b0;
	assign hw_sdDqml_o = 1'b0;

	// assign hw_sdData_io	= DQ;

	wire clock;
	wire reset;
	wire pll_reset;
	// wire send;
	// wire rx_in;
	// wire key1;
	wire pll;
	wire pll_notphased;
	wire locked;
	
	reg [7:0] led_reg = 8'b1000_0001;
	
	assign hw_LEDs	= led_reg;

	// wire w_hw_KEY0;
	// wire w_hw_KEY1;
	// wire w_hw_KEY2;
	// wire w_hw_KEY3;

// ----------------------------------------------------------------------------
// `define MODEL_TECH
`ifdef MODEL_TECH

	reg	r_hw_DIP_Switch3 = 1'b1;

	reg r_reset ;
	// reg r_send 	;
	// reg r_rx_in ;
	reg r_clock ;
	
	assign pll_reset = r_reset;

	// assign send = r_send;
	
	// assign rx_in = r_rx_in;
	reg	[15:0 ]r_test_word;
	
	assign w_test_word = r_test_word;
	
	reg r_hw_KEY0;
	reg r_hw_KEY1;
	reg r_hw_KEY2;
	reg r_hw_KEY3;
	
	assign w_hw_KEY0 = r_hw_KEY0;
	assign w_hw_KEY1 = r_hw_KEY1;
	assign w_hw_KEY2 = r_hw_KEY2;
	assign w_hw_KEY3 = r_hw_KEY3;
	
	assign clock = r_clock;
	
	always
		#1 r_clock = !r_clock;
	
	initial begin
		#0 		r_hw_KEY0 = 0;
		// #200	r_hw_KEY0 = 1;
	end
	
	initial begin
		#0 r_hw_KEY1 = 1;
		#427 r_hw_KEY1 = 0;
	end
	
	initial begin
		#0 r_hw_KEY2 = 0;
	end
	
	initial begin
		#0 r_hw_KEY3 = 0;
	end
	
	reg [7:0] 	test_data = 8'hea;	// NOP 0xea

	initial begin
		// #0 		test_data = 8'h00;
		// #192 	test_data = 8'h01;
		// #10		test_data = 8'h02;
		// #10		test_data = 8'h03;
		// #10		test_data = 8'h04;
		// #10		test_data = 8'h05;
		// #10		test_data = 8'hA9;
		// #10		test_data = 8'h00;
		// #10		test_data = 8'hA9;
		// #10		test_data = 8'h5a;
		// #10		test_data = 8'h85;	// sta
		// #10		test_data = 8'hef;
		// #10		test_data = 8'hea;	// nop
		// #10		test_data = 8'hea;	// nop
		// #10		test_data = 8'hea;	// nop
		// #10		test_data = 8'ha2;	// ldx
		// #10		test_data = 8'h55;	// data
		// #10		test_data = 8'hea;	// nop
		// #10		test_data = 8'hea;	// nop
		// #10		test_data = 8'hea;	// nop
		// #10		test_data = 8'hea;	// nop
		// #10		test_data = 8'ha0;	// ldy
		// #10		test_data = 8'h55;	// data
		#0 		test_data = 8'h00;
		// #222 	test_data = 8'h01;
		// #1		test_data = 8'h02;
		// #1		test_data = 8'h03;
		// #1		test_data = 8'h04;
		// #1		test_data = 8'h05;
		// #1		test_data = 8'h06;
		// #1		test_data = 8'h07;
		// #1		test_data = 8'h08;
		// #1		test_data = 8'h09;
		// #1		test_data = 8'h0a;
		// #1		test_data = 8'h0b;
		// #1		test_data = 8'h0c;
		// #1		test_data = 8'h0d;
		// #1		test_data = 8'h0e;
		// #1		test_data = 8'h0f;
		// #1		test_data = 8'h10;
		// #1		test_data = 8'h11;
		// #1		test_data = 8'h12;
		// #1		test_data = 8'h13;
		// #1		test_data = 8'h14;
		// #1		test_data = 8'h15;
		// #1		test_data = 8'h16;
		// #1		test_data = 8'h17;
		// #1		test_data = 8'h18;
		// #1		test_data = 8'h19;
		// #1		test_data = 8'h1a;
		// #1		test_data = 8'h1b;
		// #1		test_data = 8'h1c;
		// #1		test_data = 8'h1d;
		// #1		test_data = 8'h1e;
		// #1		test_data = 8'h1f;
		#338		test_data = 8'haa;
		
	end
	
	initial begin
		// $dumpfile("uart_tb.vcd");
		// $dumpvars(0, top);
	#0 r_clock = 0;
	#0 r_reset = 1;
	// // #0 r_send = 0;
	// // #0 r_rx_in = 1;
	#0 r_test_word = 16'h00;
	
	#10	r_reset = 0;
	// #200 r_test_word = 16'h1234;
		// // #10 r_send  = 1;
		// // #10 r_rx_in	= 0;
		// // #868 r_rx_in	= 1;
		// // #868 r_rx_in	= 0;
		// // #6000 r_rx_in	= 1;
		// // #2500 r_rx_in	= 0;
		// // #868 r_rx_in	= 1;
		// // #868 r_rx_in	= 0;
		// // #868 r_rx_in	= 0;
		// // #868 r_rx_in	= 1;
		// // #868 r_rx_in	= 0;
		// // #3500 r_rx_in	= 1;
		// // #5000 r_rx_in	= 0;
		// $finish;
	end
`else
	assign clock		= hw_clk;
	assign pll_reset 	= !hw_reset;
	// assign w_test_word = test_word;
	// assign w_hw_KEY0 = hw_KEY[0];
	// assign w_hw_KEY1 = hw_KEY[1];
	// assign w_hw_KEY2 = hw_KEY[2];
	// assign w_hw_KEY3 = hw_KEY[3];
	// assign rx_in 	= hw_DIG_in;
	// assign send 	= !hw_KEY[0];
`endif	
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

	// Reset
	assign reset	= !locked;

	// ------------------------------------------------------------------------
	// --- pll ----------------------------------------------------------------
	// ------------------------------------------------------------------------
	// CL3 : 143 Mhz !!!
	// CL2 : 100 Mhz !!!	<--- now
	pll_133 pll_100MHz_cl2 (
		.areset(pll_reset),
		.inclk0(clock),
		.c0(pll),
		.c1(pll_notphased),
		.locked(locked)
	);

	wire 			ram_init_ready;
	wire 			cpu_we_n;

	wire 	[11:0] 	addr_init;
	wire 	[11:0] 	addr_rw;
	wire 	[15:0] 	address_bus;

	wire 	[7:0] 	data_to_cpu;
	wire 	[7:0] 	data_from_cpu;
	
	assign addr = (ram_init_ready == 1'b0) ? addr_init : addr_rw;
	
	wire w_refresh;

    // 6502 registers (MSB) PC, SP, P, Y, X, A (LSB)

	// assign hw_LEDs[1] = w_refresh;

	reg cpuen = 1'b0;
	reg [2:0] cpu_count = 3'b00;
	
	always@(posedge clock) begin
		cpuen <= 1'b0;
		
		if(cpu_count == 5) begin
			cpuen <= 1'b1;
			cpu_count = 3'b000;
		end
		
		cpu_count = cpu_count + 1'b1;
	end
	
`ifdef MODEL_TECH
	wire [7:0] w_reg_a;
	wire [7:0] w_reg_x;
	wire [7:0] w_reg_y;
	wire [7:0] w_reg_p;
	wire [7:0] w_reg_sp;
	wire [15:0] w_reg_pc;
	wire [63:0] w_regs;

	assign w_reg_a	= w_regs[7:0];
	assign w_reg_x	= w_regs[15:8];
	assign w_reg_y	= w_regs[23:16];
	assign w_reg_p	= w_regs[31:24];
	assign w_reg_sp	= w_regs[39:32];
	assign w_reg_pc	= w_regs[55:40];
	
	assign enable = (r_hw_DIP_Switch3) ? 1'b1 : ~w_refresh;
`else	
	// wire enable;
	// assign enable = (hw_DIP_Switch3) ? 1'b1 : ~w_refresh;
`endif

	// ------------------------------------------------------------------------
	// --- CPU ----------------------------------------------------------------
	// ------------------------------------------------------------------------
	T65 cpu1(
		// .Enable 		(1'b1),				//	=> '1',
		.Enable 		(cpuen),			//	=> '1',
		.Mode 			(2'b00),			//	=> "00",
		.Res_n 			(ram_init_ready),	//	=> n_reset,
		.Clk 			(clock),		//	=> cpuClock,
		.Rdy 			(1'b1),				//	=> '1',
		.Abort_n 		(1'b1),				//	=> '1',
		.IRQ_n 			(1'b1),				//	=> '1',
		.SO_n 			(1'b1),				//	=> '1',
		
// `define NMI
`ifdef NMI		
		.NMI_n 			(w_refresh),		//	=> '1',
`else		
		.NMI_n 			(1'b1),		//	=> '1',
`endif		
`ifdef MODEL_TECH		
		.Regs			(w_regs),
`endif
		.R_W_n 			(cpu_we_n),			//	=> n_WR,
		.addr			(address_bus),		//	=> cpuAddress,
		.DI 			(data_to_cpu),		//	=> cpuDataIn,
		.DO 			(data_from_cpu)		//	=> cpuDataOut
	);

	// ------------------------------------------------------------------------
	// --- NMI refresh
	// ------------------------------------------------------------------------
	// refresh refresh_i (
		// .clk(clock),
		// .reset(reset),
		// .q(w_refresh)
	// );
	
	wire	selector_rom;
	wire	selector_ram_ext;
	wire	selector_ram_int;
	wire	selector_uart;
	wire	selector_ds1302;

	// ------------------------------------------------------------------------
	// --- SELECCTOR RAM / ROM
	// ------------------------------------------------------------------------
	ram_rom_select ram_rom_select_inst (

		.addr(address_bus),
		.rom(selector_rom),
		.ram_int(selector_ram_int),
		.uart(selector_uart),
		.ds1302(selector_ds1302),
		.ram(selector_ram_ext)
	);
	
`ifdef MODEL_TECH
	wire 	[3:0]	select;
	
	assign select = { selector_uart, selector_ram_ext, selector_ram_int, selector_rom };
/*
 * easy to read names in simulator output
 */
	reg [8*6-1:0] select_memory;

always @*
    case( select ) 
		4'b0001 : select_memory = "ROM";
		4'b0010 : select_memory = "INT";
		4'b0100 : select_memory = "EXT";
		4'b1000 : select_memory = "UART";
		default: select_memory = "XXX";
	endcase
`endif		

	wire [7:0] data_from_rom;
	
	// ------------------------------------------------------------------------
	// --- ROM
	// ------------------------------------------------------------------------
	rom rom_inst (
		.clk(clock),
		.addr_a(address_bus[12:0]), 	//addr_b,
		.data(8'h00),
		.waddr_a(13'h0000),
		.we_a(1'b0),
		// .clk(pll_notphased),
		.q_a(data_from_rom)	//, q_b
	);
	
	wire [7:0]	ram_intern_out;
	
	// ------------------------------------------------------------------------
	// --- RAM INTERN
	// ------------------------------------------------------------------------
	single_clock_wr_ram ram_intern (
		.q(ram_intern_out),					// output reg [7:0] q,
		.d(data_from_cpu),					// input [7:0] d,
		.write_address(address_bus[11:0]), 		// input [11:0] write_address, read_address,
		.read_address(address_bus[11:0]),	
		.we(~cpu_we_n && selector_ram_int),						// input we, clk
		// .clk(pll_notphased)
		.clk(clock)
	);

	wire 			bbb;
	wire 			ccc;
	wire			serialClock;
	wire	[7:0]	data_from_uart; //  = 8'haa;
	
	assign bbb = ~selector_uart || cpuen || cpu_we_n;
	assign ccc = ~selector_uart || cpuen || ~cpu_we_n;

	// ------------------------------------------------------------------------
	// --- UART
	// ------------------------------------------------------------------------
	bufferedUART bufferedUART_i (
		.clk 		(clock),			// => clk,
		// .clk 		(clk_10MHz),	// => clk,
		.n_wr 		(bbb),				// => bbb,
		.n_rd 		(ccc),				// => ccc,
		.n_int 		(),					// => n_int1,
		.regSel 	(address_bus[0]),	// => cpuAddress(0),
		.dataIn 	(data_from_cpu),	// => cpuDataOut,
		.dataOut 	(data_from_uart),	// => interface1DataOut,
		.rxClock 	(serialClock),		// => serialClock,
		.txClock 	(serialClock),		// => serialClock,
		.rxd 		(hw_DIG_in),		// => rxd1,
		.txd 		(hw_DIG_out2),		// => txd1,
		.n_cts 		(1'b0),				// => '0',
		.n_dcd 		(1'b0),				// => '0',
		.n_rts 		(),					// => rts1,
		.status0	(),					// => open,
		.status1	()					// => open
	);

	reg [15:0] 	serialClkCount;
	
	assign serialClock = serialClkCount[15];
	// ------------------------------------------------------------------------
	// --- serialClock --------------------------------------------------------
	// ------------------------------------------------------------------------
	always@(posedge clock or posedge reset) begin
		if(reset)
			serialClkCount <= 16'd0;
		else
			serialClkCount <= serialClkCount + 16'd2416;
	end

	wire	init_n_cs;
    wire	init_n_RAS;
    wire	init_n_CAS;
    wire	init_n_WE;
	
	wire	rw_n_cs;
    wire	rw_n_RAS;
    wire	rw_n_CAS;
    wire	rw_n_WE;
	
	assign n_cs		= (ram_init_ready) ? rw_n_cs : init_n_cs;
	assign n_RAS	= (ram_init_ready) ? rw_n_RAS : init_n_RAS;
	assign n_CAS	= (ram_init_ready) ? rw_n_CAS : init_n_CAS;
	assign n_WE		= (ram_init_ready) ? rw_n_WE : init_n_WE;

	// ------------------------------------------------------------------------
	// --- SDRAM INIT
	// ------------------------------------------------------------------------
	sdram_init sdram_init_i (
	
		// inputs
		.sys_clk(),
		.pll(pll),
		.reset(reset),

		// outputs
		.ready(ram_init_ready),
		
		// RAM Connections
		.clk(ram_clk),
		.cke(cke),
		.addr(addr_init),
		.n_cs	(init_n_cs),
		.n_RAS	(init_n_RAS),
		.n_CAS	(init_n_CAS),
		.n_WE	(init_n_WE)
	);

	wire [7:0] data_from_ram;
	wire [7:0] data_from_ds3102;

	wire	[7:0] ds1302_status;
	
	assign ds1302_status = { 7'b_0000_000, ds_trigger };

//`define SIM_RAM
`ifdef SIM_RAM
	assign data_to_cpu = data_from_ram;
`else
	assign data_to_cpu = 
	(address_bus == 16'hffef)	? 	ds1302_status:
		selector_ds1302		? 	data_from_ds3102 : 
		selector_uart		? 	data_from_uart : 
		selector_rom		? 	data_from_rom : 
		selector_ram_int	? 	ram_intern_out : 
							data_from_ram;
`endif

	// helper
	wire 	[7:0]	data_for_sdram;

	assign data_for_sdram = (selector_ram_ext && !cpu_we_n) ? data_from_cpu : 8'hz;

`ifdef MODEL_TECH
	// inout from / to sdram
	// hw_sdData_io
	reg 	[15:0] 	r_sd_data = 16'h5a5a;
	wire 	[15:0] 	w_sd_data;

	assign w_sd_data = (cpu_we_n && selector_ram_ext) ? r_sd_data : 16'bz;
`endif	
	

	// ------------------------------------------------------------------------
	// --- SDRAM READ and WRITE -----------------------------------------------
	// ------------------------------------------------------------------------
	sdram_read_write sdram_read_write_inst (
		// in
		.sys_clk(clock),
		.ena(cpuen),
		.pll(pll),
		.reset(~ram_init_ready),
		.cs_in(selector_ram_ext),
		.R_nW_in(cpu_we_n && selector_ram_ext),
		.addr_in(address_bus),
		.data_in(data_for_sdram),
		
		// out
		.data_out(data_from_ram),
		
		// RAM
		.n_cs	(rw_n_cs),	// Chip Select
		.n_RAS	(rw_n_RAS),
		.n_CAS	(rw_n_CAS),
		.n_WE	(rw_n_WE),	// Write Enable
		.addr	(addr_rw),
		// .DQ(DQ)
		`ifdef MODEL_TECH
				.DQ(w_sd_data)
		`else		
				.DQ(hw_sdData_io)
		`endif		
	);

	assign 	ds1302_CE 					= ds_trigger;
	assign 	ds1302_SCLK 				= ds_clk_out;

	// wire ds_Rw;
	// assign ds_Rw = 

	// ------------------------------------------------------------------------
	// --- ds1302 -------------------------------------------------------------
	// ------------------------------------------------------------------------
	ds1302 ds1302_i (
		.clk		(clock),
		.reset		(reset),
		.en			(selector_ds1302 && ~(address_bus == 16'hffef)),
		.Rw			(cpu_we_n),
		.clk_out	(ds_clk_out),
		.addr		(address_bus[7:0]),		// ds1302 addr: 
		.data_in	(data_from_cpu),
		.data_out   (data_from_ds3102),
		// hw
		.trigger	(ds_trigger),
		.ds1302_DATA(hw_serial_io_ds1302)
	);
	
endmodule	// top

// ----------------------------------------------------------------------------
// --- module: ds1302 ---------------------------------------------------------
// ----------------------------------------------------------------------------
module ds1302 (
	input	clk,
	input	reset,
	input	en,
	input	Rw,
	
	input	[7:0]	addr,		// ds1302 addr: 
	input	[7:0]	data_in,
	output	[7:0]	data_out,
	
	output	trigger,
	output	clk_out,
	inout	ds1302_DATA
);

	wire clock;
	
	assign clock	= clk;
	
	// reg [7:0] 	ds_data_byte = 8'b_1000_0001;
	reg [7:0] 	ds_data_byte;
	
	
	reg [7:0] 	ds_data_byte_out;
	reg [7:0] 	ds_addr_byte;
	// assign 	ds_data_byte	= data_in;
	
	reg 		ds_trigger; 
	reg			ds_clk;
	reg			ds_clk_out;
	reg 		ds_Rw;
	reg	[11:0]	ds_counter;
	reg	[4:0]	ds_bitcount;
	wire	[2:0]	ds_bitcount_mask;
	reg	[3:0]	ds_bitcount2;
	reg 		ds_old; 
	// wire 		ds_databit; 
	
	reg 		ds_trigger_in;
	reg 		tr_in_old;
	
	// DATA OUT
	assign data_out = (~ds_trigger && Rw) ? ds_data_byte_out : 8'hz;
	
	assign ds_bitcount_mask = ds_bitcount[2:0];
	
	assign trigger = ds_trigger;
	assign clk_out = ds_clk_out;

	// new trigger at request from enable
	always@(posedge clk) begin
		if(reset) begin
				ds_data_byte	<= 8'hzz;
		end
		else begin
			tr_in_old	<= en;
			if(en && ~tr_in_old) begin
				ds_trigger 		<= 1'b1;
				if(~Rw)
					ds_data_byte	<= data_in;
				else
					ds_data_byte	<= 8'hz;
				ds_addr_byte	<= { 1'b1, 3'b0, addr[2:0], Rw };
				ds_Rw			<= Rw;
			end
		end
	end

	localparam	BITS	= 8'd8 * 2;
	
	// wire	ds_bit_out	= (ds_bitcount < 4'h8) ? ds_data_byte[ds_bitcount] : 8'hz;
	assign ds1302_DATA 	= 
		(ds_bitcount < 4'd8 ) ? ds_addr_byte[ds_bitcount_mask] : 
		(ds_bitcount < 5'd16) ? ds_data_byte[ds_bitcount_mask] :
		1'h0;
	
	
	// assign ds_databit 	= (ds_bitcount < 4'h8) ? ds_data_byte[ds_bitcount] : 8'hz;
	// assign ds1302_DATA 	= ds_databit;
	
	always@(posedge clock or posedge reset) begin
		if(reset) begin
			ds_counter 		= 12'h0;
			ds_clk			= 1'b0;
			ds_clk_out		= 1'b0;
			ds_bitcount		= 1'b0;
			ds_bitcount2	= 1'b0;
			ds_old			= 1'b0;
			ds_trigger		= 1'b0;
		end
		else begin
			ds_old <= ds_clk;
			
			// detect neg. edge for trigger
			if(~ds_clk && ds_old && ~ds_clk_out && ds_bitcount == 0) begin
				// if(en)
					// ds_trigger 	<= 1'b1;
				ds_bitcount <= 1'b0;
			end
			
			// detect neg. edge for bitcount
			if(~ds_clk && ds_old) begin
				if(ds_trigger && ~ds_clk_out) begin
					ds_bitcount <= ds_bitcount + 1'b1;
				end
			end
			
			// detect pos. edge
			if(ds_clk && ~ds_old) begin
				ds_clk_out	= ~ds_clk_out;
				
				if( (ds_bitcount > 7) && ds_Rw && ds_trigger) begin
					ds_data_byte_out[ds_bitcount_mask]	= ds1302_DATA;
				end
				
			end

			// 4 clk pause
			if(ds_bitcount2 == 4) begin
				ds_bitcount 	<= 0;
				ds_bitcount2	= 0;
			end
			
			// clock divider
			if(ds_counter == (((1000 / 40)/2)) ) begin
				ds_counter 	= 12'h0;
				ds_clk		= ~ds_clk;
			
				if(ds_bitcount == BITS) begin
					ds_bitcount2 = ds_bitcount2 +1;
					ds_trigger <= 0;
				end
				
			end
			else
				ds_counter	= ds_counter + 1'd1;
		end
	end
endmodule	// ds1302

// ----------------------------------------------------------------------------
// --- module: refresh --------------------------------------------------------
// ----------------------------------------------------------------------------
module refresh (
	input clk,
	input reset,
	output	q
);

	reg [8:0] count;
	reg [8:0] count2;

	always@(posedge clk or posedge reset) begin
		if(reset) begin
			count = 1'd0;
			count2 = 1'd0;
		end
		else begin
			if(count == 9'h1ff)
				count2 = count2 + 1'd1;
			else
				count = count +1'd1;
				
			if(count2 == 8) begin
				count2 = 0;
				count = count +1'd1;
			end
		end
	end
	
	// assign q = ~count ? 1'b1 : 1'b0;
	assign q = count == 9'h1ff ? 1'b1 : 1'b0;
	
endmodule

// ----------------------------------------------------------------------------
// --- refresh2 ---------------------------------------------------------------
// ----------------------------------------------------------------------------
module refresh2 (
	input clk,
	input reset,
	output	q
);

	reg [8:0] count;

	always@(posedge clk or posedge reset) begin
		if(reset)
			count = 1'd0;
		else begin
			count = count +1'd1;
		end
	end
	
	assign q = ~count ? 1'b1 : 1'b0;
	
endmodule

// ----------------------------------------------------------------------------
// --- addr_refresh -----------------------------------------------------------
// ----------------------------------------------------------------------------
module addr_refresh (
	input clk,
	input reset,
	input ena,
	
	output [15:0] addr
	);
	
	
	reg [15:0] a;
	
	assign addr = a;
	
	always@(posedge clk or posedge reset) begin
		if(reset) begin	
			a = 16'h0000;
		end
		else
		if(ena) begin
			a = a + 16'h0100;
		end
	end
	
endmodule

// ----------------------------------------------------------------------------
// --- SDRAM read write -------------------------------------------------------
// ----------------------------------------------------------------------------
module sdram_read_write (

	// system
	input			sys_clk,
	input			ena,
	input			pll,
	input			reset,
	input			R_nW_in,
	input			cs_in,
	input	[15:0]	addr_in,		// NOW: 64kB
	input	[7:0]	data_in,
	
	output	[7:0]	data_out,
	
	// ram
	// output		 		clk,	// Clock
	// output	reg 		cke,	// Clock Enable
	// output	reg 		BA0,	// Bank select
	// output	reg 		BA1,	// Bank select
	output	[11:0]		addr,		// Address
	
	output		 		n_cs,		// Chip Select
	output		 		n_RAS,
	output		 		n_CAS,
	output		 		n_WE,		// Write Enable
	
	// output	reg 		UDQM,
	// output	reg 		LDQM,
	inout	[15:0]	DQ
	
);
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

	assign DQ 		= (!R_nW_in && cs_in) ? { 8'b00, data_in } : 16'hzz;
	assign data_out = (R_nW_in && cs_in) ? DQ[7:0] : 8'bzz;

	localparam STATE_IDLE		= 4'h0;
	localparam STATE_MODE		= 4'h1;
	localparam STATE_NOP_L		= 4'h2;
	localparam STATE_NOP_L_B1	= 4'h3;
	localparam STATE_NOP_L_B2	= 4'h4;
	localparam STATE_NOP_L_C1	= 4'h5;
	localparam STATE_NOP_L_C2	= 4'h6;
	localparam STATE_BANK		= 4'h7;	// RAS
	localparam STATE_READ		= 4'h8;	// CAS
	localparam STATE_WRITE		= 4'h9;	// CAS	TODO: check the bits for this mode
	localparam STATE_XXX_a		= 4'ha;
	localparam STATE_XXX_b		= 4'ha;
	localparam STATE_XXX_c		= 4'ha;
	localparam STATE_XXX_d		= 4'ha;
	localparam STATE_XXX_e		= 4'ha;
	localparam STATE_XXX_f		= 4'ha;
	
	// ------------------------------------------------------------------------
	// --- Mode Counter -------------------------------------------------------
	// ------------------------------------------------------------------------
	reg [10:0] count1  = 0;
	always@(posedge pll) begin
		if(ena)
			count1 = 1'd1;
		else
			count1 = count1 << 1'd1;
	end
	
	wire [3:0] w_mode1;
	
	// ------------------------------------------------------------------------
	// --- Mode select -------------------------------------------------------
	// ------------------------------------------------------------------------
	// TODO: output ctrl lines only when ext ist select
	mode_sel mode_sel_i (
		.d	(count1),
		.Rw	(R_nW_in),
		.q	(w_mode1)
	);

	wire refresh3;
	
	wire selecter_ras;
	wire selecter_cas;
	
	assign selecter_ras = 	
		(count1 == 11'b00000000010) ? 1'b1 : 
		(count1 == 11'b00000000001) ? 1'b1 :
		1'b0;
		
	assign selecter_cas = (count1 == 11'b00000001000) ? 1'b1 : 1'b0;
	
	assign n_RAS	= (cs_in || refresh3) ? w_mode1[2] : 1'bz;
	assign n_cs		= (cs_in || refresh3) ? w_mode1[3] : 1'bz;
	assign n_CAS	= (cs_in || refresh3) ? w_mode1[1] : 1'bz;
	assign n_WE		= 
		(refresh3) 		? 1'b1 :
		(!cs_in) 		? 1'bz :
		(selecter_cas) 	? R_nW_in : w_mode1[0] ;
	
	wire 	[11:0]	addr_cas;
	wire 	[11:0]	addr_ras;
	
	// wire 	auto_precharge_a10;
	// assign 	auto_precharge_a10 = test_input1;
	
	// assign addr_cas		= { 1'b0, auto_precharge_a10, 2'b0, addr_in[ 7:0] };
	// assign addr_ras		= { 1'b0, auto_precharge_a10, 2'b0, addr_in[15:8] };
	
	reg 	[7:0]	data_out_reg;
	reg 	[7:0]	data_out_reg_NO_C2;
	wire 	[11:0] 	ras_cas_n_addr;
	wire	[11:0]	w_addr;
	wire 	[15:0] 	refresh_addr;
	wire 	[15:0] 	addr_ras_cas;
	
	assign addr 		= (cs_in || refresh3) ? w_addr : 12'hz;
	assign refresh3 	= (~ena && ~cs_in) ? 1'b1 : 1'b0;
	assign addr_ras_cas = ~refresh3 ? addr_in : refresh_addr;
	
	// ------------------------------------------------------------------------
	// --- RAS CAS Select -----------------------------------------------------
	// ------------------------------------------------------------------------
	ras_cas_select ras_cas_select_inst (
		.addr(addr_ras_cas),
		// .ena(r_request_old),
		.sel_ras(selecter_ras),
		.sel_cas(selecter_cas),
		.q(w_addr)
	);

	// ------------------------------------------------------------------------
	// --- addr_refresh -------------------------------------------------------
	// ------------------------------------------------------------------------
	addr_refresh addr_refresh_i ( 
		.clk(sys_clk),
		.reset(reset),
		.ena(ena),
		
		.addr(refresh_addr)
	);
	
`ifdef MODEL_TECH

/*
 * easy to read names in simulator output
 */
reg [8*6-1:0] statenameR;

wire [1:0] m1 = { R_nW_in, cs_in };

always @*
    case( m1 ) 
		2'b11		: statenameR = "READ";
		2'b01		: statenameR = "WRITE";
		default		: statenameR = "NOTEXT";
	endcase
	
`endif	

endmodule

// ----------------------------------------------------------------------------
// --- mode select -------------------------------------------------------------
// ----------------------------------------------------------------------------
module mode_sel (
		input [10:0] d,
		input Rw,
		output [3:0] q
	);

	reg 	[3:0] 	w_mode;
	
	assign q 	= w_mode;

	always@(d) begin
		case (d)
				// Mode: n_cs, n_RAS, n_CAS, n_WE
				11'b000_0000_0001: w_mode = 4'b_zzzz;	//
				11'b000_0000_0010: w_mode = 4'b_0011;	// RAS
				11'b000_0000_0100: w_mode = 4'b_0111;	// NOP
				11'b000_0000_1000: w_mode = 4'b_0101;	// CAS	/ RW
				11'b000_0001_0000: w_mode = 4'b_0111;	// NOP
				11'b000_0010_0000: w_mode = 4'b_0111;	// NOP
				// 11'b000_0100_0000: w_mode = 4'b_0111;	// NOP
				// 11'b000_1000_0000: w_mode = 4'b_0111;	// NOP
				default:	w_mode = 4'b_zzzz;	//
		endcase
	end

endmodule

// ----------------------------------------------------------------------------
// --- ras_cas_select -------------------------------------------------------------
// ----------------------------------------------------------------------------
module ras_cas_select (
	input [15:0] 	addr,
	// input 			ena,
	input 			sel_ras,
	input 			sel_cas,
	
	output	[11:0] q
);

	wire	[11:0]	ras;
	wire	[11:0]	cas;
	
	assign ras = { 4'b0000, addr[15:8] };
	assign cas = { 4'b0100, addr[ 7:0] };

	// assign q = 	(sel_ras && ena) ? ras :
	assign q = 	sel_ras ? ras :
				sel_cas ? cas :
				12'hz;
	
endmodule

// ----------------------------------------------------------------------------
// --- ram_rom_select -------------------------------------------------------------
// ----------------------------------------------------------------------------
module ram_rom_select (

	input	[15:0]	addr,
	output	rom,
	output	ram_int,
	output	uart,
	output	ds1302,
	output	ram
);

	assign uart		= (   addr[15:1 ] == 15'b_1111_1111_1101_000 ) ? 1'b1 : 1'b0;	// FFD0_FFD1	UART
	assign ds1302	= (   addr[15:4 ] == 15'b_1111_1111_1110	 ) ? 1'b1 : 1'b0;	// FFE0-FFEF	ds1302
	assign rom 		= ( ((addr[15:12] == 4'hf) || (addr[15:12] == 4'he)) && ~uart) ? 1'b1 : 1'b0;
	assign ram_int 	= ( ( addr[15:12] == 4'h0) ) ? 1'b1 : 1'b0;
	assign ram 		= ~rom && ~ram_int && ~uart && ~ds1302;
endmodule

// ----------------------------------------------------------------------------
// --- sdram init -------------------------------------------------------------
// ----------------------------------------------------------------------------
module sdram_init (

	// system
	input	sys_clk,
	input	pll,
	input	reset,
	// input	[15:0]	addr_in,		// NOW: 64kB
	output	reg ready,
	
	// ram
	output		 		clk,	// Clock
	output	reg 		cke,	// Clock Enable
	// output	reg 		BA0,	// Bank select
	// output	reg 		BA1,	// Bank select
	output	[11:0]		addr,	// Address
	
	output		 		n_cs,	// Chip Select
	output		 		n_RAS,
	output		 		n_CAS,
	output		 		n_WE	// Write Enable
	
	// output	reg 		UDQM,
	// output	reg 		LDQM,
	// inout	[15:0]	DQ
	
);
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

	reg [15:0]	count;
	reg			init;
	wire [3:0]	mode;
	reg [3:0]	r_mode;
	wire 		r_clk;

	assign n_cs		= r_mode[3];
	assign n_RAS	= r_mode[2];
	assign n_CAS	= r_mode[1];
	assign n_WE		= r_mode[0];

	wire [4:0] initiala;
	reg trigger;

	// --- 
	always@(posedge trigger or posedge reset) begin
		if(reset) begin
		end
		else begin
		end
	end

	assign clk = pll;

	reg [11:0] addr_mode_select	= 12'b_00_0_00_010_0_000;
	
	// assign addr = (r_mode == 4'b_0000) ? addr_mode_select : addr_in;
	assign addr = (r_mode == 4'b_0000) ? addr_mode_select : 12'h0;
	
	// --- 
	always@(posedge pll or posedge reset) begin
	
		if(reset) begin
			count 	<= 0;
			trigger <= 1'b0;
			r_mode	<= 4'b1111;
			ready 	<= 1'b0;
			cke		<= 1'b0;				// <<<--- dont forget to ENABLE !!!!
			// BA0		<= 1'b0;
			// BA1		<= 1'b0;
			// addr	<= 12'b_0000_0000_0000;
			// UDQM		<= 1'b0;
			// LDQM		<= 1'b0;
		end
		else begin
			cke		<= 1'b1;
			if(!ready)
				count <= count +1'b1;
			
			if(init && r_mode == 4'b_0111) begin
				ready	<= 1'b1;
			end
			
			if(initiala) begin
				r_mode <= mode;
				trigger <= 1'b1;
			end
			else
				trigger <= 1'b0;
		end
	end

	always@(posedge pll or posedge ready) begin
		if(ready)
			init <= 1'b0;
		else
			init <= 1'b1;
	end
		
	// if(count == 143) begin			// sim 143   = 1us
	// // if(count == 14300)			// wrk 14300 = 100us

`ifdef MODEL_TECH
	assign initiala[0] = (count == 16'd143) || (count == 16'd146)  || (count == 16'd157) || (count == 16'd168)
		? 1'b1 : 1'b0;												// load nop command
	assign initiala[1] = (count == 16'd144) ? 1'b1 : 1'b0;			// precharge
	assign initiala[2] = (count == 16'd145) || (count == 16'd156) 
		? 1'b1 : 1'b0;												// auto refresh
	assign initiala[3] = (count == 16'd167) ? 1'b1 : 1'b0;			// load Mode command
	assign initiala[4] = (count == 16'd171) ? 1'b1 : 1'b0;			// ready
`else
	assign initiala[0] = (count == 16'd14143) || (count == 16'd14146)  || (count == 16'd14157) || (count == 16'd14168)
		? 1'b1 : 1'b0;												// load nop command
	assign initiala[1] = (count == 16'd14144) ? 1'b1 : 1'b0;			// precharge
	assign initiala[2] = (count == 16'd14145) || (count == 16'd14156) 
		? 1'b1 : 1'b0;												// auto refresh
	assign initiala[3] = (count == 16'd14167) ? 1'b1 : 1'b0;			// load Mode command
	assign initiala[4] = (count == 16'd14171) ? 1'b1 : 1'b0;			// ready
`endif

	// --- 
	sdram_init_mode sdram_init_mode_i (
		.initiala	(initiala),
		.mode		(mode)
	);
		
endmodule

// ----------------------------------------------------------------------------
// --- sdram_init_mode --------------------------------------------------------
// ----------------------------------------------------------------------------
module sdram_init_mode (
	input 	[4:0]	initiala,
	output 	[3:0] 	mode
);
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
	
	reg [3:0] w_mode;
	
	assign mode = w_mode;

	// --- 
	always@(initiala) begin
		case (initiala)
			// Mode: n_cs, n_RAS, n_CAS, n_WE
			5'b_0_0001: w_mode = 4'b_1000;	// nop
			5'b_0_0010: w_mode = 4'b_0010;	// precharge
			5'b_0_0100: w_mode = 4'b_0001;	// auto refresh
			5'b_0_1000: w_mode = 4'b_0000;	// mode
			5'b_1_0000: w_mode = 4'b_0111;	// ready
			default:	w_mode = 4'b_zzzz;	//
		endcase
	end
endmodule

// ----------------------------------------------------------------------------
// --- 10MHz Clock ------------------------------------------------------------
// ----------------------------------------------------------------------------
module freq_div(
	input clk,
	output reg q = 0
);
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

	reg [27:0] count = 0;

	// --- 
	always@(posedge clk) begin
	// if(count == 17_734_475) begin
		if(count == 4) begin			// 10MHz
		// if(count == 8) begin			// 5.xxxMHz
			count <= 0;
			q <= !q;
		end
			else count <= count +1'd1;
	end
endmodule
