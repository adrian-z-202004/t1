module rom (
	input clk,
	input [12:0] 		addr_a, 			//addr_b,
	input [7:0]			data,	// NOT USED
	input [12:0] 		waddr_a, //NOT USED
	input 				we_a,		// NOT USED
	output reg [7:0] 	q_a					//, q_b
);
	parameter data_width = 8;
	// parameter addr_width = 1024;
	parameter addr_width = 13;
	
	
	// assign data 	= 8'h00;
	// assign waddr_a 	= 16'h0000;
	// assign we_a		= 1'b0;
	
	reg [7:0] rom[2**13-1:0];
	
	initial 	// Read the memory contents in the file
				//dual_port_rom_init.txt.
	begin
		$readmemh("d:/Storm/test.txt", rom);
	end
	
	always @ (posedge clk)
	begin
		q_a <= rom[addr_a];
		// q_b <= rom[addr_b];
	end
endmodule
