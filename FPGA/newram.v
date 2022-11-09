module single_clock_wr_ram(
	output reg [7:0] q,
	input [7:0] d,
	input [11:0] write_address, read_address,
	input we, clk
);
	// reg [7:0] mem [16384-1:0];
	reg [7:0] mem [4096-1:0];

	initial 	// Read the memory contents in the file
				//dual_port_rom_init.txt.
	begin
		$readmemh("./fillmem.txt", mem);
	end

	
	always @ (posedge clk) begin
		if (we)
			mem[write_address] = d;
		q = mem[read_address]; // q does get d in this clock cycle if
		// we is high
	end
endmodule
