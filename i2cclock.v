module i2cclock(
	input          clk,
	input          reset,
   input [15:0]   divider,
	output reg     new_clk
);

reg [15:0]	counter;

always @(posedge clk, posedge reset) begin
	if(reset) begin
		counter <= 16'd0; new_clk <= 1'b0;
	end
	else begin
		if(counter == divider) begin
			new_clk <= ~new_clk;
			counter <= 16'd0;
		end
		else
			counter <= counter + 1'b1;
	end
end

endmodule
