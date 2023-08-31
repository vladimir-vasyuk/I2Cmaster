module i2cclock(
	input          clk,
   input [15:0]   divider,
	output reg     new_clk
);

reg [15:0]	counter;

always @(posedge clk) begin
	if(counter == divider) begin
		new_clk <= ~new_clk;
		counter <= 16'd0;
	end
	else
		counter <= counter + 1'b1;
end

endmodule
