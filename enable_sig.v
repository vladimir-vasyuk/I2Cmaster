module enable_sig(
   input    clk,
   input    reset,
   input    start,
   input    busy,
   output   enable
);


reg prev_start;
reg enar = 1'b0;
assign enable = enar;

always @(posedge clk) begin
	prev_start <= start;
end

wire  start_front_pos = ~prev_start & start;

always @(posedge clk, posedge reset) begin
	if(reset) begin
		enar <= 1'b0;
   end
	else begin
		if(busy) begin
			enar <= 1'b0;
		end
		else
			if (start_front_pos) begin
				enar <= 1'b1;
			end
	end
end

endmodule
