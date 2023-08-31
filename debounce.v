// Синхронизация и антидребезг
module debounce(
	input			clk,
	input			but,
	output reg	signal,
   output      sigup,
   output      sigdn
);

reg [1:0]   insig;
reg [15:0]  cnt;
wire        cnt_max = &cnt;

always @(posedge clk) begin
	insig[0] <= ~but; insig[1] <= insig[0];
end

wire idle = (signal == insig[1]);

always @(posedge clk) begin
   if(idle)
      cnt <= 16'b0;
   else begin
      cnt <= cnt + 1'b1;
      if(cnt_max)
         signal <= ~signal;
   end
end

assign sigdn = ~idle & cnt_max & signal;
assign sigup = ~idle & cnt_max & ~signal;

endmodule
