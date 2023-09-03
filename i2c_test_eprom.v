//////////////////////////////////////////////////////////////////////////////////
// Main module for i2c_mmaster testing.
// Used EP4CE10E22C8 with AT24C08 EEPROM chip (OMDAZZ RZ301)
//
// Button 4 - read eeprom (sequential)
// Button 3 - read eeprom (1 byte, fixed address)
// Button 2 - write eeprom (sequential)
// Reset    - initialize memory block
//
//////////////////////////////////////////////////////////////////////////////////
module i2c_test_eprom(
   input		      clk,
   input		      reset,
   input		      startr,
   input		      startr1,
   input		      startw,
	inout		      scl,
	inout		      sda
);
// I2C block
wire [7:0]	reg_adr;                // I2C register address (address pointer for the EPROM)
wire [6:0]	dev_adr;                // I2C device address
wire        busy;                   // I2C busy signal
wire  		rw;                     // I2C operation code
reg         ur;                     // I2C use-register vode
wire [15:0] datnum;                 // Number of data bytes
wire        nclk;                   // I2C clock
reg  [15:0]	divider = 16'd31;       // 50MHz clock divider (125 - 100KHz, 31 ~ 400KHz, 12 ~ 1MHz)
wire        enable;                 // Enable I2C operations
wire        dvalid;                 // Received data is valid (read operation)
wire        newdat;                 // New data request (write operation)

// Button signals
wire        start_read;             // Start read operation
wire        start_read1;            // Start read operation
wire        start_write;            // Start write operation
wire        start_init;             // Start init RAM operation
wire        start_reset;            // Start reset operation
wire        read_sig;               // Read button pressed
wire        read1_sig;              // Read button pressed
wire        write_sig;              // Write button pressed
wire        init_sig;               // Init button pressed
wire 			reset_sig;              // Reset button pressed

i2cclock i2c_clk(
	.clk(clk),
   .divider(divider),
	.new_clk(nclk)
);

debounce but_reset(
	.clk(nclk),
	.but(reset),
	.signal(reset_sig),
   .sigup(start_reset)
);

debounce but_read(
	.clk(nclk),
	.but(startr),
	.signal(read_sig),
   .sigup(start_read)
);

debounce but_write(
	.clk(nclk),
	.but(startw),
	.signal(write_sig),
   .sigup(start_write)
);

debounce but_read1(
	.clk(nclk),
	.but(startr1),
	.signal(read1_sig),
   .sigup(start_read1)
);


enable_sig enam(
   .clk(nclk),
   .reset(reset_sig),
   .start(read_sig | read1_sig | write_sig),
   .busy(busy),
   .enable(enable)
);

i2c_mmaster i2c_master_my(
   .clock_i(nclk),
   .reset_i(reset_sig),
   .enable_i(enable),
   .rw_i(rw),
   .ur_i(ur),
   .dat_i(obout),
   .regadr_i(reg_adr),
   .devadr_i(dev_adr),
   .datnum_i(datnum),
   .dat_o(in_data),
   .busy_o(busy),
   .dvalid_o(dvalid),
   .newdat_o(newdat),
   .sda(sda),
   .scl(scl)
);


// RAM block
wire [7:0]	in_data, ibout, obout;  // Data busses
reg  [9:0]  badrw, badrr;           // Address registers
reg  [7:0]  init_data;              // Initialization data
wire        ibclk, obclk;           // Clock signals
wire        ibwn, obwn;             // Write-enable signals
assign ibwn = (op_cycle & op_code)? dvalid : 1'b0;
assign ibclk = nclk;
assign obclk = nclk;

bufram ibuf(
	.address(badrr),
	.clock(ibclk),
	.data(in_data),
	.wren(ibwn),
	.q(ibout)
);

bufram obuf(
	.address(badrw),
	.clock(obclk),
	.data(init_data),
	.wren(obwn),
	.q(obout)
);


// Control block
reg [9:0]   ncountw, ncountr;       // Number of bytes for read/write operations
reg         op_cycle = 1'b0;
reg         op_code = 1'b0;
reg         idle = 1'b0;
wire [9:0]  regw;
reg  [3:0]  func = 4'b0;
reg  [2:0]  busy_wait = 3'b0;
reg         op_cycle_stop = 1'b0;
wire        busy_wait_max = &busy_wait;

assign regw = (op_cycle)? (op_code? badrr : badrw) : 10'b1;
assign datnum = (op_cycle)? (op_code? {6'b0,ncountr} : {6'b0,ncountw}) : 16'b1;
assign dev_adr = {5'b10100,regw[9],regw[8]};
assign reg_adr = regw[7:0];
assign rw = op_code;
assign obwn = func[3] & ~idle & op_cycle;

always @(posedge nclk) begin
   if(start_reset) begin
      func <= 4'b1000;
   end
   else begin
      if(op_cycle_stop)
         func <= 4'b0;
      else
         if(|func == 1'b0)
            func <= {1'b0,start_write,start_read1,start_read};
   end
end

always @(posedge nclk) begin
   if(start_reset) begin
      op_code <= 1'b0;
      op_cycle <= 1'b0;
      idle <= 1'b0;
      busy_wait <= 3'b0;
   end
   else begin
      if(|func == 1'b0)
         op_cycle_stop <= 1'b0;
      else begin
         if(~op_cycle & ~op_cycle_stop) begin
            busy_wait <= 3'b0;
            op_cycle <= 1'b1;
            case(func)
               4'b0100: begin
                  op_code <= 1'b0;
                  badrw <= 10'b0;
                  ncountw <= 10'd16;
               end
               4'b0001: begin
                  op_code <= 1'b1;
                  badrr <= 10'b0;
                  ncountr <= 10'd16;
                  ur <= 1'b0;
               end
               4'b0010: begin
                  op_code <= 1'b1;
                  badrr <= 10'b0;
                  ncountr <= 10'd1;
                  ur <= 1'b1;
               end
               4'b1000: begin
                  badrw <= 10'b0;
                  init_data <= 8'hAA;
                  ncountw <= 10'd1023;
                  op_code <= 1'b0;
                  idle <= 1'b0;
               end
            endcase
         end
         else begin
            if(~busy_wait_max)
               busy_wait <= busy_wait + 1'b1;
            case(func)
               4'b0100: begin
                  if(~busy & busy_wait_max) begin
                     op_cycle <= 1'b0;
                     op_cycle_stop <= 1'b1;
                  end
                  if(newdat) begin
                     badrw <= badrw + 1'b1;
                     ncountw <= ncountw - 1'b1;
                  end
                  if(|ncountw == 1'b0) begin
                     op_cycle <= 1'b0;
                     op_cycle_stop <= 1'b1;
                  end
               end
               4'b0001: begin
                  if(~busy & busy_wait_max) begin
                     op_cycle <= 1'b0;
                     op_cycle_stop <= 1'b1;
                  end
                  if(dvalid) begin
                     badrr <= badrr + 1'b1;
                     ncountr <= ncountr - 1'b1;
                  end
                  if(|ncountr == 1'b0) begin
                     op_cycle <= 1'b0;
                     op_cycle_stop <= 1'b1;
                  end
               end
               4'b0010: begin
                  if(~busy & busy_wait_max) begin
                     op_cycle <= 1'b0;
                     op_cycle_stop <= 1'b1;
                  end
                  if(dvalid) begin
                     badrr <= badrr + 1'b1;
                     ncountr <= ncountr - 1'b1;
                  end
                  if(|ncountr == 1'b0) begin
                     op_cycle <= 1'b0;
                     op_cycle_stop <= 1'b1;
                  end
               end
               4'b1000: begin
                  if(idle)
                     idle <= ~idle;
                  else begin
                     init_data <= init_data + 1'b1;
                     badrw <= badrw + 1'b1;
                     ncountw <= ncountw - 1'b1;
                     idle <= ~idle;
                  end
                  if(|ncountw == 1'b0) begin
                     op_cycle <= 1'b0;
                     op_cycle_stop <= 1'b1;
                  end
               end
            endcase
         end
      end
   end
end

endmodule
