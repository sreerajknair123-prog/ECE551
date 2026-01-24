module Segway_tb();
			
//// Interconnects to DUT/support defined as type wire /////
wire SS_n,SCLK,MOSI,MISO,INT;				// to inertial sensor
wire A2D_SS_n,A2D_SCLK,A2D_MOSI,A2D_MISO;	// to A2D converter
wire RX_TX;
wire PWM1_rght, PWM2_rght, PWM1_lft, PWM2_lft;
wire piezo,piezo_n;
wire cmd_sent;
wire rst_n;					// synchronized global reset

////// Stimulus is declared as type reg ///////
reg clk, RST_n;
reg [7:0] cmd;				// command host is sending to DUT
reg send_cmd;				// asserted to initiate sending of command
reg signed [15:0] rider_lean;
reg [11:0] ld_cell_lft, ld_cell_rght,steerPot,batt;	// A2D values
reg OVR_I_lft, OVR_I_rght;

///// Internal registers for testing purposes??? /////////


////////////////////////////////////////////////////////////////
// Instantiate Physical Model of Segway with Inertial sensor //
//////////////////////////////////////////////////////////////	
SegwayModel iPHYS(.clk(clk),.RST_n(RST_n),.SS_n(SS_n),.SCLK(SCLK),
                  .MISO(MISO),.MOSI(MOSI),.INT(INT),.PWM1_lft(PWM1_lft),
				  .PWM2_lft(PWM2_lft),.PWM1_rght(PWM1_rght),
				  .PWM2_rght(PWM2_rght),.rider_lean(rider_lean));				  

/////////////////////////////////////////////////////////
// Instantiate Model of A2D for load cell and battery //
///////////////////////////////////////////////////////
ADC128S_FC iA2D(.clk(clk),.rst_n(RST_n),.SS_n(A2D_SS_n),.SCLK(A2D_SCLK),
             .MISO(A2D_MISO),.MOSI(A2D_MOSI),.ld_cell_lft(ld_cell_lft),.ld_cell_rght(ld_cell_rght),
			 .steerPot(steerPot),.batt(batt));			
	 
////// Instantiate DUT ////////
Segway iDUT(.clk(clk),.RST_n(RST_n),.INERT_SS_n(SS_n),.INERT_MOSI(MOSI),
            .INERT_SCLK(SCLK),.INERT_MISO(MISO),.INERT_INT(INT),.A2D_SS_n(A2D_SS_n),
			.A2D_MOSI(A2D_MOSI),.A2D_SCLK(A2D_SCLK),.A2D_MISO(A2D_MISO),
			.PWM1_lft(PWM1_lft),.PWM2_lft(PWM2_lft),.PWM1_rght(PWM1_rght),
			.PWM2_rght(PWM2_rght),.OVR_I_lft(OVR_I_lft),.OVR_I_rght(OVR_I_rght),
			.piezo_n(piezo_n),.piezo(piezo),.RX(RX_TX));

//// Instantiate UART_tx (mimics command from BLE module) //////
UART_tx iTX(.clk(clk),.rst_n(rst_n),.TX(RX_TX),.trmt(send_cmd),.tx_data(cmd),.tx_done(cmd_sent));

/////////////////////////////////////
// Instantiate reset synchronizer //
///////////////////////////////////
rst_synch iRST(.clk(clk),.RST_n(RST_n),.rst_n(rst_n));

initial begin
  
  /// Your magic goes here ///

  clk = 0;
  RST_n = 0;
  OVR_I_lft = 0;
  OVR_I_rght = 0;
  rider_lean = 16'h0;
  @(posedge clk);
  @(negedge clk);
  RST_n = 1;
  ld_cell_lft = 12'd0000; // Trial test
  ld_cell_rght = 12'd0000; // Trial test
  steerPot = 12'h800; // Middle Value
  batt = 12'h1000; // More than
  repeat (50000) @(posedge clk);
  send_cmd = 0;
  repeat (10) @(posedge clk);
  ////////////////////////////////////////////////////////////////
  //////BLE Transmits a Valid 'G' input for Segway Power up///////
  ////////////////////////////////////////////////////////////////
  cmd = 8'h47; //G - Power Up
  send_cmd = 1; //start transmission - A Pulse of trmt signal

  repeat (50000) @(posedge clk);

  ld_cell_lft = 12'd1000; // Trial test
  ld_cell_rght = 12'd1000; // Trial test

  repeat (1) @(posedge clk);
  send_cmd = 0; //deassert trmt after one clock cycle
  @(posedge cmd_sent); // Wait for Transmission complete signal to go further
  if(!iDUT.iAuth.pwr_up)
      $fatal("ERROR !! Power is Not UP when a valid connect Signal comes in");

  repeat (250000) @(posedge clk);

  rider_lean = 16'h0FFF;
  repeat (1000000) @(posedge clk);

  rider_lean = 16'h0;
  repeat (1000000) @(posedge clk);

  $display("Here we go!!!");
  $stop();

end

always
  #10 clk = ~clk;

endmodule	
