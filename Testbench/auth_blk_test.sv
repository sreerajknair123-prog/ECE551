/////////////////////////////////////////////////////////////////////////////////////////////
//To check that when the rider jumps off - Segway power is still up and it stays in STANDBY//
/////////////////////////////////////////////////////////////////////////////////////////////
// Checks for lft_spd and right speed during power down - Expected to be Zero ///////////////
/////////////////////////////////////////////////////////////////////////////////////////////
module Segway_tb();

import tb_tasks::*;		
//// Interconnects to DUT/support defined as type wire /////
wire SS_n,SCLK,MOSI,MISO,INT;				// to inertial sensor
wire A2D_SS_n,A2D_SCLK,A2D_MOSI,A2D_MISO;	// to A2D converter
wire RX_TX;
logic PWM1_rght, PWM2_rght, PWM1_lft, PWM2_lft;
wire piezo,piezo_n;
logic cmd_sent;
logic rst_n;					// synchronized global reset

////// Stimulus is declared as type reg ///////
logic clk, RST_n;
logic [7:0] cmd;				// command host is sending to DUT
logic send_cmd;				// asserted to initiate sending of command
logic signed [15:0] rider_lean;
logic [11:0] ld_cell_lft, ld_cell_rght,steerPot,batt;	// A2D values
logic OVR_I_lft, OVR_I_rght;

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
  ///////////////////////////////////////////
  ///////////Initialize the Inputs///////////
  ///////////////////////////////////////////
  RST_n = 0;
  clk = 0;
  OVR_I_lft = 0;
  OVR_I_rght = 0;
  rider_lean = 16'h0;
  @(posedge clk);
  @(negedge clk);
  RST_n = 1;
  ld_cell_lft = 12'd0; 
  ld_cell_rght = 12'd0;
  steerPot = 12'd800; 
  batt = 12'd1000; 
  send_cmd = 0;
  cmd = 0;

////////////////////////////////////////////////////////////////////////////////
/////////Parallel Tasks for rider lean and Auth Block configurations////////////
///////////////////////////////////////////////////////////////////////////////
fork 
    begin
        ///////////////////////////////////////////////////////
        ////////////rider lean step function///////////////////
        ///////////////////////////////////////////////////////
        repeat (150000) @(posedge clk);
        repeat (25000) @(posedge clk);
        if(!iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is Not UP before I am updating the rider lean");
        repeat (250000) @(posedge clk);
        rider_lean = 16'h0FFF;
        repeat (1000000) @(posedge clk);
        rider_lean = 16'h0;
        repeat (1000000) @(posedge clk);
    end
    begin
        ////////////////////////////////////////////////////////////////
        //////BLE Transmits a Valid 'G' input for Segway Power up///////
        ////////////////////////////////////////////////////////////////
        cmd = 8'h47; //G - Power Up
        send_cmd = 1; //start transmission - A Pulse of send_cmd signal
        // rider_off = 0; // Rider is ON
        ld_cell_lft = 12'd350; 
        ld_cell_rght = 12'd350;
        steerPot = 12'd800; 
        batt = 12'd1000; 
        repeat (1) @(posedge clk);
        repeat (50000) @(posedge clk);
        send_cmd = 0; //deassert send_cmd after one clock cycle
        @(posedge cmd_sent); // Wait for Transmission complete signal to go further
        repeat (25000) @(posedge clk);
        if(!iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is Not UP when a valid connect Signal comes in");
        repeat (25000) @(posedge clk);

        //////////////////////////////////////////////////////////////////////////////
        //////BLE Transmits a Valid 'S' input that indicate a Disconnect Signal///////
        //////////////////////////////////////////////////////////////////////////////
        cmd = 8'h53; //S - Towards Shut Down Again
        send_cmd = 1; //start transmission - A Pulse of send_cmd signal
        repeat (1) @(posedge clk);
        repeat (50000) @(posedge clk);
        send_cmd = 0; //deassert send_cmd after one clock cycle
        @(posedge cmd_sent); // Wait for Transmission complete signal to go further
        repeat (25000) @(posedge clk);
        if(!iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is Not UP while Segway is on STANDY and the rider is still not OFF yet");
        repeat (25000) @(posedge clk);

        ///////////////////////////////////////////////////////////////////////////////////////
        //////BLE Transmits a Valid 'G' input Again for Segway Power up////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////
        ////Expected to have a steady power up state as long as long as the rider is still ON//
        cmd = 8'h47; //G - Power Up
        send_cmd = 1; //start transmission - A Pulse of send_cmd signal
        // rider_off = 0; // Rider is ON
        ld_cell_lft = 12'd350; 
        ld_cell_rght = 12'd350;
        steerPot = 12'd800; 
        batt = 12'd1000; 
        repeat (1) @(posedge clk);
        repeat (50000) @(posedge clk);
        send_cmd = 0; //deassert send_cmd after one clock cycle
        @(posedge cmd_sent); // Wait for Transmission complete signal to go further
        repeat (25000) @(posedge clk);
        if(!iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is Not UP when a valid connect Signal comes in");
        repeat (25000) @(posedge clk);

        ///////////////////////////////////////////////////////////////////////////////////
        //////BLE Transmits a Valid 'S' input Again that indicate a Disconnect Signal//////
        ////////////////Expected to be on Standby until the rider is off///////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        cmd = 8'h53; //S - Towards Shut Down Again
        send_cmd = 1; //start transmission - A Pulse of send_cmd signal
        repeat (1) @(posedge clk);
        repeat (50000) @(posedge clk);
        send_cmd = 0; //deassert send_cmd after one clock cycle
        @(posedge cmd_sent); // Wait for Transmission complete signal to go further
        repeat (25000) @(posedge clk);
        if(!iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is Not UP while Segway is on STANDY and the rider is still not OFF yet");
        repeat (25000) @(posedge clk);

        ///////////////////////////////////////////////////////////////////////////////////
        ////////////////Rider Gets Off After a BLE Disconnect - Segway Should Shut down////
        ////////////////////Whenever Rider is Off it should be a ShutDown//////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        // rider_off = 1;
        ld_cell_lft = 12'd0; 
        ld_cell_rght = 12'd0;
        steerPot = 12'd800; 
        batt = 12'd1000;
        repeat (2) @(posedge clk);
        repeat (25000) @(posedge clk);
        if(iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is UP while rider is OFF");
        if((iDUT.iBAL.SegwayMath_inst.lft_spd!=0) | (iDUT.iBAL.SegwayMath_inst.rght_spd))
            $fatal("ERROR !! Left Speed and Right Speed is Not Zero when Power is Not Up");
        repeat (10000) @(posedge clk);
        repeat (25000) @(posedge clk);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////
        //////BLE Transmits a Valid 'G' input for Segway Power up///////
        ////////////////////////////////////////////////////////////////
        cmd = 8'h47; //G - Power Up
        send_cmd = 1; //start transmission - A Pulse of send_cmd signal
        // rider_off = 0; // Rider is ON
        ld_cell_lft = 12'd350; 
        ld_cell_rght = 12'd350;
        steerPot = 12'd800; 
        batt = 12'd1000; 
        repeat (1) @(posedge clk);
        repeat (50000) @(posedge clk);
        send_cmd = 0; //deassert send_cmd after one clock cycle
        @(posedge cmd_sent); // Wait for Transmission complete signal to go further
        repeat (25000) @(posedge clk);
        if(!iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is Not UP when a valid connect Signal comes in");
        repeat (25000) @(posedge clk);

        //////////////////////////////////////////////////////////////////////////////
        //////BLE Transmits a Valid 'S' input that indicate a Disconnect Signal///////
        //////////////////////////////////////////////////////////////////////////////
        cmd = 8'h53; //S - Towards Shut Down Again
        send_cmd = 1; //start transmission - A Pulse of send_cmd signal
        repeat (1) @(posedge clk);
        repeat (50000) @(posedge clk);
        send_cmd = 0; //deassert send_cmd after one clock cycle
        @(posedge cmd_sent); // Wait for Transmission complete signal to go further
        repeat (25000) @(posedge clk);
        if(!iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is Not UP while Segway is on STANDY and the rider is still not OFF yet");
        repeat (25000) @(posedge clk);

        ///////////////////////////////////////////////////////////////////////////////////
        ////////////////Rider Gets Off After a BLE Disconnect - Segway Should Shut down////
        ////////////////////Whenever Rider is Off it should be a ShutDown//////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        // rider_off = 1;
        ld_cell_lft = 12'd0; 
        ld_cell_rght = 12'd0;
        steerPot = 12'd800; 
        batt = 12'd1000;        
        repeat (2) @(posedge clk);
        repeat (25000) @(posedge clk);
        if(iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is UP while rider is OFF");
        if((iDUT.iBAL.SegwayMath_inst.lft_spd!=0) | (iDUT.iBAL.SegwayMath_inst.lft_spd))
            $fatal("ERROR !! Left Speed and Right Speed is Not Zero when Power is Not Up");            
        repeat (10000) @(posedge clk);
        repeat (25000) @(posedge clk);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////
        //////BLE Transmits a Valid 'G' input for Segway Power up///////
        ////////////////////////////////////////////////////////////////
        cmd = 8'h47; //G - Power Up
        send_cmd = 1; //start transmission - A Pulse of send_cmd signal
        // rider_off = 0; // Rider is ON
        ld_cell_lft = 12'd350; 
        ld_cell_rght = 12'd350;
        steerPot = 12'd800; 
        batt = 12'd1000; 
        repeat (1) @(posedge clk);
        repeat (50000) @(posedge clk);
        send_cmd = 0; //deassert send_cmd after one clock cycle
        @(posedge cmd_sent); // Wait for Transmission complete signal to go further
        repeat (25000) @(posedge clk);
        if(!iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is Not UP when a valid connect Signal comes in");
        repeat (25000) @(posedge clk);

        //////////////////////////////////////////////////////////////////////////////
        //////BLE Transmits a Valid 'S' input that indicate a Disconnect Signal///////
        //////////////////////////////////////////////////////////////////////////////
        cmd = 8'h53; //S - Towards Shut Down Again
        send_cmd = 1; //start transmission - A Pulse of send_cmd signal
        // rider_off = 1; //RIDER is OFF the Segway in this Case
        ld_cell_lft = 12'd0; 
        ld_cell_rght = 12'd0;
        steerPot = 12'd800; 
        batt = 12'd1000;        
        repeat (1) @(posedge clk);
        repeat (50000) @(posedge clk);
        send_cmd = 0; //deassert send_cmd after one clock cycle
        @(posedge cmd_sent); // Wait for Transmission complete signal to go further
        repeat (25000) @(posedge clk);
        if(iDUT.iAuth.pwr_up)
            $fatal("ERROR !! Power is UP when Segway is on ON stage but the rider is still not ON yet");
        if((iDUT.iBAL.SegwayMath_inst.lft_spd!=0) | (iDUT.iBAL.SegwayMath_inst.lft_spd))
            $fatal("ERROR !! Left Speed and Right Speed is Not Zero when Power is Not Up");            
        repeat (25000) @(posedge clk);
    end
join

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  $display("Here we go!!!");
  $stop();

end

always
  #10 clk = ~clk;

endmodule	