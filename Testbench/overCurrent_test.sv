///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////To Test what happens if we vary the Battery input///////////////////////////////////////
///// Expectation is Piezo flagging battery low Sound and also the trigger input to the piezo should go high///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

module Segway_tb();

localparam BATT_THRES = 12'h800;
//import tb_tasks::*;		
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
logic OVR_I_lft, OVR_I_rght,OVR_I_shtdwn;

///// Internal registers for testing purposes??? /////////
logic signed [11:0] lft_speed_point1;
logic signed [11:0] rght_speed_point1;
logic signed [11:0] lft_speed_point2;
logic signed [11:0] rght_speed_point2;
logic signed [11:0] lft_speed_point3;
logic signed [11:0] rght_speed_point3;
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
			.piezo_n(piezo_n),.piezo(piezo),.RX(RX_TX),.OVR_I_shtdwn(OVR_I_shtdwn));

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
    steerPot = 12'h7ff;  // Middle Value
    batt = 12'd1000; 
    send_cmd = 0;
    cmd = 0;
    //////////////////////////////////////////////////////////////////////////
    //////////////Segway Power Up/////////////////////////////////////////////
    ////////////wait for RX is Ready followed by wait for power up ///////////
    ///////////////////////////////////////////////////////////////////////// 
    repeat (10) @(posedge clk);
    send_cmd = 0;
    repeat (10) @(posedge clk);            
    cmd = 8'h47; //G - Power Up
    send_cmd = 1; //start transmission - A Pulse of trmt signal
    repeat (1) @(posedge clk);
    send_cmd = 0; //deassert trmt after one clock cycle
    @(posedge iDUT.iAuth.rx_rdy)
    $display(" RX transceiver is ready as it completed the reception");
    @(posedge clk);
    @(posedge clk);

    repeat (500) @(posedge clk);
     ///////////////////////////////////////////////////////
    ////////////Rider Steps on - Moderate weight///////////
    ///////////////////////////////////////////////////////
    //###//option 1://##// - Moderate Weight - Expected : rider_off = 0 , en_steer = 1 , pwr_up = 1 -> lft_spd!=0 , rght_spd !=0 , (lft_spd == rght_spd) (As SteerPOT is MiD value lft and right torque should be same)
    ld_cell_lft = 12'd350;
    ld_cell_rght = 12'd350;
    steerPot = 12'h7ff; 
    batt = 12'd1000;
    ////////Wait Until Steer is Enabled///////////
    @(posedge iDUT.en_steer);
    @(posedge clk);
    if (!(iDUT.iBAL.SegwayMath_inst.lft_spd == iDUT.iBAL.SegwayMath_inst.rght_spd)) begin
        $fatal("Left and Right Speed seems to vary when there is an balanced rider");
        $stop();
    end

    $display("Starting ovr_I_shutdown test");

    // repeat (100) @(posedge clk);



    /////////////////////////////////////////////////////////////////////
    //////////////////////////// ovr_I_lft test begin ///////////////////
    ///////////////////////////////////////////////////////////////////////

    @ (posedge iDUT.iDRV.ovr_I_blank_L);
      repeat (32) begin
        if (iDUT.iDRV.ovr_I_blank_L === 1) begin
            OVR_I_lft = 1;
            @ (posedge clk);
            OVR_I_lft = 0;
            repeat (2) @(posedge clk);
        end
      end
      if (OVR_I_shtdwn === 1'b1) begin
        $display("Fail, Test case 1:FAIL Shutdown should have not gone high as the overcurrent was asserted during the blankning period");
        $stop();
      end
      else 
        $display("Passed, Test case 1: 32 OVR_I_lft asserted within the blanking period - Shuntdown did not go high");



      // OVR_I_lft - 22 high within 2047 10 high outside - No shutdown
      @ (posedge iDUT.iDRV.ovr_I_blank_L);
      repeat (22) begin
        if (iDUT.iDRV.ovr_I_blank_L === 1) begin
            OVR_I_lft = 1;
            @ (posedge clk);
            OVR_I_lft = 0;
            repeat (2) @(posedge clk);
        end
      end
      @ (negedge iDUT.iDRV.ovr_I_blank_L);
      repeat (10) begin
        if (iDUT.iDRV.ovr_I_blank_L !== 1) begin
            OVR_I_lft = 1;
            @ (posedge clk);
            OVR_I_lft = 0;
            repeat (2) @(posedge clk);
        end
      end
      if (OVR_I_shtdwn === 1'b1) begin
        $display("Fail, Test case 2:FAIL Shutdown should have not gone high as the overcurrent was asserted during the blankning period");
        $stop();
      end
      else 
        $display("Passed, Test case 2: 22 OVR_I_lft asserted within the blanking period and 20 asserted outside- Shuntdown did not go high");


      // OVR_I_lft - high 32 times outside 2047 window - Shutdown
      @ (negedge iDUT.iDRV.ovr_I_blank_L);
      repeat (32) begin
        @(posedge iDUT.iDRV.PWM_synch);
        if (iDUT.iDRV.ovr_I_blank_L !== 1) begin
            OVR_I_lft = 1;
            @ (posedge clk);
            OVR_I_lft = 0;
            repeat (5) @(posedge clk);
        end
      end
      if (OVR_I_shtdwn !== 1'b1) begin
        $display("Fail, Test case 3:FAIL Shutdown should have gone high as the overcurrent was asserted outside the blankning period");
        $stop();
      end
      else begin
        $display("Passed, Test case 3: 32 OVR_I_lft asserted during the blanking period - Shuntdown should go high");
      end

      repeat(5) @(posedge clk)


    /////////////////////////////////////////////////////////////////////
    //////////////////////////// ovr_I_lft test end ///////////////////
    ///////////////////////////////////////////////////////////////////////

    RST_n = 0;
    clk = 0;
    OVR_I_lft = 0;
    OVR_I_rght = 0;
    rider_lean = 16'h0;
    @(posedge clk);
    @(negedge clk);
    RST_n = 1;

    repeat(50) @(posedge clk)
    


    /////////////////////////////////////////////////////////////////////
    //////////////////////////// in between left & right test begin /////
    ///////////////////////////////////////////////////////////////////////

    //OVR_I_rght - high 32 times within 2047 clocks (every 50 clocks) - No shutdown
      @ (posedge iDUT.iDRV.ovr_I_blank_L);
      repeat (32) begin
        if (iDUT.iDRV.ovr_I_blank_L === 1) begin
            OVR_I_rght = 1;
            @ (posedge clk);
            OVR_I_rght = 0;
            repeat (2) @(posedge clk);
        end
      end
      if (OVR_I_shtdwn === 1'b1) begin
        $display("Fail, Test case 4:FAIL Shutdown should have not gone high as the overcurrent was asserted during the blankning period");
        $stop();
      end
      else 
        $display("Passed, Test case 4: 32 OVR_I_rght asserted within the blanking period - Shuntdown did not go high");

      // OVR_I_rght - 30 high within 2047 10 high outside - No shutdown

      @ (posedge iDUT.iDRV.ovr_I_blank_L);
      repeat (22) begin
        if (iDUT.iDRV.ovr_I_blank_L === 1) begin
            OVR_I_rght = 1;
            @ (posedge clk);
            OVR_I_rght = 0;
            repeat (2) @(posedge clk);
        end
      end
      @ (negedge iDUT.iDRV.ovr_I_blank_L);
      repeat (10) begin
        if (iDUT.iDRV.ovr_I_blank_L !== 1) begin
            OVR_I_rght = 1;
            @ (posedge clk);
            OVR_I_rght = 0;
            repeat (2) @(posedge clk);
        end
      end
      if (OVR_I_shtdwn === 1'b1) begin
        $display("Fail, Test case 5:FAIL Shutdown should have not gone high as the overcurrent was asserted during the blankning period");
        $stop();
      end
      else 
        $display("Passed, Test case 5: 22 OVR_I_rght asserted within the blanking period and 20 asserted outside- Shuntdown did not go high");

      //OVR_I_rght - high 40 times outside 2047 window - Shutdown
      @ (negedge iDUT.iDRV.ovr_I_blank_L);
      repeat (32) begin
        @(posedge iDUT.iDRV.PWM_synch);
        if (iDUT.iDRV.ovr_I_blank_L !== 1) begin
            OVR_I_rght = 1;
            @ (posedge clk);
            OVR_I_rght = 0;
            repeat (5) @(posedge clk);
        end
      end
      if (OVR_I_shtdwn !== 1'b1) begin
        $display("Fail, Test case 6:FAIL Shutdown should have gone high as the overcurrent was asserted outside the blankning period");
        $stop();
      end
      else 
        $display("Passed, Test case 6: 32 OVR_I_rght asserted during the blanking period - Shuntdown should go high");


    /////////////////////////////////////////////////////////////////////
    //////////////////////////// in between left & right test end /////
    ///////////////////////////////////////////////////////////////////////


    RST_n = 0;
    clk = 0;
    OVR_I_lft = 0;
    OVR_I_rght = 0;
    rider_lean = 16'h0;
    @(posedge clk);
    @(negedge clk);
    RST_n = 1;

    repeat(50) @(posedge clk)


    /////////////////////////////////////////////////////////////////////
    //////////////////////////// right test begin ////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    @(posedge iDUT.iDRV.PWM_synch);
      OVR_I_rght = 1;
      @ (posedge clk);
      OVR_I_rght = 0;
      // Half in OVR_I_lft and OVR_I_rght - high 40 times within 2047 clocks (every 50 clocks) - No shutdown
      @ (posedge iDUT.iDRV.ovr_I_blank_L);
      repeat (16) begin
        if (iDUT.iDRV.ovr_I_blank_L === 1) begin
            OVR_I_lft = 1;
            @ (posedge clk);
            OVR_I_lft = 0;
            repeat (5) @(posedge clk);
            OVR_I_rght = 1;
            @ (posedge clk);
            OVR_I_rght = 0;
            repeat (5) @(posedge clk);
        end
      end
      if (OVR_I_shtdwn === 1'b1) begin
        $display("Fail, Test case 7:FAIL Shutdown should have not gone high as the overcurrent was asserted during the blankning period");
        $stop();
      end
      else 
        $display("Passed, Test case 7: 32 OVR_I_lft asserted within the blanking period - Shuntdown did not go high");
      
      // Half in OVR_I_lft and OVR_I_rght -  30 high within 2047 10 high outside - No shutdown
    @ (posedge iDUT.iDRV.ovr_I_blank_L);
      repeat (11) begin
        if (iDUT.iDRV.ovr_I_blank_L === 1) begin
            OVR_I_lft = 1;
            @ (posedge clk);
            OVR_I_lft = 0;
            repeat (2) @(posedge clk);
            OVR_I_rght = 1;
            @ (posedge clk);
            OVR_I_rght = 0;
            repeat (2) @(posedge clk);
        end
      end
      @ (negedge iDUT.iDRV.ovr_I_blank_L);
      repeat (5) begin
        if (iDUT.iDRV.ovr_I_blank_L != 1) begin
            OVR_I_lft = 1;
            @ (posedge clk);
            OVR_I_lft = 0;
            repeat (2) @(posedge clk);
            OVR_I_rght = 1;
            @ (posedge clk);
            OVR_I_rght = 0;
            repeat (2) @(posedge clk);
        end
      end
      if (OVR_I_shtdwn === 1'b1) begin
        $display("Fail, Test case 8:FAIL Shutdown should have not gone high as the overcurrent was asserted during the blankning period");
        $stop();
      end
      else 
        $display("Passed, Test case 8: 22 OVR_I_rght asserted within the blanking period and 20 asserted outside- Shuntdown did not go high");


      // Half in OVR_I_lft and OVR_I_rght - high 40 times outside 2047 window - Shutdown
      @ (negedge iDUT.iDRV.ovr_I_blank_L);
      repeat (32) begin
        @(posedge iDUT.iDRV.PWM_synch);
        if (iDUT.iDRV.ovr_I_blank_L !== 1) begin
            OVR_I_lft = 1;
            @ (posedge clk);
            OVR_I_lft = 0;
            repeat (5) @(posedge clk);
            OVR_I_rght = 1;
            @ (posedge clk);
            OVR_I_rght = 0;
            repeat (5) @(posedge clk);
        end
      end
      if (OVR_I_shtdwn !== 1'b1) begin
        $display("Fail, Test case 9:FAIL Shutdown should have gone high as the overcurrent was asserted outside the blankning period");
        $stop();
      end
      else 
        $display("Passed, Test case 9: 32 OVR_I_rght asserted during the blanking period - Shuntdown should go high");


    /////////////////////////////////////////////////////////////////////
    //////////////////////////// right test end ////////////////////////////
    ///////////////////////////////////////////////////////////////////////





    
  repeat(5) @(posedge clk);
  $display ("All test cases passed!");
  $stop();
end

always
#10 clk = ~clk;

endmodule

