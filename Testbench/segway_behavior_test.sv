//////////////////////////////////////////////////////////////////////////
//////////////////////////FULL Function Test//////////////////////////////
//////////////////////////////////////////////////////////////////////////
// 1. Initialize the Inputs
// 2. Non 'G' BLE Input 'S' 
// 3. Non 'G' BLE Invalid Input
// 4. Segway Power Up BLE Input 'G'
// 5. No weight on Segway and behavior Check
// 6. underweight rider on Segway and behavior Check
// 7. Non Balanced Rider is On the Segway
// 8. Rider Steps on - Moderate weight
// 9. Making the Rider Lean Backwards
// 10. STEP Function - Lean Forward
// 11. Non 'G' BLE Input 'S' Segway Should not shut down when the rider is still ON
// 12. Steering the Segway to the Left
// 13. Steering the Segway to the right
// 14. During Steering to the Right The Rider goes out of Balance
// 15. Keeping the Rider imbalanced Rider is leaning forward
// 16. RIDER IS BALANCED ON THE SEGWAY
// 17. Now we give the Maximum Lean as a step function
// 18. Rider Gets off the Segway
// 19. Send Shut Down Signal 'S' input for Shut down of the Segway
//////////////////////////////////////////////////////////////////////////

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
    steerPot = 12'h7ff;  // Middle Value
    batt = 12'h900;  // Good Battery 
    send_cmd = 0;
    cmd = 0;
    //////////////////////////////////////////////////////////////////////////
    //////////// Non 'G' BLE Input 'S' ///////////////////////////////////////
    // No Power UP - Segway stays shut down  /////////////////////////////////
    repeat (10) @(posedge clk);
    send_cmd = 0;
    repeat (10) @(posedge clk);            
    cmd = 8'h53; //S - 'Shut Down'
    send_cmd = 1; //start transmission - A Pulse of trmt signal
    repeat (1) @(posedge clk);
    send_cmd = 0; //deassert trmt after one clock cycle
    @(posedge iDUT.iAuth.rx_rdy);
    $display(" RX transceiver is ready as it completed the reception");
    @(posedge clk);
    @(posedge clk);
    if(iDUT.iAuth.pwr_up) 
        $fatal("ERROR !! Power is UP in the next clock edge when a 'S' connect Signal comes in");
    ///Wait for Transmission complete signal to go further//
    @(posedge cmd_sent);
    $display("Wait for Transmission complete signal to go further - Done");
    ///////////////////Wait for some time before sending the next transaction///////
    repeat (50000) @(posedge clk);           
    //////////////////////////////////////////////////////////////////////////
    //////////// Non 'G' BLE Invalid Input  //////////////////////////////////
    // No Power UP - Segway stays shut down /////////////////////////////////
    repeat (10) @(posedge clk);
    send_cmd = 0;
    repeat (10) @(posedge clk);            
    cmd = 8'hFF; //Invalid Input
    send_cmd = 1; //start transmission - A Pulse of trmt signal
    repeat (1) @(posedge clk);
    send_cmd = 0; //deassert trmt after one clock cycle
    @(posedge iDUT.iAuth.rx_rdy)
    $display(" RX transceiver is ready as it completed the reception");
    @(posedge clk);
    @(posedge clk);
    if(iDUT.iAuth.pwr_up) 
        $fatal("ERROR !! Power is UP in the next clock edge when a invalid connect Signal comes in");
    ///Wait for Transmission complete signal to go further//
    @(posedge cmd_sent);
    $display("Wait for Transmission complete signal to go further - Done");
    ///////////////////Wait for some time before sending the next transaction///////
    repeat (50000) @(posedge clk);                      
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
    if(!iDUT.iAuth.pwr_up) 
        $fatal("ERROR !! Power is Not UP in the next clock edge when a valid connect Signal comes in");
    ////////////////////////////////////////////////////////
    ///Wait for Transmission complete signal to go further//
    ////////////////////////////////////////////////////////
    @(posedge cmd_sent);
    $display("Wait for Transmission complete signal to go further - Done");
    ////////////////////////////////////////////////////////
    ////Wait for some time before the rider STEPS ON////////
    ////////////////////////////////////////////////////////
    repeat (50000) @(posedge clk); 
    ///////////////////////////////////////////////////////
    //// Initially Nobody steps on Segway /////////////////
    /////////// Segway remains in Shut Down////////////////
    //###//option 2://##// - No Weight - Expected : rider_off = 1 , en_steer = 0 , pwr_up = 0 -> lft_spd = 0 , rght_spd = 0
    ld_cell_lft = 12'd0; 
    ld_cell_rght = 12'd0;
    steerPot = 12'h7ff; // Doesn't matter as the weight is Under
    batt = 12'h900;
    ////Wait for some time before an underweight rider STEPS ON//////////
    /////////////////////////////////////////////////////////////////////
    repeat (50000) @(posedge clk); 
    ///////Check for state of pwr_up , left speed and right speed before the next rider steps ON///
    if (!iDUT.iSTR.rider_off | (iDUT.iBAL.SegwayMath_inst.lft_spd != 0) | (iDUT.iBAL.SegwayMath_inst.rght_spd!= 0))
        $fatal("During No Weight Segways seems to be powered ON or Moving");
    //###//option 4://##// - Under Weight  - Expected : rider_off = 1 , en_steer = 0 , pwr_up = 0 -> lft_spd =0 , rght_spd = 0
    ld_cell_lft = 12'd200; 
    ld_cell_rght = 12'd200;
    steerPot = 12'h7ff; 
    batt = 12'h900;
    ////Wait for some time before a moderate weight rider STEPS ON//////////
    ////////////////////////////////////////////////////////////////////////
    repeat (50000) @(posedge clk); 
    ///////Check for state of pwr_up , left speed and right speed before the next rider steps ON///
    if (!iDUT.iSTR.rider_off | (iDUT.iBAL.SegwayMath_inst.lft_spd != 0) | (iDUT.iBAL.SegwayMath_inst.rght_spd!= 0))
        $fatal("During Under Weight Segways seems to be powered ON or Moving ");    
    ////////////////////////////////////////////////////////////////////////
    //////// Non Balanced Rider is On the Segway////////////////////////////
    //###//option 3://##// - Varying Wieght - Expected : rider_off = 0 , en_steer = 0 , pwr_up = 1 -> lft_spd != 0 , rght_spd != 0 and (lft_spd == rght_spd)
    ld_cell_lft = 12'd650; 
    ld_cell_rght = 12'd50;
    steerPot = 12'h7ff; 
    batt = 12'h900;
    /////// Wait Until Rider is ON ///////////////////////////////////
    @(negedge iDUT.iSTR.rider_off);
    if (iDUT.iSTR.en_steer)
        $fatal("Steer Seems to enabled even when there is an unbalanced rider");
    if (!(iDUT.iBAL.SegwayMath_inst.lft_spd == iDUT.iBAL.SegwayMath_inst.rght_spd))
        $fatal("Left and Right Speed seems to vary when there is an unbalanced rider");
    ///////////////////////////////////////////////////////
    ////////////Rider Steps on - Moderate weight///////////
    ///////////////////////////////////////////////////////
    //###//option 1://##// - Moderate Weight - Expected : rider_off = 0 , en_steer = 1 , pwr_up = 1 -> lft_spd!=0 , rght_spd !=0 , (lft_spd == rght_spd) (As SteerPOT is MiD value lft and right torque should be same)
    ld_cell_lft = 12'd350;
    ld_cell_rght = 12'd350;
    steerPot = 12'h7ff; 
    batt = 12'h900;
    ////////Wait Until Steer is Enabled///////////
    @(posedge iDUT.iSTR.en_steer);
    if (!(iDUT.iBAL.SegwayMath_inst.lft_spd == iDUT.iBAL.SegwayMath_inst.rght_spd))
        $fatal("Left and Right Speed seems to vary when there is an balanced rider");   
    repeat (50000) @(posedge clk); 

    ////////////////// Making the Rider Lean Backwards //////////////////////
    // Don't exceed 0x1FFF postitive or 0xE000 negative for the rider lean input ////
    repeat (250000) @(posedge clk);
    rider_lean = 16'hE000;
    repeat (1000000) @(posedge clk);
    if (iDUT.iBAL.SegwayMath_inst.lft_spd > 0 && iDUT.iBAL.SegwayMath_inst.rght_spd > 0)
        $fatal("Left and Right Speed seems to not negative (Reverse) ");   
    rider_lean = 16'h0;
    repeat (1000000) @(posedge clk);

    ///////////////////////////////////////////////////////
    /////////// STEP Function - Lean Forward///////////////
    ///////////////////////////////////////////////////////
    repeat (250000) @(posedge clk);
    rider_lean = 16'h0FFF;
    repeat (1000000) @(posedge clk);
    if (iDUT.iBAL.SegwayMath_inst.lft_spd < 0 && iDUT.iBAL.SegwayMath_inst.rght_spd < 0)
        $fatal("Left and Right Speed seems to not negative (Reverse) ");    
    rider_lean = 16'h0;
    repeat (1000000) @(posedge clk);

    ///////////////////////////////////////////////////////////////////////////////////
    //////////// Non 'G' BLE Input 'S' ////////////////////////////////////////////////
    // Segway Should not shut down when the rider is still ON /////////////////////////
    repeat (10) @(posedge clk);
    send_cmd = 0;
    repeat (10) @(posedge clk);            
    cmd = 8'h53; //S - 'Shut Down'
    send_cmd = 1; //start transmission - A Pulse of trmt signal
    repeat (1) @(posedge clk);
    send_cmd = 0; //deassert trmt after one clock cycle
    @(posedge iDUT.iAuth.rx_rdy);
    $display(" RX transceiver is ready as it completed the reception");
    @(posedge clk);
    @(posedge clk);
    if(!iDUT.iAuth.pwr_up) 
        $fatal("ERROR !! Power is Not UP in the next clock edge when a 'S' connect Signal comes in when the rider is till ON");
    ///Wait for Transmission complete signal to go further//
    @(posedge cmd_sent);
    $display("Wait for Transmission complete signal to go further - Done");
    ///////////////////Wait for some time before sending the next transaction///////
    repeat (50000) @(posedge clk);

    if (!iDUT.iSTR.en_steer)
        $fatal("ERROR !! Enable STEER IS NOT SUPPOSED TO GO LOW!!!");

    //////////////////////////////////////////
    //// Steering the Segway to the Left /////
    steerPot = 12'hE00;
    repeat (50000) @(posedge clk);
    if (iDUT.iBAL.SegwayMath_inst.lft_spd < 0 && iDUT.iBAL.SegwayMath_inst.rght_spd > 0)
        $fatal("ERROR !! Segway is not rotating Left!!!");
    repeat (50000) @(posedge clk);
    //////////////////////////////////////////
    //// Steering the Segway to the right /////    
    steerPot = 12'h200;
    repeat (50000) @(posedge clk);
    if (iDUT.iBAL.SegwayMath_inst.lft_spd > 0 && iDUT.iBAL.SegwayMath_inst.rght_spd < 0)
        $fatal("ERROR !! Segway is not rotating Right!!!");
    repeat (50000) @(posedge clk);

    //// During Steering to the Right The Rider goes out of Balance ///////////
    ////////////////////////////////////////////////////////////////////////
    //////// Non Balanced Rider is On the Segway////////////////////////////
    //###//option 3://##// - Varying Wieght - Expected : rider_off = 0 , en_steer = 0 , pwr_up = 1 -> lft_spd != 0 , rght_spd != 0 and (lft_spd == rght_spd)
    ld_cell_lft = 12'd10; 
    ld_cell_rght = 12'd690;
    batt = 12'h900;
    ////////////////////////////////////////////////////
    repeat (50000) @(posedge clk);
    /////// Wait Until Rider is ON ///////////////////////////////////
    if(iDUT.iSTR.rider_off)
        $fatal(" ERROR !! RIDER is expected to be still ON the Device even when there is an imbalance");
    if (iDUT.iSTR.en_steer)
        $fatal("Steer Seems to be enabled even when there is an unbalanced rider");
    if (!(iDUT.iBAL.SegwayMath_inst.lft_spd == iDUT.iBAL.SegwayMath_inst.rght_spd))
        $fatal("Left and Right Speed seems to vary when there is an unbalanced rider");

    //////////////////////////////////////////////////////////////
    ////Keeping the Rider imbalanced Rider is leaning forward/////
    //////////////////////////////////////////////////////////////
    repeat (50000) @(posedge clk);
    /////////// STEP Function - Lean Forward///////////////
    ///////////////////////////////////////////////////////
    lft_speed_point1 = iDUT.iBAL.SegwayMath_inst.lft_spd;
    rght_speed_point1 = iDUT.iBAL.SegwayMath_inst.rght_spd;
    repeat (100) @(posedge clk);
    fork 
        begin
            rider_lean = 16'h0FFF;
            repeat (1000000) @(posedge clk);
        end
        begin
            repeat (300000) @(posedge clk);
            lft_speed_point2 = iDUT.iBAL.SegwayMath_inst.lft_spd;
            rght_speed_point2 = iDUT.iBAL.SegwayMath_inst.rght_spd;
            repeat (300000) @(posedge clk);
            lft_speed_point3 = iDUT.iBAL.SegwayMath_inst.lft_spd;
            rght_speed_point3 = iDUT.iBAL.SegwayMath_inst.rght_spd;
        end
    join
    if ((lft_speed_point1 > lft_speed_point2)   || (lft_speed_point1 > lft_speed_point3) || 
        (rght_speed_point1 > rght_speed_point2) || (rght_speed_point1 > rght_speed_point3)) begin
        $fatal("Left and Right Speed both seems to be not negative (Reverse) lft_speed_point1 = %d ,lft_speed_point2 = %d,lft_speed_point3 = %d,rght_speed_point1 = %d,rght_speed_point2 = %d,rght_speed_point3 = %d",lft_speed_point1,lft_speed_point2,lft_speed_point3,rght_speed_point1,rght_speed_point2,rght_speed_point3);
    end
    if (iDUT.iBAL.SegwayMath_inst.lft_spd < 0 && iDUT.iBAL.SegwayMath_inst.rght_spd < 0)
        $fatal("Left and Right Speed both seems to be not negative (Reverse) ");
    rider_lean = 16'h0;
    repeat (1000000) @(posedge clk);
    /////////////RIDER IS BALANCED ON THE SEGWAY///////////////
    ///////////////////////////////////////////////////////
    //###//option 1://##// - Moderate Weight - Expected : rider_off = 0 , en_steer = 1 , pwr_up = 1 -> lft_spd!=0 , rght_spd !=0 , (lft_spd == rght_spd) (As SteerPOT is MiD value lft and right torque should be same)
    ld_cell_lft = 12'd350; 
    ld_cell_rght = 12'd350;
    steerPot = 12'h7ff; 
    batt = 12'h900;
    ////////Wait Until Steer is Enabled///////////
    @(posedge iDUT.iSTR.en_steer);
    if (!(iDUT.iBAL.SegwayMath_inst.lft_spd == iDUT.iBAL.SegwayMath_inst.rght_spd))
        $fatal("Left and Right Speed seems to vary when there is an unbalanced rider");   
    repeat (50000) @(posedge clk); 

    ///////////////////////////////////////////////////////
    ////Now we give the Maximum Lean as a step function////
    /////////// STEP Function - Lean Forward///////////////
    ///////////////////////////////////////////////////////
    repeat (250000) @(posedge clk);
    rider_lean = 16'h1FFF; // MAXIMUM FORWARD LEAN
    repeat (1000000) @(posedge clk);
    if (iDUT.iBAL.SegwayMath_inst.lft_spd < 0 && iDUT.iBAL.SegwayMath_inst.rght_spd < 0)
        $fatal("Left and Right Speed seems to not negative (Reverse) ");    
    rider_lean = 16'h0;
    repeat (1000000) @(posedge clk);
    
    ////////////////////////////////////////////////////////////////////////////////
    /////////// Segway remains will Shut Down as the Rider is Off now///////////////
    ////////////////////////////////////////////////////////////////////////////////
    ld_cell_lft = 12'd0; 
    ld_cell_rght = 12'd0;
    steerPot = 12'h7ff; // Doesn't matter as the weight is Under
    batt = 12'h900; // Good Battery
    /////////////////////////////////////////////////////////////////////
    repeat (50000) @(posedge clk); 
    if (!iDUT.iSTR.rider_off )
        $fatal("ERROR!! Rider is Not Flagged as OFF the Segway");
    if(iDUT.iAuth.pwr_up) 
        $fatal("ERROR !! Power Should Go Down as the Rider is off now");    

    ////Check that too_fast has gone high
    // @(posedge iDUT.iBAL.SegwayMath_inst.too_fast) // Will timeout and fail if this doesn't occur
    // ///////////////////////////////////////////////////////////////////////////////////
    // /////Waiting for the pwr_up to go low  with a timeout Check in parallel////////////
    // ///////////////////////////////////////////////////////////////////////////////////
    // fork
    //     begin :POWER_DWN_CHK1
    //         @(negedge iDUT.iAuth.pwr_up);
    //         $display("Yahoo! Test Passed");
    //         disable TIMEOUT1;
    //         $stop;
    //     end
    //     begin : TIMEOUT1
    //         repeat (700000) @(posedge clk);
    //         $fatal("POWER_DWN_CHK1 Timed out!!! - TEST FAIL"); // I want it to stop the simulation and display fatal error
    //     end
    // join
    $display("Yahoo! Test Passed");
    $stop;
    end

    always
    #10 clk = ~clk;

endmodule	


