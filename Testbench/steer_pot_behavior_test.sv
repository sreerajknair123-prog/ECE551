//////////////////////////////////////////////////////////////////////////////////
//////////////////////////STEER POT Behavior Test/////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//Varying the Steer Pot Inputs and validating the behavior of the speed outputs///
///////////////////////////along with en_steer////////////////////////////////////
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
        steerPot = 12'h7ff;  // Middle Value
        batt = 12'd1000; 
        send_cmd = 0;
        cmd = 0;
        ////////////////////////////////////////
        //////////////Segway Power Up///////////
        ////////////////////////////////////////  
        repeat (10) @(posedge clk);
        send_cmd = 0;
        repeat (10) @(posedge clk);            
        cmd = 8'h47; //G - Power Up
        send_cmd = 1; //start transmission - A Pulse of trmt signal
        repeat (1) @(posedge clk);
        send_cmd = 0; //deassert trmt after one clock cycle
        //////////////////////////////////////////////////////////////////////////
        ////////////wait for RX is Ready followed by wait for power up ///////////
        /////////////////////////////////////////////////////////////////////////
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
        ////////////////////////////////////////////////////////
        ////Wait for some time before the rider STEPS ON////////
        ////////////////////////////////////////////////////////
        repeat (50000) @(posedge clk); 
        ///////////////////////////////////////////////////////
        ////////////Rider Steps on - Moderate weight///////////
        ////////// And Steer Pot in the Mid value//////////////
        ///////////////////////////////////////////////////////
        ld_cell_lft = 12'd350; 
        ld_cell_rght = 12'd350;
        steerPot = 12'h0; // Starting at the leftmost end
        batt = 12'd1000;
        ////////Wait Until Steer is Enabled///////////
        @(posedge iDUT.iSTR.en_steer);
        repeat (50) @(posedge clk); 
        if (iDUT.iBAL.SegwayMath_inst.lft_spd == iDUT.iBAL.SegwayMath_inst.rght_spd)
            $fatal("Left and Right Speed seems to vary when there is a balanced rider");   
        repeat (50000) @(posedge clk); 
        //////////////////////////////////////////////////////////
        ////////////rider lean Ramp up function///////////////////
        /////////////////////////////////////////////////////////
        //// Stable Forward Lean for the remaining time////////
        ///////////////////////////////////////////////////////
        /////////// STEP Function - Lean Forward///////////////
        ///////////////////////////////////////////////////////
        repeat (250000) @(posedge clk);
        rider_lean = 16'h0FFF;
        repeat (1000000) @(posedge clk);
        if (iDUT.iBAL.SegwayMath_inst.lft_spd < 0 && iDUT.iBAL.SegwayMath_inst.rght_spd < 0)
            $fatal("Left and Right Speed seems to be negative (Reverse) ");    

        for ( int i = 0; i < 254; i++) begin
            repeat (10000) @(posedge clk);
            steerPot <= steerPot + 16'h10;
            if ((i == 16)) begin // Value of steer pot at this point is around x100
                if (iDUT.iBAL.SegwayMath_inst.lft_spd > 0 || iDUT.iBAL.SegwayMath_inst.rght_spd < 0 || iDUT.iBAL.SegwayMath_inst.rght_spd < iDUT.iBAL.SegwayMath_inst.lft_spd) // Expected:  lft_speed: -ve right_speed: +ve right_speed > lft_speed
                $fatal("steer pot: x100 Expected:  lft_speed: -ve right_speed: +ve right_speed > lft_speed");    
            end
            else if  (i == 48) // Value of steer pot at this point is around x300
            begin
                if (iDUT.iBAL.SegwayMath_inst.lft_spd > 0 || iDUT.iBAL.SegwayMath_inst.rght_spd < 0 || iDUT.iBAL.SegwayMath_inst.rght_spd < iDUT.iBAL.SegwayMath_inst.lft_spd) // Expected:  lft_speed: -ve right_speed: +ve right_speed > lft_speed
                $fatal("steer pot: x300 Expected:  lft_speed: -ve right_speed: +ve right_speed > lft_speed");    
            end
            else if (i == 112) // Value of steer pot at this point is around x700
            begin
                if (iDUT.iBAL.SegwayMath_inst.lft_spd < 0 || iDUT.iBAL.SegwayMath_inst.rght_spd < 0 || iDUT.iBAL.SegwayMath_inst.rght_spd < iDUT.iBAL.SegwayMath_inst.lft_spd) // Expected:  lft_speed: +ve right_speed: +ve right_speed > lft_speed
                $fatal("steer pot:x700 Expected:  lft_speed: +ve right_speed: +ve right_speed > lft_speed");    
            end
            else if (i == 128) // Value of steer pot at this point is around x800
            begin
                if (iDUT.iBAL.SegwayMath_inst.lft_spd < 0 || iDUT.iBAL.SegwayMath_inst.rght_spd < 0 || iDUT.iBAL.SegwayMath_inst.rght_spd != iDUT.iBAL.SegwayMath_inst.lft_spd) // Expected:  lft_speed: +ve right_speed: +ve right_speed = lft_speed
                $fatal("steer pot: x800 Expected:  lft_speed: +ve right_speed: +ve right_speed = lft_speed");    
            end
            else if (i == 160) // Value of steer pot at this point is around xA00
            begin
                if (iDUT.iBAL.SegwayMath_inst.lft_spd < 0 || iDUT.iBAL.SegwayMath_inst.rght_spd < 0 || iDUT.iBAL.SegwayMath_inst.rght_spd > iDUT.iBAL.SegwayMath_inst.lft_spd) // Expected:  lft_speed: +ve right_speed: +ve right_speed < lft_speed
                $fatal("steer pot: xA00 Expected:  lft_speed: +ve right_speed: +ve right_speed < lft_speed");    
            end
            else if (i == 160) // Value of steer pot at this point is around xD00
            begin
                if (iDUT.iBAL.SegwayMath_inst.lft_spd < 0 || iDUT.iBAL.SegwayMath_inst.rght_spd > 0 || iDUT.iBAL.SegwayMath_inst.rght_spd > iDUT.iBAL.SegwayMath_inst.lft_spd) // Expected:  lft_speed: +ve right_speed: -ve right_speed < lft_speed
                $fatal("steer pot: xD00 Expected:  lft_speed: +ve right_speed: -ve right_speed < lft_speed");    
            end
            else if (i == 229) // Value of steer pot at this point is around xE50
            begin
                if (iDUT.iBAL.SegwayMath_inst.lft_spd < 0 || iDUT.iBAL.SegwayMath_inst.rght_spd > 0 || iDUT.iBAL.SegwayMath_inst.rght_spd > iDUT.iBAL.SegwayMath_inst.lft_spd) // Expected:  lft_speed: +ve right_speed: -ve right_speed < lft_speed
                $fatal("steer pot: xE50 Expected:  lft_speed: +ve right_speed: -ve right_speed < lft_speed");    
            end
        end
        for ( int i = 0; i < 256; i++) begin
            repeat (10000) @(posedge clk);
            steerPot <= steerPot - 16'h10;
        end
        repeat (25000) @(posedge clk);
        $display("Yahoo! Test Passed");
        $stop;
    end
    always
    #10 clk = ~clk;
endmodule	


