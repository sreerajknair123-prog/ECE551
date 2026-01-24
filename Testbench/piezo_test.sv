// `timescale 1ns/1ps
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

////Added missing signal from mtr_drv//////
reg OVR_I_shtdwn;
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
			.PWM2_rght(PWM2_rght),.OVR_I_lft(OVR_I_lft),.OVR_I_rght(OVR_I_rght),.OVR_I_shtdwn(OVR_I_shtdwn),// Added OVR_I_shtdwn signal : TODO - Not required?? Can be removed??
			.piezo_n(piezo_n),.piezo(piezo),.RX(RX_TX));

//// Instantiate UART_tx (mimics command from BLE module) //////
UART_tx iTX(.clk(clk),.rst_n(rst_n),.TX(RX_TX),.trmt(send_cmd),.tx_data(cmd),.tx_done(cmd_sent));

/////////////////////////////////////
// Instantiate reset synchronizer //
///////////////////////////////////
rst_synch iRST(.clk(clk),.RST_n(RST_n),.rst_n(rst_n));

/////////////////////////////////////////////////////////////////
//////// Function - To calculate the frequency /////////////////
/////////////////////////////////////////////////////////////////
int counter;

// fork
//   begin : PIEZO_CHECK_1
//       $display("start PIEZO_CHECK_1");
//       @(posedge piezo_n);
//       $display("start PIEZO_CHECK_1 - posedge of piezo_n");
//       do begin
//           counter = counter + 1;
//           @(posedge clk);
//       end
//       while (1);
//       $display("end PIEZO_CHECK_2");
      
//   end
//   begin : PIEZO_CHECK_2
//       $display("start PIEZO_CHECK_2");
//       @ (negedge piezo_n);
//       disable PIEZO_CHECK_1;
//       counter = 0;
//       $display("end PIEZO_CHECK_2");
//   end
// join


initial 

  begin

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
    steerPot = 12'h7ff;   // Middle Value
    batt = 12'h900;       // Good Battery
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

    ////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////
    //// Input given: Moderate weight - Side to Side balanced //////// 
    //// Expected : Steer will be enabled and FanFare should play ///
    ///////////////////////////////////////////////////////////////////

    repeat (5) @ (posedge clk);
    ld_cell_lft = 12'd350; 
    ld_cell_rght = 12'd350; 
    steerPot = 12'h800;
    
    fork
      begin : PIEZO_CHECK_1
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_2
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_1;
          $display("end PIEZO_CHECK_2");
      end
    join 

    // G6
    @ (posedge piezo);
    if ((counter > 240) && (counter < 260)) // Note G6 frequency check - 15'h7C90
      $display ("Frequency of Note G6 is not right in en_steer mode");
    repeat (2048) @  (posedge clk);
    
        fork
      begin : PIEZO_CHECK_3
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_4
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_3;
          $display("end PIEZO_CHECK_2");
      end
    join 

    // C7
    @ (posedge piezo);
    if ((counter > 180) && (counter < 200)) // Note C7 frequency check - 15'h5D52
      $display ("Frequency of Note C7 is not right in en_steer mode");
    repeat (2048) @ (posedge clk);


        fork
      begin : PIEZO_CHECK_5
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_6
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_5;
          $display("end PIEZO_CHECK_2");
      end
    join 

      // E7
    @ (posedge piezo);
    if ((counter > 140) && (counter < 160)) // Note E7 frequency check - 15'h4A11
      $display ("Frequency of Note E7 is not right in en_steer mode");
    repeat (2048) @  (posedge clk);


        fork
      begin : PIEZO_CHECK_7
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_8
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_7;
          $display("end PIEZO_CHECK_2");
      end
    join   

      // G7
    @ (posedge piezo);
    if ((counter > 115) && (counter < 135)) // Note G7 frequency check - 15'h3E48
      $display ("Frequency of Note G7 is not right in en_steer mode");
    repeat (3072) @  (posedge clk);


        fork
      begin : PIEZO_CHECK_9
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_10
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_9;
          $display("end PIEZO_CHECK_2");
      end
    join 

      // E7_22
    @ (posedge piezo);
    if ((counter > 140) && (counter < 160)) // Note E7_22 frequency check - 15'h4A11
      $display ("Frequency of Note E7_22 is not right in en_steer mode");
    repeat (1024) @  (posedge clk);


        fork
      begin : PIEZO_CHECK_11
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_12
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_11;
          $display("end PIEZO_CHECK_2");
      end
    join 

      // G7_25
    @ (posedge piezo);
    if ((counter > 115) && (counter < 135)) // Note G7_25 frequency check - 15'h3E48
      $display ("Frequency of Note G7_25 is not right in en_steer mode");
    repeat (8192) @ (posedge clk);


        fork
      begin : PIEZO_CHECK_13
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_14
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_13;
          $display("end PIEZO_CHECK_2");
      end
    join 
    

    ///////////////////////////////////////////////////////////////////////////////
    //// Input given: Moderate weight - Side to Side balanced - BATTERY LOW//////////
    //// Expected : Battery low takes priority and FanFare should play in reverse ///
    /////////////////////////////////////////////////////////////////////////////////
    batt = 12'h500;

      // G7_25
    @ (posedge piezo);
    if ((counter > 115) && (counter < 135)) // Note G7_25 frequency check - 15'h3E48
      $display ("Frequency of Note G7_25 is not right in batt_low mode");
    repeat (8192) @ (posedge clk);

        fork
      begin : PIEZO_CHECK_15
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_16
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_15;
          $display("end PIEZO_CHECK_2");
      end
    join 

        // E7_22
    @ (posedge piezo);
    if ((counter > 140) && (counter < 160)) // Note E7_22 frequency check - 15'h4A11
      $display ("Frequency of Note E7_22 is not right in batt_low mode");
    repeat (1024) @  (posedge clk);

        fork
      begin : PIEZO_CHECK_17
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_18
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_17;
          $display("end PIEZO_CHECK_2");
      end
    join 

    // G7
    @ (posedge piezo);
    if ((counter > 115) && (counter < 135)) // Note G7 frequency check - 15'h3E48
      $display ("Frequency of Note G7 is not right in batt_low mode");
    repeat (3072) @  (posedge clk);

        fork
      begin : PIEZO_CHECK_19
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_20
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_19;
          $display("end PIEZO_CHECK_2");
      end
    join 

    // E7
    @ (posedge piezo);
    if ((counter > 140) && (counter < 160))  // Note E7 frequency check - 15'h4A11
      $display ("Frequency of Note E7 is not right in batt_low mode");
    repeat (2048) @  (posedge clk);

        fork
      begin : PIEZO_CHECK_21
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_22
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_21;
          $display("end PIEZO_CHECK_2");
      end
    join 

    // C7
    @ (posedge piezo);
    if ((counter > 180) && (counter < 200)) // Note C7 frequency check - 15'h5D52
      $display ("Frequency of Note C7 is not right in batt_low mode");
    repeat (2048) @ (posedge clk);

        fork
      begin : PIEZO_CHECK_23
          $display("start PIEZO_CHECK_1");
          @(posedge piezo_n);
          counter = 0;
          $display("start PIEZO_CHECK_1 - posedge of piezo_n");
          do begin
              counter = counter + 1;
              @(posedge clk);
          end
          while (1);
          $display("end PIEZO_CHECK_2");
          
      end
      begin : PIEZO_CHECK_24
          $display("start PIEZO_CHECK_2");
          @ (negedge piezo_n);
          disable PIEZO_CHECK_23;
          $display("end PIEZO_CHECK_2");
      end
    join 


    // G6
    @ (posedge piezo);
    if ((counter > 240) && (counter < 260)) // Note G6 frequency check - 15'h7C90
      $display ("Frequency of Note G6 is not right in batt_low mode");
    repeat (2048) @  (posedge clk);

    ///////////////////////////////////////////////////////////////////////////////
    //// Input given: Moderate weight - Side to Side balanced - TOO FAST//////////
    //// Expected : TOO FAST takes priority and FanFare should play in reverse ///
    /////////////////////////////////////////////////////////////////////////////////

    //     fork
    //   begin : PIEZO_CHECK_1
    //       $display("start PIEZO_CHECK_1");
    //       @(posedge piezo_n);
    //       counter = 0;
    //       $display("start PIEZO_CHECK_1 - posedge of piezo_n");
    //       do begin
    //           counter = counter + 1;
    //           @(posedge clk);
    //       end
    //       while (1);
    //       $display("end PIEZO_CHECK_2");
          
    //   end
    //   begin : PIEZO_CHECK_2
    //       $display("start PIEZO_CHECK_2");
    //       @ (negedge piezo_n);
    //       disable PIEZO_CHECK_1;
    //       $display("end PIEZO_CHECK_2");
    //   end
    // join 


    // G6

    //**TO_DO : give too_fast stimuli
    // @ (posedge piezo);
    // if ((counter > 240) && (counter < 260))// Note G6 frequency check - 15'h7C90
    //   $display ("Frequency of Note G6 is not right in en_steer mode");
    // repeat (131072) @  (posedge clk);

    //     fork
    //   begin : PIEZO_CHECK_1
    //       $display("start PIEZO_CHECK_1");
    //       @(posedge piezo_n);
    //       counter = 0;
    //       $display("start PIEZO_CHECK_1 - posedge of piezo_n");
    //       do begin
    //           counter = counter + 1;
    //           @(posedge clk);
    //       end
    //       while (1);
    //       $display("end PIEZO_CHECK_2");
          
    //   end
    //   begin : PIEZO_CHECK_2
    //       $display("start PIEZO_CHECK_2");
    //       @ (negedge piezo_n);
    //       disable PIEZO_CHECK_1;
    //       $display("end PIEZO_CHECK_2");
    //   end
    // join 
    
    // // C7
    // @ (posedge piezo);
    // if ((counter < 180) && (counter > 200)) // Note C7 frequency check - 15'h5D52
    //   $display ("Frequency of Note C7 is not right in en_steer mode");
    // repeat (131072) @ (posedge clk);

    //     fork
    //   begin : PIEZO_CHECK_1
    //       $display("start PIEZO_CHECK_1");
    //       @(posedge piezo_n);
    //       counter = 0;
    //       $display("start PIEZO_CHECK_1 - posedge of piezo_n");
    //       do begin
    //           counter = counter + 1;
    //           @(posedge clk);
    //       end
    //       while (1);
    //       $display("end PIEZO_CHECK_2");
          
    //   end
    //   begin : PIEZO_CHECK_2
    //       $display("start PIEZO_CHECK_2");
    //       @ (negedge piezo_n);
    //       disable PIEZO_CHECK_1;
    //       $display("end PIEZO_CHECK_2");
    //   end
    // join 

    //   // E7
    // @ (posedge piezo);
    // if ((counter < 140) && (counter > 160))  // Note E7 frequency check - 15'h4A11
    //   $display ("Frequency of Note E7 is not right in en_steer mode");
    // repeat (131072) @  (posedge clk);

    //     fork
    //   begin : PIEZO_CHECK_1
    //       $display("start PIEZO_CHECK_1");
    //       @(posedge piezo_n);
    //       counter = 0;
    //       $display("start PIEZO_CHECK_1 - posedge of piezo_n");
    //       do begin
    //           counter = counter + 1;
    //           @(posedge clk);
    //       end
    //       while (1);
    //       $display("end PIEZO_CHECK_2");
          
    //   end
    //   begin : PIEZO_CHECK_2
    //       $display("start PIEZO_CHECK_2");
    //       @ (negedge piezo_n);
    //       disable PIEZO_CHECK_1;
    //       $display("end PIEZO_CHECK_2");
    //   end
    // join 



    $display("Yahoo !!! Test Passed");
    $stop();
  end

always
  #10 clk = ~clk;

endmodule	



