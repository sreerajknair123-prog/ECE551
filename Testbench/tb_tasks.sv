package tb_tasks;
    // Any global parameters if needed to be defined here

    //Any Tasks to be defined here
    ////////////////////////////////////////////////////////////////
    //////////Reseting at the negedge of clock//////////////////////
    ////////////////////////////////////////////////////////////////
    task automatic init_reset_sequence(ref clk, output RST_n,  output OVR_I_lft,  output OVR_I_rght,  output signed [15:0]rider_lean,  output [11:0]ld_cell_lft, 
                               output [11:0]ld_cell_rght,  output [11:0]steerPot,  output [11:0]batt, output send_cmd,  output [7:0]cmd, 
                               ref PWM1_rght, ref PWM2_rght, ref PWM1_lft, ref PWM2_lft);
        begin
            RST_n = 0;
            clk = 0;
            OVR_I_lft = 0;
            OVR_I_rght = 0;
            rider_lean = 16'h0;
            @(posedge clk);
            @(negedge clk);
            RST_n = 1;
            ld_cell_lft = 12'd0; 
            ld_cell_rght = 12'd000;
            steerPot = 12'd800; 
            batt = 12'd1000; 
            send_cmd = 0;
            cmd = 0;
            PWM1_rght = 0;
            PWM2_rght = 0;
            PWM1_lft = 0;
            PWM2_lft = 0;
        end
    endtask

    ////////////////////////////////////////////////////////////////
    //////BLE Transmits a Valid 'G' input for Segway Power up///////
    ////////////////////////////////////////////////////////////////
    task automatic power_up_SendG(ref clk, input send_cmd_trig, input [7:0] cmd_to_sent, ref cmd_sent, ref send_cmd, ref [7:0]cmd);
        begin
            repeat (50000) @(posedge clk);
            send_cmd = 0;
            repeat (10) @(posedge clk);            
            cmd = cmd_to_sent; //G - Power Up
            send_cmd = send_cmd_trig; //start transmission - A Pulse of trmt signal
            repeat (50000) @(posedge clk);
            send_cmd = 0; //deassert trmt after one clock cycle
            @(posedge cmd_sent);// Wait for Transmission complete signal to go further
            repeat (50000) @(posedge clk);
        end
    endtask
    
    ////////////////////////////////////////////////////////////////
    //////BLE Transmits a Valid 'S' input for Segway Shut Down//////
    ////////////////////////////////////////////////////////////////
    task automatic power_down_SendS(ref clk,output send_cmd, output [7:0] cmd, ref cmd_sent);
        begin
            repeat (50000) @(posedge clk);
            cmd = 8'h53; //S - Shut Down
            send_cmd = 1; //start transmission - A Pulse of trmt signal
            repeat (50000) @(posedge clk);
            @(posedge cmd_sent);// Wait for Transmission complete signal to go further
            repeat (50000) @(posedge clk);            
        end
    endtask

endpackage