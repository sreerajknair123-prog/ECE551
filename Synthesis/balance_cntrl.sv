//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////balance control module/////////////////////////////////////////////////////////////
/////////////////Instantiates and connect PID Module with the Segway Math Module//////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
module balance_cntrl #(parameter fast_sim = 0)( //fast_sim parameterised at the balance control module level
    input  logic clk,
    input  logic rst_n,
    input  logic vld,
    input  logic pwr_up,
    input  logic rider_off,
    input  logic signed [15:0]ptch,
    input  logic signed [15:0]ptch_rt,
    input  logic       [11:0] steer_pot,      // Unsigned 12-bit steering potentiometer
    input  logic              en_steer,       // Steering enable
    output logic signed [11:0] lft_spd,       // Signed 12-bit left motor speed/torque
    output logic signed [11:0] rght_spd,      // Signed 12-bit right motor speed/torque
    output logic              too_fast 
    );
    wire [11:0] PID_cntrl_o;
    wire [7:0] ss_tmr_o;
    ////Instantiation of Updated PID Module/////////
    PID #(.fast_sim(fast_sim)) PID_inst(
        .rst_n(rst_n),
        .clk(clk),
        .vld(vld),
        .pwr_up(pwr_up),
        .rider_off(rider_off),
        .ptch(ptch),
        .ptch_rt(ptch_rt),
        .ss_tmr(ss_tmr_o),
        .PID_cntrl(PID_cntrl_o));
    ////Instantiation of Segway Math////////////////
    SegwayMath SegwayMath_inst(
        .rst_n(rst_n),
        .clk(clk),        
        .PID_cntrl(PID_cntrl_o),      // Signed 12-bit control from PID
        .ss_tmr(ss_tmr_o),         // Unsigned 8-bit soft-start timer
        .steer_pot(steer_pot),      // Unsigned 12-bit steering potentiometer
        .en_steer(en_steer),       // Steering enable
        .pwr_up(pwr_up),         // Power-up status
        .lft_spd(lft_spd),       // Signed 12-bit left motor speed/torque
        .rght_spd(rght_spd),      // Signed 12-bit right motor speed/torque
        .too_fast(too_fast)        // Speed limit warning
    );
endmodule