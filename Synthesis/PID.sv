/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////PID Math module updated to add sequential elements////////////////////////////
/////////////////// needed and create the full PID controller for theSegway//////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
module PID #(parameter fast_sim = 0) (
input  logic clk,
input  logic rst_n,
input  logic vld,
input  logic pwr_up,
input  logic rider_off,
input  logic signed [15:0]ptch,
input  logic signed [15:0]ptch_rt,
output logic [7:0]ss_tmr,
output logic signed [11:0]PID_cntrl);

localparam P_COEFF = 5'h09;

logic signed [14:0] P_term;
reg signed [14:0] I_term;
logic signed [12:0] D_term;
wire signed [9:0] pitch_err_sat;
wire signed [12:0] D_term_temp;
wire signed [15:0] PID_cntrl_temp;
wire signed [17:0] ptch_err_sat_intg_add_val;
wire signed [17:0] ptch_err_sat_intg_add_val_freeze;
wire signed [17:0] ptch_err_sat_intg_add_val_freeze_rider_chk;
reg signed [17:0] integrator;
wire signed [17:0] pitch_err_sat_sgn_ext;
wire overflow;
reg [26:0] long_tmr;

logic [8:0] incr_cnt;

////////Default to 1 as most of the time we are simulating with ModelSim///////////////
/////And when we test this with FPGA we have to override it with 0/////////////////////

generate 
    if (fast_sim)     // Fast Sim Case
        assign incr_cnt = 9'd256;
    else            //Normal case
        assign incr_cnt = 9'd1;
endgenerate

///////////////////////////////////////
// 15 to 10 bit Saturation ///////////
/////////////////////////////////////
assign pitch_err_sat = ptch[15]?((&ptch[14:9])?ptch[9:0]:10'b1000000000):
                                   ((~|ptch[14:9])?ptch[9:0]:10'b0111111111);

assign P_term = pitch_err_sat * $signed(5'h09);
assign D_term_temp = {{3{ptch_rt[15]}},ptch_rt[15:6]};
assign D_term = ~D_term_temp + 1;

///////////////////////////////////////
// Integrator logic //////////////////
/////////////////////////////////////
assign pitch_err_sat_sgn_ext = {{8{pitch_err_sat[9]}},pitch_err_sat[9:0]};
assign ptch_err_sat_intg_add_val = pitch_err_sat_sgn_ext + integrator;
assign overflow = ((integrator[17] == pitch_err_sat_sgn_ext[17]) && (integrator[17]!= ptch_err_sat_intg_add_val[17])) ? 1'b1 : 1'b0;
assign ptch_err_sat_intg_add_val_freeze = (vld & ~overflow) ? ptch_err_sat_intg_add_val : integrator;
assign ptch_err_sat_intg_add_val_freeze_rider_chk = rider_off ? 18'h00000 : ptch_err_sat_intg_add_val_freeze;

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        integrator <= 18'h00000;
    end
    else begin
        integrator <= ptch_err_sat_intg_add_val_freeze_rider_chk;
    end
end
generate 
    if (fast_sim)    
        assign I_term = integrator[17]? ((&integrator[17:15])?integrator[15:1]:15'b100_0000_0000_0000):
                                        ((~|integrator[17:15])?integrator[15:1]:15'b011_1111_1111_1111);
    else
        assign I_term = {{3{integrator[17]}},{integrator[17:6]}}; //Normal Case
endgenerate
///////////////////////////////////////
// One Shot SS Timer - The PID unit will also generate the soft start timer (ss_tmr[7:0]) that //
// is used in SegwayMath to ensure on power up the Segway does not jerk to a start //
/////////////////////////////////////   
/////////Update : ss_timer - Increment the 27bit counter by 256 instead of 1////////////
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        long_tmr <= 27'h0000000;
    end
    else if (pwr_up) begin 
        if (&(long_tmr[26:19])) begin
            long_tmr <= long_tmr;
        end
        else begin
            long_tmr <= long_tmr + incr_cnt; //Updated for fast_sim implementation
        end
    end
    else begin
       long_tmr <= 27'h0000000; 
    end
end
assign ss_tmr = long_tmr[26:19];
//Final PID output with 15 to 12 bit saturation//
assign PID_cntrl_temp = {P_term[14],P_term} + {I_term[14],I_term} + {{3{D_term[12]}},D_term};

reg signed [15:0] PID_cntrl_reg;
always_ff @(posedge clk, negedge rst_n) begin
    if(!rst_n) begin
        PID_cntrl_reg <= '0;
    end
    else 
        PID_cntrl_reg <= PID_cntrl_temp;
end

/////////////////////////////
///15 to 12 bit Saturation///
////////////////////////////
assign PID_cntrl = PID_cntrl_reg[15]?((&PID_cntrl_reg[14:11])?PID_cntrl_reg[11:0]:12'b1000_0000_0000):
                                      ((~|PID_cntrl_reg[14:11])?PID_cntrl_reg[11:0]:12'b0111_1111_1111);

endmodule