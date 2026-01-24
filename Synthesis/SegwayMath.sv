
module SegwayMath(
    input logic clk,
    input logic rst_n,
    input  logic signed [11:0] PID_cntrl,      // Signed 12-bit control from PID
    input  logic       [7:0]  ss_tmr,         // Unsigned 8-bit soft-start timer
    input  logic       [11:0] steer_pot,      // Unsigned 12-bit steering potentiometer
    input  logic              en_steer,       // Steering enable
    input  logic              pwr_up,         // Power-up status
    output logic signed [11:0] lft_spd,       // Signed 12-bit left motor speed/torque
    output logic signed [11:0] rght_spd,      // Signed 12-bit right motor speed/torque
    output logic              too_fast        // Speed limit warning
);

localparam MIN_DUTY = 13'h0A8;
localparam LOW_TORQUE_BAND = 7'h2A;
localparam GAIN_MULT = 4'h4;
localparam STEER_POT_MIDVAL = 12'h7FF;

wire [11:0]steer_pot_limited;
wire signed [8:0]ss_timer_s_e;
wire signed [19:0]PID_cntrl_scaled;
wire signed [11:0]PID_ss;
wire signed [12:0]PID_ss_ext;
wire signed [11:0]steer_pot_sub_signed_temp;
wire signed [12:0]steer_pot_sub_signed; 
wire signed [12:0]steer_pot_sub_signed_scaled;
wire signed [12:0]lft_torque;
wire signed [12:0]rght_torque;
wire signed [12:0]lft_torque_comp;
wire signed [12:0]lft_shaped;
wire signed [12:0]rght_torque_comp;
wire signed [12:0]rght_shaped;
wire lft_torque_band_mux_en;
wire rght_torque_band_mux_en;
wire signed [12:0]lft_torque_comp_band_comp_out;
wire signed [12:0]rght_torque_comp_band_comp_out;
wire signed [19:0]lft_torque_GAIN_MULT;
wire signed [19:0]rght_torque_GAIN_MULT;
wire signed [12:0]lft_torque_abs;
wire signed [12:0]rght_torque_abs;
wire signed [12:0]lft_shaped_abs;
wire signed [12:0]rght_shaped_abs;

/*Section 1*/
assign ss_timer_s_e = $signed({1'b0, ss_tmr[7:0]});
assign PID_cntrl_scaled = PID_cntrl * ss_timer_s_e;

reg signed [19:0]PID_cntrl_scaled_reg;
always_ff @ (posedge clk , negedge rst_n)
    if(!rst_n) begin
        PID_cntrl_scaled_reg <= '0;
    end
    else begin
        PID_cntrl_scaled_reg <= PID_cntrl_scaled;
    end


assign PID_ss = PID_cntrl_scaled_reg >>> 8; // Divide by 256
assign PID_ss_ext = {PID_ss[11], PID_ss};

/*Section 2*/
assign steer_pot_limited = (steer_pot < 12'h200) ? 12'h200 : (steer_pot > 12'hE00) ? 12'hE00 : steer_pot;
assign steer_pot_sub_signed_temp = $signed(steer_pot_limited) - $signed(STEER_POT_MIDVAL); // 12-bit result
assign steer_pot_sub_signed = {steer_pot_sub_signed_temp[11], steer_pot_sub_signed_temp}; // Sign-extend to 13 bits
assign steer_pot_sub_signed_scaled = {{3{steer_pot_sub_signed[12]}},steer_pot_sub_signed[12:3]} + {{4{steer_pot_sub_signed[12]}},steer_pot_sub_signed[12:4]};
assign lft_torque  = en_steer ?  (PID_ss_ext + steer_pot_sub_signed_scaled) : PID_ss_ext;
assign rght_torque = en_steer ?  (PID_ss_ext - steer_pot_sub_signed_scaled) : PID_ss_ext;

reg signed [12:0]lft_torque_reg;
reg signed [12:0]rght_torque_reg;

always_ff @ (posedge clk , negedge rst_n)
    if(!rst_n) begin
        lft_torque_reg <= '0;
        rght_torque_reg <= '0;
    end
    else begin
        lft_torque_reg <= lft_torque;
        rght_torque_reg <= rght_torque;
    end


/*Section 3*/
assign lft_torque_comp = lft_torque_reg[12]? (lft_torque_reg - $signed(MIN_DUTY)) : (lft_torque_reg + $signed(MIN_DUTY));
assign lft_torque_abs = lft_torque_reg[12] ? -lft_torque_reg : lft_torque_reg;
assign lft_torque_band_mux_en = lft_torque_abs < LOW_TORQUE_BAND ? 1'b0 : 1'b1;
assign lft_torque_GAIN_MULT = lft_torque_reg * $signed(GAIN_MULT);
assign lft_torque_comp_band_comp_out = lft_torque_band_mux_en ?  lft_torque_comp : lft_torque_GAIN_MULT;
assign lft_shaped = pwr_up?  lft_torque_comp_band_comp_out : 13'h0000 ;

assign rght_torque_comp = rght_torque_reg[12]? (rght_torque_reg - $signed(MIN_DUTY)) : (rght_torque_reg + $signed(MIN_DUTY));
assign rght_torque_abs = rght_torque_reg[12] ? -rght_torque_reg : rght_torque_reg;
assign rght_torque_band_mux_en = rght_torque_abs < LOW_TORQUE_BAND ? 1'b0 : 1'b1; 
assign rght_torque_GAIN_MULT = rght_torque_reg * $signed(GAIN_MULT);
assign rght_torque_comp_band_comp_out = rght_torque_band_mux_en ? rght_torque_comp : rght_torque_GAIN_MULT;
assign rght_shaped = pwr_up? rght_torque_comp_band_comp_out : 13'h0000 ;

/*Section 4*/
assign lft_spd = lft_shaped[12]?((lft_shaped[11]==1'b1)?lft_shaped[11:0]:12'b1000_0000_0000):
                                ((lft_shaped[11]==1'b0)?lft_shaped[11:0]:12'b0111_1111_1111);

assign rght_spd = rght_shaped[12]?((rght_shaped[11]==1'b1)?rght_shaped[11:0]:12'b1000_0000_0000):
                                  ((rght_shaped[11]==1'b0)?rght_shaped[11:0]:12'b0111_1111_1111);

assign lft_shaped_abs = lft_spd[11] ? -lft_spd : lft_spd;
assign rght_shaped_abs = rght_spd[11] ? -rght_spd : rght_spd;
assign too_fast = (lft_shaped_abs > $signed(12'd1536)) || (rght_shaped_abs > $signed(12'd1536));

endmodule