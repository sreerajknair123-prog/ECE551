/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////PWM11 Module intatiating counter and SR flip flops to Perform Pulse Width Modulation//////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
module PWM11(input clk,input rst_n, input [10:0]duty, output PWM1, PWM2, output PWM_synch,output ovr_I_blank);
localparam NONOVERLAP = 11'h040;
logic [10:0] cnt;
logic PWM1_in_S, PWM1_in_R, PWM2_in_S, PWM2_in_R;
logic ovr_I_blank_range1, ovr_I_blank_range2;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////Counter Instance for a 11 bit Counter/////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
counter #(11) counter_11bit(.clk_counter(clk), .rst_counter_n(rst_n), .en(1'b1), .cnt_out(cnt));
//////////Local temporary assignments for SR Flip flop and blank out range calculation
assign PWM1_in_S = (cnt >= NONOVERLAP) ? 1'b1 : 1'b0; 
assign PWM1_in_R = (cnt >= duty) ? 1'b1 : 1'b0;
assign PWM2_in_S = (cnt >= (duty + NONOVERLAP)) ? 1'b1 : 1'b0;
assign PWM2_in_R = &cnt; // all bits are 1
assign PWM_synch = ~|cnt; // all bits are 0
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////2 SR FF Instances for PWM1 and PWM Calculation////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
sr_ff_async sr_ff_async1_PWM1(.clk_srff(clk), .reset_srff_n(rst_n), .S(PWM1_in_S), .R(PWM1_in_R), .Q(PWM1));
sr_ff_async sr_ff_async2_PWM2(.clk_srff(clk), .reset_srff_n(rst_n), .S(PWM2_in_S), .R(PWM2_in_R), .Q(PWM2));
//////////blank out range calculation///////////////////////////////////////////////////////////////
assign ovr_I_blank_range1 = ((NONOVERLAP < cnt) && (cnt < NONOVERLAP + 11'd128))? 1'b1 : 1'b0;
assign ovr_I_blank_range2 = ((NONOVERLAP+duty< cnt) && (cnt < NONOVERLAP + duty + 11'd128))? 1'b1 : 1'b0;
assign ovr_I_blank = ovr_I_blank_range1 || ovr_I_blank_range2;
endmodule
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////11-bit free-running counter. It ramps from 0 to 2047 then rolls//////////////////////////////////
////////////////////////over to 0 again////////////////////////////////////////////////////////////////////
module counter #(parameter  WIDTH=8)(clk_counter,rst_counter_n,en,cnt_out);
    input clk_counter,rst_counter_n,en;
    output [WIDTH-1:0] cnt_out;
    reg [WIDTH-1:0] cnt_out;
    always_ff @(posedge clk_counter or negedge rst_counter_n) begin
        if (!rst_counter_n)
            cnt_out <= 0;
        else if (en)
            cnt_out <= cnt_out + 1;
    end
endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////Set and Rest flop for flopping the cnt value around OVERLAP and duty //////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
module sr_ff_async (
    input  wire clk_srff,
    input  wire reset_srff_n,
    input  wire S,    
    input  wire R,     
    output reg  Q      
);
  always @(posedge clk_srff or negedge reset_srff_n) begin
    if (!reset_srff_n) begin
      Q <= 1'b0;               
    end else begin
      case ({S, R})
        2'b10: Q <= 1'b1;
        2'b01: Q <= 1'b0;
        2'b00: Q <= Q;
        2'b11: Q <= 1'b0;
      endcase
    end
  end
endmodule