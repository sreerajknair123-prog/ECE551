module steer_en #(parameter fast_sim = 0) (
    input logic clk,
    input logic rst_n,
    input logic [11:0] lft_ld, 
    input logic [11:0] rght_ld, 
    output logic en_steer,
    output logic rider_off
);

    localparam MIN_RIDER_WT  = 13'h200; 
    localparam WT_HYSTERESIS = 13'h40;

    logic tmr_full;
    logic clr_tmr;
    logic sum_gt_min;
    logic sum_lt_min;
    logic diff_gt_1_4;
    logic diff_gt_15_16;
    logic [12:0] ld_sum;
    logic signed [12:0] ld_diff; // Made it 13 bit (to accomodate a case of -4095)
    logic [11:0] abs_diff;
    logic [25:0] cntr_26bit;
    reg reg_clr_tmr;

    ////////steer_en_SM module instantiation//////
    steer_en_SM iDUT(.clk(clk),.rst_n(rst_n),.tmr_full(tmr_full),.sum_gt_min(sum_gt_min),
                    .sum_lt_min(sum_lt_min),.diff_gt_1_4(diff_gt_1_4),
                    .diff_gt_15_16(diff_gt_15_16),.clr_tmr(clr_tmr),.en_steer(en_steer),
                    .rider_off(rider_off));

    //////////clr_tmr flopped as it is coming from a state machine/////////
    always_ff @(posedge clk or negedge rst_n)
        if(!rst_n)
            reg_clr_tmr <= 0;
        else
            reg_clr_tmr <= clr_tmr;

    ////1.34s timer from 50Mhz Clock//////
    ////26bit counter
    always_ff @(posedge clk or negedge rst_n)
        if(!rst_n)
            cntr_26bit <= 0;
        else if (reg_clr_tmr | tmr_full) // Clearing it whenever the clr_tmr inputs comes in and also whenever we hit the required 1.34s time
            cntr_26bit <= 0;
        else
            cntr_26bit <= cntr_26bit + 1;

    ////////fast_sim generate case//////
    generate 
        if (!fast_sim)     // Fast Sim Case       
            assign tmr_full = (cntr_26bit == 26'd67000000);//Count value for 1.34s
        else              //Normal case
            assign tmr_full = &(cntr_26bit[14:0]);   
    endgenerate

    ////Arithmetic operatiosn to generate the inputs the steer_en state machine//////
    assign ld_sum        = lft_ld + rght_ld;
    assign ld_diff       = lft_ld - rght_ld;
    assign abs_diff      = ld_diff[12] ? (~ld_diff + 1) : ld_diff;
    assign diff_gt_15_16 = (abs_diff > (ld_sum - {4'b0,ld_sum[12:4]})  ); //Being a sum(+ve) performing arithmetic right shift for scaling
    assign diff_gt_1_4   = (abs_diff > {2'b0,ld_sum[12:2]}); //Being a sum(+ve) performing arithmetic right shift for scaling
    assign sum_gt_min    = (ld_sum > (MIN_RIDER_WT + WT_HYSTERESIS));
    assign sum_lt_min    = (ld_sum < (MIN_RIDER_WT - WT_HYSTERESIS));

endmodule