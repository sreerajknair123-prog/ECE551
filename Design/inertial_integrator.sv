// Team Name : SyntheSizers
// Members : Abhinarayani Velu Radhakrishnan
//           Aditi Kapil Chintawar 
//           Shreya Somani
//           Sreeraj Kannakarankodi

module inertial_integrator (
    input logic clk,
    input logic rst_n,
    input logic vld,
    input logic signed [15:0] ptch_rt,
    input logic signed [15:0] AZ,
    output logic signed [15:0] ptch
);

    // This offest is due to soldering the gyro/accelerometer on the application device
    localparam PTCH_RT_OFFSET = 16'h0050;
    localparam AZ_OFFSET = 16'h00A0;

    logic signed [26:0] ptch_int;               // Accumulating ptch_rt here
    logic signed [15:0] ptch_rt_comp;           // The compensated value of ptch_rt after accounting for the PTCH_RT_OFFSET
    logic signed [15:0] AZ_comp;                // The compensated value of the AZ componenet from the accelerometer using AZ_OFFSET
    logic signed [25:0] ptch_acc_product;       // Pitch from accelerometer with the fudge factor multiplied
    logic signed [15:0] ptch_acc;               // This is the pitch angle calculated from accel only
    logic signed [26:0] fusion_ptch_offset ;    // Fusion pitch offset decided by comparing ptch_acc and ptch 

    // Calculating the compensated values of pitch rate from gyro and the AZ componenet from the accelerometer
    assign ptch_rt_comp = ptch_rt - $signed(PTCH_RT_OFFSET);
    assign AZ_comp = AZ - $signed(AZ_OFFSET);

    // Calclating the pitch purely from the accelerometer
    assign ptch_acc_product = AZ_comp * $signed(327);                           // 327 is a fudge factor
    assign ptch_acc = {{3{ptch_acc_product[25]}},ptch_acc_product[25:13]};      // This is the pitch angle calculated from accel only

    assign fusion_ptch_offset = (ptch_acc > ptch) ? 27'd1024 : -27'd1024;       // Deciding the fusion ptch offset according to the values of ptch_acc and ptch

    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            ptch_int <= 27'b0;
        end    
        else if (vld) begin
            ptch_int <= ptch_int - {{11{ptch_rt_comp[15]}},ptch_rt_comp} + fusion_ptch_offset;  // Accumulating pitch rate
        end
    end
    assign ptch = ptch_int[26:11];  // We have essentially divided the value of ptch_int by 11 as use it as the final pitch - this is arrived from trial and error over a real segway

endmodule 