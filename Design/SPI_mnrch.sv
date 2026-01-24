`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
///////SPI (Serial Peripheral Interconnect) Monarch Tranceiver Module/////////////
//////////////////////////////////////////////////////////////////////////////////
module SPI_mnrch(
    input wire clk, //System Clock 50MHz
    input wire rst_n, // Active Low Reset
    input wire wrt, // To initiate SPI Transaction
    input wire [15:0] wt_data, // Data to be Transmitted to Serf
    output reg done, //flopped transaction Done out
    output logic [15:0] rd_data, // the MISO Data's sampled
    output reg SS_n, //Active Low Signal - To tell which serf is selected
    output wire SCLK, // GENERATED SCLK that goes towards Serfs
    output wire MOSI, // from Monarch to Serf serial output
    input wire MISO  //from Serf to Monarch - Serial input
 );
//// Signals specific to SCLK generation
reg [3:0] sclk_div; // SCLK Div counter reg
logic ld_SCLK; // Coming from State Machine
reg [3:0]bit_cnt4; // 4 Bit counter reg for tracking number of shifts done
logic smpl; // Sample signal for MISO based on the sclk_div counter
logic shft_im;//shift imminent - About to occur
logic shft; // Shft signal coming from State Machine
reg [15:0]shft_reg;   // Shift Register common for both MOSI and MISO
reg [15:0]shft_reg_D; // Intermediate Shift Register common for both MOSI and MISO
reg MISO_smpl; // Register to sample the MISO
logic init; // Will come from SM whenever or only when we get wrt
logic done15; // To check for count the shifts - Will go to SM - 15 times
logic set_done; // signal from state machine that act as SET/RESET for done signal generation

////////////////////////////////////////////////////////////////////////
//////////////SCLK generation - 1/16th of the System Clock///////////////
////////////////////////////////////////////////////////////////////////
//To generate the required SCLK we need a 4bit counter that count 16 times per SCLK cycle
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        sclk_div <= 0;
    else if (ld_SCLK)
        sclk_div <= 4'b1011; //To set the Front porch (Gap with SS_n) of 4 clock cycles - Aisle state and SCLK = 1 at the beginning
    else if (!ld_SCLK)
        sclk_div <= sclk_div + 1; //Increment and roll over when ld_SCLK is down
end
assign SCLK = sclk_div[3]; //SCLK is tapped into the MSB of the Clock divider
//////////////////////////////////////////////
/////////////////////DECODE///////////////////
//////////////////////////////////////////////
//When SCLK falls - SHIFT the Shift Register - 1111 - shft_im signal
//When SCLK rises - SAMPLE MISO  - 0111
assign smpl = (sclk_div == 4'b0111); // MISO will be sampled in the next increment of the clkdiv counter
assign shft_im = (sclk_div == 4'b1111);// It will be shifted in the next +1 count when it becomes 0000
////////////////////////////////////////////////////////////////
/////////////////////4 bit Counter for DONE//////////////////////
////////////////////////////////////////////////////////////////
//To keep a track of how many times shift has happened
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        bit_cnt4 <= 0;
    else if (init)
        bit_cnt4 <= 4'b0000; 
    else if (shft)
        bit_cnt4 <= bit_cnt4 + 1; //Every time we shift the shift register , we increments it.
    else
        bit_cnt4 <= bit_cnt4; // Otherwise Maintain
end
assign done15 = &bit_cnt4; //Will be set once the count value is 4'b1111
/////////////////////////////////////////////////////////
/////////////////////SHIFT REGISTER//////////////////////
/////////////////////////////////////////////////////////
always @(posedge clk or negedge rst_n) begin //Note that they are running on clk and not SCLK
    if(!rst_n)
        MISO_smpl <= 0;
    else if (smpl)
        MISO_smpl <= MISO;
end
assign shft_reg_D = {shft_reg[14:0],MISO_smpl};
/////////////////////////////////////////////////////////////////////////
////////Shifting in Sample MISO into the Shift Register//////////////////
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        shft_reg <= 0; //RESET before loading new wt_data during init
    else if (init ) //Comes from SM when wrt input goes high (When Transmission Starts)
        shft_reg <= wt_data; // Initial loading of the parallel 16 bit wt_data input
    else if ({init,shft} == 2'b01) //shft comes from SM in the next count after shft_imm is met
        shft_reg <= shft_reg_D; // Shift only when the 4 bit counter counter reaches 4'b1111
    else if ({init,shft} == 2'b00)
        shft_reg <= shft_reg; // Freeze until next init happens again
end
assign MOSI = shft_reg[15]; // The shifted out value - MSB forms "MOSI"
/////////////////////////////////////////////////////////////////////////////////
////////////////////////////SR FF///////////////////////////////////////////////
//////// set_done and init act as SET and RESET for SS_n and done///////////////
///////////////////////////////////////////////////////////////////////////////
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        SS_n <= 1; //PRESET
    else if (!set_done && !init) 
            SS_n <= SS_n;
    else if (!set_done && init)  // Transaction Start
            SS_n <= 0; // Will be synchronous to transaction start which is basically wrt
    else if (set_done && !init)  // Transaction Done - set_done comes from the State Machine BACK_PORCH state
            SS_n <= 1; 
end
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        done <= 0; //RESET
    else if (!set_done && !init) 
            done <= done;
    else if (!set_done && init)  // Transaction Start
            done <= 0;
    else if (set_done && !init)  // Transaction Done - set_done comes from the State Machine BACK_PORCH state
            done <= 1;
end
assign rd_data = shft_reg; // Final MOSI Value - Will be valid only when set_done
/////////////////////////////////////////////////////////
/////////////////////STATE MACHINE//////////////////////
/////////////////////////////////////////////////////////
typedef enum reg [2:0] { IDLE, FRONT_PORCH , WORK_HORSE , BACK_PORCH} state_t;
state_t state,next_state;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        state  <= IDLE;
    else
        state <= next_state;
end
always_comb begin 
    ///////outputs initialized to avoid latch formation/////
    init = 0;
    ld_SCLK = 1;//defaulted to 1 for SCLK to be defauted to be 1
    shft = 0;
    next_state = state;
    set_done = 0;
    case (state)
        IDLE : begin
        if(wrt) begin
            init = 1; //This will set SS_n to Low 
            ld_SCLK = 0;//Same time as when init is initialized SCLK needs to be loaded
            next_state = FRONT_PORCH;
        end
        end
        FRONT_PORCH : begin // First Rising edge of SCLK - NO SHIFT
            ld_SCLK = 0;
            if(shft_im) begin
                shft = 0;
                next_state = WORK_HORSE;            
            end
        end
        WORK_HORSE : begin
            ld_SCLK = 0;
            /////////////////Sample on the Rise of the SCLK // Shift on the Fall of SCLK////////////////
            ///////Leave this state if the Bit Counter 1 - Only for the last shift in the BACK PORCH////
            if (bit_cnt4 == 4'b1111) 
                next_state = BACK_PORCH;
            else if(shft_im) 
                shft = 1; 
        end
        BACK_PORCH: begin
            ////////inhibit the last fall of SCLK//////////////////////////////
            ///// Shift the shift reg where the SCLK would have fallen/////////
            ld_SCLK = 0;
            if(shft_im) begin
                ld_SCLK = 1;
                shft = 1; 
                next_state = IDLE;
                set_done = 1;//This will be flopped to become set_done(To avoid glitching)
            end
        end
        default: // Always give default to avoid unwanted loops in case of actual external hardware issues
            next_state = IDLE;
    endcase
end
endmodule
`default_nettype wire
