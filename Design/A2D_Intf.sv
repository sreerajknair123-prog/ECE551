module A2D_intf(
    input wire rst_n,
    input wire clk,
    input wire nxt, //Trigger to start sending out transctions from SPI to A2D
    output reg [11:0] lft_ld,
    output reg [11:0] rght_ld,
    output reg [11:0] steer_pot,
    output reg [11:0] batt,
    output reg SS_n, //Active Low Signal - To tell which serf is selected - 
    output wire SCLK, // GENERATED SCLK that goes towards Serfs
    output wire MOSI, // from Monarch to Serf serial output
    input wire MISO  //from Serf to Monarch - Serial input
);

    logic wrt;
    logic done;
    logic [15:0]wrt_data;
    logic [15:0]rd_data;

    /////////Monarch Instance/////////
    SPI_mnrch SPI_mnrch_inst(
    .clk(clk), //System Clock 50MHz
    .rst_n(rst_n),
    .wrt(wrt), //Comes from the state machine
    .wt_data(wrt_data), 
    .done(done), //Goes to the state machine
    .rd_data(rd_data), // Goes to the 4 Registers in A2D_Intf
    .SS_n(SS_n), //Active Low Signal - To tell which serf is selected
    .SCLK(SCLK), // SPI Clock supplied to A2D for sampling
    .MOSI(MOSI), // from Monarch to A2D serial output
    .MISO(MISO)  //from A2D to Monarch - Serial input
    );

    logic en_lft_load_reg;
    logic en_rght_load_reg;
    logic en_steer_pot_reg;
    logic en_batt_reg;
    wire [15:0] wrt_data_chn0;
    wire [15:0] wrt_data_chn4;
    wire [15:0] wrt_data_chn5;
    wire [15:0] wrt_data_chn6;
    //First Transaction Write data for each channels going to the SPI then to the A2D Converter
    assign wrt_data_chn0 = {2'b00,3'b000,11'h000};
    assign wrt_data_chn4 = {2'b00,3'b100,11'h000};
    assign wrt_data_chn5 = {2'b00,3'b101,11'h000};
    assign wrt_data_chn6 = {2'b00,3'b110,11'h000};
    //////////////////////////////////////////////////////////////////////
    ////////////////rd_data assignment to the Hold Registers//////////////
    always @(posedge clk or negedge rst_n) 
        if(!rst_n) begin
            lft_ld <=0;
            rght_ld <=0;
            steer_pot <=0;
            batt <='hFFF; //Making the battery out by default not as low
        end
        else if (en_lft_load_reg)
            lft_ld <= rd_data;
        else if (en_rght_load_reg)
            rght_ld <= rd_data;
        else if (en_steer_pot_reg)
            steer_pot <= rd_data;
        else if (en_batt_reg)
            batt <= rd_data;

    ///////////////////////////////////
    ////Round Robin 2 bit cointer//////
    //Round Robin conversions on Channel 0 , 4, 5 and 6
    // Every time a nxt comes the round robin counter increments by 1 :
    // 1st nxt - 00 -> lft_load_reg
    // 2nd nxt - 01 -> rght_load_reg
    // 3rd nxt - 10 -> steer_pot_reg
    // 4th nxt - 11 -> batt_reg
    // 1st nxt - 00 -> Then overrides the value in reg_lft_ld and so on
    reg [1:0] rrb_cnt;
    logic nxt_trig_in;
    logic wait_1cycle;
    always @(posedge clk or negedge rst_n) // TODO : Should I just look for nxt signal and not on clock?
    if(!rst_n)
        rrb_cnt <=0;
    else if(nxt_trig_in) // Every time nxt comes in --> Should i just replace it with nxt? - Do I really need to flop it????
        rrb_cnt <= rrb_cnt + 1;

    assign wrt_data =   (rrb_cnt == 2'b00) ? wrt_data_chn0 :
                        (rrb_cnt == 2'b01) ? wrt_data_chn4 :
                        (rrb_cnt == 2'b10) ? wrt_data_chn5 :
                        (rrb_cnt == 2'b11) ? wrt_data_chn6 : 16'hx;

     //The registers get refreshed once every 4 conversions
    //State machine goes full loop for every nxt - This is just tracking the Monarch
    reg reg_wait_1cycle;
    always @(posedge clk or negedge rst_n) // TODO : Should I just look for nxt signal and not on clock?
    if(!rst_n)
        reg_wait_1cycle <= 1'b0;
    else if(wait_1cycle) // Every time nxt comes in --> Should i just replace it with nxt?
        reg_wait_1cycle <= 1'b1;
    /////////////////////////////////////////////////////////
    /////////////////////STATE MACHINE//////////////////////
    /////////////////////////////////////////////////////////
    typedef enum reg [1:0] {IDLE , TRANSCTN1 , DEAD_PERIOD , TRANSCTN2 } state_t;
    state_t state,next_state;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            state  <= IDLE;
        else
            state <= next_state;
        end
    always_comb begin 
        next_state = state;
        nxt_trig_in = 0;
        wait_1cycle = 0;
        wrt = 0;
        en_lft_load_reg = 0;
        en_rght_load_reg = 0;
        en_steer_pot_reg = 0;
        en_batt_reg = 0;

        case (state)
            IDLE : begin
                if(nxt) begin //Every time a pulse comes in on nxt we initiate an SPI transaction and we increment the 2bit counter
                    next_state = TRANSCTN1;
                    wrt = 1;
                end
            end
            TRANSCTN1 : begin
                if (done) begin // We wait until the SPI transaction is sent and done by the Monarch
                    wait_1cycle = 1; //Use this variable and flopping it outside for 1 clock cycle
                    next_state = DEAD_PERIOD; // Then we go to the Deadperiod to wait for 1 clock cycle
                end
            end     
            DEAD_PERIOD : begin
                wrt = 0; // We will trigger Write again for the second transaction
                if (reg_wait_1cycle) begin//Flopped for one clock cycle
                    next_state = TRANSCTN2;
                    wrt = 1; // Sending a transaction - But Doesn't matter the wrt_data as we are just reading from the SPI at this time
                end
            end             
            TRANSCTN2 : begin
                if (done) begin
                    nxt_trig_in = 1; //Triggering the Round Robin counter to  increment by 1
                    next_state = IDLE;
                    if (rrb_cnt == 2'b00)
                        en_lft_load_reg = 1;
                    else if (rrb_cnt == 2'b01)
                        en_rght_load_reg = 1;
                    else if (rrb_cnt == 2'b10)
                        en_steer_pot_reg = 1;
                    else if (rrb_cnt == 2'b11)
                        en_batt_reg = 1;
                end
            end  
            default:
                next_state = IDLE;
        endcase
    end
endmodule