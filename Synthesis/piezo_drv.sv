module piezo_drv #(
    parameter logic fast_sim = 0)
    (input clk,
    input rst_n,
    input en_steer,
    input too_fast,
    input batt_low,
    output piezo,
    output piezo_n);

    //3sec counter
    logic three_sec_done;
    logic [27:0] three_sec_counter;
    logic three_sec_start;

    //Duration counter
    logic [27:0] duration_counter;
    logic [27:0] duration;
    logic [27:0] next_duration;
    logic duration_done;

    //frequency generator
    logic [14:0] time_period_counter;
    logic [14:0] frequency_clk_cnt;
    logic [14:0] next_frequency_clk_cnt;
    logic [13:0] half_frequency_clk_cnt;

    //states
    typedef enum logic [2:0] { IDEAL,G6,C7,E7_23,G7,E7_22,G7_24} state_t;
    state_t state,next_state;

    //fast sim
    logic [6:0] increments;


    //fastsim increments
    generate
        if (fast_sim)
        assign increments = 7'd64;
        else
        assign increments = 7'd1;
    endgenerate

    logic [27:0] three_sec_value_reg;
    always_ff @( posedge clk, negedge rst_n ) begin 
        if (!rst_n)
            three_sec_value_reg <= 28'd150000000;
    end

    //3 sec counter 150,000,000 cycles  resets the counter when duration of a particular stage done
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n)
        three_sec_counter <= 28'b0;
        else if (three_sec_start)
        three_sec_counter <= 28'b0;
        else
        three_sec_counter <= three_sec_counter + increments;
    end
    assign three_sec_done = (three_sec_counter == three_sec_value_reg) ? 1'b1 : 0;


    // Method 1:
    // // Using a reg for three sec done signal
    // logic three_sec_done_reg;

    // always @(posedge clk or negedge rst_n) begin
    //     if (!rst_n)
    //         three_sec_done_reg <= 1'b0;
    //     else if (duration_done)
    //         three_sec_done_reg <= 1'b0; // reset whenever you reset counter
    //     else if (three_sec_counter >= 28'd150000000)
    //         three_sec_done_reg <= 1'b1;
    //     else
    //         three_sec_done_reg <= 1'b0;
    // end
    // assign three_sec_done = three_sec_done_reg;

    // Method 2:
    // assign three_sec_done = (three_sec_counter == 28'd150000000) ? 1'b1 : 1'b0;

    //counter for duration
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n)
        duration_counter <= 28'b0;
        else if (duration_counter == duration)
        duration_counter <= 28'b0;
        else
        duration_counter <= duration_counter + increments;
    end
    assign duration_done = (duration_counter == duration) ? 1'b1 : 0; 

    //register for duration value
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n)
        duration <= 28'b0;
        else
        duration <= next_duration;
        
    end

    //frequency generator
    always @ (posedge clk, negedge rst_n) begin
        if(!rst_n)
        time_period_counter <= 0;
        else if (time_period_counter >= frequency_clk_cnt) //frequency_clk_cnt is full time period
        time_period_counter <= 0;
        else
        time_period_counter <= time_period_counter + increments;
    end

    assign half_frequency_clk_cnt = frequency_clk_cnt >> 1'b1;
    assign piezo = (time_period_counter <= half_frequency_clk_cnt) ? 1 : 0;  //half_frequency_clk_cnt is half time period  
    assign piezo_n = ~piezo;

    //frequency's clock count register
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n)
        frequency_clk_cnt <= 15'b0;
        else
        frequency_clk_cnt <= next_frequency_clk_cnt;
        
    end

    //state machine register
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n)
        state <= IDEAL;
        else
        state <= next_state;
    end

    //FSM
    always_comb begin : FSM
        next_state = state;
        next_frequency_clk_cnt = frequency_clk_cnt;
        next_duration = duration;
        three_sec_start = 0;

        case (state)
            IDEAL : begin
                    if((too_fast))begin
                        next_state = G6;
                        next_frequency_clk_cnt = 15'd31888;
                        next_duration =  28'h0800000;
                        three_sec_start = 1'b1;
                    end
                    else if(batt_low)  begin
                        next_state = G7_24;
                        next_frequency_clk_cnt = 15'd15944;
                        next_duration = 28'h2000000;
                        three_sec_start = 1'b1;
                    end
                    else if((en_steer))begin
                        next_state = G6;
                        next_frequency_clk_cnt = 15'd31888;
                        next_duration =  28'h0800000;
                        three_sec_start = 1'b1;
                    end
                    else begin
                        next_frequency_clk_cnt = 15'd0;
                    end

                end
            G6 : begin
                    if (duration_done &&  (too_fast))begin
                        next_state = C7;
                        next_frequency_clk_cnt = 15'd23890;
                        next_duration = 28'h0800000;
                    end

                    else if(duration_done &&  batt_low && three_sec_done)begin
                        next_state = G7_24;
                        next_frequency_clk_cnt = 15'd15944;
                        next_duration = 28'h2000000;
                        three_sec_start = 1'b1;
                    end
                    else if(duration_done &&  batt_low && ~three_sec_done)begin
                        next_state = state;
                        next_frequency_clk_cnt = 15'd0;
                        next_duration = 28'd0;
                        // next_duration = 28'd150000001;
                    end
                    else if(duration_done)begin
                        next_state = C7;
                        next_frequency_clk_cnt = 15'd23890;
                        next_duration = 28'h0800000;
                    end
                    // else if ((~en_steer && ~too_fast && ~batt_low )) begin
                    //     next_state = IDEAL;
                    //     next_frequency_clk_cnt = 15'd0;
                    // end
            end
            C7 : begin
                    if (duration_done &&  (too_fast))begin
                        next_state = E7_23;
                        next_frequency_clk_cnt = 15'd18961;
                        next_duration = 28'h0800000;
                    end
                    else if(duration_done &&  batt_low)begin
                        next_state = G6;
                        next_frequency_clk_cnt = 15'd31888;
                        next_duration = 28'h0800000;
                    end
                    else if(duration_done)begin
                        next_state = E7_23;
                        next_frequency_clk_cnt = 15'd18961;
                        next_duration = 28'h0800000;
                    end
            end
            E7_23 : begin
                    if(duration_done && too_fast)begin
                        next_state = G6;
                        next_frequency_clk_cnt = 15'd31888;
                        next_duration = 28'h0800000;
                    end
                    else if(duration_done &&  batt_low)begin
                        next_state = C7;
                        next_frequency_clk_cnt = 15'd23890;
                        next_duration = 28'h0800000;
                    end
                    else if(duration_done && en_steer)begin    ////*****?
                        next_state = G7;
                        next_frequency_clk_cnt = 15'd15944;
                        next_duration = 28'h0C00000;
                    end
                    else if ((~en_steer && ~too_fast && ~batt_low )) begin
                        next_state = IDEAL;
                        next_frequency_clk_cnt = 15'd0;
                    end
            end
            G7 : begin
                    if(duration_done &&  batt_low)begin
                        next_state = E7_23;
                        next_frequency_clk_cnt = 15'd18961;
                        next_duration = 28'h0800000;
                    end
                    else if(duration_done)begin
                        next_state = E7_22;
                        next_frequency_clk_cnt = 15'd18961;
                        next_duration = 28'h0400000;
                    end
            end
            E7_22 : begin
                    if(duration_done &&  batt_low)begin
                        next_state = G7;
                        next_frequency_clk_cnt = 15'd15944;
                        next_duration = 28'h0C00000;
                    end
                    else if(duration_done)begin
                        next_state = G7_24;
                        next_frequency_clk_cnt = 15'd15944;
                        next_duration = 28'h2000000;
                    end
            end
            G7_24 : begin
                    if(duration_done && too_fast)begin
                        next_state = G6;
                        next_frequency_clk_cnt = 15'd3188;
                        next_duration = 28'h0800000;
                    end
                    else if(duration_done && batt_low)begin
                        next_state = E7_22;
                        next_frequency_clk_cnt = 15'd18961;
                        next_duration = 28'h0400000;
                    end
                    else if (duration_done &&  en_steer && three_sec_done) begin
                        next_state = G6;
                        next_frequency_clk_cnt = 15'd3188;
                        next_duration = 28'h0800000;
                        three_sec_start = 1'b1;
                    end
                    else if (duration_done &&  en_steer && ~three_sec_done) begin
                        next_state = state;
                        next_frequency_clk_cnt = 15'd0;
                        next_duration = 28'd0;
                        // next_duration = 28'd15000001;
                    end
                    else if ((~en_steer && ~too_fast && ~batt_low )) begin
                        next_state = IDEAL;
                        next_frequency_clk_cnt = 15'd0;
                    end
            end
            default: begin
                next_state = IDEAL;
                next_frequency_clk_cnt = 15'd0;
            end
        endcase
    end

endmodule
