/////////////////////////////////////////////////////////////
/////Universal Asynchronous Receiver Transmitter (UART)//////
////////////Transmitter Implementation ///////////////////////
/////////////////////////////////////////////////////////////
module UART_tx(
    input logic clk,
    input logic rst_n,
    input logic trmt,
    input logic [7:0] tx_data,
    output logic TX,
    output logic tx_done
);
////////////////////////
/////Wires/ports///////
///////////////////////
wire [1:0] LS; //load/shift
logic load, shift;
wire [1:0] LST; //load/shift/transmitting
logic [3:0] bit_cnt; //bit counter
logic [12:0] baud_cnt; //baud counter
logic transmitting;
logic [8:0] tx_shft_reg; //shift register
logic set_done;
////////////////////////////////////
/////assignments for SM outputs//////
////////////////////////////////////
assign shift = (baud_cnt == 13'd5207) ? 1'b1 : 1'b0; //5207 for 9600 baud with 50MHz clock
assign LS =  {load,shift};
assign LST = {load|shift,transmitting};
////////////////////////////
//Shift Register///////////
///////////////////////////
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin 
        tx_shft_reg <= 9'h1FF; //We need to PRESET
    end
    else if ((LS == 2'b10)||(LS ==2'b11)) begin
        tx_shft_reg <= {tx_data,1'b0}; //load data into shift register
    end
    else if (LS == 2'b01) begin
        tx_shft_reg <= {1'b1,tx_shft_reg[8:1]};//right shift with 0 fill
    end
    else if (LS == 2'b00) begin
        tx_shft_reg <= tx_shft_reg; //hold
    end   
end
assign TX = tx_shft_reg[0]; //output LSB first - As we are doing right shift
///////////////////////////////
//baud counter - 13bit/////////
///////////////////////////////
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        baud_cnt <= 13'b0;
    end
    else if ((LST == 2'b11)||(LST == 2'b10)) begin
        baud_cnt <= 13'b0;//Load
    end
    else if (LST == 2'b01) begin
        baud_cnt <= baud_cnt + 1;
    end
    else if (LST == 2'b00) begin
        baud_cnt <= baud_cnt;
    end
end
//////////////////////////////
//bit counter - 4 bit ////////
//////////////////////////////
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        bit_cnt <= 4'b0;
    end
    else if ((LS == 2'b11)||(LS == 2'b10)) begin
        bit_cnt <= 4'b0;//Load
    end
    else if (LS == 2'b01) begin
        bit_cnt <= bit_cnt + 1;
    end
    else if (LS == 2'b00) begin
        bit_cnt <= bit_cnt;
    end
end
/////////////////////////////////////////////////////////////////////////////////
////////////////////////////SR FF///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        tx_done <= 0;
    else begin
        if (set_done ==0 && load ==0) 
            tx_done <= tx_done;
        else if (set_done ==0 && load ==1)  
            tx_done <= 0;
        else if (set_done ==1 && load ==0)
            tx_done <= 1;
    end
end 
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////State Machine//////////////////////////////////////////////////////////////
//bit_cnt = 4'd10 - We are done -> set_done -> tx_done flop Done (SR flop)///////////////////////////////
//IDLE to Tranmsit thru trmt signal -> that will assert load -> that will knock done tx_done (R of SR flipflop)////
//when transmitting is one , it is counting the baud counter/////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef enum reg [1:0] {IDLE , TRANSMIT} state_t;
state_t state, next_state;
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state <= IDLE;
    end
    else begin
        state <= next_state;
    end
end
always_comb begin
    next_state = state;
    transmitting = 1'b0;
    load = 1'b0;
    set_done = 1'b0;
    case (state)
        IDLE : begin
            if (trmt) begin
                next_state = TRANSMIT;
                load = 1'b1;
            end
        end
        TRANSMIT : begin
            transmitting = 1'b1;
            load = 1'b0;
            if (bit_cnt == 4'd10) begin
                next_state = IDLE;
                set_done = 1'b1;
            end
        end
        default: 
            next_state = IDLE;
    endcase
end
endmodule