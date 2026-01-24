//`default_nettype none
/////////////////////////////////////////////////////////////
/////Universal Asynchronous Receiver Transmitter (UART)//////
////////////Receiver Implementation ///////////////////////
/////////////////////////////////////////////////////////////
module UART_rx(
    input logic clk,
    input logic rst_n,
    input logic RX,
    input logic clr_rdy,
    output logic [7:0] rx_data,
    output logic rdy
);
////////////////////////
/////Wires/ports///////
///////////////////////
wire [1:0] ST_SH_R; //start/shift
logic start, shift;
wire [1:0] ST_SH; //start/shift/transmitting
logic [12:0] baud_cnt; //baud counter
logic receiving;
logic set_rdy;
logic [3:0] bit_cnt; //bit counter
logic [8:0] rx_shft_reg; //shift register
logic RX_ff,RX_2ff;
////////////////////////////////////
/////assignments for SM outputs//////
////////////////////////////////////
assign ST_SH =  {start,shift};
assign ST_SH_R = {start|shift,receiving};
//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////Double flopping RX///////////////////////////////////
/////////Since they have same clock and reset conditions 1 always block is sufficent//////
//////////////////////////////////////////////////////////////////////////////////////////
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        RX_ff <= 1'b1; // PRESET - As IDLE stat if RX lane is high (0 can wrongly indicate it as a Start bit)
        RX_2ff <= 1'b1; // PRESET - As IDLE stat if RX lane is high (0 can wrongly indicate it as a Start bit)
    end
    else begin
        RX_ff  <= RX;   
        RX_2ff <= RX_ff;   
    end
end
////////////////////////////
//Shift Register///////////
///////////////////////////
//RX comes into MSB - We are oing a shift 10 times into a 9 bit register - START bit comes first
//Sampling D0 at the MSB of the data shift register
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin 
        rx_shft_reg <= 9'h1FF; //Do We need to PRESET? -Yes becuase the normal state is high
    end
    else if (shift) begin
        rx_shft_reg <= {RX_2ff,rx_shft_reg[8:1]};
    end
    else begin
        rx_shft_reg <= rx_shft_reg; //freeze
    end  
end
assign rx_data = rx_shft_reg[7:0]; // Thus ensuring the start bit is falling off.
////////////////////////////////////////////////////////////
//bit counter - 4 bit //////////////////////////////////////
////////////////////////////////////////////////////////////
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        bit_cnt <= 4'b0;
    end
    else if (ST_SH == 2'b00) begin
        bit_cnt <= bit_cnt;
    end
    else if (ST_SH == 2'b01) begin
        bit_cnt <= bit_cnt + 1;
    end
    else 
        bit_cnt <= 4'b0;
end
assign shift =  (baud_cnt == 13'd0) ? 1'b1 : 1'b0; //5207 for 9600 baud with 50MHz clock  - Down counter check for 0
////////////////////////////////////////////////////////////////
////////////////////baud counter - 13bit - Down Counter/////////
/////////When start bit is detected, load 2604 else load 5205///
////////////////////////////////////////////////////////////////
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        baud_cnt <= 13'd5208;
    end
    else if (ST_SH_R == 2'b10 || ST_SH_R == 2'b11) begin
        baud_cnt <= start? 13'd2604 : 13'd5208;
    end
    else if (ST_SH_R == 2'b00) begin
        baud_cnt <= baud_cnt;
    end
    else if (ST_SH_R == 2'b01) begin
        baud_cnt <= baud_cnt - 1;
    end
end
/////////////////////////////////////////////////////////////////////////////////
////////////////////////////SR FF///////////////////////////////////////////////
//////// start and clr_rdy act as a synchronous clear to the flop///////////////
///////////////////////////////////////////////////////////////////////////////
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rdy <= 0;
    end
    else begin
        if (set_rdy  == 0 && (start|clr_rdy) == 0) 
            rdy <= rdy;
        else if (set_rdy == 0 && (start|clr_rdy) == 1)  
            rdy <= 0;
        else if (set_rdy == 1 && (start|clr_rdy) == 0)
            rdy <= 1;
    end
end 
///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////State Machine////////////////////////////////////////////
//////////////// Look at the RX line for start bit - Looking for a 0///////////////////
////////////////////// raising the rdy flag at the very end////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
typedef enum reg [1:0] {IDLE , RECEIVE} state_t;
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
    receiving = 1'b0;
    start = 1'b0;
    set_rdy = 1'b0;
    case (state)
        IDLE : begin
            if (!RX_2ff) begin
                next_state = RECEIVE;
                start = 1'b1;
            end
        end
        RECEIVE : begin
            receiving = 1'b1;
            start = 1'b0;
            if (bit_cnt == 4'd10) begin
                next_state = IDLE;
                set_rdy = 1'b1;
            end
        end
        default: 
            next_state = IDLE;
    endcase
end
endmodule
//`default_nettype wire