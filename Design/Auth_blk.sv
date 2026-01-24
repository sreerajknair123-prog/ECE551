////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////Auth BLK  for Authorisation Control for the //////////////////////////////////////////
//////////////////////////////BLE Connection link from an external BLE device///////////////////////////////////////
//BLE(Phone) to BLE(Segway) Linking --> Transmission of the received via UART TX to RX to the Authorization Block///
// Which will in turn authorize or unauthorize (Power Up / Standby / Shutdown) based on the received inputs/////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module Auth_blk(
    input logic clk,
    input logic rst_n, // Active Low Reset
    input logic RX, // Connected to the UART TX which is in turn connected to the BLE module
    input logic rider_off, // Coming from en_steer block
    output logic pwr_up
);
///Local Assignments
logic [7:0]rx_data_in;
logic clr_rx_rdy_o;
logic rx_rdy;
///////UART RX Instance///////
UART_rx dut_rx(
    .clk(clk),
    .rst_n(rst_n),
    .RX(RX), // Coming from TX
    .clr_rdy(clr_rx_rdy_o), //input comes from auth_SM
    .rx_data(rx_data_in), //output goes to auth_SM
    .rdy(rx_rdy) // output goes to auth_Sm
);
//////// clr_rdy assignment/////////
assign clr_rx_rdy_o = rx_rdy? 1'b1 : 1'b0;  // We just need to clear it whenever rx_rdy comes in
///////////////////////////////////////////////
///////////////Auth State Machine//////////////
///////////////////////////////////////////////
////State 1 : SEGWAY_OFF - Will shift to SEGWAY_ON only if it received a valid 'G' data input
////State 2 : SEGWAY_ON - Will Power up , but if it receives a valid 'S' input it will 
////          go to SEGWAY_STANDBY (Power up still remain asserted). In this situaion if the 
////          rider also goes off it will jump to SEGWAY_OFF state
////State 3 : SEGWAY_STANDBY - Power is still up but if the rider goes off it will go to SEGWAY_OFF 
////          state amd Power is Down(Shut Down). Incase in this state a valid 'G' input comes in state will go back to 
////          SEGWAY_ON State.
typedef enum reg [1:0] {SEGWAY_OFF,SEGWAY_ON,SEGWAY_STANDBY} state_t;
state_t state,next_state;
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        state <= SEGWAY_OFF;
    else
        state <=next_state;
end
always_comb begin
    pwr_up = 0;
    next_state = state;
    case (state)
        SEGWAY_OFF : begin
            pwr_up = 0;//Deasserted only when the SEGWAY is fully off and the rider is off
            if((rx_data_in == 8'h47) && rx_rdy) begin
                next_state = SEGWAY_ON;
            end
        end
        SEGWAY_ON : begin
            pwr_up = 1; //Power Up is Asserted When the Connection is Established by 'G'
            if ((rx_data_in == 8'h53) && rx_rdy) begin //Disconnect due to some reason (But not yet sure if the rider is OFF or not)
                if(rider_off)
                    next_state = SEGWAY_OFF; // Should Shut Down if the rider gets off after the Disconnect
                else
                    next_state = SEGWAY_STANDBY;// Should StandBy if the rider is not off yet
            end
        end
        SEGWAY_STANDBY : begin
            pwr_up = 1;
            if(rider_off) // Will shut down if the rider is OFF - High prio - While on STANDBY mode
                next_state = SEGWAY_OFF;
            else if ((rx_data_in == 8'h47) && rx_rdy) begin// Case where rider is not off yet and the device again gets an Appropriate Authentication it has to go back to the ON stage
               if (!rider_off)
                    next_state = SEGWAY_ON;
            end
        end
        default :
                next_state = SEGWAY_OFF;
    endcase
end
endmodule