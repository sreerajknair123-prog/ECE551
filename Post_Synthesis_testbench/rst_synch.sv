////////////////////////////////////////////////////////////////////////////////////////
//////////synchronizer that takes in the raw push button signal and ////////////////////
//////////creates a signal that is deasserted at the negative edge of clock////////////
//////////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps

module rst_synch(RST_n, clk, rst_n);

    input RST_n;
    input clk;
    output rst_n;
    //////temporary wires for the flops///////
    reg Q_1ff, Q_2ff;
    //We want the reset de-asserted on the opposite edge of clocks that other flops are active on.
    // Thus Reset de-asserted on the negative edge of the clock
    always_ff @(negedge clk, negedge RST_n) 
      if (!RST_n) begin
        Q_1ff <= 0;
        Q_2ff <= 0;
        end
      else begin
        Q_1ff <= 1'b1;
        Q_2ff <= Q_1ff;
        end
    assign rst_n = Q_2ff;

endmodule