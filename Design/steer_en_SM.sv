///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////STEERING ENABLE STATE MACHINE///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module steer_en_SM(clk,rst_n,tmr_full,sum_gt_min,sum_lt_min,diff_gt_1_4,
                   diff_gt_15_16,clr_tmr,en_steer,rider_off);

  input clk;				// 50MHz clock
  input rst_n;				// Active low asynch reset
  input tmr_full;			// asserted when timer reaches 1.3 sec
  input sum_gt_min;			// asserted when left and right load cells together exceed min rider weight
  input sum_lt_min;			// asserted when left_and right load cells are less than min_rider_weight

  /////////////////////////////////////////////////////////////////////////////
  // HEY HOFFMAN...you are a moron.  sum_gt_min would simply be ~sum_lt_min. 
  // Why have both signals coming to this unit??  ANSWER: What if we had a rider
  // (a child) who's weigth was right at the threshold of MIN_RIDER_WEIGHT?
  // We would enable steering and then disable steering then enable it again,
  // ...  We would make that child crash(children are light and flexible and 
  // resilient so we don't care about them, but it might damage our Segway).
  // We can solve this issue by adding hysteresis.  So sum_gt_min is asserted
  // when the sum of the load cells exceeds MIN_RIDER_WEIGHT + HYSTERESIS and
  // sum_lt_min is asserted when the sum of the load cells is less than
  // MIN_RIDER_WEIGHT - HYSTERESIS.  Now we have noise rejection for a rider
  // who's weight is right at the threshold.  This hysteresis trick is as old
  // as the hills, but very handy...remember it.
  //////////////////////////////////////////////////////////////////////////// 

  input diff_gt_1_4;		// asserted if load cell difference exceeds 1/4 sum (rider not situated)
  input diff_gt_15_16;		// asserted if load cell difference is great (rider stepping off)
  output logic clr_tmr;		// clears the 1.3sec timer
  output logic en_steer;	// enables steering (goes to balance_cntrl)
  output logic rider_off;	// held high in intitial state when waiting for sum_gt_min
///////////////////////////////////////////////////////////////////////////
// Defining 3 states for the SM /////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
  typedef enum reg [1:0] {INIT,STEADY_CHK,STEER_EN} state_t;
  state_t state,nxt_state;
///////////////////////////////////////////////////////////////////////////////
// 2 always block seperate for comb logic and flop logic for state tansition //
///////////////////////////////////////////////////////////////////////////////
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) 
      state <= INIT ;
    else 
      state <= nxt_state;
  end
  always_comb begin
    rider_off = 0;
    clr_tmr = 0;
    nxt_state = state;
    en_steer = 0;
    case(state)
      INIT: begin
          rider_off = 1;
          en_steer = 0;
          if(sum_gt_min) 
            nxt_state = STEADY_CHK;
        end
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// STEAD STATE Check wil move to steer en state only if the diff_gt_1_4 stays low for 1.34s (timer full condtion)////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      STEADY_CHK: begin
        if (!diff_gt_1_4 && tmr_full) // if the timer is full (which also means that diff_gt_1_4 stayed low throughout as we will be clearing if at all it goes high)
          nxt_state = STEER_EN;
        else if (diff_gt_1_4) begin //it should maintain the steady state if the diff is greater than the 1/4th of the the sum
          clr_tmr = 1;//Clearing the timer to 0
          nxt_state = STEADY_CHK;
        end
        else if (sum_lt_min)
          nxt_state = INIT;
      end
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// STEER Enable can go back to steady state or INIT state based on 2 seperate conditions////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      STEER_EN: begin
        en_steer = 1;
        if (sum_lt_min) 
          nxt_state = INIT;
        else if (diff_gt_15_16) begin
          nxt_state = STEADY_CHK;
          en_steer = 0;
        end
      end 
      default :  
          nxt_state = INIT;
    endcase
  end
endmodule