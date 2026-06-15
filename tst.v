module tst(
			input clk_50,
			input m1_a,
			input m1_b,
			
			output reg l1,
			output reg l2
			);
			

 reg pulse_1, pulse_2;
 reg [13:0] duty_cycle_l;
 reg  [13:0] duty_cycle_r;
 wire ena1,enb1,in11,in21,in31,in41,p1;

initial 
	begin
	pulse_1=m1_a;
	pulse_2=m1_b;
	duty_cycle_l=5000;
	duty_cycle_r=5000;
	l1=1;
	l2=1;
	
	end
	
//	 botrun u1(
//    .clk50(clk_50),
//	 .duty_cycle_l(duty_cycle_l),
//	 .duty_cycle_r(duty_cycle_l),
//	 
//	 
//   
//     .ena(ena1),
//   .in1(in11),
//    .in2(in21),
//      .enb(enb1),
//     .in3(in31),
//     .in4(in41),
//	  .pwm_signal(p1)
//);
			
			
endmodule 