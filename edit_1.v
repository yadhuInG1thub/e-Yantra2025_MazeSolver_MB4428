//latest botrun for harcoded
module botrun( //change module name to botrun

	input clk50,
	input m1_a, //right
	input m2_a, //left
	input ir_left,
	input ir_right,
	input ir_mid,
	 
   
    output reg ena, //right motor enable
    output reg in1, //
    output reg in2, //
    output reg enb, //left motor enable
    output reg in3, //
    output reg in4, //
	 //next direction using hardcode/mazeSolver
 
	input e_left,e_right,e_mid, //echoes of ultrasonic
	output t_right,t_left,t_mid, //triggers of ultrasonic
	output  [15:0]left_dist,
	output  [15:0]right_dist,
	output  [15:0]mid_dist,
 
	 ///////////
	 output reg [1:0] exp_state,
	 output reg mid_signal, //whether mid_dist is greater than a threshold
	 output reg right_signal,
	 output reg left_signal,
	 output reg en_explorer,
	 output [2:0]move_in
	
	 
);

localparam straight = 3'd1;
localparam left = 3'd2;
localparam right = 3'd3;
localparam uturn = 3'd4;
localparam stop = 3'd0;
localparam update = 3'd5;
localparam delay = 3'd6;

localparam left_normal = 5750;
localparam right_normal = 6400;
localparam straight_rpm = 1200;


reg rst_left;
reg rst_right;
reg rst_mid;

//wire rst_left;
//wire rst_right;
//wire rst_mid;

wire op_left;
wire op_right;
wire op_mid;
 
	 
	
//wire [2:0] move_state;
reg [18:0] rpm_right;
reg [18:0] rpm_left;


reg [13:0] duty_right;
reg  [13:0] duty_left;
reg [13:0] pwm_count;

reg pwm_signal;

reg move_update; //to give posedge input to mazesolver
reg [2:0] move_state_fsm; //register in the current module, includes update state as well

reg[1:0] wait_cycles; //to wait for the maze solver to respond

reg [18:0] rpm_new;

reg output_en; //output from mazeSolver given or not

reg reset_value; //signal to indicate that rpm count is to be reset (occurs at the transition to delay states)

reg [26:0] clk_delay=0;

reg s2; //To check for 1960 pulses

reg left_sig;
reg right_sig;
reg mid_sig;

ultrasonic left_ultra(.clk_50M(clk50), .reset(rst_left), .echo_rx(e_left), .trig(t_left), .op(op_left), .distance_out(left_dist));
ultrasonic right_ultra(.clk_50M(clk50), .reset(rst_right), .echo_rx(e_right), .trig(t_right), .op(op_right), .distance_out(right_dist));
ultrasonic mid_ultrA(.clk_50M(clk50), .reset(rst_mid), .echo_rx(e_mid), .trig(t_mid), .op(op_mid), .distance_out(mid_dist));

//mazesolver mazesolve(.move_update(move_update), .clk50(clk50), .output_en(output_en), .move_state(move_state));

//maze_explorer explore(
//    .clk(clk50),
//    .rst_n(1),
//    .left(left_sig), 
//	 .mid(mid_sig), 
//	 .right(right_sig), 
//    .move(move_in),
//	 .output_updated(out_updated) ,
//	 .enable(en_exp)
//);

reg left_turn_start;
reg right_turn_start;

reg [7:0] pointer;

reg [2:0] moves [0:119];

//try to remove
//reg output_en;
reg [2:0] move_state;
//end

initial begin
	left_turn_start = 0;
	right_turn_start = 0;
	move_update = 1;
	s2=0;
	reset_value = 0;
	
	pwm_count=0;
    ena = 1'b0;
    in1 = 1'b0;
    in2 = 1'b0;
    enb = 1'b0;
    in3 = 1'b0;
    in4 = 1'b0;
	 duty_left=0;
	 duty_right=0;
	 
	 rst_left = 1;
	 rst_mid = 1;
	 rst_right = 1;
	
	
	rpm_right=0;
	rpm_left=0;
	rpm_new = 0;
	wait_cycles = 1;
	move_update = 0;
	move_state_fsm = straight;
	left_sig = 1;
	mid_sig = 0;
	right_sig = 1;
	pointer = 0;
	output_en =0;
	moves[0] = straight;
	moves[1] = left;
	moves[2] = left;
	moves[3] = right;
	moves[4] = straight;
	moves[5] = straight;
	moves[6] = right;
	moves[7] = straight;
	moves[8] = right;
	moves[9] = left;
	moves[10] = left;
	moves[11] = uturn; ////////////////////////////
	moves[12] = left;
	moves[13] = left;
	moves[14] = right;
	moves[15] = right;
	moves[16] = straight;
	moves[17] = left;
	moves[18] = right;
	moves[19] = left;
	moves[20] = straight;
	moves[21] = left;
	moves[22] = left;
	moves[23] = uturn;
	moves[24] = left;
	moves[25] = left;
	moves[26] = right;
	moves[27] = left;
	moves[28] = left;
	moves[29] = uturn;
	moves[30] = right;
	moves[31] = right;
	moves[32] = left;
	moves[33] = left;
	moves[34] = uturn;
	moves[35] = straight;
	moves[36] = straight;
	moves[37] = right;
	moves[38] = straight;
	moves[39] = right;
	moves[40] = left;
	moves[41] = right;
	moves[42] = straight;
	moves[43] = left;
	moves[44] = left;
	moves[45] = straight;
	moves[46] = right;
	moves[47] = straight;
	moves[48] = straight;
	moves[49] = right;
	moves[50] = uturn;
	moves[51] = left;
	moves[52] = right;
	moves[53] = left;
	moves[54] = uturn;
	moves[55] = left;
	moves[56] = straight;
	moves[57] = straight;
	moves[58] = right;
	moves[59] = right;
	moves[60] = left;
	moves[61] = left;
	moves[62] = straight;
	moves[63] = left;
	moves[64] = right;
	moves[65] = right;
	moves[66] = uturn;
	moves[67] = left;
	moves[68] = left;
	moves[69] = right;
	moves[70] = straight;
	moves[71] = right;
	moves[72] = right;
	moves[73] = left;
	moves[74] = left;
	moves[75] = straight;
	moves[76] = right;
	moves[77] = straight;
	moves[78] = left;
	moves[79] = right;
	moves[80] = right;
	moves[81] = straight;
	moves[82] = right;
	moves[83] = straight;
	moves[84] = left;
	moves[85] = straight;
	moves[86] = right;
	moves[87] = left;
	moves[88] = left;
	moves[89] = straight;
	moves[90] = straight;
	moves[91] = straight;
	moves[92] = straight;
	moves[93] = left;
	moves[94] = left;
	moves[95] = right;
	moves[96] = left;
	moves[97] = straight;
	moves[98] = left;
	moves[99] = left;
	moves[100] = uturn;
	moves[101] = right;
	moves[102] = right;
	moves[103] = straight;
	moves[104] = right;
	moves[105] = left;
	moves[106] = right;
	moves[107] = left;
	moves[108] = left;
	moves[109] = straight;
	moves[110] = left;
	moves[111] = right;
	moves[112] = left;
	moves[113] = right;
	moves[114] = right;
	moves[115] = straight;
	moves[116] = right;
	moves[117] = uturn;
	moves[118] = right;
	moves[119] = stop;
end

always @(posedge m1_a) begin 
  
  if (reset_value)//(left_turn_start or right_turn_start) //For turning, different rpm count is needed. old value is discarded.
	rpm_right<=0;
  else
   rpm_right<=rpm_right+1;
  
end

always @(posedge m2_a) begin   
  
  if (reset_value)//(left_turn_start or right_turn_start)
	rpm_left<=0;
  else
   rpm_left<=rpm_left+1;
  
end

always @(posedge m1_a) begin

  if(s2) //s2(move_en) is modified (for the condition of checking against 1960) in clk50 block
	rpm_new<=0;
  else
   rpm_new <= rpm_new + 1;
end

always @(posedge clk50) begin

	if (!s2) begin	//executed until 1960 pulses are completed	
		if (pwm_count < duty_right && pwm_count < duty_left ) begin
		  pwm_signal <= 1'b1;
		  ena <= 1'b1;
		  enb <= 1'b1;
		  end
		  
		  else if (pwm_count < duty_right && pwm_count > duty_left ) 
		 begin 
		  pwm_signal <= 1'b0;
		  ena <= 1'b1;
		  enb <= 1'b0;
		  end
		  
		  else if (pwm_count > duty_right && pwm_count < duty_left ) 
		 begin 
		  pwm_signal <= 1'b0;
		  ena <= 1'b0;
		  enb <= 1'b1;
		  end
		  
		  else begin 
		  pwm_signal <= 1'b0;
		  ena <= 1'b0;
		  enb <= 1'b0;
		  end
		  
		  if (pwm_count == 14'd10000) 
		  pwm_count <= 14'd00;
		  else 
		  pwm_count <= pwm_count + 14'd01;
		  
	end
	if (move_state == straight) begin //deviation control with IR, no delay in changing the PWM
		if (!ir_left) begin
			duty_left <= left_normal + 500;
			duty_right <= right_normal - 500;
		end
		else if (!ir_right) begin
			duty_left <= left_normal - 500;
			duty_right <= right_normal + 500;
		end
		else begin
			duty_left <= left_normal;
			duty_right <= right_normal;
		end
	end
	
	case(move_state_fsm)
		straight:begin
		if (op_mid && mid_dist > 0 && mid_dist < 20) begin //if obstacle detected in front,override the straight movement.
				reset_value <= 1;	
				ena <= 0;
				enb <= 0;
				move_state_fsm <= delay;
			end
		else begin
			if (rpm_new >= straight_rpm) begin//1960) begin //*********************should be included in middle block?
				s2<=1;		
			end
			if(rpm_new < straight_rpm )begin
				in1 <= 1'b0;
				in2 <= 1'b1;				
				in3 <= 1'b1;
				in4 <= 1'b0;
				reset_value <= 0;
			end
			
			else begin
			reset_value <= 1;		
			ena <= 0;
			enb <= 0;
			move_state_fsm <= delay;
			//rest_mid =0;
			end
			end
		end
		
		left:begin
			if(rpm_left < 12'd310 )begin	
				in1 <= 1'b0;
				in2 <= 1'b1;					
				in3 <= 1'b0;
				in4 <= 1'b1;					
			end
			else begin			
				reset_value <= 1;	
				ena <= 0;
				enb <= 0;
				move_state_fsm <= delay;		
			end
		end
		
		right : begin
			if(rpm_left < 12'd300 )begin					
				in1 <= 1'b1;
				in2 <= 1'b0;					
				in3 <= 1'b1;
				in4 <= 1'b0;		
			end
			else begin
				ena <= 0;			
				enb <= 0;
				reset_value <= 1;													
				move_state_fsm <= delay;
			end
		end
		
		uturn : begin
				if(rpm_left < 12'd600 )begin	
						in1 <= 1'b1;
						in2 <= 1'b0;					
						in3 <= 1'b1;
						in4 <= 1'b0;
						reset_value <= 0;
				end 
				else begin			
						reset_value <= 1;	
						ena <= 0;
						enb <= 0;
						move_state_fsm <= delay;
				end
		end
		
		
		update: begin 
			ena <= 0;
			enb <= 0;
			rst_left <= 1;
			rst_mid <= 1;
			rst_right <= 1;
			reset_value <= 0;
			move_update <= 0;
			
			
			
			if (pointer < 119) begin
				move_state_fsm <= moves[pointer];
				//output_en <= 1;
				pointer <= pointer + 1;
			end
			else begin
				move_state_fsm <= stop;
			end
//			if (output_en) begin
//				move_state_fsm <= move_state;
//			end
//			else begin
//				//move_update <= 1;
//				move_state_fsm <= update;
//			end
			
			
//			if (wait_cycles != 0) begin
//				move_update <= 1;
//				wait_cycles <= wait_cycles -1;
//				move_state_fsm <= update;
//			end
//			else begin
//				wait_cycles <= 2'b01;
//				move_update <= 0;
//				move_state_fsm <= move_state;
//			end
			
//			move_state <= moves[pointer];
//			if (pointer < 119)
//				pointer <= pointer + 1;
//			else begin
//				ena <= 0;
//				enb <= 0;
//			end
		end
		
		delay : begin
			if (clk_delay > 50000) begin
				move_update <= 1;
				rst_left <= 0;
				rst_mid <= 0;
				rst_right <= 0;
				reset_value <= 0;
				move_state_fsm <= update;
				clk_delay <= 0;
			end
			else begin
				clk_delay <= clk_delay + 1;
				move_state_fsm <= delay;
			end
		end
		
		stop:begin
			ena <= 0;
			enb <= 0;
		end
		
	endcase
	
//	if (rpm_new >= 1960) //*********************should be included in middle block?
//		s2<=1;
		
end
endmodule