module botrun2( //change module name to botrun

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
	 
	 
	
	output reg [2:0] move_state, //next direction using hardcode/mazeSolver
 
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

reg go_to_straight; //to tell the delay operation to proceed to straight (after a turning)


reg rst_left;
reg rst_right;
reg rst_mid;

//wire rst_left;
//wire rst_right;
//wire rst_mid;

wire op_left;
wire op_right;
wire op_mid;

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
reg en_exp;
reg out_updated;

reg adjust; //0 -left detected, 1 - right detected

ultrasonic left_ultra(.clk_50M(clk50), .reset(rst_left), .echo_rx(e_left), .trig(t_left), .op(op_left), .distance_out(left_dist));
ultrasonic right_ultra(.clk_50M(clk50), .reset(rst_right), .echo_rx(e_right), .trig(t_right), .op(op_right), .distance_out(right_dist));
ultrasonic mid_ultrA(.clk_50M(clk50), .reset(rst_mid), .echo_rx(e_mid), .trig(t_mid), .op(op_mid), .distance_out(mid_dist));

mazesolver mazesolve(.move_update(move_update), .clk50(clk50), .output_en(output_en), .move_state(move_state));

maze_explorer explore(
    .clk(clk50),
    .rst_n(1),
    .left(left_sig), 
	 .mid(mid_sig), 
	 .right(right_sig), 
    .move(move_in),
	 .output_updated(out_updated) ,
	 .enable(en_exp)
);

reg left_received;
reg right_received;
reg mid_received;

initial begin
//	left_turn_start = 0;
//	right_turn_start = 0;
	move_state = 0;
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
	
	go_to_straight = 0;
	rpm_right=0;
	rpm_left=0;
	rpm_new = 0;
	wait_cycles = 1;
	move_update = 0;
	move_state_fsm = delay;
	left_sig = 1;
	mid_sig = 0;
	right_sig = 1;
    left_received = 0;
    right_received = 0;
    mid_received = 0;
    en_exp = 0;
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
			// if (rpm_new >= straight_rpm) begin//1960) begin //*********************should be included in middle block?
			// 	s2<=1;		
			// end
			if(rpm_left < straight_rpm )begin
				in1 <= 1'b0;
				in2 <= 1'b1;				
				in3 <= 1'b1;
				in4 <= 1'b0;
				reset_value <= 0;
			end
			
			else begin
            s2<=1;
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
                go_to_straight <= 1;		
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
                go_to_straight <= 1;												
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
			
			//****************non-harcoded********************
			if (op_left) begin
				left_sig <= (left_dist >= 200) ? 0 : 1;
                left_received <= 1;
			end
			if (op_right) begin
				right_sig <= (right_dist >= 200) ? 0 : 1;
                right_received <= 1;
			end
			if (op_mid) begin
				mid_sig <= (mid_dist >= 200) ? 0 : 1;
                mid_received <= 1;
			end

            if (left_received && right_received && mid_received) begin
                en_exp <= 1;
            end
            
            if (out_updated) begin
                move_state_fsm <= move_in;
                left_received <= 0;
                right_received <= 0;
                mid_received <= 0;
            end
			
			//****************end of non-harcoded********************
			
			
			// if (output_en) begin
			// 	move_state_fsm <= move_state;
			// end
			// else begin
			// 	//move_update <= 1;
			// 	move_state_fsm <= update;
			// end


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
                if (go_to_straight) begin
                    move_state_fsm <= straight;
                    go_to_straight <= 0;
                    reset_value <= 0;
                    clk_delay <= 0;
                end
                else begin
				    move_update <= 1;
				    rst_left <= 0;
				    rst_mid <= 0;
				    rst_right <= 0;
				    reset_value <= 0;
				    move_state_fsm <= update;
				    clk_delay <= 0;
                end
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