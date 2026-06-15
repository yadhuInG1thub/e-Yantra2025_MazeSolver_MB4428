module botrun(

	input clk50,
	input m1_a,
	input m2_a,
	input ir_left,
	input ir_right,
	input ir_mid,
	 
   
    output reg ena,
    output reg in1,
    output reg in2,
    output reg enb,
    output reg in3,
    output reg in4,
	 
	 
	
	output reg [2:0] move_state,
 
	input e_left,e_right,e_mid,
	output t_right,t_left,t_mid,
	
//	output reg [15:0] mid_dist_reg,
//	output reg [15:0] left_dist_reg,
//	output reg [15:0]  right_dist_reg,
// 
	 ///////////
	 output reg [1:0] exp_state,
	 output reg mid_signal,
	 output reg right_signal,
	 output reg left_signal,
	 output reg en_explorer,
	 output [2:0]move_in,
	 
	 output [2:0] move_maze,
	 output move_done,
	 
	 output op_mid,
	 output reg [8:0] str,u,lftt,ryt,
	 //output wire [7:0] position,
	 output reg[15:0] dist_chk,
	 output left_maze, right_maze, mid_maze,
	
	input ir_sensor,
	output  uart_txx1,
	output servo_pulse,
	input dout,
	inout sensor,
	output ack,
	output rsts
 	
	 
);
	localparam straight = 3'd1;
	localparam left = 3'd2;
	localparam right = 3'd3;
	localparam uturn = 3'd4;
	localparam stop = 3'd0;
	reg sp_move;
	reg [7:0] pointer;
	reg  ack_reg;
	reg [2:0] moves [0:119];
	wire signed [7:0] position;

//	reg [15:0] mid_dist_reg;
//	reg [15:0] left_dist_reg;
//	reg [15:0]  right_dist_reg;

 	reg pwm_signal;
	reg l1;
	reg l2;
 	reg [15:0] cnt_a;
	reg [15:0] cnt_b;
 	reg [2:0] state;
 	reg [14:0] s1_count;

	reg [18:0] rpm_right;
   reg [18:0] rpm_left;

   wire rst_left;
   wire rst_right;
   wire rst_mid;

  wire [15:0]left_dist;
	wire  [15:0]right_dist;
	wire [15:0]mid_dist;

reg rest_left;
reg rest_right;
reg rest_mid;
reg k1;

assign rst_left = rest_left;
assign rst_right = rest_right;
assign rst_mid = rest_mid;


wire op_left;
wire op_right;






wire mid_sig;
wire left_sig;
wire right_sig;
wire out_updated;
wire en_exp;
reg deadend;
wire dead;
assign dead = deadend;

assign mid_sig = mid_signal;
assign left_sig = left_signal;
assign right_sig = right_signal;
assign en_exp = en_explorer;

//maze_explorer (
//    .clk(clk50),
//    .rst_n(1),
//    .left(left_sig), 
//	 .mid(mid_sig), 
//	 .right(right_sig), 
//    .move(move_in),
//	 .output_updated(out_updated) ,
//	 .enable(en_exp)
//);
//wire ack;

mpimsend work( .clk50(clk50),.ir_sensor(ir_sensor),.uart_txx1(uart_txx1),.dout(dout),.servo_pulse(servo_pulse),.sensor(sensor),.ack(ack),.deadend(dead),.reset_servo(rsts));

ultrasonic let(.clk_50M(clk50), .reset(1), .echo_rx(e_left), .trig(t_left), .op(op_left), .distance_out(left_dist));
ultrasonic rght(.clk_50M(clk50), .reset(1), .echo_rx(e_right), .trig(t_right), .op(op_right), .distance_out(right_dist));
ultrasonic md(.clk_50M(clk50), .reset(1), .echo_rx(e_mid), .trig(t_mid), .op(op_mid), .distance_out(mid_dist));



wire execute;

reg execute_reg, left_maze_reg, right_maze_reg, mid_maze_reg, move_reg, move_done_reg;

assign execute = execute_reg;
assign left_maze = left_maze_reg;
assign right_maze = right_maze_reg;
assign mid_maze = mid_maze_reg;
//assign move_maze = move_reg;
//assign move = move_reg;
//assign move_done = move_done_reg;


reg [13:0] duty_right;
reg  [13:0] duty_left;
reg [13:0] pwm_count;
reg reset_pulse;
reg reset_value;
reg [18:0] rpm_new;

reg [15:0] cnt_1k;
reg clk_1k;
reg [9:0] one_s;
reg s1,s2;


reg t1,z1,z2;

reg [2:1] move;

reg [11:0] left_rpulse;
reg [11:0] right_rpulse;
reg [1:0] def_turn=0;
reg [26:0] clk_delay;
reg [19:0] q1_count;
reg dcont=0;
//reg [8:0] str,u,lftt,ryt;



localparam q1=0;
localparam q2=1;
localparam q3=2;
localparam q4=3;
localparam q5=4;
localparam q6=5;

reg [15:0] mid_dist_reg;
 reg [15:0] left_dist_reg;
 reg [15:0]  right_dist_reg;
 

initial begin
///start 
	rest_left = 1;
	rest_right = 1;
	rest_mid =1;
////start
ack_reg = 0;
	pwm_count=0;
    ena = 1'b0;
    in1 = 1'b0;
    in2 = 1'b0;
    enb = 1'b0;
    in3 = 1'b0;
    in4 = 1'b0;
	 duty_left=0;
	 duty_right=0;
	 sp_move =0;
	
	rpm_right=0;
	rpm_left=0;
	
	cnt_1k=0;
	one_s=0;
	
	cnt_a=0;
	cnt_b=0;
	t1=0;
	s1=0;
	s2=0;
	z1=0;
	z2=0;
	state=q1;
	
	rpm_new = 0;
	s1_count = 0;
	 q1_count=0;
	 mid_dist_reg=0;
	left_dist_reg=0;
	right_dist_reg = 0;
	 
	 move=3'b001;
	 move_state= straight;
	 left_rpulse=0;
	 right_rpulse=0;
	 reset_value = 0;
	 en_explorer = 0;
	 exp_state = 0;
	 
	 pointer = 0;
	 
	 str=0;
	 u=0;
	 lftt=0;
	 ryt=0;
	 k1=0;
	 execute_reg = 0;
	 left_maze_reg=0;
	  right_maze_reg=0;
	   mid_maze_reg=0;
		clk_delay=0;
		deadend =0;

//	 moves[0] = 3'b101;
//moves[1] = left;
//moves[2] = left;
//moves[3] = right;
//moves[4] = straight;
//moves[5] = straight;
//moves[6] = right;
//moves[7] = straight;
//moves[8] = right;
//moves[9] = left;
//moves[10] = left;
//moves[11] = uturn; ////////////////////////////
//moves[12] = left;
//moves[13] = left;
//moves[14] = right;
//moves[15] = right;
//moves[16] = straight;
//moves[17] = left;
//moves[18] = right;
//moves[19] = left;
//moves[20] = straight;
//moves[21] = left;
//moves[22] = straight;//
//moves[23] = left;
//moves[24] = right;
//moves[25] = left;
//moves[26] = left;
//moves[27] = uturn;
//moves[28] = right;
//moves[29] = right;
//moves[30] = left;
//moves[31] = left;
//moves[32] = uturn;
//moves[33] = straight;
//moves[34] = right;
//moves[35] = uturn;
//moves[36] = right;
//moves[37] = right;
//moves[38] = straight;
//moves[39] = right;
//moves[40] = left;
//moves[41] = right;
//moves[42] = straight;
//moves[43] = left;
//moves[44] = left;
//moves[45] = straight;
//moves[46] = right;
//moves[47] = straight;
//moves[48] = straight;
//moves[49] = right;
//moves[50] = uturn;
//moves[51] = left;
//moves[52] = right;
//moves[53] = left;
//moves[54] = uturn;
//moves[55] = left;
//moves[56] = straight;
//moves[57] = straight;
//moves[58] = right;
//moves[59] = right;
//moves[60] = left;
//moves[61] = left;
//moves[62] = straight;
//moves[63] = left;
//moves[64] = right;
//moves[65] = right;
//moves[66] = uturn;
//moves[67] = left;
//moves[68] = left;
//moves[69] = right;
//moves[70] = straight;
//moves[71] = right;
//moves[72] = right;
//moves[73] = left;
//moves[74] = left;
//moves[75] = straight;
//moves[76] = right;
//moves[77] = straight;
//moves[78] = left;
//moves[79] = right;
//moves[80] = right;
//moves[81] = straight;
//moves[82] = right;
//moves[83] = straight;
//moves[84] = left;
//moves[85] = straight;
//moves[86] = right;
//moves[87] = left;
//moves[88] = left;
//moves[89] = straight;
//moves[90] = straight;
//moves[91] = straight;
//moves[92] = straight;
//moves[93] = left;
//moves[94] = left;
//moves[95] = right;
//moves[96] = left;
//moves[97] = straight;
//moves[98] = left;
//moves[99] = left;
//moves[100] = uturn;
//moves[101] = right;
//moves[102] = right;
//moves[103] = straight;
//moves[104] = right;
//moves[105] = left;
//moves[106] = right;
//moves[107] = left;
//moves[108] = left;
//moves[109] = straight;
//moves[110] = left;
//moves[111] = right;
//moves[112] = left;
//moves[113] = right;
//moves[114] = right;
//moves[115] = straight;
//moves[116] = right;
//moves[117] = uturn;
//moves[118] = right;
//moves[119] = stop;
end


////////////////////////////nov
always @(posedge m1_a) begin

rpm_new <= rpm_new + 1;
// if(move_en==1)
// rpm_new<=0;
end

always @(posedge clk50) begin
	ack_reg <= ack;
	if (mid_dist != 0) begin
	 mid_dist_reg <= mid_dist;
	 end
	 
	 if (left_dist != 0) begin
	 left_dist_reg <= left_dist;
	 end
	 
	 if (right_dist != 0) begin
	 right_dist_reg <= right_dist;
	 end

	  if(s2==0)
	 begin
  
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
		
	   else begin
	  ena <= 1'b0;
	  enb <= 1'b0;
		end

		if (dcont==0) begin

	case (state)
	
	q1:begin//normal
	
	duty_left<=8500;
	 duty_right<=8500;
	
	if (ir_right==0 ) begin
	state<=q2;
	
	q1_count<=s1_count+300;
	end
	else
	state<=q4;
	 
	end
	
	q2:begin//stop
	duty_left<=0;
	 duty_right<=0;
	 z1<=1;
	 state<=q3;	 
	 end
	
	  q3: begin//correction - right
	   z1<=0;
	 duty_right<=8500; //##############################
	 duty_left<=9800; //##############################
	 if(rpm_right>=150)begin //##############################
	 state<=q1;
	
	 end
	 end
	 
	 q4:begin//left check
	if (ir_left==0 )begin
	
	state<=q5;
	q1_count<=s1_count+300;
	end
	else
	state<=q1;
	end
	
	q5:begin//stop
	duty_left<=0;
	 duty_right<=0;
	z2<=1; 
	 state<=q6;
	 end
	
	  q6: begin//left correction
	   z2<=0;
	 duty_right<=9500;
	 duty_left<=8500; //##############################
	 if(rpm_left>=300) begin //##############################
	 state<=q1;
	
	 end
	 end
	 
	 endcase
	 
	 end
//	 
//	 else
//	 dcont<=1;
//	 //move
//	 rest_mid = 1;
//	 rest_right = 0;
//	 rest_left = 0;
//	 
//	 if(dcont == 0) begin
//		if(ir_left == 0 )begin
//			duty_left<=10000;
//			duty_right<=9000;
//		end
//		else if(ir_right == 0)begin
//			duty_left<=9000; //##############################
//			duty_right<=10000; //##############################
//		end
//		else begin
//			duty_left<=9000;
//			duty_right<=9000;
//		end
//	 end
//	 
//	 else begin
//		dcont <= 1;		
//	 end
	 

 case(move_state)
//
		left : begin 
			dcont<=1;
			clk_delay <= 0;
			duty_left<=8500; //##############################
	 duty_right<=8500; //##############################
			//nxt_command = 0;
			reset_value <= 0;
			if(right_rpulse < 12'd150 )begin					
				in1 <= 1'b1;
				in2 <= 1'b0;					
				in3 <= 1'b1;
				in4 <= 1'b0;		
			end
			else begin
			ena <= 0;			
			enb <= 0;
			reset_value <= 1;													
			move_state <=  straight;

			end
		end
//
		right : begin
		dcont<=1;
		clk_delay <= 0;
			duty_left<=9500; //##############################
	     duty_right<=9500; //##############################
			//nxt_command = 0;
			reset_value <= 0;
			if(right_rpulse < 12'd150 )begin					
				in1 <= 1'b0;
				in2 <= 1'b1;					
				in3 <= 1'b0;
				in4 <= 1'b1;		
			end
			else begin
			ena <= 0;			
			enb <= 0;
			reset_value <= 1;													
			move_state <= straight;

			end
		end
//
		uturn : begin //u turn
		dcont<=1;
			clk_delay <= 0;	
			duty_left<=8950; //##########################
	 duty_right<=8950; //##############################
			//nxt_command = 0;			
			reset_value <= 0;
			if(right_rpulse < 12'd380 )begin	
				in1 <= 1'b1;
				in2 <= 1'b0;					
				in3 <= 1'b1;
				in4 <= 1'b0;				
			end 
			else begin			
			reset_value <= 1;	
				ena <= 0;
				enb <= 0;
				move_state <=straight;
	end
		end
//
 3'b110 : begin //straight
		dcont <= 0;
		  	clk_delay <= 0;			
			
		  reset_value <= 0;
//			
//		   if(sp_move == 1)begin
//				if((left_rpulse <= 12'd600 || right_rpulse <= 12'd600) && mid_dist_reg > 300 )begin
//				in1 <= 1'b0;
//				in2 <= 1'b1;				
//				in3 <= 1'b1;
//				in4 <= 1'b0;
//			end
//			
//			else begin
//				reset_value <= 1;		
//				ena <= 0;
//				enb <= 0;
//				move_state <= 3'b111;
//				 rest_left<=1;
//	          rest_mid<=1;
//	         rest_right<=1;
//				
//			end
//			end
//			else begin
				if((((left_rpulse <= 12'd600) || (right_rpulse <= 12'd600)) && (mid_dist_reg > 70)) || ((ir_sensor == 0) && (left_dist_reg<=170) && ((left_rpulse <= 12'd650) || (right_rpulse <= 12'd650))&& (right_dist_reg<=170)))begin
				in1 <= 1'b0;
				in2 <= 1'b1;				
				in3 <= 1'b1;
				in4 <= 1'b0;
			end/////////////////////////////////////////////////////////changed
			
			else begin
			reset_value <= 1;		
			ena <= 0;
			enb <= 0;
			move_state <= 3'b111;
			  rest_left<=1;
	          rest_mid<=1;
	         rest_right<=1;
			
			end
			//end
		end
//
		straight : begin // 1 sec delay
		dcont<=1;
				ena <= 0;
				enb <= 0;
				 rest_left<=1;
	          rest_mid<=1;
	         rest_right<=1;
				
				
				reset_value <= 1;
			if(clk_delay > 15000000 ) begin
				move_state <= 3'b110; // to straight
			end
			else begin
				clk_delay <= clk_delay+1;
			end
			//////////////////////////////////////////////////////////////
			
			
		end
		
		
//		
		3'b101 : begin	// update state
		ena <= 0;
		enb <= 0;
		 z1<=0;
		z2<=0;
		
		 left_maze_reg <= ((left_dist_reg >= 150 ) )?0:1;
		right_maze_reg <= ((right_dist_reg >= 150 ) )?0:1;
		mid_maze_reg <= ((mid_dist_reg >= 150) )?0:1;
		
		
		rest_mid <=0;
		rest_right <= 0;
		rest_left <= 0;
		execute_reg <= 0;
		
		if (left_dist_reg>=170)
		move_state<=left;
		else if (mid_dist_reg>=170)
		move_state<=straight;
		else if (right_dist_reg>=170)
		move_state<=right;
		else if(left_dist_reg<=170 && mid_dist_reg<=170 && right_dist_reg<=170) begin
		if (ir_sensor==0)begin
		move_state<=3'b000;
		deadend <= 1;
		end
		else
		move_state<=uturn;
		
		end
		else if (left_dist_reg>=400 && mid_dist_reg>=400 && right_dist_reg>=400)begin
		move_state<=3'b000;
		
		end
		else
		move_state<=uturn;
		
//		if(move_maze!=0)
//		move_state <= move_maze;
//		if (k1== 0) begin
//		dist_chk<=left_dist_reg;
//		k1=1;
//		end
		
		if(move_maze == uturn)
		u<=u+1;
		if(move_maze == straight)
		str<=str+1;
		if(move_maze == left)
		lftt<=lftt+1;
		if(move_maze == right)
		ryt<=ryt+1;
		
		
		 
		
		
//		if(left_dist_reg >= 200)begin
//			move_state <= left;
//		end
//		else if (mid_dist_reg >= 200)begin
//			move_state <= straight;
//		end
//		else if (right_dist_reg >= 200)begin
//			move_state <= right;
//		end
//		else begin
//			move_state <= uturn;
//		end
		
//		if (pointer < 120) begin
//		move_state <= moves[pointer];
//			pointer = pointer + 1;
//			end
//		else begin
//			ena <= 0;
//			enb <= 0;
//		end	
//		
//		if (pointer == 93)begin
//			sp_move <= 1;
//		end	
//		else begin
//			sp_move <= 0;
//		end 
	end
//


  3'b111:begin
 

 
	dcont <= 1;
	   z1<=1;
		z2<=1;
				rest_mid =0;
				rest_right = 0;
				rest_left = 0;
				
				
				
				ena <= 0;
				enb <= 0;
				en_explorer <=0;
//				

				
				reset_value <= 1;
			if(clk_delay > 10000000 ) begin
				move_state <= 3'b101;// to update

			end
			else begin				
				clk_delay <= clk_delay+1;
			end
			
			if(clk_delay == 9999998) begin
			
			
				execute_reg <= 1;
		
			end
//			if (clk_delay == 10000000) begin
//				execute_reg <= 0;
//			end
end

	3'b000: begin 
//	ena<=0;
//	enb<=0;
	dcont <= 1; 
	duty_left <=0;
	duty_right <=0;
	deadend <=0;
	in1<=0;
	in2<=0;
	in3<=0;
	in4<=0;
	if(ack_reg)
		move_state<=uturn;

	end
	

endcase



	
	
	  
	 
end

always @(posedge m2_a) begin



 rpm_right<=rpm_right+1;
 
 if (z1==1)
 rpm_right<=0;
 
end


always @(posedge m1_a) begin

 rpm_left<=rpm_left+1;
 

 if (z2==1)
 rpm_left<=0;
 end
 
 
always @(posedge m1_a) begin

	if (reset_value == 1)
right_rpulse <=0 ;
else 
right_rpulse <= right_rpulse + 12'd1;


end

always @(posedge m2_a) begin
if (reset_value == 1)
left_rpulse <=0 ;
else 
left_rpulse <= left_rpulse + 12'd1;


end




always @(posedge clk_1k) begin

//if (rpm_new >= 1960)
//s2<=1;

s1_count<=s1_count+1;



end

always @(posedge clk50) begin

if(cnt_1k>=25000) begin
clk_1k<=~clk_1k;
cnt_1k<=0;

end
else
cnt_1k<=cnt_1k+1;

end
endmodule
///////////nov

//module botrun(
//
//	input clk50,
//	input m1_a,
//	input m2_a,
//	input ir_left,
//	input ir_right,
//	input ir_mid,
//	 
//   
//    output reg ena,
//    output reg in1,
//    output reg in2,
//    output reg enb,
//    output reg in3,
//    output reg in4,
//	 
//	 
//	
//	output reg [2:0] move_state,
// 
//	input e_left,e_right,e_mid,
//	output t_right,t_left,t_mid,
//	output  [15:0]left_dist,
//	output  [15:0]right_dist,
//	output  [15:0]mid_dist,
// 
//	 ///////////
//	 output reg [1:0] exp_state,
//	 output reg mid_signal,
//	 output reg right_signal,
//	 output reg left_signal,
//	 output reg en_explorer,
//	 output [2:0]move_in,
//	 
//	 output [2:0] move_maze,
//	 output move_done,
//	 
//	 output op_mid
//	
//	 
//);
//	localparam straight = 3'd1;
//	localparam left = 3'd2;
//	localparam right = 3'd3;
//	localparam uturn = 3'd4;
//	localparam stop = 3'd0;
//	localparam update = 3'd5;
//	reg sp_move;
//	reg [7:0] pointer;
//
//	reg [2:0] moves [0:119];
//
//	reg [15:0] mid_dist_reg;
//	reg [15:0] left_dist_reg;
//	reg [15:0]  right_dist_reg;
//
// 	reg pwm_signal;
//	reg l1;
//	reg l2;
// 	reg [15:0] cnt_a;
//	reg [15:0] cnt_b;
// 	reg [2:0] state;
// 	reg [14:0] s1_count;
//
//	reg [18:0] rpm_right;
//reg [18:0] rpm_left;
//
//wire rst_left;
//wire rst_right;
//wire rst_mid;
//
//reg rest_left;
//reg rest_right;
//reg rest_mid;
//
//assign rst_left = rest_left;
//assign rst_right = rest_right;
//assign rst_mid = rest_mid;
//
//
//wire op_left;
//wire op_right;
//
//
//
//
//
//
//wire mid_sig;
//wire left_sig;
//wire right_sig;
//wire out_updated;
//wire en_exp;
//
//assign mid_sig = mid_signal;
//assign left_sig = left_signal;
//assign right_sig = right_signal;
//assign en_exp = en_explorer;
//
////maze_explorer (
////    .clk(clk50),
////    .rst_n(1),
////    .left(left_sig), 
////	 .mid(mid_sig), 
////	 .right(right_sig), 
////    .move(move_in),
////	 .output_updated(out_updated) ,
////	 .enable(en_exp)
////);
//
//
//ultrasonic let(.clk_50M(clk50), .reset(rst_left), .echo_rx(e_left), .trig(t_left), .op(op_left), .distance_out(left_dist));
//ultrasonic rght(.clk_50M(clk50), .reset(rst_right), .echo_rx(e_right), .trig(t_right), .op(op_right), .distance_out(right_dist));
//ultrasonic md(.clk_50M(clk50), .reset(rst_mid), .echo_rx(e_mid), .trig(t_mid), .op(op_mid), .distance_out(mid_dist));
//
//wire execute, left_maze, right_maze, mid_maze;
//reg execute_reg, left_maze_reg, right_maze_reg, mid_maze_reg, move_reg, move_done_reg;
//
//assign execute = execute_reg;
//assign left_maze = left_maze_reg;
//assign right_maze = right_maze_reg;
//assign mid_maze = mid_maze_reg;
////assign move = move_reg;
////assign move_done = move_done_reg;
//
//maze_explore_solve explore(
//	.clk(clk50),
//	.execute(execute),
//	.rst_n(1),
//	.left(left_maze),
//	.right(right_maze),
//	.mid(mid_maze),
//	.move(move_maze),
//	.move_done(move_done)
//
//);
//reg [13:0] duty_right;
//reg  [13:0] duty_left;
//reg [13:0] pwm_count;
//reg reset_pulse;
//reg reset_value;
//reg [18:0] rpm_new;
//
//reg [15:0] cnt_1k;
//reg clk_1k;
//reg [9:0] one_s;
//reg s1,s2;
//
//
//reg t1,z1,z2;
//
//reg [2:1] move;
//
//reg [11:0] left_rpulse;
//reg [11:0] right_rpulse;
//reg [1:0] def_turn = 0;
//reg [26:0] clk_delay = 0;
//reg [19:0] q1_count;
//reg dcont=0;
//
//
//
//localparam q1=0;
//localparam q2=1;
//localparam q3=2;
//localparam q4=3;
//localparam q5=4;
//localparam q6=5;
//
//
//initial begin
/////start 
//	execute_reg=0;
//	left_maze_reg=0;
//	right_maze_reg=0;
//	mid_maze_reg=0;
//	move_reg=0;
//	move_done_reg=0;
//	
//	rest_left = 0;
//	rest_right = 0;
//	rest_mid =0;
//////start
//	pwm_count=0;
//    ena = 1'b0;
//    in1 = 1'b0;
//    in2 = 1'b0;
//    enb = 1'b0;
//    in3 = 1'b0;
//    in4 = 1'b0;
//	 duty_left=0;
//	 duty_right=0;
//	 sp_move =0;
//	
//	rpm_right=0;
//	rpm_left=0;
//	
//	cnt_1k=0;
//	one_s=0;
//	
//	cnt_a=0;
//	cnt_b=0;
//	t1=0;
//	s1=0;
//	s2=0;
//	z1=0;
//	z2=0;
//	state=q1;
//	
//	rpm_new = 0;
//	s1_count = 0;
//	 q1_count=0;
//	 
//	 move=3'b001;
//	 move_state= update;
//	 left_rpulse=0;
//	 right_rpulse=0;
//	 reset_value = 0;
//	 en_explorer = 0;
//	 exp_state = 0;
//	 
//	 pointer = 0;
//
////	 moves[0] = 3'b101;
////moves[1] = left;
////moves[2] = left;
////moves[3] = right;
////moves[4] = straight;
////moves[5] = straight;
////moves[6] = right;
////moves[7] = straight;
////moves[8] = right;
////moves[9] = left;
////moves[10] = left;
////moves[11] = uturn; ////////////////////////////
////moves[12] = left;
////moves[13] = left;
////moves[14] = right;
////moves[15] = right;
////moves[16] = straight;
////moves[17] = left;
////moves[18] = right;
////moves[19] = left;
////moves[20] = straight;
////moves[21] = left;
////moves[22] = straight;//
////moves[23] = left;
////moves[24] = right;
////moves[25] = left;
////moves[26] = left;
////moves[27] = uturn;
////moves[28] = right;
////moves[29] = right;
////moves[30] = left;
////moves[31] = left;
////moves[32] = uturn;
////moves[33] = straight;
////moves[34] = right;
////moves[35] = uturn;
////moves[36] = right;
////moves[37] = right;
////moves[38] = straight;
////moves[39] = right;
////moves[40] = left;
////moves[41] = right;
////moves[42] = straight;
////moves[43] = left;
////moves[44] = left;
////moves[45] = straight;
////moves[46] = right;
////moves[47] = straight;
////moves[48] = straight;
////moves[49] = right;
////moves[50] = uturn;
////moves[51] = left;
////moves[52] = right;
////moves[53] = left;
////moves[54] = uturn;
////moves[55] = left;
////moves[56] = straight;
////moves[57] = straight;
////moves[58] = right;
////moves[59] = right;
////moves[60] = left;
////moves[61] = left;
////moves[62] = straight;
////moves[63] = left;
////moves[64] = right;
////moves[65] = right;
////moves[66] = uturn;
////moves[67] = left;
////moves[68] = left;
////moves[69] = right;
////moves[70] = straight;
////moves[71] = right;
////moves[72] = right;
////moves[73] = left;
////moves[74] = left;
////moves[75] = straight;
////moves[76] = right;
////moves[77] = straight;
////moves[78] = left;
////moves[79] = right;
////moves[80] = right;
////moves[81] = straight;
////moves[82] = right;
////moves[83] = straight;
////moves[84] = left;
////moves[85] = straight;
////moves[86] = right;
////moves[87] = left;
////moves[88] = left;
////moves[89] = straight;
////moves[90] = straight;
////moves[91] = straight;
////moves[92] = straight;
////moves[93] = left;
////moves[94] = left;
////moves[95] = right;
////moves[96] = left;
////moves[97] = straight;
////moves[98] = left;
////moves[99] = left;
////moves[100] = uturn;
////moves[101] = right;
////moves[102] = right;
////moves[103] = straight;
////moves[104] = right;
////moves[105] = left;
////moves[106] = right;
////moves[107] = left;
////moves[108] = left;
////moves[109] = straight;
////moves[110] = left;
////moves[111] = right;
////moves[112] = left;
////moves[113] = right;
////moves[114] = right;
////moves[115] = straight;
////moves[116] = right;
////moves[117] = uturn;
////moves[118] = right;
////moves[119] = stop;
//end
//
//
//////////////////////////////nov
//always @(posedge m1_a) begin
//
//rpm_new <= rpm_new + 1;
//// if(move_en==1)
//// rpm_new<=0;
//end
//
//always @(posedge clk50) begin
//	
//	if (mid_dist != 0) begin
//	 mid_dist_reg = mid_dist;
//	 end
//	 
//	 if (left_dist != 0) begin
//	 left_dist_reg = left_dist;
//	 end
//	 
//	 if (right_dist != 0) begin
//	 right_dist_reg = right_dist;
//	 end
//
//	  if(s2==0)
//	 begin
//  
//   if (pwm_count < duty_right && pwm_count < duty_left ) begin
//	  pwm_signal <= 1'b1;
//	  ena <= 1'b1;
//	  enb <= 1'b1;
//	  end
//	  
//	  else if (pwm_count < duty_right && pwm_count > duty_left ) 
//	 begin 
//	  pwm_signal <= 1'b0;
//	  ena <= 1'b1;
//	  enb <= 1'b0;
//	  end
//	  
//	  else if (pwm_count > duty_right && pwm_count < duty_left ) 
//	 begin 
//	  pwm_signal <= 1'b0;
//	  ena <= 1'b0;
//	  enb <= 1'b1;
//	  end
//	  
//	  else begin 
//	  pwm_signal <= 1'b0;
//	  ena <= 1'b0;
//	  enb <= 1'b0;
//	  end
//	  
//    if (pwm_count == 14'd10000) 
//	  pwm_count <= 14'd00;
//	  else 
//	  pwm_count <= pwm_count + 14'd01;
//	  
//	end
//		
//	   else begin
//	  ena <= 1'b0;
//	  enb <= 1'b0;
//		end
//
//		if (dcont==0) begin
//
//	case (state)
//	
//	q1:begin//normal
//	
//	duty_left<=9000;
//	 duty_right<=9000;
//	
//	if (ir_right==0 ) begin
//	state<=q2;
//	
//	q1_count<=s1_count+300;
//	end
//	else
//	state<=q4;
//	 
//	end
//	
//	q2:begin//stop
//	duty_left<=0;
//	 duty_right<=0;
//	 z1<=1;
//	 state<=q3;	 
//	 end
//	
//	  q3: begin//correction - right
//	   z1<=0;
//	 duty_left<=9000; //##############################
//	 duty_right<=10000; //##############################
//	 if(rpm_right>=150)begin //##############################
//	 state<=q1;
//	
//	 end
//	 end
//	 
//	 q4:begin//left check
//	if (ir_left==0 )begin
//	
//	state<=q5;
//	q1_count<=s1_count+300;
//	end
//	else
//	state<=q1;
//	end
//	
//	q5:begin//stop
//	duty_left<=0;
//	 duty_right<=0;
//	z2<=1; 
//	 state<=q6;
//	 end
//	
//	  q6: begin//left correction
//	   z2<=0;
//	 duty_left<=10000;
//	 duty_right<=9000; //##############################
//	 if(rpm_left>=300) begin //##############################
//	 state<=q1;
//	
//	 end
//	 end
//	 
//	 endcase
//	 
//	 end
//	 
//	 else
//	 dcont<=1;
//	 //move
////	 rest_mid = 1;
////	 rest_right = 0;
////	 rest_left = 0;
////	 
////	 if(dcont == 0) begin
////		if(ir_left == 0 )begin
////			duty_left<=10000;
////			duty_right<=9000;
////		end
////		else if(ir_right == 0)begin
////			duty_left<=9000; //##############################
////			duty_right<=10000; //##############################
////		end
////		else begin
////			duty_left<=9000;
////			duty_right<=9000;
////		end
////	 end
////	 
////	 else begin
////		dcont <= 1;		
////	 end
//	 
//
// case(move_state)
////
//		left : begin //right turn
//			dcont<=1;
//			clk_delay <= 0;
//			duty_left<=8500; //##############################
//			duty_right<=8500; //##############################
//			//nxt_command = 0;
//			reset_value <= 0;
//			if(right_rpulse < 12'd150 )begin					
//				in1 <= 1'b0;
//				in2 <= 1'b1;					
//				in3 <= 1'b0;
//				in4 <= 1'b1;		
//			end
//			else begin
//			ena <= 0;			
//			enb <= 0;
//			reset_value <= 1;													
//			move_state <=  3'b110;
//
//			end
//		end
////
//		right : begin //left turn
//		dcont<=1;
//		clk_delay <= 0;
//			duty_left<=8500; //##############################
//	 duty_right<=8500; //##############################
//			//nxt_command = 0;
//			reset_value <= 0;
//			if(right_rpulse < 12'd150 )begin					
//				in1 <= 1'b1;
//				in2 <= 1'b0;					
//				in3 <= 1'b1;
//				in4 <= 1'b0;		
//			end
//			else begin
//			ena <= 0;			
//			enb <= 0;
//			reset_value <= 1;													
//			move_state <= 3'b110;
//
//			end
//		end
////
//		uturn : begin //u turn
//		dcont<=1;
//			clk_delay <= 0;	
//			duty_left<=8500; //##############################
//			duty_right<=8500; //##############################
//			//nxt_command = 0;			
//			reset_value <= 0;
//			if(right_rpulse < 12'd380 )begin	
//				in1 <= 1'b1;
//				in2 <= 1'b0;					
//				in3 <= 1'b1;
//				in4 <= 1'b0;				
//			end 
//			else begin			
//			reset_value <= 1;	
//				ena <= 0;
//				enb <= 0;
//				move_state <=3'b110;
//	end
//		end
////
//		straight: begin //straight
//		dcont <= 0;
//		  	clk_delay <= 0;			
//			
//		  reset_value <= 0;
//			
//		   if(sp_move == 1)begin
//				if((left_rpulse <= 12'd600 || right_rpulse <= 12'd600) && mid_dist_reg > 300 )begin
//				in1 <= 1'b0;
//				in2 <= 1'b1;				
//				in3 <= 1'b1;
//				in4 <= 1'b0;
//			end
//			
//			else begin
//				reset_value <= 1;		
//				ena <= 0;
//				enb <= 0;
//				move_state <= 3'b111;
//				rest_mid <= 0;
//			end
//			end
//			else begin
//				if((left_rpulse <= 12'd600 || right_rpulse <= 12'd600) && mid_dist_reg > 70 )begin
//				in1 <= 1'b0;
//				in2 <= 1'b1;				
//				in3 <= 1'b1;
//				in4 <= 1'b0;
//			end
//			
//			else begin
//			reset_value <= 1;		
//			ena <= 0;
//			enb <= 0;
//			move_state <= 3'b111;
//			rest_mid <= 0;
//			end
//			end
//		end
////
//		3'b110 : begin // 1 sec delay
//				dcont<=1;
//				ena <= 0;
//				enb <= 0;
//				
//				rest_mid = 1;
//				rest_right = 0;
//				rest_left = 0;
//				
//				reset_value <= 1;
//			if(clk_delay > 15000000 ) begin
//				move_state <= straight; // to straight
//			end
//			else begin
//				clk_delay <= clk_delay+1;
//			end
//		end
//		
//		stop : begin // 1 sec delay
//		ena <= 0;
//        enb <= 0;
//
//		end
////		
//		3'b101 : begin	// update state
//		ena <= 0;
//		enb <= 0;
//		execute_reg <= 1;
//		left_maze_reg = (left_dist_reg <= 200)?1:0;
//		right_maze_reg = (right_dist_reg <= 200)?1:0;
//		mid_maze_reg = (mid_dist_reg <= 200)?1:0;
//		if (move_done) begin
//			move_state <= move_maze;
//		end
//		else begin
//			move_state <= 3'b101;
//			
//		end
////		if (pointer < 120) begin
////		move_state <= moves[pointer];
////			pointer = pointer + 1;
////			end
////		else begin
////			ena <= 0;
////			enb <= 0;
////		end	
////		
////		if (pointer == 93)begin
////			sp_move <= 1;
////		end	
////		else begin
////			sp_move <= 0;
////		end 
//	end
////
//  3'b111:begin
// 
//	dcont <= 1;
//				rest_mid = 0;
//				rest_right = 0;
//				rest_left = 0;
//				
//				ena <= 0;
//				enb <= 0;
//				en_explorer <=0;
////				
//
//				
//				reset_value <= 1;
//			if(clk_delay > 10000000 ) begin
//				move_state <= 3'b101;// to update
//
//			end
//			else begin
//				clk_delay <= clk_delay+1;
//			end
//end
////
////	default: begin
////			ena<=0;
////			enb<=0;
////			end
//endcase
//
//
//
//	
//	
//	  
//	 
//end
//
//always @(posedge m2_a) begin
//
//
//
// rpm_right<=rpm_right+1;
// 
// if (z1==1)
// rpm_right<=0;
// 
//end
//
//
//always @(posedge m1_a) begin
//
// rpm_left<=rpm_left+1;
// 
//
// if (z2==1)
// rpm_left<=0;
// end
//always @(posedge m1_a) begin
//
//	if (reset_value == 1)begin
//right_rpulse <=0 ;
//end
//else begin
//right_rpulse <= right_rpulse + 12'd1;
//end
//
//
//end
//
//always @(posedge m2_a) begin
//if (reset_value == 1)begin
//left_rpulse <=0 ;
//end
//else begin
//left_rpulse <= left_rpulse + 12'd1;
//end
//
//
//end
//
//
//
//
//always @(posedge clk_1k) begin
//
////if (rpm_new >= 1960)
////s2<=1;
//
//s1_count<=s1_count+1;
//
//
//
//end
//
//always @(posedge clk50) begin
//
//if(cnt_1k>=25000) begin
//clk_1k<=~clk_1k;
//cnt_1k<=0;
//
//end
//else
//cnt_1k<=cnt_1k+1;
//
//end
//endmodule
/////////////nov