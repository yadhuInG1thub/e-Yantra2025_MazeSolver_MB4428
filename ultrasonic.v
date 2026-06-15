/*
# Team ID:          eYRC#4428
# Theme:            Mazesolver Bot
# Author List:      Agathiyan , Sanjay ,Israel stephen , Yadhu nandhan
# Filename:         t1b_ultrasonic.v
# File Description: in this file we scaled 3125KHz clock into 195KHz clock and also generated a pwm signal for 195KHz signal.
# Global variables: none
*/
/*
Module HC_SR04 Ultrasonic Sensor

This module will detect objects present in front of the range, and give the distance in mm.

Input:  clk_50M - 50 MHz clock
        reset   - reset input signal (Use negative reset)
        echo_rx - receive echo from the sensor

Output: trig    - trigger sensor for the sensor
        op     -  output signal to indicate object is present.
        distance_out - distance in mm, if object is present.
*/

// module Declaration
module ultrasonic(
    input clk_50M, reset, echo_rx,
    output reg trig,
    output reg op,
    output wire [15:0] distance_out
);
/*defing registers ,
count_us     => this register is used to count delays in micro seconds , like 1us and 10us.
dist         => this register is used to assign distance_out wire the distance value in mm.
opout        => this register is used to asssign op wire the value based on a condition.
clk_count_ms => this register is used to count delay in milli seconds.
trig_enable  => this register is used to enable trigger pulse for 10us.
echo_count   => this regiter counts the number of pulses where echo pin is high.
*/

reg [15:0] distance ;
reg [30:0] echo_count; 
reg [20:0] clk_count ;
reg [8:0]clk_count_us ;
reg [1:0]state;

initial begin
    trig = 0;
	 distance = 0;
	 echo_count = 0;
	 clk_count= 0;
	 clk_count_us =0;
	 state = st_1us;
end

assign distance_out = distance;

parameter st_1us = 0, st_trig = 1, st_ech = 2;
/*
st_1us    => this state gives 1us delay.
st_trig   => this state is place holder.
st_ech    => this state check eho pins value and calculate distance.
*/


always @ (posedge clk_50M)begin
	/*
	this always block works in 50 MHz clock and give input and optput for the ultrasonic sensor along with reset pin.
	*/

	if (reset == 0) begin
	//for reset pin at active low to make all register and working to their default values.
		state <= st_1us;
		echo_count<=00;
		clk_count<=00;
		distance <= 0;
		clk_count_us <=0;
		trig <= 0;
		op <= 0;
	end

	else begin
		if (clk_count > 1000000) begin
			state = st_1us; //has to be blocking statement, so that once 600000 cycles have been counted, no other state transition can be done
			clk_count = 0;
			clk_count_us =0;
			op<=0;
		end
			clk_count <= clk_count+1;
		
		case(state) 

			st_1us : begin
				op <= 0;
				if (clk_count_us >= 50) begin 
					state <= st_trig;
					clk_count_us <= 0;
				end 
				else  begin
					clk_count_us <= clk_count_us + 1;
					state <= st_1us;
				end

			end

			st_trig :begin
				op <= 0;
				if (clk_count_us >= 500)begin
					trig <= 0;
					clk_count_us <= 0;
					echo_count <= 0;
					state <= st_ech;
					clk_count <= 0;
				end
				else begin
					trig <= 1;
					clk_count_us <= clk_count_us+1;
					state <= st_trig;
				end
			end 
			
			st_ech : begin
				if (echo_rx == 1) begin
					 echo_count <= echo_count+1;
					 state <= st_ech;
				 end
				else begin 
					distance <= ((echo_count * 10)/2944); 
					op <= 1;
				end 
			end
		endcase
		end
	end
endmodule

////////////////////////

//
//module ultrasonic(
//    input clk_50M, reset, echo_rx,
//    output reg trig,
//    output reg op,
//    output wire [15:0] distance_out
//);
///*defing registers ,
//count_us     => this register is used to count delays in micro seconds , like 1us and 10us.
//dist         => this register is used to assign distance_out wire the distance value in mm.
//opout        => this register is used to asssign op wire the value based on a condition.
//clk_count_ms => this register is used to count delay in milli seconds.
//trig_enable  => this register is used to enable trigger pulse for 10us.
//echo_count   => this regiter counts the number of pulses where echo pin is high.
//*/
//
//reg [15:0] distance ;
//reg [16:0] echo_count; 
//reg [20:0] clk_count ;
//reg [8:0]clk_count_us ;
//
//initial begin
//    trig = 0;
//	 distance = 0;
//	 echo_count = 0;
//	 clk_count= 0;
//	 clk_count_us =0;
//	 op = 0;
//end
//
//assign distance_out = distance;
//
//parameter st_1us = 0, st_trig = 1, st_ech = 2;
///*
//st_1us    => this state gives 1us delay.
//st_trig   => this state is place holder.
//st_ech    => this state check eho pins value and calculate distance.
//*/
//reg [1:0]state = st_1us;
//
//always @ (posedge clk_50M)begin
///*
//this always block works in 50 MHz clock and give input and optput for the ultrasonic sensor along with reset pin.
//*/
//
//if (reset == 0) begin
////for reset pin at active low to make all register and working to their default values.
//state = st_1us;
//echo_count=00;
//clk_count=00;
//distance = 0;
//clk_count_us =0;
//op = 0;
//end
//
//else begin
//if (clk_count > 600001)begin
//state = st_1us;
//clk_count = 0;
//clk_count_us =0;
//end
//clk_count = clk_count+1;
//case(state) 
//
//st_1us : begin
//op = 0;
//if ( clk_count_us == 50) begin 
//state= st_trig;
//clk_count_us = 0;
//end 
//else  begin
//clk_count_us = clk_count_us + 1;
//end
//
//end
//
//st_trig :begin
//op = 0;
//if (clk_count_us == 500)begin
//trig = 0;
//echo_count = 0;
//state = st_ech;
//clk_count = 0;
//end
//else begin
//trig = 1;
//clk_count_us = clk_count_us+1;
//state = st_trig;
//end
//end 
//st_ech : begin
//op = 0;
//if (echo_rx == 1) begin
// echo_count = echo_count+1;
// end
//else begin 
//op = 1;
//distance = ((echo_count * 10)/2944); 
//end 
//end
//endcase
//end
//end
//endmodule