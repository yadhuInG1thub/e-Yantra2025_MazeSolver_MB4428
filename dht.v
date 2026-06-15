module dht ( input clk_50M,
    input reset,
    inout sensor,
    output reg [7:0] T_integral,
    output reg [7:0] RH_integral,
    output reg [7:0] T_decimal,
    output reg [7:0] RH_decimal,
    output reg [7:0] Checksum,
    output reg data_valid,
	 output reg [2:0] state,
	 output reg [5:0] bit_count,
	 output reg [15:0] high_count,
	 output reg m1,
	 output reg m2
	 
);

//    initial begin
//        T_integral = 0;
//        RH_integral = 0;
//        T_decimal = 0;
//        RH_decimal = 0;
//        Checksum = 0;
//        data_valid = 0;
//		  
//    end
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

//FSM STATE DEFINITIONS
localparam s_START_18ms = 3'b000;
localparam s_START_40us = 3'b001;
localparam s_START_COMM_1 = 3'b010;
localparam s_START_COMM_2 = 3'b011; 
localparam s_WAIT_HALF = 3'b100;
localparam s_SECOND_HALF = 3'b101;
localparam s_LOAD = 3'b110;

//Initial count value of 1'b1 since fsm takes one extra cycle
localparam clock_init = 1'b1;

//REGISTER DEFINITIONS :
//clk_count : To count number of cycles to find elapsed duration (for 18ms, 40us, 50us and 80us)
//high_count : To count number of cycles of HIGH after the 50us
//state : Register to store the current state
//bit_count : for keeping count of bits received thus far
// total_word: 40 bit register to store all 40 bit data received for one frame
// sensor_out = to drive the sensor when it is output
// drive_enable : if this bit is 1, then sensor line is output, if 0, it is input
 
 
reg [20:0] clk_count; 
//= clock_init; 

 
reg [39:0] total_word;
reg sensor_out = 0; 
reg drive_enable = 1; 
initial begin
state = 0;
bit_count = 0;
clk_count = clock_init;
		//state <= s_START_18ms;
		T_integral = 0;
      RH_integral = 0;
      T_decimal = 0;
      RH_decimal = 0;
      Checksum = 0;
      data_valid = 0;
		 high_count = clock_init; 
		 total_word = 0;
		 sensor_out = 0;
		 drive_enable = 1;
		 total_word = 0; 
		 m1=0;
		  m2=0;
end

//sensor : Logic to decide if sensor will be input or output based on drive_enable
assign sensor = (drive_enable) ? sensor_out: 1'bz ; 

always @(posedge clk_50M) begin 

/*
Purpose: 
---------

Work on 50Mhz clk to implement the FSM to interface with dht11 sensor by following the below given steps :

1) Pull sensor line low 18ms and then high for 40us to initiate communication
2) Read the sensor response of 80us LOW and 80us HIGH. This indicates the sensor is ready to transmit data.
3) Read each bit
	- 0: 50us low and 28us HIGH
	- 1: 50us low and 70us HIGH
4) Store the total frame
	8 bits ➜ Relative Humidity (integer part)
	8 bits ➜ Relative Humidity (decimal part)
	8 bits ➜ Temperature (integer part)
	8 bits ➜ Temperature (decimal part)
	8 bits ➜ Checksum (RH integer + RH decimal + T intger +T Decimal)
5) Output the data with the data_valid pulse after cheking checksum condition
*/

	//default values of registers when reset becomes low
	if (!reset) begin
		clk_count <= clock_init;
		state <= s_START_18ms;
		T_integral <= 0;
      RH_integral <= 0;
      T_decimal <= 0;
      RH_decimal <= 0;
      Checksum <= 0;
      data_valid <= 0;
			high_count <= clock_init; 
		 total_word <= 0;
		 sensor_out <= 0;
		 drive_enable <= 1;
	end
	
	else begin
	
		case (state)
		
			//State to initiate the communication 
			//by pulling the sensor line LOW for 18ms (900000 cycles)
			s_START_18ms: begin
				drive_enable <= 1'b1; //sensor is output
				sensor_out <= 1'b0; //sensor is pulled LOW
				data_valid <= 1'b0; //This prevents the data_valid signal from persisting after 1 cycle
				total_word<=0; //To initialize the word again for the new input
				
				if (clk_count >= 900000) begin //900000 cycles correspond to 18ms in 50Mhz clk
						clk_count <= clock_init; //If crossed 18ms, go to next state
						state <= s_START_40us;
				end
				else  begin
						clk_count <= clk_count + 1; //else, wait for completion of 18ms 
				end
			end
			
			//State to implement next part of start signal
		   //	where sensor line is pulled HIGH for 40 us (2000 cycles)
			
			s_START_40us: begin
				drive_enable <= 1'b1; //sensor is output
				sensor_out <= 1'b1; //sensor is pulled HIGH
				
				if (clk_count >= 2000) begin //2000 cycles correspond to 40us in 50Mhz clk
							clk_count <= clock_init;
							state <= s_START_COMM_1; // Ready to start reading from dht11 after 40us
				end
				if ((sensor === 1'b1) && (clk_count<2000)) //wait for 40us 			
							clk_count <= clk_count + 1;
			end

		   //State to detect communication initiation signal from the sensor
		   //The sensor responds with 80us low and 80us high after the start signal
			s_START_COMM_1 : 
				begin
					drive_enable <= 1'b0; //sensor is input
					if (clk_count >= 4000) begin //4000 cycles correspond to 80us in 50Mhz clk
						state <= s_START_COMM_2;
						clk_count <= clock_init; //after 80us low, go to detect 80us high
					end
					if (clk_count < 4000) begin //else, wait for 80us to complete
						clk_count <= clk_count + 1; 
					end
				end
				
			//This state is to detect the 80us high
			s_START_COMM_2:
				begin
					drive_enable <= 1'b0; //sensor is input
					if (clk_count >= 4000) begin //after 80us HIGH, go to detect 50us LOW of starting of each bit
							state <= s_WAIT_HALF;
							clk_count <= clock_init;
					end
					else begin
							clk_count <= clk_count + 1; //else, wait for 80us
					end
				end
				
			//For every bit, the sensor responds with 50us low
			//This state is to detect the same:
			s_WAIT_HALF:
				begin
					drive_enable <= 1'b0; //sensor is input
					if (bit_count == 40) begin //If received all the 40 bits, go to load state
						state <= s_LOAD;
						clk_count <=clock_init;
					end
					if (clk_count >= 2500) begin //If not received all bits, go to the measuring state after the 50us
							state <= s_SECOND_HALF;
							clk_count <= clock_init;
							high_count <= clock_init;
					end
					else
						clk_count <= clk_count + 1; //else, wait for 50us
				end
				
			//Following the 50us low, the bit is decided to be 0 or 1
		   //based on the duration of the subsequent HIGH signal.
		   //This state counts the duration of the subsequent high and loads the bit in the register
			//28 us for 0 or 70 us for 1
			s_SECOND_HALF:
				begin
					drive_enable <= 1'b0; //sensor is input
					//once the line goes low in this state, it denotes that the high duration for the particular bit is over
					if (((sensor == 1'b0) && (high_count >= 1000)) || (high_count >= 3000)) begin //|| (sensor === 1'bz)) begin
						state <= s_LOAD; //in that case, go to loading state
						clk_count <= clock_init;
					end
					else begin
						//until the line is high, count the duration by incrementing high_count to a max limit of 2047
						if ((sensor == 1'b1))// || (high_count < 2047)) 
							high_count <= high_count + 1;
					end
				end
				
			
		   //This state loads the received bits to the ports
		   //Checks the checksum condition
			//And gives out data_valid accordingly
			s_LOAD:
				begin
					drive_enable <= 1'b0; //sensor is input
					
					if (bit_count <= 38) begin // before all 40 bits have been received
						bit_count <= bit_count + 1; //record how many bits have been read so far
						state <= s_WAIT_HALF; //go to wait for 50us for the next bit
						clk_count <= clock_init;						
						
						//1400 cycles is 28us, that is, a 0 bit's HIGH duration and 3500 for bit 1. 2000 given for safe margin
						total_word <= { total_word[38:0], (high_count >= 2000) };//load the current bit in the 40-bit word	
					end
					
					else begin //once 40 bits have been received
    
						
						
						if (total_word[7:0] == total_word[39:32] + total_word[31:24] + total_word[23:16] + total_word[15:8]) begin
							data_valid <= 1'b1;	//data_valid is one, only if checksum condition is satisfied
                            RH_integral <= total_word[39:32]; //first 8 bits are integral part of relative humidity
						RH_decimal <= total_word[31:24]; //next 8 bits are decimal part of relative humidity
						T_integral <= total_word[23:16]; //next 8 bits are integral part of temperature
						T_decimal <= total_word[15:8]; //next 8 bits are decimal part of temperature
						Checksum <= total_word[7:0]; //last 8 bits are checksum
						
						
                  end
						bit_count <= 1'b0; //after 40 bits, reset bit count to 0						
						state <= s_START_18ms; //go to initial state of start signal for the next set of values
						clk_count <= clock_init;
					end
						
				end
				default:
					begin
						state <= s_START_18ms; //default state is start signal state
						clk_count <= clock_init;
						drive_enable <= 1'b0; //sensor is by default input
					end
		endcase
	end

end

//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////
  
endmodule