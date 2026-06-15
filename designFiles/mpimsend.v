 module mpimsend (
    input        clk50,     
    input        ir_sensor,
    output       uart_txx1,
   ////////
	output [1:0] state,
	output tx_done,
	output reg [5:0] char_index,
	output reg [3:0] point_number,
	output reg start_trans,
	output servo_pulse,
	inout sensor,
	input dout,
	output reg ack,
	input deadend,
	output reg reset_servo
);

uart_tx trans(.clk50M(clk50), .tx_start(tx_start) , .data(tx_data) , .tx(uart_txx1) , .tx_done(tx_done) , .state(state));
servo_runn run (.clk(clk50), .reset(reset_servo), .servo_pulse(servo_pulse));
dht sensor_dht (.clk_50M(clk50), .reset(1), .sensor(sensor), .T_integral(dht_temp) , .RH_integral(dht_humid));
moisture_sensor sensor2(.dout(dout), .clk50(clk50) , .sent_1(soil_input));

wire [7:0]soil_input;

wire [7:0]dht_temp;
wire [7:0]dht_humid;

//reg reset_servo;

wire [7:0] soil_input1;
wire [7:0] soil_input2;

wire [7:0] dht_temp1;
wire [7:0] dht_temp2;

wire [7:0] dht_humid1;
wire [7:0] dht_humid2;


assign dht_temp1 = (dht_temp/10)+48;
assign dht_temp2 = (dht_temp%10)+48;

assign dht_humid1 = (dht_humid/10)+48;
assign dht_humid2 = (dht_humid%10)+48;

reg mpim_fsm;
localparam idle = 1'b0;
localparam execute = 1'b1;
reg [7:0]  tx_data;
reg last_ir;
reg waitt;
reg [2:0] counter = 0;
reg tx_start;
reg clk_3125KHz;
reg rtt;
reg [28:0]delay;
reg [28:0] main_delay;
initial begin
	mpim_fsm = idle;
	tx_data = "M";
	char_index = 0;
	point_number = 0;
	tx_start = 0;
	start_trans = 0;
	last_ir = 0;
	clk_3125KHz = 0;
	waitt = 0;
	rtt = 0;
	delay = 0;
	reset_servo = 0;
	ack = 0;
	main_delay = 0;
end

always @ (posedge clk50) begin
    if (!counter) clk_3125KHz = ~clk_3125KHz; // toggles clock signal
    counter = counter + 1'b1; // increment counter // after 7 it resets to 0
end

always @(posedge clk50)begin
//case(mpim_fsm)
//	idle:begin
//	ack <= 0;
//	
//	end
//endcase
 
 if (ir_sensor == 0 && deadend) begin
	start_trans <= 1;
	rtt <= 1;
	reset_servo <=1;
	
 end
 if (ir_sensor == 1 && last_ir == 0)
 ack <=0;
 
 
 
 if(start_trans == 1) begin
		
 
	 if(delay < 400000000) begin
		delay = delay + 1;
	 end
	 
 else begin
 
	if(rtt == 1)begin
		rtt <= 0;
	if (point_number < 9)
				point_number = point_number + 1;
				else 
				point_number = 0;
	end

			case (char_index)
                0: tx_data = "M";
                1: tx_data = "P";
                2: tx_data = "I";
                3: tx_data = "M";
                4: tx_data = "-";
                5: begin
					 tx_data = 8'd48 + point_number;
					 end
                6: tx_data = "-";
                7: tx_data = "#";
					 8 :tx_data = "M";
					  9 :tx_data = "M";
					   10 :tx_data = "-";
						11 :tx_data = 8'd48 + point_number;
						12 :tx_data = "-";
						13 :tx_data = soil_input;
						14: tx_data = "-";
                15: tx_data = "#";
					 16: tx_data = "T";
                17: tx_data = "H";
                18: tx_data = "-";
                19: begin
					 tx_data = 8'd48 + point_number;
					 end
					 20 :tx_data = "-";
						21 :tx_data = dht_temp1;
						22 :tx_data = dht_temp2;
						23 :tx_data = "-";
						24 :tx_data = dht_humid1;
						25 :tx_data = dht_humid2;
						26: tx_data = "-";
                27: tx_data = "#";
                default: tx_data <= 8'h00;
            endcase
			
		
			
		if(state == 0) begin
			tx_start <= 1;
			waitt = 1;
		end
		else begin
		tx_start <= 0;
		end
		
		if(state == 3) begin
			if(char_index < 28 ) begin
				if(waitt == 1)begin
					char_index = char_index + 1;
					waitt <= 0;
				end
			end
				else begin
				ack <= 1;
				reset_servo<=0;
				start_trans <= 0;
				char_index <= 0;
				delay <= 0;
				end 
			
		end	
	end
	end
	last_ir <= ir_sensor;	
	end
endmodule