module servo_runn (
    input clk,
	 input reset,
	 output reg servo_pulse
);

reg [28:0] counter;
reg [1:0]turn;
reg step;
initial begin
	servo_pulse = 0;
	counter = 0;
	turn = 0;
	step = 0;
end



always @(posedge clk) begin
	
	if(reset == 1) begin
	
		if(step == 0)begin
			 case (turn)
			 0 : begin
				if(counter == 999999 ) begin //20ms
						counter = 0;
						turn = 1;
				 end
				 else begin
					counter = counter + 1;
				 end
					if(counter < 50000 ) begin
						servo_pulse <= 1;
					end
					
					else begin
						servo_pulse <= 0;
					end
				end
				1:begin
				servo_pulse <= 0;
				 if(counter == 300000000 ) begin
						counter = 0;
						step = 1;
						turn = 0;
				 end
				 else begin
					counter = counter + 1;
				 end
				end
			endcase
		end
		
		if(step == 1 )begin
			 case (turn)
			 0 : begin
				if(counter == 999999 ) begin
				counter = 0;
				turn = 1;
		 end
		 else begin
			counter = counter + 1;
		 end
			if(counter < 100000 ) begin
				servo_pulse <= 1;
			end
			
			else begin
				servo_pulse <= 0;
			end
			end
				1:begin
				servo_pulse <= 0;
				 if(counter == 100000000 ) begin
						counter = 0;
						step = 1;
						turn = 0;
				 end
				 else begin
					counter = counter + 1;
				 end
				end
			endcase
		end
	end
	else begin
		servo_pulse = 0;
	counter = 0;
	turn = 0;
	step = 0;
	end
		 
end

endmodule