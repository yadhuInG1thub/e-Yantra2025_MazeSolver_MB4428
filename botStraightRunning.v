module botStraightRunning
(
        input clk50

);
wire [15:0] left_dist;
wire [15:0] right_dist;
wire [15:0] mid_dist;

wire echo_left;
wire trig_left;
wire op_left;

wire echo_right;
wire trig_right;
wire op_right;

wire echo_mid;
wire trig_mid;
wire op_mid;

wire [15:0] right_rpm;
wire [15:0] left_rpm;

reg [1:0] pwm;

parameter st_move = 0, st_check = 1;
reg state;

initial begin
        right_dist = 16'b0;
        mid_dist  = 16'b0;

        right_rpm = 16'b0;
        left_rpm = 16'b0;

        pwm = 2'b11;
end

//botrun 

ultrasonic left(.clk_50M(clk50), .reset(1), .echo_rx(echo_left), .trig(trig_left), .op(op_left), .distance_out(left_dist));
ultrasonic right(.clk_50M(clk50), .reset(1), .echo_rx(echo_right), .trig(trig_right), .op(op_right), .distance_out(left_right));
ultrasonic mid(.clk_50M(clk50), .reset(1), .echo_rx(echo_mid), .trig(trig_mid), .op(op_mid), .distance_out(left_mid));

always @(posedge clk50)begin
        case(state)

        st_move : begin
                if(left_rpm < 16'd1280 )begin
                        state = 0;    
                        if (left_dist < 16'd30)begin
                                pwm = 2'b01;
                        end  
                        else if (right_dist < 16'd30)begin
                                pwm = 2'b10;
                        end         
                        else begin
                                pwm = 2'b11;
                        end     
                end
                else begin
                        state = st_check;
                end
        end

        st_check : begin
                if (op_left == 1 && op_right == 1 && op_mid == 1)begin
                        if(left_dist >= right_dist && left_dist >= mid_dist)begin
                                move = 2'b00;
                        end
                        else if(right_dist >= left_dist && right_dist >= mid_dist)begin
                                move = 2'b01;
                        end
                        else if(mid_dist >= left_dist && mid_dist >= right_dist)begin
                                move = 2'b10;
                        end
                        else begin
                                move = 2'b11;
                        end
                        state = st_move;
                end
                else begin
                        state = st_check;
                end
        end

        endcase
end
endmodule

