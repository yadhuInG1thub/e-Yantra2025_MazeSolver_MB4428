module clk_1k(
		input clk50,
		output reg clk_1k
		
);

reg [15:0] cnt_1k;


initial begin

cnt_1k=0;

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