module uart_tx(
    input clk50M,
    input  tx_start,
	 input [7:0] data,
    output reg tx, tx_done,
	 output reg [1:0] state 
);

initial begin
    tx = 1'b1;        
    tx_done = 1'b0;
	 state = IDLE;	 
end

//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE//////////////////
 
// UART Timing Parameter
// CLKS_PER_BIT = Clock cycles required per bit at 115200 baud
// With clk = 3.125 MHz --> 3125000 / 115200 which is approximately 27 cycles. So the 27 cycles is set in the CLKS_PER_BIT parameter
reg [9:0] CLKS_PER_BIT =434;
reg [2:0] counter = 0; // counts 0 to 7


// UART State Machine Definitions

localparam IDLE       = 0;   // UART line idle (in the idle state, the tx is always 1)
localparam START_BIT  = 1;   // Transmitting start bit (the start bit is always 0)
localparam DATA_BITS  = 2;   // Transmitting 8 data bits
localparam STOP_BIT   = 3;   // Transmitting stop bit (the stop bit is always 1)

// Internal Registers

//changed to output reg         // Current state of the machine
reg [9:0] clk_count = 0;        // Counts clock cycles for each bit duration
reg [2:0] bit_index = 0;        // Selects which data bit is being transmitted

// Main UART Transmission Logic
/*PURPOSE
 This always block handles the full UART transmission sequence.
 In one transmission sequence, it sends the start bit, shifts out data bits, outputs the parity and stop bits,
 and finally generates a 1-clock tx_done pulse once the frame is fully sent.
*/
always @(posedge clk50M) begin
    tx_done <= 1'b0;    // tx_done is a pulse; reset it every clock unless STOP_BIT sets it.

    case(state)

        // IDLE STATE
        // UART line stays HIGH. Wait for tx_start to begin transmission.
		  
        IDLE: begin
            clk_count <= 0;
            bit_index <= 0;
            tx <= 1'b1;     // Idle line high

            if (tx_start) begin
                tx <= 1'b0; // On tx_start = 1, immediately pull tx LOW to generate the start bit.
                state <= START_BIT;
            end
        end

        //  START BIT STATE
        // Hold tx = 0 for exactly CLKS_PER_BIT clock cycles.
		  // After this period, go to transmitting the first data bit.
        START_BIT: begin
            if (clk_count < CLKS_PER_BIT-1) begin
                clk_count <= clk_count + 1;  // Hold start bit for the CLKS_PER_BIT clock cycles
            end
            else begin
                clk_count <= 0;// Clear the counter and the bit index counter not to interfere the count
                bit_index <= 0;// with other states. Then move to the next state=> DATA BITS transmission
                state <= DATA_BITS;

                // Load MSB first of the data since we transmit from MSB to LSB
                tx <= data[0];
            end
        end

        //  DATA BITS STATE
        // Transmit each data bit for CLKS_PER_BIT duration.
		 // Data is transmitted MSB → LSB (reverse index).
		 DATA_BITS: begin
			  if(clk_count < CLKS_PER_BIT-1) begin
					clk_count <= clk_count + 1;// Wait inside the duration of this data bit
			  end else begin
					clk_count <= 0;// Completed sending one data bit

					if(bit_index < 7) begin
						 // Move to next bit
						 bit_index <= bit_index + 1;

						 // Output next data bit. Reverse indexing:
						 // When bit_index=0 → data[6]
						 // When bit_index=1 → data[5] ... etc.
						 tx <= data[bit_index+1];
					end else begin
					clk_count <= 0;
					tx <= 1'b1;   // Stop bit is always 1
					state <= STOP_BIT;
					end
			  end
		 end

        
        //  STOP BIT STATE
        //  Send stop bit and signal completion
         
        // Stop bit is simply HIGH for one bit duration.
		 // After this, tx_done is asserted for one clock.
		 STOP_BIT: begin
			  if(clk_count < CLKS_PER_BIT-1) begin
					// Slightly shorter wait ensures clean alignment before IDLE
					clk_count <= clk_count + 1;
			  end else begin
					clk_count <= 0;

					tx_done <= 1'b1;  // Signal: "Transmission Completed"
					state <= IDLE;    // Return to IDLE and wait for next start request
			  end
		 end
        // DEFAULT STATE
		  // if the curr_state bits goes into any other value other than the given states, for a safety, the system is returned to the idle state
        default: 
				state <= IDLE; 
    endcase
end

//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE//////////////////

endmodule