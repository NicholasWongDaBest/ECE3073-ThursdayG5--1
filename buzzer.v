module music_player (
    input clk,                 // 50 MHz clock
    input [4:0] note_id,       // 5-bit input from Nios II PIO
    output reg buzzer_pin      // 1-bit output to physical buzzer
);

reg [23:0] limit;          // Internal wire to hold the current note's limit
reg [23:0] counter = 0;    // The stopwatch

// --- THE LOOK-UP TABLE ---
// Quartus will automatically synthesize this into a MUX!
always @(*) begin
case(note_id)
        5'd0:  limit = 24'd0;       // Rest / Silence
        
        // --- First Octave ---
        5'd1:  limit = 24'd143176;  // Note F3  (174.61 Hz)
        5'd2:  limit = 24'd135135;  // Note F#3 (185.00 Hz)
        5'd3:  limit = 24'd127551;  // Note G3  (196.00 Hz)
        5'd4:  limit = 24'd120395;  // Note G#3 (207.65 Hz)
        5'd5:  limit = 24'd113636;  // Note A3  (220.00 Hz)
        5'd6:  limit = 24'd107259;  // Note A#3 (233.08 Hz)
        5'd7:  limit = 24'd101239;  // Note B3  (246.94 Hz)
        5'd8:  limit = 24'd95555;   // Note C4  (261.63 Hz) - Middle C
        5'd9:  limit = 24'd90194;   // Note C#4 (277.18 Hz)
        5'd10: limit = 24'd85132;   // Note D4  (293.66 Hz)
        5'd11: limit = 24'd80352;   // Note D#4 (311.13 Hz)
        5'd12: limit = 24'd75843;   // Note E4  (329.63 Hz)

        // --- Second Octave ---
        5'd13: limit = 24'd71586;   // Note F4  (349.23 Hz)
        5'd14: limit = 24'd67569;   // Note F#4 (369.99 Hz)
        5'd15: limit = 24'd63776;   // Note G4  (392.00 Hz)
        5'd16: limit = 24'd60197;   // Note G#4 (415.30 Hz)
        5'd17: limit = 24'd56818;   // Note A4  (440.00 Hz)
        5'd18: limit = 24'd53630;   // Note A#4 (466.16 Hz)
        5'd19: limit = 24'd50620;   // Note B4  (493.88 Hz)
        5'd20: limit = 24'd47778;   // Note C5  (523.25 Hz)
        5'd21: limit = 24'd45096;   // Note C#5 (554.37 Hz)
        5'd22: limit = 24'd42566;   // Note D5  (587.33 Hz)
        5'd23: limit = 24'd40177;   // Note D#5 (622.25 Hz)
        5'd24: limit = 24'd37922;   // Note E5  (659.25 Hz)
        
        default: limit = 24'd0;     // Catch-all: defaults to silence for inputs 25-31
    endcase
end

// --- THE COUNTER LOGIC ---
always @(posedge clk) begin
    if (limit == 0) begin
        counter <= 0;
        buzzer_pin <= 0;            // Keep pin low, no sound
    end 
    else if (counter >= limit) begin
        counter <= 0;               // Reset stopwatch
        buzzer_pin <= ~buzzer_pin;  // Toggle the buzzer wire
    end 
    else begin
        counter <= counter + 1;     // Keep counting
    end
end

endmodule