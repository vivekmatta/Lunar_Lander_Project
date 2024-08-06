`default_nettype none
// Empty top module

module top (
  // I/O ports
  input  logic hz100, reset,
  input  logic [20:0] pb,
  output logic [7:0] left, right,
         ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0,
  output logic red, green, blue,

  // UART ports
  output logic [7:0] txdata,
  input  logic [7:0] rxdata,
  output logic txclk, rxclk,
  input  logic txready, rxready
);

    logic [3:0] disp_ctrl;
    logic [4:0] keyout;
    logic keyclk;

    // Instantiate keysync module
    keysync key_sync(
        .clk(hz100), 
        .rst(reset),
        .keyin(pb[19:0]), 
        .keyout(keyout),
        .keyclk(keyclk)
    );

    // Logic to update disp_ctrl based on keyout
    always_ff @(posedge keyclk or posedge reset) begin
        if (reset)
            disp_ctrl <= 4'b1000; // Default to altitude display
        else begin
            case (keyout)
                5'd16: disp_ctrl <= 4'b0001; // W pressed
                5'd17: disp_ctrl <= 4'b0010; // X pressed
                5'd18: disp_ctrl <= 4'b0100; // Y pressed
                5'd19: disp_ctrl <= 4'b1000; // Z pressed
                default: disp_ctrl <= disp_ctrl; // No change
            endcase
        end
    end

    // Instantiate lunarlander module
    lunarlander lunar_lander_inst(
        .hz100(hz100), 
        .reset(reset), 
        .in(pb[19:0]), 
        .ss7(ss7), .ss6(ss6), .ss5(ss5), 
        .ss3(ss3), .ss2(ss2), .ss1(ss1), .ss0(ss0),
        .red(red), .green(green)
    );


 
endmodule

module lunarlander #(
    parameter FUEL = 16'h800,
    parameter ALTITUDE = 16'h4500,
    parameter VELOCITY = 16'h0,
    parameter THRUST = 16'h5,
    parameter GRAVITY = 16'h5
)(
    input logic hz100, reset,
    input logic [19:0] in,
    output logic [7:0] ss7, ss6, ss5,
    output logic [7:0] ss3, ss2, ss1, ss0,
    output logic red, green
);

    // Internal state variables
    logic [15:0] alt, vel, fuel, thrust;
    logic [15:0] alt_n, vel_n, fuel_n, thrust_n;
    logic land, crash;
    logic wen; // Write enable for memory

    // Internal signal for display control
    logic [3:0] disp_ctrl;

    // Clock prescaler instantiation
    logic hzX; // Slower clock output
    logic [7:0] clock_limit = 8'd24; 
    clock_psc clk_prescaler (
        .clk(hz100),
        .rst(reset),
        .lim(clock_limit),
        .hzX(hzX)
    );

    // Instantiate memory module
    ll_memory mem(
        .clk(hzX), .rst(reset), .wen(wen),
        .alt_n(alt_n), .vel_n(vel_n), .fuel_n(fuel_n), .thrust_n(thrust_n),
        .alt(alt), .vel(vel), .fuel(fuel), .thrust(thrust)
    );

    // Instantiate ALU module
    ll_alu alu(
        .alt(alt), .vel(vel), .fuel(fuel), .thrust(thrust),
        .alt_n(alt_n), .vel_n(vel_n), .fuel_n(fuel_n)
    );

    // Instantiate control module
    ll_control ctrl(
        .clk(hzX), .rst(reset),
        .alt(alt), .vel(vel),
        .land(land), .crash(crash), .wen(wen)
    );

    // Key input handling
    logic [4:0] keyout;
    logic keyclk;
    keysync key_sync(
        .clk(hz100), .rst(reset),
        .keyin(in), .keyout(keyout), .keyclk(keyclk)
    );

    // Handling display control based on key inputs
    always_comb begin
            case (keyout)
                5'd16: disp_ctrl = 4'b0001; // W pressed
                5'd17: disp_ctrl = 4'b0010; // X pressed
                5'd18: disp_ctrl = 4'b0100; // Y pressed
                5'd19: disp_ctrl = 4'b1000; // Z pressed
                default: disp_ctrl = 4'b0; // No change
            endcase
    end

    // Thrust handling based on key input
    always_ff @(posedge keyclk or posedge reset) begin
        if (reset) 
            thrust_n <= THRUST;
        else if (~keyout[4]) 
            thrust_n <= {12'b0, keyout[3:0]};
    end

    // Instantiate display module
    ll_display display(
        .clk(keyclk), .rst(reset),
        .land(land), .crash(crash),
        .disp_ctrl(disp_ctrl),
        .alt(alt), .vel(vel), .fuel(fuel), .thrust(thrust),
        .ss7(ss7), .ss6(ss6), .ss5(ss5),
        .ss3(ss3), .ss2(ss2), .ss1(ss1), .ss0(ss0),
        .red(red), .green(green)
    );

    // Other module code as needed...
endmodule

module ll_memory #(
  parameter ALTITUDE = 16'h4500,
  parameter VELOCITY = 16'h0,
  parameter FUEL     = 16'h800,
  parameter THRUST   = 16'h5
)(
  input logic        clk, rst, wen,
  input logic [15:0] alt_n, vel_n, fuel_n, thrust_n,
  output logic [15:0] alt, vel, fuel, thrust
);

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      alt    <= ALTITUDE;
      vel    <= VELOCITY;
      fuel   <= FUEL;
      thrust <= THRUST;
    end else if (wen) begin
      alt    <= alt_n;
      vel    <= vel_n;
      fuel   <= fuel_n;
      thrust <= thrust_n;
    end
  end

endmodule

module ll_alu #(parameter GRAVITY = 16'h5)(
  input logic [15:0] alt, vel, fuel, thrust,
  output logic [15:0] alt_n, vel_n, fuel_n
);
  //parameter GRAVITY = 16'h5;

  // Internal buses for calculated values
  logic [15:0] alt_c, vel_c, fuel_c;

  // Adjust thrust based on fuel availability
  logic [15:0] thrust_adj;
  assign thrust_adj = (fuel == 16'h0) ? 16'h0 : thrust;

  // Calculate new altitude
  bcdaddsub4 add_alt(.a(alt), .b(vel), .op(1'b0), .s(alt_c));

  // Calculate new velocity
  logic [15:0] vel_temp;
  bcdaddsub4 sub_vel(.a(vel), .b(GRAVITY), .op(1'b1), .s(vel_temp));
  bcdaddsub4 add_thrust(.a(vel_temp), .b(thrust_adj), .op(1'b0), .s(vel_c));

  // Calculate new fuel
  bcdaddsub4 sub_fuel(.a(fuel), .b(thrust), .op(1'b1), .s(fuel_c));

  // Handle special cases and assign outputs
  assign alt_n = (alt_c[15] == 1'b0 && alt_c != 16'h0) ? alt_c : 16'h0;
  assign vel_n = (alt_c[15] == 1'b0 && alt_c != 16'h0) ? vel_c : 16'h0;
  assign fuel_n = (fuel_c[15] == 1'b0 && fuel_c != 16'h0) ? fuel_c : 16'h0;
endmodule

module ll_control (
    input logic clk,
    input logic rst,
    input logic [15:0] alt,
    input logic [15:0] vel,
    output logic land,
    output logic crash,
    output logic wen
);

    logic [15:0] ground_level;
    logic [15:0] neg_thirty_bcd; // BCD representation of -30
    logic ground_reached, velocity_less_than_neg_thirty;

    // (ten's complement of 30)
    assign neg_thirty_bcd = 16'h9970;

    // Use bcdaddsub4 module to calculate if ground level reached
    bcdaddsub4 alt_vel_sum (.a(alt), .b(vel), .op(1'b0), .s(ground_level));

    // Determine if the lander has reached the ground
    assign ground_reached = ground_level >= 16'h8000; // BCD for negative numbers starts with a 1

    // Determine if the velocity is less than -30
    assign velocity_less_than_neg_thirty = vel < neg_thirty_bcd;

    // Next state logic for land, crash and wen
    logic land_next, crash_next, wen_next;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset the outputs
            land <= 1'b0;
            crash <= 1'b0;
            wen <= 1'b0;
        end 
        else begin
            // Assign the next state to the outputs
            land <= land_next;
            crash <= crash_next;
            wen <= wen_next;
        end
    end

    // Determine the next state logic for land, crash and wen
    always_comb begin
        // Default next state values
        land_next = land;
        crash_next = crash;
        wen_next = wen;

        if (ground_reached) begin
            if (velocity_less_than_neg_thirty) begin
                crash_next = 1'b1;
                wen_next = 1'b0;
            end 
            else begin
                land_next = 1'b1;
                wen_next = 1'b0;
            end
        end 
        else if (!land && !crash) begin
            // Enable write if the lander has not landed or crashed
            wen_next = 1'b1;
        end
    end

endmodule

module ll_display (
    input logic clk, rst,
    input logic land, crash,
    input logic [3:0] disp_ctrl,
    input logic [15:0] alt, vel, fuel, thrust,
    output logic [7:0] ss7, ss6, ss5, ss3, ss2, ss1, ss0,
    output logic red, green
);

    // Internal variables for display
    logic [15:0] display_value, next_display_value;
    logic [3:0] val_digits[3:0];
    logic negative;
    logic [23:0] next_word;

    // Selecting the value to display
    always_comb begin
        case({ss7, ss6, ss5})
            24'b01110111_00111000_01111000 : display_value = alt;
            24'b00111110_01111001_00111000 : display_value = vel;
            24'b01101111_01110111_01101101 : display_value = fuel;
            24'b01111000_01110110_01010000 : display_value = thrust;
            default: display_value = alt;
        endcase
        negative = ({ss7, ss6, ss5} == 24'b00111110_01111001_00111000) && display_value[15];
    end

    // Convert the value to positive if negative
    logic [15:0] abs_value;
    bcdaddsub4 convert_to_positive(
        .a(16'h0), 
        .b(display_value), 
        .op(negative), 
        .s(abs_value)
    );

    // Split the value into digits
    assign val_digits[0] = abs_value[3:0];
    assign val_digits[1] = abs_value[7:4];
    assign val_digits[2] = abs_value[11:8];
    assign val_digits[3] = abs_value[15:12];

    // Instantiate ssdec for each digit
    logic [6:0] ss3_digit; // Separate digit part for ss3

    // Combine the digit and the minus sign for ss3
    always_comb begin
    if (negative) 
        ss3 = 8'b01000000; // Display minus sign for negative values
    else
        ss3 = {1'b0, ss3_digit}; // Concatenate to make 7 bits
    end

    ssdec disp_digit0(.in(val_digits[0]), .enable(1'b1), .out(ss0[6:0]));
    ssdec disp_digit1(.in(val_digits[1]), .enable(|abs_value[15:4]), .out(ss1[6:0]));
    ssdec disp_digit2(.in(val_digits[2]), .enable(|abs_value[15:8]), .out(ss2[6:0]));
    ssdec disp_digit3(.in(val_digits[3]), .enable(negative | (|abs_value[15:12])), .out(ss3_digit));

    // Messages for ss7-ss5
    always_comb begin
        case(disp_ctrl)
            4'b1000: next_word = 24'b01110111_00111000_01111000; // ALT
            4'b0100: next_word = 24'b00111110_01111001_00111000; // VEL
            4'b0010: next_word = 24'b01101111_01110111_01101101; // GAS (Fuel)
            4'b0001: next_word = 24'b01111000_01110110_01010000; // THR (Thrust)
            default: next_word = {ss7, ss6, ss5}; // Default to ALT
        endcase
    end

    always_ff @(posedge clk, posedge rst) begin

      if (rst) begin
        {ss7, ss6, ss5} <= 24'b01110111_00111000_01111000;
      end

      else begin
        {ss7, ss6, ss5} <= next_word;
      end
    end


    // LED Outputs
    assign red = crash;
    assign green = land;

    // Additional display logic as needed...

endmodule

module fa (
  input logic a,
  input logic b,
  input logic ci,
  output logic s,
  output logic co
);

  // Sum calculation using XOR
  assign s = a ^ b ^ ci;

  // Carry-out calculation
  assign co = (a & b) | (b & ci) | (ci & a);

endmodule

module fa4 (
  input logic [3:0] a,
  input logic [3:0] b,
  input logic ci,
  output logic [3:0] s,
  output logic co
);
  // Internal wires for carry connections between the full adders
  logic c1, c2, c3;
  // Instantiating four fa modules
  fa fa0(.a(a[0]), .b(b[0]), .ci(ci), .s(s[0]), .co(c1));
  fa fa1(.a(a[1]), .b(b[1]), .ci(c1), .s(s[1]), .co(c2));
  fa fa2(.a(a[2]), .b(b[2]), .ci(c2), .s(s[2]), .co(c3));
  fa fa3(.a(a[3]), .b(b[3]), .ci(c3), .s(s[3]), .co(co));
endmodule

module bcdadd1 (
  input logic [3:0] a,
  input logic [3:0] b,
  input logic ci,
  output logic [3:0] s,
  output logic co
);
 
  logic [3:0] binary_sum;
  logic binary_co;
  
  fa4 fa4_inst(.a(a), .b(b), .ci(ci), .s(binary_sum), .co(binary_co));
  
  logic [3:0] correction;
  assign correction = (binary_sum > 4'd9 || binary_co) ? 4'd6 : 4'd0;
  
  fa4 correction_adder(.a(binary_sum), .b(correction), .ci(1'b0), .s(s), .co());
  
  assign co = correction[2];
endmodule

module bcdadd4 (
  input logic [15:0] a,
  input logic [15:0] b,
  input logic ci,
  output logic [15:0] s,
  output logic co
);
  
  logic c1, c2, c3;
  
  // Instantiating four bcdadd1 modules
  bcdadd1 bcdadd1_0(.a(a[3:0]),   .b(b[3:0]),   .ci(ci), .s(s[3:0]),   .co(c1));
  bcdadd1 bcdadd1_1(.a(a[7:4]),   .b(b[7:4]),   .ci(c1), .s(s[7:4]),   .co(c2));
  bcdadd1 bcdadd1_2(.a(a[11:8]),  .b(b[11:8]),  .ci(c2), .s(s[11:8]),  .co(c3));
  bcdadd1 bcdadd1_3(.a(a[15:12]), .b(b[15:12]), .ci(c3), .s(s[15:12]), .co(co));
endmodule

module bcd9comp1 (
  input logic [3:0] in,
  output logic [3:0] out
);
  always_comb begin
    case (in)
      4'd0: out = 4'd9;
      4'd1: out = 4'd8;
      4'd2: out = 4'd7;
      4'd3: out = 4'd6;
      4'd4: out = 4'd5;
      4'd5: out = 4'd4;
      4'd6: out = 4'd3;
      4'd7: out = 4'd2;
      4'd8: out = 4'd1;
      4'd9: out = 4'd0;
      default: out = 4'dx;
    endcase
  end
endmodule

module bcdaddsub4 (
 input logic [15:0] a, b,
 input logic op,
 output logic [15:0] s
);
 logic [3:0] c;
 logic [15:0] va;
 logic [15:0] ans;
 
 bcd9comp1 bcd1(.in(b[3:0]), .out(va[3:0]));
 bcd9comp1 bcd2(.in(b[7:4]), .out(va[7:4]));
 bcd9comp1 bcd3(.in(b[11:8]), .out(va[11:8]));
 bcd9comp1 bcd4(.in(b[15:12]), .out(va[15:12]));
 assign ans[15:0] = (op == 1) ? va[15:0] : b[15:0];
 
 bcdadd4 bd1(.a(a[15:0]), .b(ans[15:0]), .ci(op), .s(s[15:0]), .co(c[0]));
 
 
endmodule

module ssdec (
  input logic [3:0] in,
  input logic enable,
  output logic [6:0] out
);

  logic [6:0] seg[15:0];
 
  assign seg[4'h0] = 7'b0111111;
  assign seg[4'h1] = 7'b0000110;
  assign seg[4'h2] = 7'b1011011;
  assign seg[4'h3] = 7'b1001111;
  assign seg[4'h4] = 7'b1100110;
  assign seg[4'h5] = 7'b1101101;
  assign seg[4'h6] = 7'b1111101;
  assign seg[4'h7] = 7'b0000111;
  assign seg[4'h8] = 7'b1111111;
  assign seg[4'h9] = 7'b1100111;
  assign seg[4'ha] = 7'b1110111;
  assign seg[4'hb] = 7'b1111100;
  assign seg[4'hc] = 7'b0111001;
  assign seg[4'hd] = 7'b1011110;
  assign seg[4'he] = 7'b1111001;
  assign seg[4'hf] = 7'b1110001;
   
  always_comb begin  
    if (enable == 1'b1)
      out[6:0] = seg[in];
     
    else
      out[6:0] = 7'd0;
  end
   
endmodule

module clock_psc (
    input wire clk,
    input wire rst,
    input wire [7:0] lim,
    output reg hzX
);

    logic [7:0] counter;
    logic [7:0] next_counter;
    logic hzX_scaled;

    always_comb begin
        if (counter == lim)
            next_counter = 8'd0;
        else
            next_counter = counter + 1;
    end

    always_ff @(posedge clk, posedge rst) begin
        if (rst) begin
            hzX_scaled <= 1'b0;
            counter <= 8'd0;
        end
        else begin
            counter <= next_counter;

            if (counter == lim) begin
              hzX_scaled <= ~hzX_scaled;
            end
        end
    end

    assign hzX = (lim == 0) ? clk : hzX_scaled;

endmodule

module keysync (
    input wire clk,
    input wire rst,
    input wire [19:0] keyin,
    output reg [4:0] keyout,
    output reg keyclk
);

    logic [1:0] sync;

    // Define the logic for keyout outputs
    assign keyout[0] = keyin[1] | keyin[3] | keyin[5] | keyin[7] | keyin[9] | keyin[11] | keyin[13] | keyin[15] | keyin[17] | keyin[19];
    assign keyout[1] = keyin[2] | keyin[3] | keyin[6] | keyin[7] | keyin[10] | keyin[11] | keyin[14] | keyin[15] | keyin[18] | keyin[19];
    assign keyout[2] = keyin[4] | keyin[5] | keyin[6] | keyin[7] | keyin[12] | keyin[13] | keyin[14] | keyin[15];
    assign keyout[3] = keyin[8] | keyin[9] | keyin[10] | keyin[11] | keyin[12] | keyin[13] | keyin[14] | keyin[15];
    assign keyout[4] = keyin[16] | keyin[17] | keyin[18] | keyin[19];

    assign keyclk = sync[1];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sync[0] <= 1'b0;
            sync[1] <= 1'b0;
        end
        else begin
            sync[0] <= |keyin;
            sync[1] <= sync[0];
        end
    end

endmodule