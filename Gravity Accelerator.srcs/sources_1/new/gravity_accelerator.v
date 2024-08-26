`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.03.2024 17:53:52
// Design Name: 
// Module Name: gravity_accelerator
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module gravity_accelerator(
    input clk,                              // Clock input
    input reset,                            // Reset input
    input enable,                           // Enable input
    input signed [31:0] gravity,            // Gravity force input
    input signed [31:0] init_velocity,      // Initial velocity input
    input signed [31:0] init_position,      // Initial position input
    input signed [31:0] mass,               // Mass of the object
    input signed [31:0] time_step,          // Time step for simulation
    input signed [31:0] lower_bound,        // Lower boundary for position
    input signed [31:0] upper_bound,        // Upper boundary for position
    input signed [31:0] damping_factor,     // Damping factor for velocity decay
    input signed [31:0] friction_coefficient, // Coefficient of friction
    input signed [31:0] obstacle_position_0, // Position of obstacle 0
    input signed [31:0] obstacle_position_1, // Position of obstacle 1
    input signed [31:0] obstacle_position_2, // Position of obstacle 2
    input signed [31:0] obstacle_size_0,    // Size of obstacle 0
    input signed [31:0] obstacle_size_1,    // Size of obstacle 1
    input signed [31:0] obstacle_size_2,    // Size of obstacle 2
    output reg signed [31:0] velocity,      // Output velocity of the accelerated object
    output reg signed [31:0] position,      // Output position of the accelerated object
    output reg [2:0] obstacle_collision    // Output indicating collision with obstacles
);

// Internal parameters
parameter FIXED_BITS = 24;                 // Number of fractional bits for fixed-point arithmetic
parameter GRAVITY_BITS = 16;               // Number of bits to represent gravity force
parameter VELOCITY_BITS = 24;              // Number of bits to represent velocity
parameter POSITION_BITS = 32;              // Number of bits to represent position
parameter BOUNDARY_BITS = 16;              // Number of bits for boundary
parameter DAMPING_BITS = 16;               // Number of bits for damping factor
parameter HIT_THRESHOLD = 10;              // Threshold for collision detection
parameter FRICTION_BITS = 16;              // Number of bits for friction coefficient
parameter ANGULAR_VELOCITY_BITS = 16;      // Number of bits for angular velocity
parameter ANGULAR_ACCELERATION_BITS = 16;  // Number of bits for angular acceleration

// Constants
localparam GRAVITY_SCALE = 9.81;           // Earth's gravity constant in m/s^2
localparam TIME_SCALE = 1;                 // Default time scale for simulation
localparam MASS_SCALE = 1;                 // Mass scale for simulation
localparam OBSTACLE_SIZE_SCALE = 1;        // Obstacle size scale for simulation

// Internal registers
reg signed [FIXED_BITS+VELOCITY_BITS-1:0] acc;   // Linear acceleration (fixed-point)
reg signed [FIXED_BITS+VELOCITY_BITS-1:0] vel;   // Linear velocity (fixed-point)
reg signed [FIXED_BITS+POSITION_BITS-1:0] pos;   // Linear position (fixed-point)
reg signed [31:0] force1;                          // Force
reg signed [31:0] timestep_scaled;                // Time step scaled
reg signed [31:0] obstacle_size_scaled[2:0];      // Scaled obstacle sizes
reg [2:0] obstacle_collision_internal;            // Internal obstacle collision signals

// Internal variables
reg crossing_lower_boundary;
reg crossing_upper_boundary;
reg [2:0] i;

always @(*) begin
    force1 = (gravity * ($signed(GRAVITY_SCALE) * $signed(MASS_SCALE) * $signed(mass))) >> (31 - GRAVITY_BITS); // Compute linear force
    timestep_scaled = (time_step * TIME_SCALE) >> (31 - FIXED_BITS); // Scale time step
    obstacle_size_scaled[0] = (obstacle_size_0 * OBSTACLE_SIZE_SCALE) >> (31 - POSITION_BITS); // Scale obstacle sizes
    obstacle_size_scaled[1] = (obstacle_size_1 * OBSTACLE_SIZE_SCALE) >> (31 - POSITION_BITS);
    obstacle_size_scaled[2] = (obstacle_size_2 * OBSTACLE_SIZE_SCALE) >> (31 - POSITION_BITS);
    crossing_lower_boundary = (pos <= lower_bound) ? 1'b1 : 1'b0; // Check if crossing lower boundary
    crossing_upper_boundary = (pos >= upper_bound) ? 1'b1 : 1'b0; // Check if crossing upper boundary
end

always @(posedge clk or posedge reset) begin
    if (reset) begin
        vel <= init_velocity << FIXED_BITS;
        pos <= init_position << FIXED_BITS;
        acc <= 0;
    end
    else if (enable) begin
        acc <= ((force1 / mass) - ((vel >> (VELOCITY_BITS - DAMPING_BITS)) * damping_factor)) - (vel * friction_coefficient >> (VELOCITY_BITS - FRICTION_BITS)); // Calculate linear acceleration with damping and friction
        vel <= vel + acc * timestep_scaled; // Update linear velocity using Euler's method
        pos <= pos + vel * timestep_scaled; // Update linear position using Euler's method
        // Boundary check and rebounding
        if (crossing_lower_boundary) begin
            vel <= -vel; // Rebound from lower boundary
            pos <= lower_bound << FIXED_BITS; // Move back to the boundary
        end
        else if (crossing_upper_boundary) begin
            vel <= -vel; // Rebound from upper boundary
            pos <= upper_bound << FIXED_BITS; // Move back to the boundary
        end
        // Collision detection with obstacles
        for (i = 0; i < 3; i = i + 1) begin
            if (pos >= (obstacle_position_0 - obstacle_size_scaled[i]) && pos <= (obstacle_position_0 + obstacle_size_scaled[i]) && vel < HIT_THRESHOLD) begin
                obstacle_collision_internal[i] <= 1'b1; // Set obstacle collision flag
            end
            else begin
                obstacle_collision_internal[i] <= 1'b0; // Reset obstacle collision flag
            end
        end
    end
end

always @(posedge clk) begin
    if (reset) begin
        obstacle_collision_internal <= 3'b000; // Reset obstacle collision signals
    end
    else if (enable) begin
        obstacle_collision <= obstacle_collision_internal; // Update external obstacle collision signal
    end
end

// Output conversion from fixed-point to integer
always @(*)
begin
    velocity = vel[FIXED_BITS+VELOCITY_BITS-1:FIXED_BITS];
    position = pos[FIXED_BITS+POSITION_BITS-1:FIXED_BITS];
end

endmodule
