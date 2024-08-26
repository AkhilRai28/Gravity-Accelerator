`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.03.2024 18:06:25
// Design Name: 
// Module Name: gravity_accelerator_tb
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


module gravity_accelerator_tb;

    // Parameters
    parameter CLK_PERIOD = 10; // Clock period in nanoseconds

    // Signals
    reg clk;
    reg reset;
    reg enable;
    reg signed [31:0] gravity;
    reg signed [31:0] init_velocity;
    reg signed [31:0] init_position;
    reg signed [31:0] mass;
    reg signed [31:0] time_step;
    reg signed [31:0] lower_bound;
    reg signed [31:0] upper_bound;
    reg signed [31:0] damping_factor;
    reg signed [31:0] friction_coefficient;
    reg signed [31:0] obstacle_position_0;
    reg signed [31:0] obstacle_position_1;
    reg signed [31:0] obstacle_position_2;
    reg signed [31:0] obstacle_size_0;
    reg signed [31:0] obstacle_size_1;
    reg signed [31:0] obstacle_size_2;
    wire signed [31:0] velocity;
    wire signed [31:0] position;
    wire [2:0] obstacle_collision;

    // Clock generation
    always #((CLK_PERIOD)/2) clk = ~clk;

    // Reset generation
    initial begin
        clk = 0;
        reset = 1;
        #50;
        reset = 0;
        #100;
        enable = 1;
        // Provide testbench stimuli here
        // Example:
        // gravity = 9.81;
        // init_velocity = 0;
        // init_position = 0;
        // mass = 10;
        // time_step = 1;
        // lower_bound = -100;
        // upper_bound = 100;
        // damping_factor = 1;
        // friction_coefficient = 1;
        // obstacle_position_0 = -50;
        // obstacle_position_1 = 0;
        // obstacle_position_2 = 50;
        // obstacle_size_0 = 10;
        // obstacle_size_1 = 15;
        // obstacle_size_2 = 20;
        #200;
        $finish;
    end

    // Instantiate the module under test
    gravity_accelerator dut (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .gravity(gravity),
        .init_velocity(init_velocity),
        .init_position(init_position),
        .mass(mass),
        .time_step(time_step),
        .lower_bound(lower_bound),
        .upper_bound(upper_bound),
        .damping_factor(damping_factor),
        .friction_coefficient(friction_coefficient),
        .obstacle_position_0(obstacle_position_0),
        .obstacle_position_1(obstacle_position_1),
        .obstacle_position_2(obstacle_position_2),
        .obstacle_size_0(obstacle_size_0),
        .obstacle_size_1(obstacle_size_1),
        .obstacle_size_2(obstacle_size_2),
        .velocity(velocity),
        .position(position),
        .obstacle_collision(obstacle_collision)
    );

    // Display simulation results
    initial begin
        $monitor("Time: %0t, Velocity: %d, Position: %d, Obstacle Collision: %b", $time, velocity, position, obstacle_collision);
    end

endmodule
