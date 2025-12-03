#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include "api.h"
#include "lemlib/api.hpp"

// ========================================
// MOTOR PORT CONFIGURATION
// ========================================
// Left side motors (use negative values for reversed motors)
const int LEFT_FRONT_MOTOR_PORT = 1;
const int LEFT_MIDDLE_MOTOR_PORT = 2;
const int LEFT_BACK_MOTOR_PORT = 3;

// Right side motors (use negative values for reversed motors)
const int RIGHT_FRONT_MOTOR_PORT = -4;
const int RIGHT_MIDDLE_MOTOR_PORT = -5;
const int RIGHT_BACK_MOTOR_PORT = -6;

// ========================================
// SENSOR PORT CONFIGURATION
// ========================================
// IMU (Inertial Measurement Unit) port
const int IMU_PORT = 10;

// Tracking wheels (if using external tracking wheels)
// Set to -1 if not using tracking wheels
const int LEFT_TRACKING_WHEEL_PORT = -1;    // ADI port for left tracking wheel encoder
const int RIGHT_TRACKING_WHEEL_PORT = -1;   // ADI port for right tracking wheel encoder
const int BACK_TRACKING_WHEEL_PORT = -1;    // ADI port for back/horizontal tracking wheel encoder

// ========================================
// ROBOT PHYSICAL DIMENSIONS
// ========================================
// All measurements in inches

// Wheel diameter (measure the wheel's diameter)
const float WHEEL_DIAMETER = 4.0;

// Drive gear ratio (driven/driving)
// For example: 48/36 = 1.333 means output is 1.333x slower than motor
// If using direct drive (no gears), set to 1.0
const float DRIVE_GEAR_RATIO = 1.0;

// Track width (distance between left and right wheels, center to center)
const float TRACK_WIDTH = 12.5;

// Wheelbase (distance between front and back wheels, center to center)
// Only needed if using external tracking wheels
const float WHEELBASE = 12.5;

// Horizontal tracking wheel offset (distance from center of robot)
// Positive = right of center, Negative = left of center
// Only needed if using back/horizontal tracking wheel
const float HORIZONTAL_TRACKING_WHEEL_OFFSET = 0.0;

// Tracking wheel diameter (if using tracking wheels)
const float TRACKING_WHEEL_DIAMETER = 2.75;

// ========================================
// PID CONSTANTS
// ========================================
// Lateral PID (forward/backward movement)
// Start with these values and tune as needed
const float LATERAL_KP = 10.0;      // Proportional gain
const float LATERAL_KI = 0.0;       // Integral gain
const float LATERAL_KD = 50.0;      // Derivative gain
const float LATERAL_SMALL_ERROR = 2.0;         // Error threshold for small error (inches)
const float LATERAL_SMALL_ERROR_TIMEOUT = 200; // Timeout for small error (ms)
const float LATERAL_LARGE_ERROR = 10.0;        // Error threshold for large error (inches)
const float LATERAL_LARGE_ERROR_TIMEOUT = 500; // Timeout for large error (ms)

// Angular PID (turning)
const float ANGULAR_KP = 4.0;
const float ANGULAR_KI = 0.0;
const float ANGULAR_KD = 40.0;
const float ANGULAR_SMALL_ERROR = 2.0;         // degrees
const float ANGULAR_SMALL_ERROR_TIMEOUT = 200; // ms
const float ANGULAR_LARGE_ERROR = 10.0;        // degrees
const float ANGULAR_LARGE_ERROR_TIMEOUT = 500; // ms

// ========================================
// DRIVE CURVE CONSTANTS
// ========================================
// Slew rate (maximum change in motor power per 10ms)
// Lower = smoother acceleration, Higher = faster acceleration
// Range: 0-127 (typically 3-10)
const float SLEW_RATE = 5.0;

// Drive curve gain (exponential scaling for joystick input)
// 0 = linear, higher = more exponential (more precise at low speeds)
// Typical range: 0.0-2.0
const float DRIVE_CURVE_GAIN = 1.0;

// Deadzone for joystick (ignore small inputs below this threshold)
// Range: 0-127
const int JOYSTICK_DEADZONE = 5;

// ========================================
// AUTONOMOUS MOVEMENT CONSTANTS
// ========================================
// Maximum speeds for autonomous movement
const int MAX_LINEAR_SPEED = 127;  // Maximum speed for linear movement (0-127)
const int MAX_TURN_SPEED = 90;     // Maximum speed for turning (0-127)

// Timeout for autonomous movements (milliseconds)
const int DEFAULT_TIMEOUT = 3000;  // 3 seconds default timeout

#endif // ROBOT_CONFIG_HPP
