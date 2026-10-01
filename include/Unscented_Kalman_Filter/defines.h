#pragma once

/** ========================================================
 *                       User Inputs
 * ======================================================= **/

// Process noise (tune this based on vibes; vibe check results with NEES)
#define LINEAR_ACCEL_STD        1       // Model linear process noise as linear acceleration m/s^2
#define STEERING_RATE_STD       0.75    // Model angular process noise as steering rate rad/s

// Parameters for Merwe Scaled Sigma Points
#define ALPHA                   1.0     // 0 <= ALPHA <=1, controls spread of sigma points from the mean, independent of dimension
#define BETA                    2.0     // 2 is the optimal result for Gaussian     

// Car geometry / physical limits
#define WHEELBASE               2.5     // distance between front and rear wheel axles, 2.5m typical of passenger car
#define MAX_STEERING_ANGLE      35      // degrees, ~+-30-40 in typical passenger cars with a rack and pinion system
                                    // MAX_STEERING_ANGLE should also be under 90 degrees to avoid asymptotes in the steering model's tan() function

// Car control parameters
#define NOMINAL_CRUISE_SPEED    10      // m/s
#define STEERING_ANGLE_CMD      0.2     // rad
#define STRAIGHT_TIME           3.0     // s
#define TURN_TIME               0.5     // s

// Sensor measurement noise (based on sensor's characteristic standard deviations)
#define GPS_POS_STD             2.0     // 2m range typical  consumer-grade GPS
#define GYRO_STD                0.1     // 0.1 rad/s typical MEMS IMU angular velocity noise
#define ACCEL_STD               0.3     // 0.3 m/s^2 typical MEMS IMU linear velocity noise

// Sensor update rates
#define GPS_RATE                5.0     // Hz, slowest update rate
#define GYRO_RATE               50.0    // Hz
#define ACCEL_RATE              40.0    // Hz