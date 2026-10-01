#pragma once

#include <random>
#include <cmath>
#include <algorithm>

/** =================================================================================
 *                       CarSimulator Class Definitions 
 * - Moves in 2D space with movements affected by Gaussian noise.
 * - Serves as the ground truth for the vehicle's actual position in the simulation.
 * - Given commands to drive along a serpentine path (i.e. never circle back on
 *   previous paths) for simulation visual clarity.
 * ================================================================================== **/
class CarSimulator{
    public:
        /**
         * @brief Constructor to initialise the attributes of the car simulator model.
         * 
         * @param speed Nominal cruise speed, m/s. Mean of the true speed.
         * @param wheelbase Distance between front and rear wheel axles, m.
         * @param steering_angle_cmd Front wheel angle during a turn, rad.
         * @param straight_time Duration of each straight segment, s.
         * @param turn_time Duration of each turn segment, s.
         * @param max_steering_rate How fast the steering angle ramps towards its scripted
         *        target, rad/s. Required to prevent the model from instantly snapping from
         *        one target angle to another.
         * @param true_accel_noise_std Std of random accelerations affecting the nominal 
         *        speed, m/s^2.
         * @param true_steer_rate_noise_std Std of random steering rate dither affecting
         *         delta, the nominal steering angle, rad/s. 
         * @param speed_reversion_rate How fast the true speed relaxes back towards the nominal
         *        cruise speed after a disturbance, s^-1.
        **/
        CarSimulator(double speed, double wheelbase, double steering_angle_cmd,
                     double straight_time, double turn_time,
                     double max_steering_rate = 1.3,
                     double true_accel_noise_std = 0.4,
                     double true_steer_rate_noise_std = 0.15,
                     double speed_reversion_rate = 0.5);

        /**
         * @brief Uses the bicycle steering model to advance the true state of the
         *        system by dt secs. Includes effects from random noise to mimic
         *        real-world conditions (e.g. uneven road surfaces, imperfect cruise
         *        control etc.).
         * 
         * @param dt Time step, s.
         * @param rng Pseudorandom number generator to serve as the seed for the
         *        standard Gaussian distribution that mimics noise effects.
        **/
        void step(double dt, std::mt19937& rng);

        /* Getter functions for private variables */
        double get_px() const {return px;}
        double get_py() const {return py;}
        double get_vActual() const {return v_actual;}
        double get_yaw() const {return yaw;}                // radians
        double get_steeringAngle () const {return delta;}   // radians
        double get_wheelbase() const {return L;}

        double get_yawRate() const {return (v_actual/L) * std::tan(delta);} // psi_dot = (v/L)*tan(delta)

    private:
        double v_nominal, L, delta_cmd, straight_time, turn_time, max_steering_rate;
        double true_accel_noise_std, true_steer_rate_noise_std, speed_reversion_rate;
        double px, py, yaw, delta, v_actual;
        int phase; // 0=straight, 1=turn left, 2=straight, 3=turn right
        double phase_clock; // stores the duration of the current movement phase (straight/left turn/right turn)
};