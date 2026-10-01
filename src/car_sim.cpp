#include "Unscented_Kalman_Filter/car_sim.h"

CarSimulator::CarSimulator(double speed, double wheelbase, double steering_angle_cmd,
                           double straight_time, double turn_time,
                           double max_steering_rate,
                           double true_accel_noise_std,
                           double true_steer_rate_noise_std,
                           double speed_reversion_rate)
    : v_nominal(speed), L(wheelbase), delta_cmd (steering_angle_cmd),
      straight_time(straight_time), turn_time(turn_time), max_steering_rate(max_steering_rate), 
      true_accel_noise_std(true_accel_noise_std), true_steer_rate_noise_std(true_steer_rate_noise_std),
      speed_reversion_rate(speed_reversion_rate){
    px = py = 0.0;
    yaw = 0.0;
    delta = 0.0;
    v_actual = speed;
    phase = 0;          
    phase_clock = 0.0;
}

void CarSimulator::step(double dt, std::mt19937& rng){
    /* Determine the steering angle command*/
    double target;
    switch(phase){
      case 0:  target = 0;          break;    // straight
      case 1:  target = delta_cmd;  break;    // turn left
      case 2:  target = 0;          break;    // straight
      default: target = -delta_cmd; break;    // turn right
    }
    double max_delta_change = max_steering_rate*dt;
    double diff = std::clamp(target - delta, -max_delta_change, max_delta_change);
    delta += diff; // effect of steering angle command on actual steering angle

    /* Generate random noise elements */
    std::normal_distribution<double> n1(0.0, 1.0);
    double accel_noise = n1(rng) * true_accel_noise_std;
    double steer_rate_noise = n1(rng) * true_steer_rate_noise_std;

    /* Ensures actual speed fluctuates around the nominal cruise speed, avoids unbounded variance growth */
    v_actual += (-speed_reversion_rate * (v_actual - v_nominal) + accel_noise) * dt;
    if(v_actual < 0.0) {v_actual = 0.0;}  // this demo does not drive in reverse

    delta += steer_rate_noise * dt; // effect of random dither on steering angle

    double yaw_rate = (v_actual/L) * std::tan(delta); // psi_dot = (v/L)*tan(delta)

    if(std::fabs(yaw_rate) > 1e-6) {
        // Car is turning
        // Calculate new car position based on the arc formed
        double new_px = px + (v_actual/yaw_rate) * (std::sin(yaw + yaw_rate*dt) - std::sin(yaw));
        double new_py = py + (v_actual/yaw_rate) * (std::cos(yaw) - std::cos(yaw + yaw_rate*dt));
        px = new_px;
        py = new_py;
    } else{
        // Car is travelling in a straight line
        px += v_actual * std::cos(yaw) * dt;
        py += v_actual * std::sin(yaw) * dt;
    }
    yaw += yaw_rate*dt;

    phase_clock += dt;
    double duration = (phase == 0 || phase == 2) ? straight_time : turn_time;
    if(phase_clock >= duration){
        // Reset the clock and switch to the next phase
        phase_clock = 0.0;
        phase = (phase + 1) % 4; // Cycles through the phases in the order 0, 1, 2, 3 repeatedly
    }
}