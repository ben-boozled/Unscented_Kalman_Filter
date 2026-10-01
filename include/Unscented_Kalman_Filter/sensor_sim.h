#pragma once

#include <iostream>
#include <random>
#include "Unscented_Kalman_Filter/UKF.h"
#include "Unscented_Kalman_Filter/car_sim.h"

/** ====================================================================
 *                          Sensor Class Definitions 
 * - Base class for the various simulated sensors.
 * - The sensor fusion loop does not need to know the sampling rates of the
 *   different sensors. It calls the tick() function on all sensors at every
 *   simulation time step. Each sensor will independently decide whether it
 *   is its time to "fire" at a given time step.
 * ===================================================================== **/
class Sensor{
    public:
        // using explicit to prevent implicit compiler type conversions
        explicit Sensor(double rate_hz) : period(1.0 / rate_hz) {} 
        virtual ~Sensor() = default;

        /**
         * @brief Advances the sensor's internal clock by dt seconds. Uses a running
         *        accumulator to determine when the sensor should fire. Once enough
         *        time has elapsed, this function generates a noisy sensor measurement
         *        from the car's true state and applies it to the Unscented Kalman
         *        Filter via the sensor's own measurement model.
         * 
         * @param dt Time step, s
         * @param Car CarSimulator object
         * @param rng Pseudorandom number generator to serve as the seed for the
         *        standard Gaussian distribution that mimics noise effects.
         * @param ukf Unscented Kalman Filter object
         * 
         * @return false if the sensor does not fire, else it returns true    
        **/
        bool tick(double dt, const CarSimulator& Car, std::mt19937& rng, UKF& ukf){
            accum += dt;
            if(accum < period) {return false;} // not time to fire
            
            // Consumes exactly one period, leftover period accrues to the next firing 
            // so the long run average rate is the same even if dt does not divide
            // period evenly
            accum -= period; 
            applyMeasurement(Car, rng, ukf);
            return true;
        }

        /* Getter functions for private variables */ 
        double rateHz() const {return 1.0/period;}
        virtual const char* name() const = 0;

    protected:
        /**
         * @brief Generates a noisy sensor measurement from the car's true state and 
         *        applies it to the Unscented Kalman Filter via the sensor's own 
         *        measurement model. 
         * 
         * @param Car CarSimulator object
         * @param rng Pseudorandom number generator to serve as the seed for the
         *        standard Gaussian distribution that mimics noise effects.
         * @param ukf Unscented Kalman Filter object
        **/
        virtual void applyMeasurement(const CarSimulator& Car, std::mt19937& rng, 
                                      UKF& ukf) = 0; // pure virtual function

    private:
        double period;         // time between firing, s
        double accum = 0.0;    // accummulated time since last firing, s

};

/** ===============================================================
 *                   GPS Class Definitions 
 * - Measures absolute position [px, py], slow but drift free.
 * ================================================================ **/
class GPSSensor : public Sensor {
    public:
        GPSSensor(double rate_hz, double std_dev) : Sensor(rate_hz), std_dev(std_dev) {}
        const char* name() const override {return "GPS";}

        bool hasReading = false; 
        double last_px = 0.0, last_py = 0.0; // storing the last reading for plotting

    protected:
        void applyMeasurement(const CarSimulator& Car, std::mt19937& rng, UKF& ukf) override {
            std::normal_distribution<double> n1(0.0, 1.0);
            last_px = Car.get_px() + n1(rng)*std_dev;
            last_py = Car.get_py() + n1(rng)*std_dev;
            ukf.updateGPS(last_px, last_py);
            // std::cout << "GPS X: " << last_px  << " | GPS Y: " << last_py << std::endl; // for debugging
            hasReading = true;
        }

    private:
        double std_dev;
};

/** ===============================================================
 *                   Gyroscope Class Definitions 
 * - Measures the yaw (psi) directly, fast but prone to drifting.
 * ================================================================ **/
class GyroSensor : public Sensor {
    public:
        GyroSensor(double rate_hz, double std_dev) : Sensor(rate_hz), std_dev(std_dev) {}    
        const char* name() const override {return "Gyroscope";}

        double lastReading = 0.0;

    protected:
        void applyMeasurement(const CarSimulator& Car, std::mt19937& rng, UKF& ukf) override {
            std::normal_distribution<double> n1(0.0, 1.0);
            lastReading = Car.get_yawRate() + n1(rng)*std_dev;
            ukf.updateGyro(lastReading);
            // std::cout << "Gyro: " << lastReading << std::endl; // for debugging
        }

    private:
        double std_dev;
};

/** ===============================================================
 *                 Accelerometer Class Definitions 
 * - Measures the body-frame lateral acceleration -> centripetal
 *   acceeration, a = v * yaw_rate, for a car travelling a curved
 *   path.
 * ================================================================ **/
class AccelSensor : public Sensor {
    public:
        AccelSensor(double rate_hz, double std_dev) : Sensor(rate_hz), std_dev(std_dev) {}
        const char* name() const override {return "Accelerometer";}

        double lastReading = 0.0;

    protected:
        void applyMeasurement(const CarSimulator& Car, std::mt19937& rng, UKF& ukf) override {
            std::normal_distribution<double> n1(0.0, 1.0);
            double accel_true = Car.get_vActual() * Car.get_yawRate();
            lastReading = accel_true + n1(rng)*std_dev;
            ukf.updateAccel(lastReading);
            // std::cout << "Accel: " << lastReading << std::endl; // for debugging
        }

    private:
        double std_dev;
};