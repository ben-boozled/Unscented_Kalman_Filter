#pragma once

#include <cmath>
#include <functional>
#include <Eigen/Dense>

#include "Unscented_Kalman_Filter/defines.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/** ================================================
 *              UKF Class Definitions 
 * ================================================= **/
class UKF{
    public:
        /**
         * @brief Constructor to initialise UKF.
         * 
         * @param x0 Initial state mean vector
         * @param P0 Initial state covariance
        **/ 
        UKF(const VectorXd& x0, const MatrixXd& P0);

        /**
         * @brief Propagate x and P forward by dt seconds using the bicycle steering model.
         *        Measurement function, h(x) = [px, py]
         * 
         * @param dt Time step in s
        **/
        void predict(double dt);

        /**
         * @brief Uses measurement function *h(x) = [px, py]* to convert the sigma 
         *        points of the prior's state vector into measurements.

         * @param pos_x_meas x position reading from the GPS 
         * @param pos_y_meas y position reading from the GPS 
        **/
        void updateGPS(double pos_x_meas, double pos_y_meas);   

        /**
         * @brief Uses measurement function *h(x) = (v/L)*tan(delta) * to convert the 
         *        sigma points of the prior's state vector into measurements.
         * 
         * @param yaw_rate_meas Yaw angular velocity reading from the gyroscope
        **/
        void updateGyro(double yaw_rate_meas);

        /**
         * @brief Uses measurement function *h(x) = v*(v/L)*tan(delta) * to convert the 
         *        sigma points of the prior's state vector into measurements.
         * 
         * @param accel_meas Acceleration reading from accelerometer
        **/
        void updateAccel(double accel_meas);

        // Getter functions for state mean and covariance
        const VectorXd& getState() const {return x;}
        const MatrixXd& getCovariance() const {return P;}

        /* Vehicle Geometry and Physical Limits */
        double L = WHEELBASE;
        double delta_max = MAX_STEERING_ANGLE * M_PI / 180; // convert to rad

        /* Process noise affecting the constant velocity and constant steering angle 
         * assumption per sec */
        double var_linear = LINEAR_ACCEL_STD*LINEAR_ACCEL_STD;
        double var_steering = STEERING_RATE_STD*STEERING_RATE_STD;

        /* Measurement Noise */ 
        double var_gps_x = GPS_POS_STD*GPS_POS_STD; 
        double var_gps_y = GPS_POS_STD*GPS_POS_STD;
        double var_gyro = GYRO_STD*GYRO_STD;
        double var_accel = ACCEL_STD*ACCEL_STD;

        /* Merwe Scaled Sigma Points Parameters */
        double alpha = ALPHA;
        double beta = BETA;
        double kappa;
        double lambda;

    private:
        /**
         * @brief Map angles to the range [-pi, pi] to correctly calculate residuals
         *        (e.g. a heading of 5 degrees vs 365 degrees should have a difference 
         *         of 10 degrees, not 360 degrees).
         * 
         * @param theta The angle to be normalised in radians.
         * 
         * @return The angle normalised to the range [-pi, pi].
        **/
        static double normaliseAngle(double theta);

        /**
         * @brief Angles that exceed the limit are clamped to the limit.
         * 
         * @param theta The angle to be clamped in radians.
         * @param limit The angle limits in radians.
         * 
         * @return An angle within the range [-limit, limit] in radians.
        **/
        static double clampAngle(double theta, double limit);

        /**
         * @brief Computes the car's yaw rate from its existing state using the
         *        formula: *psi_dot = (v/L)*tan(delta)*.
         * 
         * @param v Linear velocity of car in m/s.
         * @param delta Steering angle of car in radians.
         * 
         * @return Yaw rate in rad/s.
        **/
        double yawRateFromState(double v, double delta) const;

        /**
         * @brief Computes the weights of the sigma mean and covariance.
        **/
        void computeWeights();

        /**
         * @brief Build sigma points to capture the mean and the covariance of the
         *        augmented state.
        **/
        void generateAugmentedSigmaPoints();

        /**
         * @brief Push augmented sigma points through the non-linear bicycle process 
         *        model to obtain the predicted sigma points.
         * 
         * @param dt Time step in s
        **/
        void predictSigmaPoints(double dt);

        /**
         * @brief Performs the Unscented Transform on the transformed (predicted) sigma
         *        points to compute the mean and covariance of the prior.
        **/
        void predictMeanAndCovariance();

        /**
         * @brief Rederives X_sig from the CURRENT x and P (dt=0 so no extra noise is
         *        injected and the output from the bicycle model equals the input). 
         *        Needed because a single time step may comprise multiple sequential 
         *        updates from different sensors. After the first measurement corrects 
         *        x and P, the sigma points used by the next correction must be 
         *        regenerated from the corrected state. Else, Xsig_pred (from before 
         *        any correction) no longer matches x, which means the weighted sigma 
         *        point deviations no longer sum to zero. This can cause P to become
         *        non-positive definite, affecting subsequent uses of Cholesky()
         *        decomposition in generateAugmentedSigmaPoints().
        **/
        void resyncSigmaPointsWithCurrentState();

        /**
         * @brief Generic update function called by every sensor in the update step.
         *        Transforms the *already predicted* sigma into measurements using
         *        h(). Then applies the standard Kalman correction to compute the new
         *        x and P.
         * 
         * @param z Vector containing the measurement means
         * @param h Measurement function to transform the sigma points of the prior
         *        into measurement sigma points
         * @param R Measurement noise covariance matrix
         * @param angleIndex The index of the angle value in Vector z (enter -1 if
         *        there is no angle) 
        **/
        void update(const VectorXd& z, 
                    const std::function<VectorXd (const VectorXd&)>& h,
                    const MatrixXd& R, int angleIndex);
    
        int n_x;                    // dimension of state vector
        int n_aug;                  // dimension of augmented state vector 
        int n_sig;                  // dimension of sigma point weight vectors

        VectorXd x;                 // state mean (n_x by 1)
        MatrixXd P;                 // state covariance (n_x by n_x)
        MatrixXd Xsig_aug;          // augmented sigma points (n_aug by (2*n_aug+1))
        MatrixXd Xsig_pred;         // predicted sigma points (n_x by (2*n_aug+1))
        
        VectorXd weights_m;         // sigma point weights ((2*n_aug+1) by 1) for means
        VectorXd weights_c;         // sigma point weights ((2*n_aug+1) by 1) for covariances
};