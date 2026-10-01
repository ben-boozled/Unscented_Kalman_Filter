#include "Unscented_Kalman_Filter/UKF.h"

UKF::UKF(const VectorXd& x0, const MatrixXd& P0) : x(x0), P(P0) {
    n_x = x0.size();
    n_aug = x0.size() + 2; // +2 for linear accel and steering rate noise
    n_sig = 2*n_aug + 1;
    kappa = 3 - n_aug; // default formula, secondary spread parameter
    lambda = alpha*alpha*(n_aug + kappa) - n_aug;
    weights_m = VectorXd::Zero(n_sig);
    weights_c = VectorXd::Zero(n_sig);
}

double UKF::normaliseAngle(double theta){
    while (theta > M_PI){theta -= 2*M_PI;}
    while (theta < -M_PI){theta += 2*M_PI;}
    return theta;
}

double UKF::clampAngle(double theta, double limit){
    if(theta >= limit){return limit;}
    if(theta <= -limit){return -limit;} 
    return theta;
}

double UKF::yawRateFromState(double v, double delta) const {
    double d = clampAngle(delta, delta_max);
    return (v/L) * std::tan(d);
}

void UKF::computeWeights(){
    weights_m[0] = lambda/(n_aug + lambda);
    weights_c[0] = weights_m[0] + 1 - alpha*alpha + beta;
    for(int i=1; i<n_sig; i++){
        double w = 1 / (2*(n_aug +lambda));
        weights_m[i] = w;
        weights_c[i] = w;
    }
}

void UKF::generateAugmentedSigmaPoints(){
    computeWeights();

    VectorXd x_aug = VectorXd::Zero(n_aug);
    for(int i=0; i<n_x; i++){x_aug(i) = x(i);} // x_aug(5) and x_aug(6) are Gaussian noise with mean 0

    MatrixXd P_aug = MatrixXd::Zero(n_aug,n_aug);
    for(int i=0; i<n_x; i++){
        for(int j=0; j<n_x; j++){
            P_aug(i, j) = P(i, j);
        };
    } 
    P_aug(n_x, n_x) = var_linear; // covariances of noise terms are 0
    P_aug(n_x+1, n_x+1) = var_steering; // P_aug = [[P, 0], [0, Q]]

    Eigen::LLT <MatrixXd> lltOf_P_aug(P_aug); // Cholesky decomposition to find the "square root" of the matrix, P_aug = L*L.transpose()
    MatrixXd L = lltOf_P_aug.matrixL();       // P_aug matrix must be positive definite, lower-triangular returned
                                        
    double scale = std::sqrt(n_aug + lambda);

    // Every row contains the sigma points of the state in the first col
    Xsig_aug = MatrixXd::Zero(n_aug, n_sig);
    Xsig_aug.col(0) = x_aug; // 1st column of the sigma matrix contains the state means
    for(int j=0; j<n_aug; j++){ 
        VectorXd plus(n_aug), minus(n_aug); // the the weighted mean of the plus/minus pairs cancel each other to tend to x_aug
        for(int i=0; i<n_aug; i++){
            double d = L(i, j)*scale;       // i-th row, j-th col of the Cholesky factor means "how much dimension i moves along the spread-direction j"  
            plus(i) = x_aug(i) + d;         // sigma points from 1 to n
            minus(i) = x_aug(i) - d;        // sigma points from n+1 to 2n
        }
        Xsig_aug.col(j+1) = plus;
        Xsig_aug.col(j+1+n_aug) = minus;
    }
}

void UKF::predictSigmaPoints(double dt){
    Xsig_pred = MatrixXd::Zero(n_aug, n_sig);
    
    for(int i=0; i<n_sig; i++){
        // Extract sigma points corresponding to the current state means for predictions
        double px              = Xsig_aug(0,i);
        double py              = Xsig_aug(1,i); 
        double v               = Xsig_aug(2,i);
        double yaw             = Xsig_aug(3,i);
        double delta           = Xsig_aug(4,i);  
        double n_linear_accel  = Xsig_aug(5,i);             
        double n_steering_rate = Xsig_aug(6,i);
        
        double delta_clamped = clampAngle(delta, delta_max);
        double yaw_rate = (v/L)*std::tan(delta_clamped); // rad/s, based on the bicycle steering model
        
        double px_pred, py_pred;
        if(std::fabs(yaw_rate) > 1e-3){
            // Car is turning
            // Calculate predicted positions based on the arc formed
            // Assume constant v and yaw_rate over dt
            px_pred = px + (v/yaw_rate) * (std::sin(yaw + yaw_rate*dt) - std::sin(yaw));
            py_pred = py + (v/yaw_rate) * (std::cos(yaw) - std::cos(yaw + yaw_rate*dt));
        } else {
            // Car is moving in a straight line, handle here to avoid dividing by 0 in the if-block
            px_pred = px + v*std::cos(yaw)*dt;
            py_pred = py + v*std::sin(yaw)*dt;
        }

        double v_pred = v; // constant velocity model
        double delta_pred = delta; // constant steering angle for constant yaw rate assumption to be valid
                                   // max steering angle rate in dt=1/60 s (= (delta_max-delta_min)*dt) 
                                   // = approx 0.021rad/dt (can be approximated as 0). This inherent uncertainty
                                   // in delta is accounted for in the noise term n_steering_rate

        double yaw_pred = yaw + yaw_rate*dt;

        /* Add process noise */
        px_pred += 0.5*dt*dt * n_linear_accel * std::cos(yaw);
        py_pred += 0.5*dt*dt * n_linear_accel * std::sin(yaw);
        v_pred += dt * n_linear_accel;

        delta_pred = clampAngle(delta_pred + dt*n_steering_rate, delta_max);

        // Since psi_dot = (v/L)*tan(delta), the disturbance of the noise (n_steering_rate)
        // must go through chain rule --> d(psi_dot)/d(delta) = (v/L)*sec^2(delta)
        // Therefore, an unknown n_steering_rate affects yaw in an ACCELERATION-like manner
        double cos_delta = std::cos(delta_clamped);
        double sec2 = 1 / (cos_delta*cos_delta);
        yaw_pred += 0.5*dt*dt * (v/L) * sec2 * n_steering_rate;

        // Update predictions for all except noise terms
        Xsig_pred(0,i) = px_pred;
        Xsig_pred(1,i) = py_pred;
        Xsig_pred(2,i) = v_pred;
        Xsig_pred(3,i) = yaw_pred;
        Xsig_pred(4,i) = delta_pred; 
    }
}

void UKF::predictMeanAndCovariance(){
    VectorXd x_new = VectorXd::Zero(n_x);
    for(int i=0; i<n_sig; i++){
        for(int r=0; r<n_x; r++){
            x_new(r) += weights_m(i) * Xsig_pred(r, i);
        }
    }
    x_new(3) = normaliseAngle(x_new(3)); // normalise the heading

    MatrixXd P_new = MatrixXd::Zero(n_x, n_x);
    for(int i=0; i<n_sig; i++){
        VectorXd diff = VectorXd::Zero(n_x);
        for(int r=0; r<n_x; r++){
            diff(r) = Xsig_pred(r, i) - x_new(r);
        }
        diff(3) = normaliseAngle(diff(3));
        for(int r=0; r<n_x; r++){
            for(int c=0; c<n_x; c++){
                P_new(r, c) += weights_c(i) * diff(r) * diff(c);
            }
        }
    }

    x = x_new;
    P = P_new;
}

void UKF::predict(double dt){
    generateAugmentedSigmaPoints();
    predictSigmaPoints(dt);
    predictMeanAndCovariance();
}

void UKF::resyncSigmaPointsWithCurrentState(){
    generateAugmentedSigmaPoints();
    predictSigmaPoints(0);
}

void UKF::update(const VectorXd& z, const std::function<VectorXd (const VectorXd&)>& h,
                 const MatrixXd& R, int angleIndex){
    resyncSigmaPointsWithCurrentState();
    int n_z = z.size();

    // Convert the sigma points of the prior into measurement sigma points
    MatrixXd Zsig = MatrixXd::Zero(n_z, n_sig);
    for(int i=0; i<n_sig; i++){
        VectorXd xi = VectorXd::Zero(n_x); // xi stores the i-th sigma points of the priors
        for(int r=0; r<n_x; r++) {xi(r) = Xsig_pred(r, i);}
        VectorXd zi = h(xi); // zi converts xi into measurements
        for(int r=0; r<n_z; r++) {Zsig(r, i) = zi(r);} // Zsig stores all the zi into one matrix
    }

    // Use the Unscented Transform to compute the mean of the obtained measurement sigma points
    VectorXd z_pred = VectorXd::Zero(n_z);
    for(int i=0; i<n_sig; i++){
        for(int r=0; r<n_z; r++){
            z_pred(r, 0) += weights_m(i) * Zsig(r, i);
        }
    }
    if(angleIndex >= 0){
        z_pred(angleIndex) = normaliseAngle((z_pred(angleIndex)));
    }

    // Use the Unscented Transform to compute the covariance of the obtained measurement sigma points
    MatrixXd Pz = MatrixXd::Zero(n_z, n_z);  // covariance of measurement sigma points
    MatrixXd Pxz = MatrixXd::Zero(n_x, n_z);
    for(int i=0; i<n_sig; i++){
        VectorXd zdiff = VectorXd::Zero(n_z);
        for(int r=0; r<n_z; r++) {zdiff(r) = Zsig(r, i) - z_pred(r);}
        if(angleIndex >= 0) {zdiff(angleIndex) = normaliseAngle(zdiff(angleIndex));}

        VectorXd xdiff = VectorXd::Zero(n_x);
        for(int r=0; r<n_x; r++) {xdiff(r) = Xsig_pred(r, i) - x(r);}
        xdiff(3, 0) = normaliseAngle(xdiff(3, 0)); // normalise the heading term

        // Compute the covariance of measurement sigma points
        for(int r=0; r<n_z; r++){
            for(int c=0; c<n_z; c++) {
                Pz(r, c) += weights_c(i) * zdiff(r) * zdiff(c);
            }
        }

        // Compute the covariance of the state and measurements
        for(int r=0; r<n_x; r++){
            for(int c=0; c<n_z; c++){
                Pxz(r, c) += weights_c(i) * xdiff(r) * zdiff(c);
            }
        }
    }
    Pz = Pz + R;    // add measurement noise covariance

    MatrixXd K = Pxz*Pz.inverse();  // Kalman gain

    VectorXd y = VectorXd::Zero(n_z);   // store the residual
    for(int r=0; r<n_z; r++){y(r) = z(r) - z_pred(r);}
    if(angleIndex >= 0){y(angleIndex) = normaliseAngle(y(angleIndex));}

    // Compute the new state
    MatrixXd dx = K * y;
    for(int r=0; r<n_x; r++){x(r) += dx(r, 0);}
    x(3) = normaliseAngle(x(3));    // normalise heading
    x(4) = clampAngle(x(4), delta_max);        // clamp steering angle

    // Compute the new covariance
    P = P - K*Pz*K.transpose();
}

void UKF::updateGPS(double pos_x_meas, double pos_y_meas){
    VectorXd z {{pos_x_meas, pos_y_meas}};
    MatrixXd R = MatrixXd::Zero(2, 2);     // measurement noise matrix
    R(0, 0) = var_gps_x;
    R(1, 1) = var_gps_y; 

    // Lambda expression to form a vector of pos x and pos y from the state vector
    update(z, [](const VectorXd& x) -> VectorXd {
        VectorXd out {{x(0), x(1)}};    // Vector of px and py
        return out;
    }, R, -1);
}

void UKF::updateGyro(double yaw_rate_meas){
    VectorXd z {{yaw_rate_meas}};
    MatrixXd R {{var_gyro}};

    update(z, [this](const VectorXd& x) -> VectorXd{
        VectorXd out {{yawRateFromState(x(2), x(4))}};
        return out;
    }, R, -1);
}

void UKF::updateAccel(double accel_meas){
    VectorXd z {{accel_meas}};
    MatrixXd R {{var_accel}};

    // In the body frame, the car moves along an arc -> centripetal acceleration
    // = v^2/R = v * yaw rate
    // Also, longitudinal acceleration cannot be measured from the state vector
    // (the only longitudinal acceleration present is the acceleration noise term)
    update(z, [this](const VectorXd& x) -> VectorXd{
        double v = x(2);
        VectorXd out {{v * yawRateFromState(v, x(4))}};
        return out;
    }, R, -1);
}