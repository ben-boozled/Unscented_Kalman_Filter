# Description
The Unscented Kalman Filter (UKF) is a useful algorithm for estimating the true state of a **non-linear discrete-time** system. First, the UKF represents the probability distribution of the current system state by strategically selecting a set of sampling points (called "sigma points") located at the mean and along the covariance of the distribution. Each sigma point then gets propagated through the non-linear function of interest, forming a new warped distribution. 

![Sigma Point Transformations](https://github.com/ben-boozled/Unscented_Kalman_Filter/blob/main/Images/Sigma_Point_Transform.png)

Finally, the UKF computes a weighted combination of these transformed sigma points to calculate the new mean and covariance of the warped distribution.

## Forming the State Vector
For this project, the **base state vector** is:

$$
\begin{aligned}
x = [px, py, v, {\psi}, δ]
\end{aligned}
$$

* $$px$$ and $$py$$ are the x and y coordinates of the car in m.
* $$v$$ is the velocity of the car in m/s.
* $${\psi}$$ is the yaw in rad. Note that heading is measured **from the x-axis** (i.e. $${\psi} = 0$$ from the positive x-axis, and increases counterclockwise). 
* $$δ$$ is the front steering angle in rad, relative to the car body (positive means steering left, negative means steering right).

This base state vector is augmented by appending two noise terms to its end. The **augmented state vector** is: 

$$
\begin{aligned}
x\\_aug = [px, py, v, {\psi}, δ, n\\_a, n\\_steerd]
\end{aligned}
$$

* $$n\\_a$$ is the longitudinal acceleration noise in m/s<sup>2</sup>. 
* $$n\\_steerd$$ is the steering rate noise in rad/s.

Appending the two noise terms to the base state vector forces the noise terms through the non-linear process model. This better captures the non-linear effects of the process noise on the state variables. For instance, as covered in the `predictSigmaPoints()` function of `UKF.cpp`, the acceleration effect of $$n\\_a$$ also gets multiplied by $$cos({\psi})$$ or $$sin({\psi})$$ depending on whether it affects $$px$$ or $$py$$.

Hence, this augmented state vector will be used as the UKF's state vector.  

# Kinematic Bicycle Model
Next, let's discuss the kinematic bicycle model / bicycle steering model since the Kalman Filter relies on having a process model to run its prediction step. Assuming there is no wheel slip or skid, a car would generally be steered in a manner where its front wheels pivot around its rear wheels. This motion can be represented by a two-axle single-track model (i.e. a bicycle), as shown below:

![Bicycle steering model](https://github.com/ben-boozled/Unscented_Kalman_Filter/blob/main/Images/Bicycle%20Steering%20Model.png)

From the above,

$$
\begin{aligned}
R = \frac{L}{tan(δ)}
\end{aligned}
$$

A body rotating about the ICR at longitudinal velocity $$v$$ has the yaw rate:

$$
\begin{aligned}
\dot{\psi} = \frac{v}{R} = \frac{v⋅tan(δ)}{L}
\end{aligned}
$$

Several assumptions are also made about the model:
* The car is moving at constant longitudinal velocity (parallel to the car's body), $$v$$.
* The car has constant turn velocity, $$\dot{\psi}$$.
* The steering angle $$δ$$, is sent as a command signal.

With the above relationship and assumptions, the following equations are used to predict the state variables at the next time step:  
* For the new heading:

$$
\begin{aligned}
{\psi}_{predict} = {\psi}_{current} + \dot{\psi}⋅dt
\end{aligned}
$$

* For $$px$$ and $$py$$, if the car is moving in a straight line (i.e. $$\dot{\psi} = 0$$):

$$
\begin{aligned}
px_{predict} = px_{current} + v⋅cos({\psi})⋅dt\\
py_{predict} = py_{current} + v⋅sin({\psi})⋅dt
\end{aligned}
$$

* Else, the car is turning (i.e. $$\dot{\psi} \gt 0$$):

$$
\begin{aligned}
px_{predict} = px_{current} + \int_{0}^{dt} v⋅cos({\psi}) \\,dt =  px_{current} + \frac{v}{\dot{\psi}} (sin({\psi}_{predict}) - sin({\psi}_{current}))\\
py_{predict} = py_{current} + \int_{0}^{dt} v⋅sin({\psi}) \\,dt = py_{current} + \frac{v}{\dot{\psi}} (cos({\psi}_{current}) - cos({\psi}_{predict}))
\end{aligned}
$$

* For the steering angle, it is assumed to be constant within the same time step while the car is turning:

$$
\begin{aligned}
δ_{predict} = δ_{current}
\end{aligned}
$$

> [!NOTE]
> The state variables $$n\\_a$$ and $$n\\_steerd$$ are noise terms modelled as a standard Gaussian distribution. As such, they have a constant mean of 0 and standard deviation of 1 regardless of the time step.

# UKF Algorithm
The UKF involves the following steps: 
1. **Initialise** our belief in the system state.
2. **Predict** the system state at the next time step.
3. **Update** the state with sensor measurements (when they are available).
4. **Repeat** steps 2 and 3 iteratively.

Depending on the sampling rate, multiple predict steps may occur before an update step. An update step may also incorporate measurements from multiple sensors at the same time. The predict and update steps of the UKF are a little more complicated than the linear Kalman Filter. The next section explains them in greater detail.

## Predict Step
**1a. From the most recent state estimate, generate the sigma points, $$X$$.**  
Generate $$2N+1$$ sigma points, where $$N$$ is the dimension of the state vector. Since the augmented state vector has 7 variables, 15 sigma points are required. The first sigma point is the mean:
       
$$
\begin{aligned}
{\chi}_{0} = {\mu}
\end{aligned}
$$
     
The remaining sigma points are computed as:

$$
\begin{aligned}
{\chi}_{i} = {\mu} + [\sqrt{(\lambda + n)·P}]_{i} \quad i=1..n\\
{\chi}_{i} = {\mu} - [\sqrt{(\lambda + n)·P}]_{i-n} \quad i=(n+1)..2n\\
\lambda = \alpha^{2}·(n + \kappa) - n\\
\kappa = 3 - n
\end{aligned}
$$

* $$P$$ is the error covariance matrix of the augmented state vector. The Cholesky decomposition is used to obtain the "square root", $$L$$, of this matrix where $$L·L^{T}=P$$.
    * $$[\sqrt{P}]\_{i}=L_{i}$$ is the i-th column of the Cholesky decomposition. This column has the same number of dimensions as the augmented state vector (i.e. 7) and serves as a "direction vector" along which to choose a new sigma point. 
* $$\alpha$$ is a tuning parameter that determines the spread of the sigma points around the mean state value. It is usually in the range of $$0 \leq \alpha \leq 1$$.
* $$\kappa$$ is another scaling parameter for the sigma points. It is usually set to $$0$$ or $$3-n$$ where $$n$$ is the dimension of the state vector.
  
> [!NOTE]
> Each sigma point has values for all variables in the state vector simultaneously. This means that for this project, each of the 15 sigma points has 7 dimensions (e.g. $${\chi}_{0}$$ is represented collectively by all 7 means of the augmented state vector).
  
**1b. Generate the corresponding weights for the sigma points' mean and covariance, $$w^{m}$$ and $$w^{c}$$.**  
The weight for the mean of $${\chi}_{0}$$ is:

$$
\begin{aligned}
w_{0}^{m} = \frac{\lambda}{\lambda + n}
\end{aligned}
$$

The weight for the covariance of $${\chi}_{0}$$ is:

$$
\begin{aligned}
w_{0}^{c} = \frac{\lambda}{\lambda + n} + 1 - \alpha^{2} + \beta
\end{aligned}
$$

* $$\beta$$ is a tuning parameter. $$\beta = 2$$ is generally optimal for Gaussian problems.

The weights for the remaining sigma points are the same for both the mean and covariance. These weights can be calculated as follows:

$$
\begin{aligned}
w_{i}^{m} = w_{i}^{c} = \frac{1}{2(n + \lambda)} \quad i=1..2n
\end{aligned}
$$

**1c. Propagate each sigma point through the process model equations. The result will be the predicted sigma points at the next time step.**  
Each sigma point will be passed through the equations explained in the **Kinematic Bicycle Model** section above. The symbol $$\mathcal{Y}$$ is used to represent the transformed sigma points.  

**1d. Using the sigma point weights, recombine the predicted sigma points from step 1c to obtain the prior belief (the predicted mean and error covariance).**  

$$
\begin{aligned}
\bar{x} = \sum_{i=0}^{2n}w_{i}^{m}·\mathcal{Y}_{i}\\
\bar{P} = \sum_{i=0}^{2n}w_{i}^{c}(\mathcal{Y}_{i} - \bar{x})(\mathcal{Y}_{i} - \bar{x})^{T}
\end{aligned}
$$

The bars above $$\bar{x}$$ and $$\bar{P}$$ indicate that these are estimates of the state before incorporating any measurements (i.e. the prior belief).

## Update Step  
**2a. Map the sigma points of the prior belief into the measurement space using a self-defined measurement function h(x) for each sensor.**  
The symbol $$\mathcal{Z}$$ is used to represent the sigma points in the measurement space. 
* GPS (measures position): $$h(x) = [px, py]$$.
* Gyroscope (measures yaw rate): $$h(x) = \frac{v⋅tan(δ)}{L}$$.
* Accelerometer (measures acceleration): $$h(x) = \frac{v^{2}⋅tan(δ)}{L}$$.

> [!NOTE]
> For the accelerometer, $$h(x)$$ computes the centripetal/lateral acceleration from the sigma points because:
> - Centripetal acceleration is responsible for the car's curved movements and
> - It is the only acceleration component we can compute from the existing state vector; longitudinal/tangential acceleration is treated as an unknown in the state vector, under the $$n\\_a$$ term.

**2b. Recombine the measurement sigma points from step 2a to obtain their mean and covariance.**  

$$
\begin{aligned}
\mu_{z} = \sum_{i=0}^{2n}w_{i}^{m}·\mathcal{Z}_{i}\\
P_{z} = \sum_{i=0}^{2n}w_{i}^{c}(\mathcal{Z}_{i} - \mu_{z})(\mathcal{Z}_{i} - \mu_{z})^{T}
\end{aligned}
$$

* $$\mu_{z}$$ is the state vector of the measurement sigma points.
* $$P_{z}$$ is the error covariance matrix of the measurement sigma points.

**2c. Compute the residual.**  

$$
\begin{aligned}
y = z - \mu_{z}
\end{aligned}
$$

* $$z$$ is the measurement vector obtained from the sensor.

**2d. Compute the Kalman gain.**   

$$
\begin{aligned}
P_{xz} = \sum_{i=0}^{2n}w_{i}^{c}(\mathcal{Y}_{i} - \bar{x})(\mathcal{Z}_{i} - \mu_{z})^{T} \\
K = P_{xz}·P_{z}^{-1}
\end{aligned}
$$

* $$P_{xz}$$ is the cross covariance matrix of the state and the measurements.
  
**2e. Compute the new estimate for the state vector and error covariance matrix.**    

$$
\begin{aligned}
x = \bar{x} + Ky \\
P = \bar{P} - K·P_{z}·K^{T}
\end{aligned}
$$
