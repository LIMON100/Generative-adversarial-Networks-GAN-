# Extended Kalman Filters

The extended kf is a variant of linear kf that can operate Non-Linear system.
The non-linear system is approximated as linear system using first order approximation a around given state and covariance.

Extended kalman filter is recursive estimator that solves the linear quadratic estimation problem using minimum Mean Squared Error
(RMSE) method.


## Jacobian Matrix
In the context of the Extended Kalman Filter (EKF), the Jacobian matrix is used to linearize the mathematical model of a nonlinear system. The EKF is an extension of the Kalman Filter, a recursive algorithm for state estimation in dynamic systems. It is primarily used when the system dynamics and measurements are nonlinear. To apply the Kalman Filter or EKF to such systems, it is necessary to linearize the equations at each time step. The Jacobian matrix plays a crucial role in this linearization process.

The Jacobian matrix, denoted as "H" for measurement Jacobian and "F" for process Jacobian, is used to approximate the nonlinear dynamics and measurement models as linear functions for the Kalman Filter or EKF. Here's how it works:

Process Model Jacobian (F): In the context of EKF, the process model represents the evolution of the system state over time. The Jacobian matrix "F" is used to linearize the nonlinear process model. Given the state transition equation:

x(k+1) = f(x(k), u(k))

Here, "f" is the nonlinear process model, "x(k)" is the state at time "k," and "u(k)" is the control input at time "k." The Jacobian matrix "F" is the partial derivative of "f" with respect to "x." It approximates how small changes in the state "x" affect the evolution of the system.

Measurement Model Jacobian (H): The measurement model relates the system state to the sensor measurements. The Jacobian matrix "H" is used to linearize the nonlinear measurement model. Given the measurement equation:

z(k) = h(x(k))

Here, "h" is the nonlinear measurement model, "z(k)" is the measurement at time "k," and "x(k)" is the state at time "k." The Jacobian matrix "H" is the partial derivative of "h" with respect to "x." It approximates how small changes in the state "x" affect the predicted measurements.

In the EKF algorithm, the Jacobian matrices "F" and "H" are used to compute the Kalman Gain, which determines how much the predicted state estimate and its uncertainty should be adjusted based on new measurements.
