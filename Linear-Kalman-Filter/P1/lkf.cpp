#include "kalmanfilter.h"
#include "utils.h"

constexpr bool INIT_ON_FIRST_PREDICTION = true;
constexpr double INIT_POS_STD = 0;
constexpr double INIT_VEL_STD = 0;
constexpr double ACCEL_STD = 0;
constexpr double GPS_POS_STD = 3.0;

void KalmanFilter::predictionStep(double dt)
{
    // Initial state and covariance is 0
    if (!isInitialised() && INIT_ON_FIRST_PREDICTION)
    {
        VectorXd state = Vector4d::Zero();
        MatrixXd cov = Matrix4d::Zero();

        state<<0, 0, 5.0*cos(M_PI/4), 5.0*sin(M_PI/4);

        const double init_pos_std = INIT_POS_STD;
        const double init_vel_std = INIT_VEL_STD;
        cov(0, 0) = INIT_POS_STD * INIT_POS_STD;
        cov(1, 1) = INIT_POS_STD * INIT_POS_STD;
        cov(2, 2) = init_vel_std * init_vel_std;
        cov(3, 3) = init_vel_std * init_vel_std;

        setState(state);
        setCovariance(cov);
    }

    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Kalman Filter Prediction Step for the system in the  
        MatrixXd F = Matrix4d();
        F << 1,0,dt,0,
             0,1,0,dt,
             0,0,1,0,
             0,0,0,1;

        MatrixXd Q = Matrix2d::Zero();
        Q(0, 0) = (ACCEL_STD * ACCEL_STD);
        Q(1, 1) = (ACCEL_STD * ACCEL_STD);

        MatrixXd L = MatrixXd(4, 2);
        L << (0.5*dt*dt),0,0,(0.5*dt*dt),dt,0,0,dt;

        state = F * state;
        cov = F * cov * F.transpose() + L * Q * L.transpose();

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the GPS Measurements in the 
        // section below.
        // Hint: Assume that the GPS sensor has a 3m (1 sigma) position uncertainty.
        // Hint: You can use the constants: GPS_POS_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE 


        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
    else
    {
        // Implement the State Vector and Covariance Matrix Initialisation in the
        // section below. Make sure you call the setState/setCovariance functions
        // once you have generated the initial conditions.
        // Hint: Assume the state vector has the form [X,Y,VX,VY].
        // Hint: You can use the constants: GPS_POS_STD, INIT_VEL_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE
            VectorXd state = Vector4d::Zero();
            MatrixXd cov = Matrix4d::Zero();


            setState(state);
            setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }        
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,VX,VY]
        double psi = std::atan2(state[3],state[2]);
        double V = std::sqrt(state[2]*state[2] + state[3]*state[3]);
        return VehicleState(state[0],state[1],psi,V);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt){predictionStep(dt);}
void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map){}
void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map){}

