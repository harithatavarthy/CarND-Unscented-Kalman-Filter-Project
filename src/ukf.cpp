#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
    
  std_a_ = 0.75;

  // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.6;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
    
    
    
    ///* State dimension
    n_x_ = 5;
    
    ///* Augmented state dimension
    n_aug_ = 7;
    
    ///* Sigma point spreading parameter
    lambda_ = 3 - n_x_;
    
    ///* predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_ , 2 * n_aug_ + 1);
    
    ///* time when the state is true, in us
    time_us_ = 0.00;
    
    
    ///* initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;
    
    ///* Initialize sigma point matrix
    Xsig = MatrixXd(n_x_ , 2 * n_x_ + 1);
    
    ///* Weights of sigma points
    weights_ = VectorXd(2 * n_aug_ + 1);
    
    R_lidar_ = Eigen::MatrixXd(2,2);
    R_lidar_ << std_laspx_*std_laspx_,0,
                0,std_laspy_*std_laspy_;

    
    R_radar_ = Eigen::MatrixXd(3,3);
    
    R_radar_ << std_radr_*std_radr_, 0, 0,
                0, std_radphi_*std_radphi_, 0,
                0, 0,std_radrd_*std_radrd_;
    
    
    }

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
     //std::cout << "Process Funtcion..." << std::endl;
    
    if(((meas_package.sensor_type_ == MeasurementPackage::RADAR ) ||
        (meas_package.sensor_type_ == MeasurementPackage::LASER )) && (!is_initialized_))
    {
        // Initialize state vector and state covariance matrix
        //std::cout << "Initialization Done....." << std::endl;
        x_ << 0,0,0,0,0;
        P_ <<  0.15,0,0,0,0,
                0,0.15,0,0,0,
                0,0,1,0,0,
                0,0,0,1,0,
                0,0,0,0,1;
        
        //Initialize the timestamp variable with the timestamp taken from the first measurement.
        time_us_ = meas_package.timestamp_;
        
        // Initialize the X and Y positions of the state vector using the values from the first measurement.
        
        if(meas_package.sensor_type_ == MeasurementPackage::LASER )
        {
            x_(0) = meas_package.raw_measurements_(0);
            x_(1) = meas_package.raw_measurements_(1);
        }
        
        else if(meas_package.sensor_type_ == MeasurementPackage::RADAR )
        {
            // Convert polar coordinates to cartesian
            float rho =  meas_package.raw_measurements_(0);
            float phi = meas_package.raw_measurements_(1);
            //float rhodot = meas_package.raw_measurements_(2);
            
            x_(0) = rho * cos(phi);
            x_(1) = rho * sin(phi);
            //float vx  = rhodot * cos(phi);
            //float vy = rhodot * sin(phi);
            //float v  = sqrt(vx*vx + vy*vy);
            //x_(2) = v;
        }
        
        /* set the bool variable is_initialized_ to TRUE as we dont need to initialize the state vector and state covariance matrix again. */
        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }
    
    /************ PREDICTION ********************/
    /* Compute the difference in timestamp - required to pass as parameter to prediction process */
    float dt  = (meas_package.timestamp_ - time_us_) / 1000000.00 ;
    /* save the timestamp from the measurement for later use */
    time_us_ = meas_package.timestamp_;
    //std::cout << "px      : " << x_(0) << std::endl;
    //std::cout << "py      : " << x_(1) << std::endl;
    //std::cout << "v       : " << sqrt(x_(0)*x_(0) + x_(1)*x_(1)) << std::endl;
    //std::cout << "yaw     : " << meas_package.raw_measurements_(3) << std::endl;
    //std::cout << "yawrate : " << meas_package.raw_measurements_(4) << std::endl;

    
    Prediction(dt);
    
    /************* MEASUREMENT  UPDATE ****************/
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        UpdateLidar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        UpdateRadar(meas_package);
    }
    
    return;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
    //std::cout << "Inside Predicton ...." << std::endl;
    /**************************************************************************
    // First we will generate sigma points for the current state distribution.
    *************************************************************************/
    //std::cout << "Generate Sigma Points ...." << std::endl;
    Xsig.fill(0.0);
    
    // Calculate square root of state covariance matrix P_
    
    Eigen::MatrixXd A  = P_.llt().matrixL();
    
    //calculate sigma points ...
    //set sigma points as columns of matrix Xsig
    // Set First column of Xsig with x
    
    Xsig.col(0) = x_;
    
    // Set rest of the 10 columns of Xsig.
    
    for(int index = 0 ; index < n_x_ ; index++)
    {
        Xsig.col(index + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(index);
        Xsig.col(index + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(index);
    }
    
    /**************************************************************************
    // Next we will augment the state vector and covariance matrixes with process noise
    **************************************************************************/
    //std::cout << "Augmentation ...." << std::endl;
    //create augmented mean vector
    Eigen::VectorXd x_aug_ = Eigen::VectorXd(n_aug_);
    
    //create augmented state covariance
    Eigen::MatrixXd P_aug_ = Eigen::MatrixXd(n_aug_ , n_aug_);
    
    //create sigma point matrix
    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_ , 2 * n_aug_ + 1);
    
    //set lambda for augmented sigma points
    lambda_ = 3 - n_aug_;
    
    //set augmented mean state
    x_aug_.head(5) = x_;
    x_aug_(5) = 0;
    x_aug_(6) = 0;
    
    //set augmented covariance matrix
    P_aug_.fill(0.0);
    P_aug_.topLeftCorner(5, 5) = P_;
    P_aug_(5,5) = std_a_ * std_a_ ;
    P_aug_(6,6) = std_yawdd_ * std_yawdd_;
    
    //create square root matrix
    Eigen::MatrixXd L = P_aug_.llt().matrixL();
    
    //create augmented sigma points
    Xsig_aug.col(0) = x_aug_;
    for (int index  = 0 ; index < n_aug_ ; index++)
    {
        Xsig_aug.col(index + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(index);
        Xsig_aug.col(index + n_aug_ + 1) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(index);
    }
    
    /**************************************************************************
     // Next we will predict sigma points
    **************************************************************************/
    //std::cout << "Predict Sigma Points ...." << std::endl;
    for (int i = 0; i< 2* n_aug_ +1; i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;
        
        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;
        
        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }

    
    /*****************************************************************************
    //  Convert Predicted Sigma Points to Mean/Covariance
    ******************************************************************************/
    //std::cout << "Converted predicted Sigma points to Mean/Covariance  ...." << std::endl;
    // set weights
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    
    weights_.fill(0);
    weights_(0) = weight_0;
    

    for (int i = 1; i < 2 * n_aug_ + 1; i++)
    {  //2n+1 weights
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
        
    }
    
    //predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {  //iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }
    
    //predicted state covariance matrix
    
    P_.fill(0.0);
    //std::cout << Xsig_pred_ << std::endl;
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {  //iterate over sigma points
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //std::cout << "x_diff...." << std::endl;
        //std::cout << x_diff << std::endl;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;
        
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
        //std::cout << P_ << std::endl;
        //std::cout << "BREAK" << std::endl;
    }
    
    //std::cout << "Predicted State" <<std::endl;
    //std::cout << x_(0) << std::endl;
    //std::cout << x_(1) << std::endl;
    
    
    
    
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
    //std::cout << "Update LIDAR ...." << std::endl;
    //set measurement dimension, lidar can measure p_x and p_y
    int n_z = 2;
    
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        // extract values for better readibility
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        
        // measurement model
        Zsig(0, i) = p_x;
        Zsig(1, i) = p_y;
    }
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    //MatrixXd R = MatrixXd(n_z, n_z);
    //R << std_laspx_*std_laspx_, 0,
    //0, std_laspy_*std_laspy_;
    //S = S + R;
    S = S + R_lidar_;
    
    //create a vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_(0),
    meas_package.raw_measurements_(1);
    
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    
    /*****************************************************************************
     *  UKF Update for Lidar
     ****************************************************************************/
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    //calculate NIS
    NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
    //std::cout<<"NIS LASER"<<std::endl;
    std::cout<<NIS_laser_<<std::endl;
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
    //std::cout << "updated State - LASER" <<std::endl;
    //std::cout << x_(0) << std::endl;
    //std::cout << x_(1) << std::endl;
    

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
    //std::cout << "Update RADAR ...." << std::endl;
    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;
    
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);
        
        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;
        
        // measurement model
        Zsig(0,i) = sqrt(p_x * p_x + p_y * p_y);                          //r
        Zsig(1,i) = atan2(p_y,p_x);                                       //phi
        Zsig(2,i) = (p_x * v1 + p_y*v2 ) / sqrt(p_x * p_x + p_y * p_y);   //r_dot
    }
    
    //mean predicted measurement
    
    z_pred.fill(0.0);
    for (int i=0; i < 2 * n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    //MatrixXd R = MatrixXd(n_z,n_z);
    //R <<    std_radr_ * std_radr_, 0, 0,
    //0, std_radphi_ * std_radphi_, 0,
    //0, 0,std_radrd_ * std_radrd_;
    //S = S + R;
    S = S + R_radar_;
    
    //create a vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_(0),
        meas_package.raw_measurements_(1),
        meas_package.raw_measurements_(2);
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    //calculate NIS
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
    //std::cout<<"NIS RADAR"<<std::endl;
    //std::cout<<NIS_radar_<<std::endl;
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
    //std::cout << "updated State - RADAR" <<std::endl;
    //std::cout << x_(0) << std::endl;
    //std::cout << x_(1) << std::endl;
    
}
