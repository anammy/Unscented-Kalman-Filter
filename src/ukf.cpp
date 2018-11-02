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

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  /**** Parameters to be tuned ****/
  //For dataset1, std_a_ = 1 and std_yawdd_ = 0.1, NIS_radar and NIS_laser is greater than 7.82 and 5.99 7.6% and 9.6% of the timesteps, respectively. RMSE values did not meet criteria.
  //For dataset 1, std_a_ = 1 and std_yawdd_ = 0.3, NIS_radar and NIS_laser is greater than 7.82 and 5.99 3.6% and 2.0% of the timesteps, respectively. RMSE values did meet criteria.  
  //For dataset 1, std_a_ = 1 and std_yawdd_ = 0.2, NIS_radar_ and NIS_laser is greater than 7.82 and 5.99 4.4% and 2.8% of the timesteps, respectively. RMSE values did meet criteria.
  //For dataset 1, std_a_ = 1.5 and std_yawdd_ = 0.2, NIS_radar and NIS_laser is greater than 7.82 and 5.99 4.8% and 3.6% of the timesteps, respectively. RMSE values did not meet criteria. 
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;
  
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
  
  //Weights of sigma points
  weights_ = VectorXd(2*n_aug_ + 1);

  //Initialization flag
  is_initialized_ = false;

  //Normalized Innovation Squared (NIS) for lidar
  NIS_las_= 0.0;
  //Total number of NIS values for lidar
  NIS_Count_las_ = 0.0;
  //Number of NIS values greater than 5.99 (0.05 value from Chi-squared distribution for 2 DOF measurement space) for lidar
  NIS_HighCount_las_ = 0.0;


  //Normalized Innovation Squared (NIS) for radar
  NIS_rad_= 0.0;
  //Total number of NIS values for radar
  NIS_Count_rad_ = 0.0;
  //Number of NIS values greater than 7.82 (0.05 value from Chi-squared distribution for 3 DOF measurement space) for lidar
  NIS_HighCount_rad_ = 0.0;

  //timestamp in us
  time_us_ = 0.0;

  //Prediction sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);
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
  

  if (use_radar_ || use_laser_){
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
    if (!is_initialized_) {
      /**
      TODO:
        * Initialize the state x_ with the first measurement.
        * Create the covariance matrix.
        * Convert radar from polar to cartesian coordinates.
      */
      // first measurement
      cout << "UKF: " << endl;

      //Initial guess for state vector
      //Assume initial yaw rate is 5 deg/s (not a sudden sharp turn)
      //Assume initial yaw is approximately 60 deg (1 rad) for the first dataset and 170 deg (3 rad) for the second dataset
      //Assume small initial velocity 
      x_ << 1.0, 1.0, 1.0, 1.0, 0.09;

      //Initial guess for covariance matrix
      //Assume std deviation of px and py are smaller than 1.0 since both the std deviations for radar and lidar measurements are lesser than 1.0. The first measurement will be initialized using either a lidar or radar measurement.
      P_ << 0.15, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.15, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0;
  
      //Timestamp of first measurement
      time_us_ = meas_package.timestamp_;

      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
        cout << "Initial Measurement: Radar\n"<< endl;

        // rho and phi are the range and angle of the radar measurement
        double rho = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];

        double px = rho*cos(phi);
        double py = rho*sin(phi);

        if(px < 0.0001){px = 0.0001;}
        if(py < 0.0001){py = 0.0001;}
        
        x_(0) = px;
        x_(1) = py;
        cout << x_(0) << "\t" << x_(1) << "\n";
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        /**
        Initialize state.
        */
        cout << "Initial Measurement: Laser.\n"<< endl;
        x_(0) = meas_package.raw_measurements_[0];
        x_(1) = meas_package.raw_measurements_[1];
        cout << x_(0) << "\t" << x_(1) << "\n";
      }

      //Initialize weights
      weights_(0) = lambda_/(lambda_ + n_aug_);
      for (int i=1; i < 2*n_aug_ + 1; i++){
        weights_(i) = 0.5/(lambda_ + n_aug_);
      }

      // done initializing, no need to predict or update
      is_initialized_ = true;

      return;
     }

     //Calculated elapsed time
     double dt = (meas_package.timestamp_ - time_us_)/1000000.0;
     time_us_ = meas_package.timestamp_;
     
     //Calculate predicted mean and covariance
     Prediction(dt);

     //Update prediction with current measurement
     if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
       UpdateRadar(meas_package);
     } 
     else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){   
       UpdateLidar(meas_package);
     }
   }
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

  //create augmented mean state
  //create augmented covariance matrix
  //create square root matrix
  //create augmented sigma points
  
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);

  
  x_aug.head(n_x_) = x_;
  x_aug.tail(2) = VectorXd::Zero(2);
  
  MatrixXd Q = MatrixXd(2, 2);
  Q << std_a_*std_a_, 0,
       0, std_yawdd_*std_yawdd_;
  
  P_aug.fill(0.0);      
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;

  //calculate square root of P_aug
  MatrixXd A_aug = P_aug.llt().matrixL();
  
  Xsig_aug.col(0) = x_aug;
  
  for (int i=0; i < n_aug_; i++){
      Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ + n_aug_)*A_aug.col(i);
      Xsig_aug.col(n_aug_+i+1) = x_aug - sqrt(lambda_ + n_aug_)*A_aug.col(i);
  }
  
  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  
  double yawk=0, vk=0, yawratek=0, noise_ak=0, noise_yawratek=0;
  
  for (int i=0; i < 2 * n_aug_ + 1; i++){
      vk = Xsig_aug(2, i);
      yawk = Xsig_aug(3, i);
      yawratek = Xsig_aug(4, i);
      noise_ak = Xsig_aug(5, i);
      noise_yawratek = Xsig_aug(6, i);
      
      Xsig_pred_.col(i) = Xsig_aug.block(0, i, n_x_, 1);

      
      if (fabs(yawratek) > 1e-3){
          Xsig_pred_(0, i) += (vk/yawratek)*(sin(yawk + yawratek*delta_t) - sin(yawk)) + 0.5*pow(delta_t,2)*cos(yawk)*noise_ak;
          Xsig_pred_(1, i) += (vk/yawratek)*(-cos(yawk + yawratek*delta_t) + cos(yawk)) + 0.5*pow(delta_t,2)*sin(yawk)*noise_ak;
          Xsig_pred_(2, i) += delta_t*noise_ak;
          Xsig_pred_(3, i) += yawratek*delta_t + 0.5*pow(delta_t,2)*noise_yawratek;
          Xsig_pred_(4, i) += delta_t*noise_yawratek;
      }
      else{
          Xsig_pred_(0, i) += vk*cos(yawk)*delta_t + 0.5*pow(delta_t,2)*cos(yawk)*noise_ak;
          Xsig_pred_(1, i) += vk*sin(yawk)*delta_t + 0.5*pow(delta_t,2)*sin(yawk)*noise_ak;
          Xsig_pred_(2, i) += delta_t*noise_ak;
          Xsig_pred_(3, i) += 0.5*pow(delta_t,2)*noise_yawratek;
          Xsig_pred_(4, i) += delta_t*noise_yawratek;
      }
  }


  //predict state mean
  //predict state covariance matrix
  x_.fill(0);
  for (int j=0; j < 2*n_aug_ + 1; j++){
    x_ += weights_(j)*Xsig_pred_.col(j);
  }

  P_.fill(0);
  for (int j=0; j < 2*n_aug_ + 1; j++){
    VectorXd x_diff = Xsig_pred_.col(j) - x_;
    NormAngle(&(x_diff(3)));
    P_ += weights_(j)*x_diff*x_diff.transpose();
  }

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

  //transform sigma points into measurement space

  //set measurement dimension, lidar can measure px and py
  int n_z_las = 2;

  //Matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_las, 2*n_aug_ + 1);
  
  for (int i=0; i < 2*n_aug_ + 1; i++){
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  //create example vector for incoming lidar measurement
  VectorXd z_las = VectorXd(n_z_las);
  z_las << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
  
  //Update State
  UpdateState(meas_package, Zsig, n_z_las, z_las);
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

  //transform sigma points into measurement space

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z_rad = 3;

  //Matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_rad, 2*n_aug_ + 1);
  
  double px, py, v, psi;
  
  for (int i=0; i < 2*n_aug_ + 1; i++){
    px = Xsig_pred_(0, i);
    py = Xsig_pred_(1, i);
    v = Xsig_pred_(2, i);
    psi = Xsig_pred_(3, i);
    Zsig(0, i) = sqrt(px*px + py*py);
    Zsig(1, i) = atan2(py,px);

    //Avoid dividing by zero    
    if (Zsig(0,i) < 0.0001) {Zsig(0, i) = 0.0001;}
    Zsig(2, i) = v*(px*cos(psi) + py*sin(psi))/Zsig(0, i);
  }

  //create example vector for incoming radar measurement
  VectorXd z_rad = VectorXd(n_z_rad);
  z_rad << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
  
  //Update State
  UpdateState(meas_package, Zsig, n_z_rad, z_rad);
}

/**
 * Normalize angles
 * @param {double} angle Angle to be normalize between -M_PI and M_PI
 */
void UKF::NormAngle(double *angle){
  while(*angle > M_PI) *angle -= 2.*M_PI;
  while(*angle < -M_PI) *angle += 2.*M_PI;
}

/**
 * Update the state using the current and predicted measurement
 * @param {MeasurementPackage} meas_package The measurement at k+1
 * @param {MatrixXd} Zsig The sigma points for the predicted measurement
 * @param {int} n_z Size of measurement vector (2 - lidar, 3 - radar)
 * @param {VectorXd} z_meas Current measurement vector at k+1
 */

void UKF::UpdateState(MeasurementPackage meas_package, MatrixXd Zsig, int n_z, VectorXd z_meas){

  //calculate mean predicted measurement
  //calculate measurement covariance matrix S 
  //calculate cross correlation matrix
  //calculate Kalman gain K;
  //update state mean and covariance matrix  

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
 
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
 
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
 
  z_pred.fill(0);
  for (int i=0; i < 2*n_aug_ + 1; i++){
    z_pred += weights_(i)*Zsig.col(i);
  }
 
  S.fill(0);
  Tc.fill(0);
  for (int i=0; i < 2*n_aug_ + 1; i++){
    
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    //angle normalization
    NormAngle(&(z_diff(1)));
    }

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormAngle(&(x_diff(3)));

    S += weights_(i)*z_diff*z_diff.transpose();
    Tc += weights_(i)*x_diff*z_diff.transpose();
  }


  //Measurement noise covariance matrices
  MatrixXd R = MatrixXd(n_z, n_z); 

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    R << std_radr_*std_radr_, 0, 0,
         0, std_radphi_*std_radphi_, 0,
         0, 0, std_radrd_*std_radrd_;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
    R << std_laspx_*std_laspx_, 0,
         0, std_laspy_*std_laspy_;
  }

  S += R;

  MatrixXd Kc = Tc*S.inverse();

  //residual
  VectorXd z_diff = z_meas - z_pred;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
  //angle normalization
  NormAngle(&(z_diff(1)));
  }

  //Update state and covariance matrix
  x_ += Kc*z_diff;

  P_ -= Kc*S*Kc.transpose();

  
  //Calculate NIS for parameter tuning
  double NIS = z_diff.transpose()*S.inverse()*z_diff;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ 
    NIS_rad_ = NIS;   
    NIS_Count_rad_ += 1.0;
    if (NIS_rad_ > 7.82) {NIS_HighCount_rad_ += 1.0;}
    cout << "NIS_radar: " << NIS_rad_ << "\n";
    cout << "% NIS_radar is greater than 7.8: " << (NIS_HighCount_rad_/NIS_Count_rad_)*100 << "%\n";
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER){ 
    NIS_las_ = NIS;   
    NIS_Count_las_ += 1.0;
    if (NIS_las_ > 5.99) {NIS_HighCount_las_ += 1.0;}
    cout << "NIS_laser: " << NIS_las_ << "\n";
    cout << "% NIS_laser is greater than 5.99: " << (NIS_HighCount_las_/NIS_Count_las_)*100 << "%\n";
  }


} 
