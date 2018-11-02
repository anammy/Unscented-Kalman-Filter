#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* Normalized Innovation Squared (NIS) for lidar
  double NIS_las_;
  ///* Total number of NIS values for lidar
  double NIS_Count_las_;
  ///* Number of NIS values greater than 5.99 (0.05 value from Chi-squared distribution for 2 DOF measurement space) for lidar
  double NIS_HighCount_las_;


  ///* Normalized Innovation Squared (NIS) for radar
  double NIS_rad_;
  ///* Total number of NIS values for radar
  double NIS_Count_rad_;
  ///* Number of NIS values greater than 7.82 (0.05 value from Chi-squared distribution for 3 DOF measurement space) for lidar
  double NIS_HighCount_rad_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Normalize angles
   * @param {double} angle: angle to be normalized between -M_PI and M_PI
   */
  void NormAngle(double *angle);

  /**
   * Update the state using the current and predicted measurement
   * @param {MeasurementPackage} meas_package The measurement at k+1
   * @param {MatrixXd} Z_sig The sigma points for the predicted measurement
   * @param {int} n_z Size of measurement vector (2 - lidar, 3 - radar)
   * @param {VectorXd} z_meas Current measurement vector at k+1
   */
   void UpdateState(MeasurementPackage meas_package, MatrixXd Zsig, int n_z, VectorXd z_meas);
};

#endif /* UKF_H */
