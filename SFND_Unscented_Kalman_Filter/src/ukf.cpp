#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
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
  std_a_ = 6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  n_aug_ = 7;
  n_x_ = 5;
  lambda_ = 3 - n_aug_;

  H_ = MatrixXd(2, n_x_);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

  R_ = MatrixXd(2, 2);
  R_ << pow(std_laspx_, 2), 0,
        0, pow(std_laspy_, 2);

  // Calculate weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_[0] = lambda_/(lambda_ + n_aug_);
  for(int i=1; i<(2*n_aug_+1); ++i){
      weights_[i] = 1/(2*(lambda_ + n_aug_));
  }

  assert(use_laser_ || use_radar_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_) {
    //cout << "Kalman Filter Initialization " << endl;

    // set the state with the initial location and zero velocity
    if(use_laser_ && meas_package.sensor_type_ == meas_package.LASER) {
        x_ << meas_package.raw_measurements_[0],
                meas_package.raw_measurements_[1],
                0, 0, 0;
        P_ <<   pow(std_laspx_, 2), 0, 0, 0, 0,
                0, pow(std_laspy_, 2), 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;
    }
    else if(use_radar_ && meas_package.sensor_type_ == meas_package.RADAR){
        double rho = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];
        x_ <<   rho * cos(phi),
                rho * sin(phi),
                0, 0, 0;
        P_ = MatrixXd::Identity(n_x_, n_x_);
        P_(0, 0) = 0.4;
        P_(1, 1) = 0.4;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  double dt = (double)(meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // predict
  Prediction(dt);

  // measurement update
  if(use_laser_ && meas_package.sensor_type_ == meas_package.LASER){
    UpdateLidar(meas_package);
  }
  else if(use_radar_ && meas_package.sensor_type_ == meas_package.RADAR){
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

    /**
     * @brief 
     * Part 1: Generate Sigma points in augmented state space
     */

    // create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // create augmented mean state
    x_aug.head(n_x_) = x_;

    // create augmented covariance matrix
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = pow(std_a_, 2);
    P_aug(n_x_ + 1, n_x_ + 1) = pow(std_yawdd_, 2);

    // create square root matrix
    MatrixXd P_aug_root = P_aug.llt().matrixL();

    // create augmented sigma points
    double scalar = sqrt(lambda_ + n_aug_);
    Xsig_aug.col(0) = x_aug;
    for(int i=1; i<Xsig_aug.cols(); ++i){
        if(i >= 1 && i <= n_aug_){
            Xsig_aug.col(i) = x_aug + (P_aug_root.col(i - 1) * scalar);
        }
        else if(i >= (n_aug_ + 1)){
            Xsig_aug.col(i) = x_aug - (P_aug_root.col(i - n_aug_ - 1) * scalar);
        }
    }

    /**
     * @brief 
     * Part 2: Predict for k+1 timestep, for all sigma points
     */

    // create matrix with predicted sigma points as columns
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    for(int i=0; i<Xsig_aug.cols(); ++i){

        double vel = Xsig_aug.col(i)[2];
        double yaw = Xsig_aug.col(i)[3];
        double yaw_rate = Xsig_aug.col(i)[4];

        double acc_noise = Xsig_aug.col(i)[5];
        double yaw_acc_noise = Xsig_aug.col(i)[6];
        VectorXd v1(5), v2(5);

        v2 << 0.5 * pow(delta_t, 2) * cos(yaw) * acc_noise,
                0.5 * pow(delta_t, 2)  * sin(yaw) * acc_noise,
                delta_t * acc_noise,
                0.5 * pow(delta_t, 2) * yaw_acc_noise,
                delta_t * yaw_acc_noise;
        if(abs(yaw_rate) < 0.001){
            v1 << vel * delta_t * cos(yaw),
                    vel * delta_t * sin(yaw),
                    0, delta_t * yaw_rate, 0;
        }
        else{
            v1 << (vel/yaw_rate)*(sin(yaw + delta_t * yaw_rate) - sin(yaw)),
                    (vel/yaw_rate)*(-cos(yaw + delta_t * yaw_rate) + cos(yaw)),
                    0, delta_t * yaw_rate, 0;

        }
        Xsig_pred_.col(i) = Xsig_aug.col(i).head(n_x_) + v1 + v2;
    }

    /**
     * @brief 
     * Part 3: 
     * 1. Approximate the non linear transformation's results (CTRV model) as Gaussian distribution
     * 2. Update x and P
     */
    // create vector for predicted state
    VectorXd x_pred = VectorXd(n_x_);

    // create covariance matrix for prediction
    MatrixXd P_pred = MatrixXd(n_x_, n_x_);

    // predict state mean
    for(int i=0; i<(2*n_aug_+1); ++i){
        x_pred += (Xsig_pred_.col(i) * weights_[i]);
    }

    // predict state covariance matrix
    for(int i=0; i<(2*n_aug_+1); ++i){
        VectorXd col = Xsig_pred_.col(i) - x_pred;
        P_pred += ((col * col.transpose()) * weights_[i]);
    }

    // update state and covariance matrix
    x_ = x_pred;
    P_ = P_pred;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  VectorXd z_pred = H_ * x_; // 2x5 5x1 = 2x1

  VectorXd z = meas_package.raw_measurements_;
  VectorXd y = z - z_pred; // 2x1

  MatrixXd Ht = H_.transpose(); // 5x2
  MatrixXd PHt = P_ * Ht; // 5x5 5x2 = 5x2
  MatrixXd S = H_ * PHt + R_; // 2x5 5x2 + 2x2 = 2x2
  MatrixXd Si = S.inverse(); // 2x2
  
  MatrixXd K = PHt * Si; // 5x2 2x2 = 5x2

  //new estimate
  x_ += (K * y); // 5x2 2x1 = 5x1
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K * H_) * P_; // 5x5 5x5 = 5x5

  VectorXd nis = y.transpose() * Si * y;
  std::cout << "NIS LIDAR: " << nis(0, 0) << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  /**
   * @brief 
   * Part 1: Project predicted sigma points from state space to measurement space
   * Approximate the same nonlinear transformation as Gaussian 
   */
  int n_z = 3;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  // transform sigma points into measurement space
  for(int i=0; i<(2 * n_aug_ + 1); ++i){
      auto px = Xsig_pred_.col(i)[0];
      auto py = Xsig_pred_.col(i)[1];
      auto vel = Xsig_pred_.col(i)[2];
      auto yaw = Xsig_pred_.col(i)[3];
      auto yaw_rate = Xsig_pred_.col(i)[4];

      Zsig.col(i)[0] = sqrt(pow(px, 2) + pow(py, 2));
      Zsig.col(i)[1] = atan(py / px);
      Zsig.col(i)[2] = (vel * (px * cos(yaw) + py * sin(yaw)))/Zsig.col(i)[0];
  }

  // calculate mean predicted measurement
  for(int i=0; i<(2*n_aug_+1); ++i){
      z_pred += (Zsig.col(i) * weights_[i]);
  }

  // calculate innovation covariance matrix S
  for(int i=0; i<(2*n_aug_+1); ++i){
      VectorXd col = Zsig.col(i) - z_pred;
      S += ((col * col.transpose()) * weights_[i]);
  }
  MatrixXd R = MatrixXd(n_z, n_z);
  R.diagonal() << pow(std_radr_, 2), pow(std_radphi_, 2), pow(std_radrd_, 2);
  S += R;

  /**
   * @brief 
   * Part 2: Perform the measurement update step
   */

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
    for(int i=0; i<(2*n_aug_ + 1); ++i){
        VectorXd c1 = Xsig_pred_.col(i) - x_;
        VectorXd c2 = Zsig.col(i) - z_pred;
        Tc += ((c1 * c2.transpose()) * weights_[i]);
    }

    // calculate Kalman gain K;
    MatrixXd Si = S.inverse();
    MatrixXd K = Tc * Si;

    VectorXd z = meas_package.raw_measurements_;
    VectorXd y = z - z_pred;

    // update state mean and covariance matrix
    x_ += K * y;
    P_ -= K * S * K.transpose();

    VectorXd nis = y.transpose() * Si * y;
    std::cout << "NIS RADAR: " << nis(0, 0) << std::endl;
}