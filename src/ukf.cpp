#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <random>

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
    std_a_ = 3.0;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.1;

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

    // Time when ths state is true
    time_us_ = 0;

    // State dimension
    n_x_ = 5;

    // Augmented state dimension
    n_aug_ = 7;

    // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;

    // Prediceted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

    // Weights of sigma points
    weights_ = VectorXd(2*n_aug_+1);
    // Set values for weights
    const double weight_0 = lambda_ / (lambda_+n_aug_);
    weights_(0) = weight_0;
    const double weight_i = 0.5/(n_aug_+lambda_);
    for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
        weights_(i) = weight_i;
    }

    // Set measurement matrix
    H_laser_ << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0;

    // Set measurement noise covariance matrix - Laser
    R_laser_ << std_laspx_*std_laspx_, 0,
                0, std_laspy_*std_laspy_;

    // Set measurement noise covariance matrix - Radar
    R_radar_ << std_radr_*std_radr_, 0, 0,
                0, std_radphi_*std_radphi_, 0,
                0, 0, std_radrd_*std_radrd_;
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

    /*******************
     *  Initialization
     ********************/
    if (!is_initialized_) {

        // initialize the state covariance matrix with a 5X5 Identity
        P_ = MatrixXd::Identity(5,5);
        // initialize time with 0
        time_us_ = meas_package.timestamp_;

        // initialize the state x_ with the first measurement
        // Convert radar from polar to cartesian coordinates and initialize state.
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

            // recover state parameters
            const double rho = meas_package.raw_measurements_(0);
            double phi = meas_package.raw_measurements_(1);

            // angle normalization
            NormalizeAngle(phi);

            // convert state from polar to cartesian coordinates
            const double x = rho * cos(phi);
            const double y = rho * sin(phi);

            // Initialize state x_
            x_ << x, y, 0, 0, 0;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            // Initialize state x_
            x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;
        }
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************
     *  Prediction   *
     *****************/
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0; // dt - expressed in seconds
    time_us_ = meas_package.timestamp_;
    Prediction(delta_t);

    /*************
     *  Update   *
     *************/
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        if (use_radar_) {
            UpdateRadar(meas_package);
        } else {
            return;
        }
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        if (use_laser_) {
            UpdateLidar(meas_package);
        } else {
            return;
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

    /**
     *  Create Augmented Sigma Points
     */

    // Create augmented mean state x_aug(k|k)
    VectorXd x_aug = VectorXd(7);
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // Create aumented state covariance P_aug(k|k)
    MatrixXd P_aug = MatrixXd(7,7);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_ * std_a_;
    P_aug(6,6) = std_yawdd_ * std_yawdd_;

    // Create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // Create aumented sigma point matrix Xsig_aug(k|k)
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
    Xsig_aug.col(0) = x_aug;
    for (int i=0; i<n_aug_; i++) {
        Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }

    /**
     *  Predict Sigma Points Xsig_pred_(k+1|k)
     */
    for (int i=0; i<2*n_aug_+1; i++) {
        // extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        // predicted state values
        double px_p, py_p;

        // avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
        }
        else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        // add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;
        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        // write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }

    /**
     *  Predict the state x_ and the state covariance matrix P_
     */

    // predict state mean x_(k+1|k)
    x_ = Xsig_pred_ * weights_;

    // create state difference matrix P_(k+1|k)
    P_.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++) {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        NormalizeAngle(x_diff(3));

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
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

    VectorXd y = meas_package.raw_measurements_ - H_laser_ * x_;
    MatrixXd S = H_laser_ * P_ * H_laser_.transpose();
    MatrixXd K = P_ * H_laser_.transpose() * S.inverse();

    // update state and state covariance
    x_ = x_ + K * y;
    P_ = (MatrixXd::Identity(5, 5) - K * H_laser_) * P_;

    // calculate NIS - Normalized Innovation Squared
    MatrixXd NIS_laser = y.transpose() * S.inverse() * y;
    cout << "\t\t\t\t\tNIS_laser = " << NIS_laser << endl;
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

    /*************************************************
     * TRANSFORM SIGMA POINTS INTO MEASUREMENT SPACE *
     *************************************************/

    // set measurement dimension, radar can measure rho, phi, rho_dot
    int n_z = 3;
    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
    // transform sigma points into measurement space
    for (int i=0; i<2*n_aug_+1; i++) {  //2n+1 sigma points

        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        // measurement modle
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);
        Zsig(1,i) = atan2(p_y, p_x);
        Zsig(2,i) = (p_x*v1 + p_y*v2) / max(0.001, Zsig(0,1));

    }

    /*****************************
     * PREDICT RADAR MEASUREMENT *
     *****************************/

    // predict mean measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++) {
        z_pred += weights_(i) * Zsig.col(i);
    }

    // innovation covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++) {

        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        NormalizeAngle(z_diff(1));

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    S = S + R_radar_;

    /****************************
     * UPDATE RADAR MEASUREMENT *
     ****************************/

    // create matrix for corss correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    // calculate cross correlatin matrix
    Tc.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++) {  //2n+1 sigma points

        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        NormalizeAngle(z_diff(1));

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        NormalizeAngle(x_diff(3));

        // Tc bewteen sigma points in state space and measurement space
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
    // angle normalization
    NormalizeAngle(z_diff(1));

    // update state and state covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // calculate NIS - Normalized Innovation Squared
    MatrixXd NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
    cout << "NIS_radar = " << NIS_radar << endl;
}

/**
   * Angle normalization helper.
   * @param phi The angle to be normalized
   */
void UKF::NormalizeAngle(double& phi) {
    phi = atan2(sin(phi), cos(phi));
}