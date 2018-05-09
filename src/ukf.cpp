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
    P_ << MatrixXd::Identity(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.2;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.2;

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
    time_us_ = 0;
    Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
    weights_ = VectorXd(2*n_aug_+1);
    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_aug_;
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
    /********************
     *  Initialization
     ********************/
    if (!is_initialized_) {
        Xsig_pred_.fill(0.0);
        time_us_ = 0;

        // initialize the state x_ with the first measurement
        // Convert radar from polar to cartesian coordinates and initialize state.
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

            // recover state parameters
            float rho = meas_package.raw_measurements_(0);
            float phi = meas_package.raw_measurements_(1);

            // angle normalization
            while (phi > M_PI) phi-=2.*M_PI;
            while (phi <-M_PI) phi+=2.*M_PI;

            // convert state from polar to cartesian coordinates
            float x = rho * cos(phi);
            float y = rho * sin(phi);

            // Initialize state x_
            x_ << x, y, 0, 0, 0;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            // Initialize state x_
            x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;
        }
        // done initializing, no need to predict or update
        is_initialized_ = true;
    }
    else {
        // Convert radar from polar to cartesian coordinates and initialize state.
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

            // recover state parameters
            float rho = meas_package.raw_measurements_(0);
            float phi = meas_package.raw_measurements_(1);

            // angle normalization
            while (phi > M_PI) phi-=2.*M_PI;
            while (phi <-M_PI) phi+=2.*M_PI;

            // convert state from polar to cartesian coordinates
            float x = rho * cos(phi);
            float y = rho * sin(phi);

            // Initialize state x_
            x_ << x, y, 0, 0, 0;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            // Initialize state x_
            x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;
        }
    }

    /*****************
     *  Prediction   *
     *****************/
    // compute the time elapsed between the current and previous measurements
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0; // dt - expressed in seconds
    time_us_ = meas_package.timestamp_;
    void Prediction(double delta_t);

    /*************
     *  Update   *
     *************/
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        void UpdateRadar(MeasurementPackage meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        void UpdateLidar(MeasurementPackage meas_package);
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

    /***************************************
     * PREDICT AUGMENTED MEAN STATE VECTOR *
     ***************************************/

    // create augmented mean state vector
    VectorXd x_aug_ = VectorXd(n_aug_);
    // create augmented state vector covariance
    MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);
    // create sigma point matrix
    MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);
    // create augmented mean state
    x_aug_.head(5) = x_;
    x_aug_(5) = 0;
    x_aug_(6) = 0;
    // create augmented covariance matrix
    P_aug_.fill(0.0);
    P_aug_.topLeftCorner(5,5) = P_;
    P_aug_(5,5) = std_a_ * std_a_;
    P_aug_(6,6) = std_yawdd_ * std_yawdd_;

    /*********************************************
     * PREDICT STATE AND STATE COVARIANCE MATRIX *
     *********************************************/

    // set weights
    double weight_0 = lambda_ / (lambda_+n_aug_);
    weights_(0) = weight_0;
    for (int i=0; i<2*n_aug_+1; i++) {  //2n+1 weights
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
    }

    // predict state mean
    x_.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    // predict state covariance matrix
    P_.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++) {  //Iterate over sigma points

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
        while (x_diff(3) <-M_PI) x_diff(3) += 2.*M_PI;

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

    // set measurement dimension, radar can measure px, py, v, yaw, yaw_dot
    int n_z = 5;
    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

    /*****************************
     * PREDICT LIDAR MEASUREMENT *
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
        while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
        while (z_diff(1) <-M_PI) z_diff(1) += 2.*M_PI;

        S = S + weights_ * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0, std_radrd_*std_radrd_;
    S = S + R;

    /****************************
     * UPDATE LIDAR MEASUREMENT *
     ****************************/

    // create matrix for corss correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    // calculate cross correlatin matrix
    Tc.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++) {  //2n+1 sigma points

        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
        while (z_diff(1) <-M_PI) z_diff(1) += 2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
        while (x_diff(3) <-M_PI) x_diff(3) += 2.*M_PI;

        // Tc bewteen sigma points in state space and measurement space
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_;
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) <-M_PI) z_diff(1) += 2.*M_PI;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // calculate NIS
    MatrixXd epsilon;
    epsilon = z_diff.transpose() * S.inverse() * z_diff;
    cout << "\nNIS epsilon = " << epsilon << endl;
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
        Zsig(2,i) = (p_x*v1 + p_y*v2) / (p_x*p_x + p_y*p_y);
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
        while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
        while (z_diff(1) <-M_PI) z_diff(1) += 2.*M_PI;

        S = S + weights_ * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0, std_radrd_*std_radrd_;
    S = S + R;

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
        while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
        while (z_diff(1) <-M_PI) z_diff(1) += 2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
        while (x_diff(3) <-M_PI) x_diff(3) += 2.*M_PI;

        // Tc bewteen sigma points in state space and measurement space
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_;
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) <-M_PI) z_diff(1) += 2.*M_PI;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // calculate NIS
    MatrixXd epsilon;
    epsilon = z_diff.transpose() * S.inverse() * z_diff;
    cout << "\nNIS epsilon = " << epsilon << endl;
}
