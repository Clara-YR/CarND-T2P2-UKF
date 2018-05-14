[//]: # (Image References)

[image0]: ./UKF_roadmap.jpg "UKF Roadmap"
[image1]: ./warining.jpg "Failed to listen to port"


# Unscented Kalman Filter

![alt text][image0]


## File Structure in src folder

* `main.cpp` - reads in data, calls a function to run the Unscented Kalman fiter, calls a function to calculate RMSE.
* `ukf.cpp` - initializes the Unscented Kalman filter, defines the predict and update dunctions
* `tools.cpp` - function to calculate RMSE

The only file I need to modify ars `ukf.cpp` and `tools.cpp`.

## Function structure in `ukf.cpp`

```
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	
	/********************
	 *  Initialization
	 ********************/
 
	/*****************
	 *  Prediction   *
	 *****************/
	 
	/*************
	 *  Update   *
	 *************/
}
```
```
void UKF::Prediction(double delta_t) {

	/***************************************
	 * PREDICT AUGMENTED MEAN STATE VECTOR *
	 ***************************************/

	/*********************************************
	 * PREDICT STATE AND STATE COVARIANCE MATRIX *
	 *********************************************/	 
}
```
```
void UKF::UpdateLidar(MeasurementPackage meas_package) {

}
```
```
void UKF::UpdateRadar(MeasurementPackage meas_package) {
	
	/*************************************************
	 * TRANSFORM SIGMA POINTS INTO MEASUREMENT SPACE *
	 *************************************************/
	 
	/*****************************
	 * PREDICT RADAR MEASUREMENT *
	 *****************************/
	 
	/****************************
     * UPDATE RADAR MEASUREMENT *
     ****************************/ 
}
```

## EKF Versus UKF Repositories
The EKF and UKF repositories are similar, but have small differences.

In the EKF project, there was a separate `KamlmanFilter` class for storing variables and calling the predict and update steps. In this project all of the Kalman filter code will go in the `ukf.cpp` file.

Also as part of your code, you will need to store laser and radar NIS. The `ukf.cpp` stater code shows where to calculate NIS.