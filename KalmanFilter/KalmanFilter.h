#pragma once
#include <Arduino.h>
#define ALTITUDESIGMA 19
#define ACCELERATIONSIGMA 6
#define MODELSIGMA 0.12

class KalmanFilter{
  private: 
    double A[3][3]; //State transition matrix
    double At[3][3]; //Tranpose of A
    double H[2][3]; //Transform system state to measurement
    double Ht[3][2]; //H tranpose
    double predict_state[3];
    double update_state[3];
    double predict_error[3][3];
    
    double current_error[3][3];
    double kalman_gain[3][2];
    
    unsigned long old_time;
    unsigned long curr_time;
    double dt;

    double altitude_variance = ALTITUDESIGMA * ALTITUDESIGMA;
    double model_variance =  MODELSIGMA * MODELSIGMA;
    double accel_variance = ACCELERATIONSIGMA * ACCELERATIONSIGMA;

    

    void predicted_state();
    void predicted_error();
    void kalman_gained();
    void adjust_state();
    void adjusted_error();
    void init();
  
  public:
    double adjust_error[3][3];
    double measurement[2];
    double current_state[3];
    KalmanFilter();
    void filter(double barometerAltitude, double accelZ);   // Main EKF filter caller
    void initial(double initialBarometerAltitude, double initialAccelZ);
};







