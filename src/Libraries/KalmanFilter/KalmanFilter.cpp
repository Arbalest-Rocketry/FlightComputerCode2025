#include "KalmanFilter.h"
KalmanFilter::KalmanFilter() {
    init();
}
void KalmanFilter::init(){
  H[0][0] = 1; H[0][1] = 0; H[0][2] = 0;
  H[1][0] = 0; H[1][1] = 0; H[1][2] = 1;

  Ht[0][0] = 1; Ht[0][1] = 0; 
  Ht[1][0] = 0; Ht[1][1] = 0;
  Ht[2][0] = 0; Ht[2][1] = 1;

  current_error[0][0] = 3; current_error[0][1] = 0; current_error[0][2] = 0;
  current_error[1][0] = 0; current_error[1][1] = 2; current_error[1][2] = 0;
  current_error[2][0] = 0; current_error[2][1] = 0; current_error[2][2] = 3;

}

void KalmanFilter::initial(double initialBarometerAltitude, double initialAccelZ) {
    old_time = millis();
    current_state[0] = initialBarometerAltitude; // Altitude
    current_state[1] = 0; // Velocity
    current_state[2] = initialAccelZ; // Acceleration
}
void KalmanFilter::filter(double barometerAltitude, double accelZ) {
  curr_time = millis();
  dt = ((double)(curr_time - old_time) / 1000);
  old_time = curr_time;

  A[0][0] = 1; A[0][1] = dt; A[0][2] = dt * dt / 2;
  A[1][0] = 0; A[1][1] = 1; A[1][2] = dt;
  A[2][0] = 0; A[2][1] = 0; A[2][2] = 1;

  At[0][0] = 1; At[0][1] = 0; At[0][2] = 0;
  At[1][0] = dt; At[1][1] = 1; At[1][2] = 0;
  At[2][0] = dt * dt / 2; At[2][1] = dt; At[2][2] = 1;

  measurement[0] = barometerAltitude;
  measurement[1] = accelZ;

  

  predicted_state();
  predicted_error();
  kalman_gained();
  adjust_state();
  adjusted_error();
  
  memcpy(current_state, update_state, sizeof(update_state));
  memcpy(current_error, adjust_error, sizeof(adjust_error));
  
}

void KalmanFilter::predicted_state(){
  // A*x(k-1)
  predict_state[0] = (A[0][0] * current_state[0]) + (A[0][1] * current_state[1]) + (A[0][2] * current_state[2]);
  predict_state[1] = (A[1][0] * current_state[0]) + (A[1][1] * current_state[1]) + (A[1][2] * current_state[2]);
  predict_state[2] = (A[2][0] * current_state[0]) + (A[2][1] * current_state[1]) + (A[2][2] * current_state[2]);
}

void KalmanFilter::predicted_error(){
  //P(k-) = A(k-1) * P(k-1) * At(k-1) + Q(k-1)
  double B[3][3]; //B = P(k-1) * At(k-1)
  B[0][0] = (current_error[0][0] * At[0][0]) + (current_error[0][1] * At[1][0]) + (current_error[0][2] * At[2][0]);
  B[0][1] = (current_error[0][0] * At[0][1]) + (current_error[0][1] * At[1][1]) + (current_error[0][2] * At[2][1]);
  B[0][2] = (current_error[0][0] * At[0][2]) + (current_error[0][1] * At[1][2]) + (current_error[0][2] * At[2][2]);

  B[1][0] = (current_error[1][0] * At[0][0]) + (current_error[1][1] * At[1][0]) + (current_error[1][2] * At[2][0]);
  B[1][1] = (current_error[1][0] * At[0][1]) + (current_error[1][1] * At[1][1]) + (current_error[1][2] * At[2][1]);
  B[1][2] = (current_error[1][0] * At[0][2]) + (current_error[1][1] * At[1][2]) + (current_error[1][2] * At[2][2]);

  B[2][0] = (current_error[2][0] * At[0][0]) + (current_error[2][1] * At[1][0]) + (current_error[2][2] * At[2][0]);
  B[2][1] = (current_error[2][0] * At[0][1]) + (current_error[2][1] * At[1][1]) + (current_error[2][2] * At[2][1]);
  B[2][2] = (current_error[2][0] * At[0][2]) + (current_error[2][1] * At[1][2]) + (current_error[2][2] * At[2][2]);

  predict_error[0][0] = (A[0][0] * B[0][0]) + (A[0][1] * B[1][0]) + (A[0][2] * B[2][0]) + model_variance;
  predict_error[0][1] = (A[0][1] * B[0][0]) + (A[0][1] * B[1][1]) + (A[0][2] * B[2][1]);
  predict_error[0][2] = (A[0][2] * B[0][0]) + (A[0][1] * B[1][2]) + (A[0][2] * B[2][2]);

  predict_error[1][0] = (A[1][0] * B[0][0]) + (A[1][1] * B[1][0]) + (A[1][2] * B[2][0]);
  predict_error[1][1] = (A[1][1] * B[0][0]) + (A[1][1] * B[1][1]) + (A[1][2] * B[2][1]);
  predict_error[1][2] = (A[1][2] * B[0][0]) + (A[1][1] * B[1][2]) + (A[1][2] * B[2][2]);

  predict_error[2][0] = (A[2][0] * B[0][0]) + (A[2][1] * B[1][0]) + (A[2][2] * B[2][0]);
  predict_error[2][1] = (A[2][1] * B[0][0]) + (A[2][1] * B[1][1]) + (A[2][2] * B[2][1]);
  predict_error[2][2] = (A[2][2] * B[0][0]) + (A[2][1] * B[1][2]) + (A[2][2] * B[2][2]);

}

void KalmanFilter::kalman_gained(){
  //K(k) = P(k-) * Ht * (H * P(k-) * Ht + R(k))^-1
  double PHt[3][2], HpH[2][2], D[2][2];
  // P(k-) * Ht
  PHt[0][0] = (predict_error[0][0] * Ht[0][0]) + (predict_error[0][1] * Ht[1][0]) +(predict_error[0][2] * Ht[2][0]);
  PHt[0][1] = (predict_error[0][0] * Ht[0][1]) + (predict_error[0][1] * Ht[1][1]) +(predict_error[0][2] * Ht[2][1]);

  PHt[1][0] = (predict_error[1][0] * Ht[0][0]) + (predict_error[1][1] * Ht[1][0]) +(predict_error[1][2] * Ht[2][0]);
  PHt[1][1] = (predict_error[1][0] * Ht[0][1]) + (predict_error[1][1] * Ht[1][1]) +(predict_error[1][2] * Ht[2][1]);

  PHt[2][0] = (predict_error[2][0] * Ht[0][0]) + (predict_error[2][1] * Ht[1][0]) +(predict_error[2][2] * Ht[2][0]);
  PHt[2][1] = (predict_error[2][0] * Ht[0][1]) + (predict_error[2][1] * Ht[1][1]) +(predict_error[2][2] * Ht[2][1]);
  //H * P(k-) * Ht + R(k)
  HpH[0][0] = (H[0][0] * PHt[0][0]) + (H[0][1] * PHt[1][0]) + (H[0][2] * PHt[2][0]) + altitude_variance;
  HpH[0][1] = (H[0][0] * PHt[0][1]) + (H[0][1] * PHt[1][1]) + (H[0][2] * PHt[2][1]);

  HpH[1][0] = (H[1][0] * PHt[0][0]) + (H[1][1] * PHt[1][0]) + (H[1][2] * PHt[2][0]);
  HpH[1][1] = (H[1][0] * PHt[0][1]) + (H[1][1] * PHt[1][1]) + (H[1][2] * PHt[2][1] + accel_variance);
  //^-1
  double det = (HpH[0][0]*HpH[1][1]) - (HpH[0][1]*HpH[1][0]);
  D[0][0] = HpH[1][1]/det;
  D[0][1] = -HpH[0][1]/det;
  D[1][0] = -HpH[1][0]/det;
  D[1][1] = HpH[0][0]/det;

  kalman_gain[0][0] = (PHt[0][0] * D[0][0]) + (PHt[0][1] * D[1][0]);
  kalman_gain[0][1] = (PHt[0][0] * D[0][1]) + (PHt[0][1] * D[1][1]);

  kalman_gain[1][0] = (PHt[1][0] * D[0][0]) + (PHt[1][1] * D[1][0]);
  kalman_gain[1][1] = (PHt[1][0] * D[0][1]) + (PHt[1][1] * D[1][1]);

  kalman_gain[2][0] = (PHt[2][0] * D[0][0]) + (PHt[2][1] * D[1][0]);
  kalman_gain[2][1] = (PHt[2][0] * D[0][1]) + (PHt[2][1] * D[1][1]);

}
void KalmanFilter::adjust_state(){
  double B[2]; //z(k) - H(k) * x(k-)
  B[0] = measurement[0] - ((H[0][0] * predict_state[0]) + (H[0][1] * predict_state[1]) +(H[0][2] * predict_state[2]));
  B[1] = measurement[1] - ((H[1][0] * predict_state[0]) + (H[1][1] * predict_state[1]) +(H[1][2] * predict_state[2]));

  update_state[0] = predict_state[0] + ((kalman_gain[0][0] * B[0]) + (kalman_gain[0][1] * B[1]));
  update_state[1] = predict_state[1] + ((kalman_gain[1][0] * B[0]) + (kalman_gain[1][1] * B[1]));
  update_state[2] = predict_state[2] + ((kalman_gain[2][0] * B[0]) + (kalman_gain[2][1] * B[1]));
}

void KalmanFilter::adjusted_error(){
  //(I - K(k) * H(k)) * P(k-)
  double B[3][3];//inside bracket

  B[0][0] = 1 - ((kalman_gain[0][0] * H[0][0]) + (kalman_gain[0][1] * H[1][0]));
  B[0][1] = -((kalman_gain[0][0] * H[0][1]) + (kalman_gain[0][1] * H[1][1]));
  B[0][2] = -((kalman_gain[0][0] * H[0][2]) + (kalman_gain[0][1] * H[1][2]));

  B[1][0] = -((kalman_gain[1][0] * H[0][0]) + (kalman_gain[1][1] * H[1][0]));
  B[1][1] = 1 - ((kalman_gain[1][0] * H[0][1]) + (kalman_gain[1][1] * H[1][1]));
  B[1][2] = -((kalman_gain[1][0] * H[0][2]) + (kalman_gain[1][1] * H[1][2]));

  B[2][0] = -((kalman_gain[2][0] * H[0][0]) + (kalman_gain[2][1] * H[1][0]));
  B[2][1] = -((kalman_gain[2][0] * H[0][1]) + (kalman_gain[2][1] * H[1][1]));
  B[2][2] = 1 - ((kalman_gain[2][0] * H[0][2]) + (kalman_gain[2][1] * H[1][2]));

  adjust_error[0][0]  = (B[0][0] * predict_error[0][0]) + (B[0][1] * predict_error[1][0]) + (B[0][2] * predict_error[2][0]);
  adjust_error[0][1]  = (B[0][0] * predict_error[0][1]) + (B[0][1] * predict_error[1][1]) + (B[0][2] * predict_error[2][1]);
  adjust_error[0][2]  = (B[0][0] * predict_error[0][2]) + (B[0][1] * predict_error[1][2]) + (B[0][2] * predict_error[2][2]);

  adjust_error[1][0]  = (B[1][0] * predict_error[0][0]) + (B[1][1] * predict_error[1][0]) + (B[1][2] * predict_error[2][0]);
  adjust_error[1][1]  = (B[1][0] * predict_error[0][1]) + (B[1][1] * predict_error[1][1]) + (B[1][2] * predict_error[2][1]);
  adjust_error[1][2]  = (B[1][0] * predict_error[0][2]) + (B[1][1] * predict_error[1][2]) + (B[1][2] * predict_error[2][2]);

  adjust_error[2][0]  = (B[2][0] * predict_error[0][0]) + (B[2][1] * predict_error[1][0]) + (B[2][2] * predict_error[2][0]);
  adjust_error[2][1]  = (B[2][0] * predict_error[0][1]) + (B[2][1] * predict_error[1][1]) + (B[2][2] * predict_error[2][1]);
  adjust_error[2][2]  = (B[2][0] * predict_error[0][2]) + (B[2][1] * predict_error[1][2]) + (B[2][2] * predict_error[2][2]);

}
