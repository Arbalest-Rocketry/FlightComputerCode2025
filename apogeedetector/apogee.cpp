#include "apogee.h"

Adafruit_BMP280 bmp;
bool reached = false;

void init_rw(RollingWindow *rw, double *BackingArray, size_t capacity){
  rw->backing_array = BackingArray; 
  rw->capacity = capacity;
  rw->size = 0;
  rw->front = 0;
  rw->sum_of_elements = 0;
}

size_t mod_rw(size_t index, size_t modulo){
  return (index % modulo + modulo) % modulo;
}

void add_data(RollingWindow *rw, double data){
  if(rw->size >= rw->capacity){
    rw->sum_of_elements -= rw->backing_array[mod_rw(rw->front, rw->capacity)];
    rw->front = mod_rw(rw->front + 1, rw->capacity );
    rw->size --;
  }
  size_t back_insertion_index = mod_rw(rw->front + rw->size, rw->capacity);
  rw->backing_array[back_insertion_index] = data;
  rw->sum_of_elements += data;
  rw->size++;
}
double get_earliest_data(RollingWindow *rw){
  return rw->backing_array[mod_rw(rw->front, rw->capacity)];
}
double get_latest_data(RollingWindow *rw){
  return rw->backing_array[mod_rw(rw->front + rw->size - 1, rw->capacity)];
}


//detector
void init_apogee_detector(ApogeeDetector *detector, double *BackingArray, size_t capacity){
  init_rw(&detector->altitude_window, BackingArray, capacity);
  detector->last_altitude = -1;
  detector->apogee_reached = 0;
  detector->decrease_count = 0;
}
void update_apogee_detector(ApogeeDetector *detector, double cur_altitude){
  add_data(&detector->altitude_window, cur_altitude);

  if(detector->apogee_reached){
    return;
  }
  Serial.print("Current altitude: ");
  Serial.println(cur_altitude);
  if(detector->last_altitude >= 0 ){
    if(cur_altitude < detector->last_altitude){
      detector->decrease_count ++;
      Serial.print("Decrease count: ");
      Serial.println(detector->decrease_count);

    }
    else{
      detector->decrease_count = 0;
    }
  }
  if(detector->decrease_count >= THRESHOLD){
      detector->apogee_reached = 1;
      Serial.println("Apogee Reached");
  }
  detector->last_altitude = cur_altitude;

}
int apogee_reached(ApogeeDetector *detector){
  return detector->apogee_reached;
}