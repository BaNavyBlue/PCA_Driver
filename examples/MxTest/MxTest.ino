#include <Mx2125.h>

Mx2125  Accelerometer(5, 6, A5) ;
uint8_t HB=0 ;  // Heartbeat counter

void setup() {
  Serial.begin(115200) ;
}

void loop() {
  float F, C ;

  Serial.print("Acceleration: ") ;
  Serial.print(Accelerometer.mx_acceleration_x()) ;
  Serial.print(",") ;
  Serial.print(Accelerometer.mx_acceleration_y()) ;
  Serial.print(" -- Tilt: ") ;
  Serial.print(Accelerometer.mx_tilt_x()) ;
  Serial.print(",") ;
  Serial.print(Accelerometer.mx_tilt_y()) ;
  Serial.print("  -- Rotation: ") ;
  Serial.print(Accelerometer.mx_rotation()) ;
  Serial.println();
}
