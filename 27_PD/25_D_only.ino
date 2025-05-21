#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9
#define PIN_SERVO 10
#define PIN_IR    A0

// Event interval parameters
#define _INTERVAL_DIST   50 // distance sensor interval (unit: ms)
#define _INTERVAL_SERVO  20 // servo interval (unit: ms)
#define _INTERVAL_SERIAL 100 // serial interval (unit: ms)

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.8    // EMA weight of new sample (range: 0 to 1)

// Servo adjustment
#define _DUTY_MAX 2700
#define _DUTY_NEU 950
#define _DUTY_MIN 700
#define _SERVO_ANGLE_DIFF  30    // Servo range in degrees
#define _SERVO_SPEED       1000  // Servo speed (degrees/sec)

// Target Distance
#define _DIST_TARGET    175 // Target distance in mm

// PID parameters
#define _KP  2.0    // proportional gain
#define _KD  3.0      // derivative gain
#define _KI  0.0      // integral gain

// Global variables
Servo myservo;
float dist_ema = 0;   // Filtered distance
unsigned long last_sampling_time_dist = 0, last_sampling_time_servo = 0, last_sampling_time_serial = 0;

// Servo speed control
int duty_change_per_interval;

// Servo position
int duty_target, duty_current;

// PID control variables
int error_current = 0, error_prev = 0;
float pterm, dterm, iterm = 0;

void setup() {
  // Initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  
  // Initialize servo
  myservo.attach(PIN_SERVO);
  duty_target = duty_current = _DUTY_NEU;
  myservo.writeMicroseconds(duty_current);    

  // Calculate maximum duty change per interval
  duty_change_per_interval = 
    (float)(_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE_DIFF) * (_INTERVAL_SERVO / 1000.0); 
  
  // Initialize serial port
  Serial.begin(1000000);  
}

void loop() {
  unsigned long time_curr = millis();
  
  // Check for event triggers
  if (time_curr >= (last_sampling_time_dist + _INTERVAL_DIST)) {
    last_sampling_time_dist += _INTERVAL_DIST;
    handleDistanceEvent();
  }
  if (time_curr >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    handleServoEvent();
  }
  if (time_curr >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    handleSerialEvent();
  }
}

void handleDistanceEvent() {
  float dist_filtered; // unit: mm
  int control;

  // Get a distance reading
  dist_filtered = volt_to_distance(ir_sensor_filtered(10, 0.5, 0)); // Median of 10 samples
  dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;

  // Update PID variables
  error_current = _DIST_TARGET - dist_ema;
  pterm = _KP * error_current;
  dterm = _KD * (error_current - error_prev);
  iterm += _KI * error_current * (_INTERVAL_DIST / 1000.0); // Simple integration

  // Limit integral term to avoid wind-up
  if (iterm > 50) iterm = 50;
  if (iterm < -50) iterm = -50;

  // Compute control signal
  control = pterm + dterm + iterm;
  error_prev = error_current;
        
  duty_target = _DUTY_NEU + control;

  // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
  if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
  if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
      
  // Turn LED on or off based on the error
  digitalWrite(PIN_LED, error_current > 0);
}

void handleServoEvent() {
  // Adjust duty_current toward duty_target
  if (duty_target > duty_current) {
    duty_current += duty_change_per_interval;
    if (duty_current > duty_target) duty_current = duty_target;
  } else {
    duty_current -= duty_change_per_interval;
    if (duty_current < duty_target) duty_current = duty_target;
  }

  // Write the updated position to the servo
  myservo.writeMicroseconds(duty_current);
}

void handleSerialEvent() {
  // Output distance and control information
  Serial.print("DIST: ");
  Serial.print(dist_ema);
  Serial.print(", ERROR: ");
  Serial.print(error_current);
  Serial.print(", P: ");
  Serial.print(pterm);
  Serial.print(", D: ");
  Serial.print(dterm);
  Serial.print(", I: ");
  Serial.print(iterm);
  Serial.print(", Duty Target: ");
  Serial.println(duty_target);
}

float volt_to_distance(int a_value) {
  // Replace this line with the calibration equation for your IR sensor
  return 1722-9.81*a_value+0.0193*a_value*a_value-1.28E-05*a_value*a_value*a_value;
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose) {
  unsigned int *ir_val, ret_val;

  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1.0)) return 0;

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL) return 0;

  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
  }

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];
  free(ir_val);

  return ret_val;
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}
