#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9
#define PIN_SERVO 10
#define PIN_IR    A0

// Event interval parameters
#define _INTERVAL_DIST    100   // Distance sensor sampling interval (ms)
#define _INTERVAL_SERVO   50    // Servo adjustment interval (ms)
#define _INTERVAL_SERIAL  500   // Serial output interval (ms)

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.2          // Exponential moving average factor (0.2 is moderate smoothing)

// Servo adjustment - Set _DUTY_MAX, _NEU, _MIN with your own numbers
#define _DUTY_MAX 2000          // Maximum servo pulse width (us)
#define _DUTY_NEU 1500          // Neutral servo pulse width (us)
#define _DUTY_MIN 1000          // Minimum servo pulse width (us)

#define _SERVO_ANGLE_DIFF 45    // Difference in servo angles (degrees)
#define _SERVO_SPEED 90         // Servo speed (degrees per second)

#define _BANGBANG_RANGE 350     // Bang-bang control range (us)

// Target Distance
#define _DIST_TARGET 175        // Target distance (mm)

// Global variables
Servo myservo;

float dist_ema = 0.0;           // Filtered distance measurement (mm)
int duty_change_per_interval;   // Maximum duty change per interval
int duty_target;                // Target servo duty cycle
int duty_current;               // Current servo duty cycle
int duty_adj;                   // Adjustment level (not used in this implementation)

unsigned long last_sampling_time_dist = 0;   // Distance sensor sampling timestamp
unsigned long last_sampling_time_servo = 0;  // Servo adjustment timestamp
unsigned long last_sampling_time_serial = 0; // Serial output timestamp

bool event_dist = false, event_servo = false, event_serial = false;

void setup() {
  // Initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);

  duty_target = duty_current = _DUTY_NEU;
  myservo.writeMicroseconds(duty_current);

  // Initialize serial port
  Serial.begin(9600);

  // Convert angular speed into duty change per interval
  duty_change_per_interval =
    (float)(_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE_DIFF) * (_INTERVAL_SERVO / 1000.0);
}

void loop() {
  unsigned long time_curr = millis();

  // Wait until the next event time
  if (time_curr >= (last_sampling_time_dist + _INTERVAL_DIST)) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  if (event_dist) {
    float dist_filtered; // Filtered distance
    event_dist = false;

    // Get a distance reading from the distance sensor
    dist_filtered = volt_to_distance(ir_sensor_filtered(10, 0.5, 0));
    dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;

    // Bang-bang control
    if (dist_ema > _DIST_TARGET) {
      duty_target = _DUTY_NEU - _BANGBANG_RANGE;
      digitalWrite(PIN_LED, LOW);
    } else if (dist_ema < _DIST_TARGET) {
      duty_target = _DUTY_NEU + _BANGBANG_RANGE;
      digitalWrite(PIN_LED, HIGH);
    }
  }

  if (event_servo) {
    event_servo = false;

    // Adjust duty_current toward duty_target
    if (duty_target > duty_current) {
      duty_current += duty_change_per_interval;
      if (duty_current > duty_target)
        duty_current = duty_target;
    } else if (duty_target < duty_current) {
      duty_current -= duty_change_per_interval;
      if (duty_current < duty_target)
        duty_current = duty_target;
    }

    // Servo arm protection
    if (duty_current < _DUTY_MIN)
      duty_current = _DUTY_MIN;
    else if (duty_current > _DUTY_MAX)
      duty_current = _DUTY_MAX;

    // Update servo position
    myservo.writeMicroseconds(duty_current);
  }

  if (event_serial) {
    event_serial = false;

    // Output the read value to the serial port
    Serial.print("TARGET:"); Serial.print(_DIST_TARGET);
    Serial.print(",DIST:"); Serial.print(dist_ema);
    Serial.print(",duty_target:"); Serial.print(duty_target);
    Serial.print(",duty_current:"); Serial.println(duty_current);
  }
}

float volt_to_distance(int a_value) {
  // Convert analog reading to distance (example equation, replace with actual calibration data)
  return (6762.0 / (a_value - 9) - 4.0) * 10.0;
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose) {
  unsigned int *ir_val, ret_val;
  unsigned int start_time;

  if (verbose >= 2)
    start_time = millis();

  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1))
    return 0;

  if (position == 1.0)
    position = 0.999;

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;

  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
  }

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];
  free(ir_val);

  return ret_val;
}
