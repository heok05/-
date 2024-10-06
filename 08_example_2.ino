// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}

void loop() { 
  float distance;

  // wait until next sampling time. // polling
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // 거리값에 따른 LED 밝기 제어
  int brightness = 255;  // LED를 기본적으로 OFF (가장 밝지 않은 상태)로 설정
  if ((distance >= _DIST_MIN) && (distance <= _DIST_MAX)) {
    if (distance <= 150) {
      brightness = map(distance, _DIST_MIN, 150, 255, 128);  // 100mm에서 최대 밝기, 150mm에서 50% 밝기
    } else if (distance <= 200) {
      brightness = map(distance, 150, 200, 128, 0);  // 150mm에서 50% 밝기, 200mm에서 최대 밝기
    } else if (distance <= 250) {
      brightness = map(distance, 200, 250, 0, 128);  // 200mm에서 최대 밝기, 250mm에서 50% 밝기
    } else {
      brightness = map(distance, 250, _DIST_MAX, 128, 255);  // 250mm에서 50% 밝기, 300mm에서 꺼짐
    }
  }
  analogWrite(PIN_LED, brightness);  // LED 밝기 제어

  // output the distance to the serial port
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
  Serial.println("");

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
