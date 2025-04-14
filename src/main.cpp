#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

#include "PID.h"

#define SW1 35
#define SW2 37

#define LED1 5
#define LED2 8
#define SERVO_PIN 17

// rename this pins for better debugging
#define SENSOR_PISO_IZQ 11 // piso
#define SENSOR_LATERAL_IZQ 9
#define SENSOR_FRONTAL_IZQ 10
#define SENSOR_FRONTAL_DER 7
#define SENSOR_LATERAL_DER 6
#define SENSOR_PISO_DER 3 // piso

#define MAX_SENSORS 6

#define RIGHT_PWM_PIN 1
#define RIGHT_DIR_PIN 12
#define RIGHT_DIS_PIN 14

#define LEFT_PWM_PIN 2
#define LEFT_DIR_PIN 13
#define LEFT_DIS_PIN 14

#define GO 21
#define RDY 18


uint8_t sensor_states[MAX_SENSORS];
const int freq = 12000;
const int resolution = 10;
const int right_pwm_channel = 2;
const int left_pwm_channel = 3;

#define MAX_SPEED pow(2, resolution) - 1

uint8_t switch_value = 0;


bool DONE = false;
bool DONE2 = false;

bool flag1 = false;
bool flag2 = false;
bool flag3 = false;
bool flag4 = false;
bool flag5 = false;


Adafruit_NeoPixel NEO = Adafruit_NeoPixel(1, 38, NEO_GRB + NEO_KHZ800);
#define MAX_BRIGHTNESS 100
// colors
const uint32_t RED = NEO.Color(255, 0, 0);
const uint32_t GREEN = NEO.Color(0, 255, 0);
const uint32_t BLUE = NEO.Color(0, 0, 255);
const uint32_t YELLOW = NEO.Color(255, 255, 0);
const uint32_t CYAN = NEO.Color(0, 255, 255);
const uint32_t MAGENTA = NEO.Color(255, 0, 255);

Servo servoFlag;

PID trackerPID(850.0, 0.0, 50.0, MAX_SPEED, -MAX_SPEED);

hw_timer_t * sensor_timer = NULL;

// @todo: check which sensors are actually used
void IRAM_ATTR sensorTimerInterruption(){
  sensor_states[0] = analogRead(SENSOR_PISO_IZQ);
  sensor_states[1] = analogRead(SENSOR_LATERAL_IZQ);
  sensor_states[2] = analogRead(SENSOR_FRONTAL_IZQ);
  sensor_states[3] = analogRead(SENSOR_FRONTAL_DER);
  sensor_states[4] = analogRead(SENSOR_LATERAL_DER);
  sensor_states[5] = analogRead(SENSOR_PISO_DER);
}

void setMotorSpeed(int lvel, int rvel){
  lvel = constrain(lvel, -255, 255);
  rvel = constrain(rvel, -255, 255);

  digitalWrite(LEFT_DIS_PIN, LOW);
  digitalWrite(RIGHT_DIS_PIN, LOW);

  (lvel <= 0) ? digitalWrite(LEFT_DIR_PIN, HIGH) : digitalWrite(LEFT_DIR_PIN, LOW);
  (rvel >= 0) ? digitalWrite(RIGHT_DIR_PIN, HIGH) : digitalWrite(RIGHT_DIR_PIN, LOW);

  ledcWrite(left_pwm_channel, abs(lvel));
  ledcWrite(right_pwm_channel, abs(rvel));
}

void motorBrake(){
  digitalWrite(LEFT_DIS_PIN, HIGH);
  digitalWrite(RIGHT_DIS_PIN, HIGH);

  ledcWrite(left_pwm_channel, 0);
  ledcWrite(right_pwm_channel, 0);
}

void setNeoColor(uint32_t color, int brightness = MAX_BRIGHTNESS){
  NEO.setBrightness(brightness);
  NEO.setPixelColor(0, color);
  NEO.show();
}

void setLEDS(bool led1, bool led2){
  digitalWrite(LED1, led1);
  digitalWrite(LED2, led2);
}

int switchValue(){
  bool sw1 = digitalRead(SW1) == HIGH;
  bool sw2 = digitalRead(SW2) == HIGH;
  
  return (sw1 ? 1 : 0) + (sw2 ? 2 : 0);
}

void statusLED(int option){
  switch (option){
    case 1:
      setLEDS(HIGH, LOW);
      break;
    case 2:
      setLEDS(LOW, HIGH);
      break;
    case 3:
      setLEDS(HIGH, HIGH);
      break;
    default:
      setLEDS(LOW, LOW);
      break;
  }
}

void tracker(int base_speed){
  static int position = 0;
  // calculate the position based on the sensor states
  int weighted_sum = (2) * sensor_states[1] + (1) * sensor_states[2] + (-1) * sensor_states[3] + (-2) * sensor_states[4];
  int sensor_sum = sensor_states[1] + sensor_states[2] + sensor_states[3] + sensor_states[4];
  // check if the sensor sum is 0 to avoid division by zero
  if (sensor_sum != 0) position = weighted_sum / sensor_sum;

  int output = trackerPID.compute(0, position);

  int left_speed = base_speed + output;
  int right_speed = base_speed - output;

  left_speed = constrain(left_speed, -MAX_SPEED, MAX_SPEED);
  right_speed = constrain(right_speed, -MAX_SPEED, MAX_SPEED);

  setMotorSpeed(left_speed, right_speed);
}


void returnToSender(){
  setMotorSpeed(-550, -550);
  delay(350);
  setMotorSpeed(875, -875); // This is the perfect amount of power and time to make a 180
  delay(295);
}

void roll(){
  setMotorSpeed(20, 20);
  delay(50);
  setMotorSpeed(-70, 70);
  delay(180);
  setMotorSpeed(70, 70);
  delay(350);
  setMotorSpeed(-100, 100);
}

void dodge(){
  setMotorSpeed(875, -875);
  delay(int(115));
  setMotorSpeed(735, 735);
  delay(int(400));
}

void startFight(bool flag){
  while (1 && !flag){
    setMotorSpeed(435, 435);
    if (sensor_states[0] == HIGH || sensor_states[5] == HIGH){
      DONE = true;
      return;
    }
  }
}

void startTurn(){
  setMotorSpeed(150, -150);
  delay(200);
  motorBrake();
}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  sensor_timer = timerBegin(3, 80, true);
  timerAttachInterrupt(sensor_timer, &sensorTimerInterruption, true);
  timerAlarmWrite(sensor_timer, 5000, true);
  timerAlarmEnable(sensor_timer);

  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SENSOR_PISO_IZQ, INPUT);
  pinMode(SENSOR_LATERAL_IZQ, INPUT);
  pinMode(SENSOR_FRONTAL_IZQ, INPUT);
  pinMode(SENSOR_FRONTAL_DER, INPUT);
  pinMode(SENSOR_LATERAL_DER, INPUT);
  pinMode(SENSOR_PISO_DER, INPUT);
  // pinMode(SENSOR_PIN6, INPUT);

  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIS_PIN, OUTPUT);

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(LEFT_DIS_PIN, OUTPUT);

  ledcSetup(right_pwm_channel, freq, resolution);
  ledcAttachPin(RIGHT_PWM_PIN, right_pwm_channel);

  ledcSetup(left_pwm_channel, freq, resolution);
  ledcAttachPin(LEFT_PWM_PIN, left_pwm_channel);

  servoFlag.attach(SERVO_PIN);
  servoFlag.write(0);


  NEO.begin();
  NEO.setBrightness(255);
  NEO.show();

  // delay(1500);
  // // returnToSender();
  // dodge();
  // setNeoColor(MAGENTA, MAX_BRIGHTNESS);
  // motorBrake();


}
void loop() {
  int base_speed;
  if (digitalRead(GO) == HIGH){ 
    if (!(sensor_states[0] == HIGH) && !(sensor_states[5] == HIGH)) returnToSender(); // check if the robot is on the line
    startFight(DONE);
    switch_value = switchValue();
    statusLED(switch_value);
    switch (switch_value){
      case 1: // First strategy, based velocity is low
      setNeoColor(BLUE, MAX_BRIGHTNESS);
        base_speed = 535;
        tracker(base_speed);
        break;
      case 2: // Second strategy, based velocity is high
        setNeoColor(GREEN, MAX_BRIGHTNESS);
        base_speed = 800;
        tracker(base_speed);
        break;
      case 3: // Third strategy, when robots face each other
        setNeoColor(YELLOW, MAX_BRIGHTNESS);
        base_speed = 700;
        if (!flag1) dodge();
        flag1 = true;
        tracker(base_speed);
        break;
      default:
        
        break;
    }

  } else if (digitalRead(RDY) == HIGH){
    DONE = false;
    setNeoColor(RED, MAX_BRIGHTNESS);
    motorBrake();
  }
}

