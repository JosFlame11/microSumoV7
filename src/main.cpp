#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define SW1 35
#define SW2 37

#define LED1 5
#define LED2 8
#define SERVO_PIN 17
#define SENSOR_PIN1 11
#define SENSOR_PIN2 9
#define SENSOR_PIN3 10
#define SENSOR_PIN4 7
#define SENSOR_PIN5 6
#define SENSOR_PIN6 4
#define SENSOR_PIN7 3

#define RIGHT_PWM_PIN 1
#define RIGHT_DIR_PIN 12
#define RIGHT_DIS_PIN 14

#define LEFT_PWM_PIN 2
#define LEFT_DIR_PIN 13
#define LEFT_DIS_PIN 14

#define GO 21
#define RDY 18

#define MAX_BRIGHTNESS 50

uint8_t sensor_states[7];
const int freq = 10000;
const int resolution = 8;
const int right_pwm_channel = 0;
const int left_pwm_channel = 1;

uint8_t switch_value = 0;

// float kp = 1.0;
// float ki = 0.0;
// float kd = 0.0; 

bool DONE = false;
bool DONE2 = false;

bool flag1 = false;
bool flag2 = false;
bool flag3 = false;
bool flag4 = false;
bool flag5 = false;


Adafruit_NeoPixel NEO = Adafruit_NeoPixel(1, 38, NEO_GRB + NEO_KHZ800);

hw_timer_t * sensor_timer = NULL;

// @todo: check which sensors are actually used
void IRAM_ATTR sensorTimerInterruption(){
  sensor_states[0] = digitalRead(SENSOR_PIN1);
  sensor_states[1] = digitalRead(SENSOR_PIN2);
  sensor_states[2] = digitalRead(SENSOR_PIN3);
  sensor_states[3] = digitalRead(SENSOR_PIN4);
  sensor_states[4] = digitalRead(SENSOR_PIN5);
  sensor_states[5] = digitalRead(SENSOR_PIN6);
  sensor_states[6] = digitalRead(SENSOR_PIN7);
}



void setMotorSpeed(int lvel, int rvel){
  lvel = constrain(lvel, -255, 255);
  rvel = constrain(rvel, -255, 255);

  digitalWrite(LEFT_DIS_PIN, LOW);
  digitalWrite(RIGHT_DIS_PIN, LOW);

  (lvel >= 0) ? digitalWrite(LEFT_DIR_PIN, HIGH) : digitalWrite(LEFT_DIR_PIN, LOW);
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

void setNeoColor(int r, int g, int b, int brightness){
  NEO.setBrightness(brightness);
  NEO.setPixelColor(0, NEO.Color(r, g, b));
  NEO.show();
}

void setLEDS(bool led1, bool led2){
  digitalWrite(LED1, led1);
  digitalWrite(LED2, led2);
}

int switchValue(){
  if (digitalRead(SW1) == HIGH && digitalRead(SW2) == LOW){
    return 1;
  } 
  else if (digitalRead(SW2) == HIGH && digitalRead(SW1) == LOW){
    return 2;
  }
  else if (digitalRead(SW1) == HIGH && digitalRead(SW2) == HIGH){
    return 3;
  }
  else{
    return 0;
  }
}

void PIDCompute(float kp = 5.0, float ki = 0.0, float kd = 2.0, int max_speed = 150){
  static int last_error = 0;
  static int i_error = 0;

  int wsum = -10 * sensor_states[2] + -5 * sensor_states[3] + 5 * sensor_states[4] + 10 * sensor_states[5];
  int sum  = sensor_states[2] + sensor_states[3] + sensor_states[4] + sensor_states[5];

  int position = wsum / sum;

  int error = 0 - position;

  int p = kp * (float)error;

  i_error = i_error + error;

  int i = ki * (float)i_error;

  int d = kd * (float)(error - last_error);

  int out = p + i + d;

  last_error = error;

  int PWML = constrain(max_speed - out, -255, 255);
  int PWMR = constrain(max_speed + out, -255, 255);
 

  setMotorSpeed(PWML, PWMR);
}

void returnToSender(){
  setMotorSpeed(-120, -120);
  delay(200);
  setMotorSpeed(125, -100);
  delay(65);
  motorBrake();
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
  setMotorSpeed(80, -80);
  delay(140);
  setMotorSpeed(45, 110);
  delay(450);
  setMotorSpeed(70, 110);
  delay(120);
  setMotorSpeed(-80, 80);
  delay(180);
  motorBrake();
}

void startFight(){
  while (1){
    setMotorSpeed(50, 50);
    if (sensor_states[0] == HIGH || sensor_states[1] == HIGH){
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
  pinMode(SENSOR_PIN1, INPUT);
  pinMode(SENSOR_PIN2, INPUT);
  pinMode(SENSOR_PIN3, INPUT);
  pinMode(SENSOR_PIN4, INPUT);
  pinMode(SENSOR_PIN5, INPUT);
  pinMode(SENSOR_PIN6, INPUT);
  pinMode(SENSOR_PIN7, INPUT);

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

  NEO.begin();
  NEO.setBrightness(255);
  NEO.show();

}
void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(GO) == HIGH){
    setNeoColor(0, 255, 0, MAX_BRIGHTNESS);
    if (!(sensor_states[0] == HIGH) && !(sensor_states[1] == HIGH)) motorBrake();
    
    setLEDS(HIGH, LOW);

    switch_value = switchValue();
    switch (switch_value){
      case 1: //Task 1
        /* code */
        setNeoColor(88, 255, 127, MAX_BRIGHTNESS);
        break;
      case 2: //Task 2
        /* code */
        setNeoColor(255, 0, 255, MAX_BRIGHTNESS);
        break;
      case 3: //Task 3
        /* code */
        setNeoColor(0, 0, 255, MAX_BRIGHTNESS);
        break;
      default:
        setNeoColor(127, 100, 200, MAX_BRIGHTNESS);
        break;
    }

  } else if (digitalRead(RDY) == HIGH){
    motorBrake();
    setNeoColor(255, 0, 0, MAX_BRIGHTNESS);
    setLEDS(LOW, HIGH);
  }
}

