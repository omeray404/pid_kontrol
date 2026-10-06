#include "PID_v2.h"

#define ENC_A 3
#define ENC_B 2
#define RPWM 5
#define LPWM 6
volatile long encoderCount = 0;
int lastEncoder = 0;

double PosInput = 0,PosOutput = 0, PosSetpoint = 0, absOutput= 0;

double Pos_Kp = 0.464510;
double Pos_Ki = 0.00001;
double Pos_Kd = 0.0;

PID PosPID(&PosInput, &PosOutput, &PosSetpoint, Pos_Kp, Pos_Ki, Pos_Kd, DIRECT);

unsigned long currentMillis=0;
unsigned long previousMillis=0;
unsigned long interval=20;

void setup() {
  Serial.begin(9600);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(5, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, CHANGE);
  
  PosPID.SetMode(AUTOMATIC);
  PosPID.SetSampleTime(interval);
  PosPID.SetOutputLimits(-255, 255);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print(PosSetpoint); Serial.print(",");
  
    Serial.print(encoderCount); Serial.print(",");

    Serial.println(PosOutput);

    PosInput = encoderCount;
    PosSetpoint = 4500;

    PosPID.Compute();
    if (PosOutput < 0) 
    {
      absOutput = abs(PosOutput);
      analogWrite(5, absOutput);
      analogWrite(6, 0);
    } 
    else if (PosOutput > 0) 
    {
      analogWrite(6, PosOutput);
      analogWrite(5, 0);
    } 
    else 
    {
      analogWrite(5, 0);
      analogWrite(6, 0);
    }

  }
}
void updateEncoder() {
  int MSB = digitalRead(ENC_A);
  int LSB = digitalRead(ENC_B);
  int encoder = (MSB << 1) | LSB;
  int sum = (lastEncoder << 2) | encoder;

  if (sum == 0b0010 || sum == 0b1001) encoderCount++;
  if (sum == 0b0001 || sum == 0b0111) encoderCount--;

  lastEncoder = encoder;
}
