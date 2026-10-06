#include <PID_v2.h>

#define PPR 400.0 
#define ENC_A 2
#define ENC_B 3

#define RPWM 5
#define LPWM 6

unsigned long dt = 20;

volatile long encoderCount = 0;

volatile int lastEncoderState = 0;

long lastEncoderCount = 0;

double SpeedInput = 0;         //ANLIK HIZI ALCAZ
double SpeedOutput = 0;        // PWM ÇIKIŞ HIZ
double SpeedSetpoint = 235.0;  //bu RPM


double Speed_Kp = 0.3;
double Speed_Ki = 0.0001;
double Speed_Kd = 0.0;

PID SpeedPID(&SpeedInput, &SpeedOutput, &SpeedSetpoint, Speed_Kp, Speed_Ki, Speed_Kd, DIRECT);

unsigned long previousMillis = 0;
unsigned long SerialMillis = 0;

void setup() {
  Serial.begin(9600);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);


  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, CHANGE);

  SpeedPID.SetMode(AUTOMATIC);
  SpeedPID.SetSampleTime(dt);
  SpeedPID.SetOutputLimits(-255, 255);
}

void loop() {
  unsigned long currentMillis = millis();


  if (currentMillis - previousMillis >= dt) {


    long currentEncoderCount = encoderCount;


    long palsDelta = currentEncoderCount - lastEncoderCount;   // 20 msde gelen encoder sayısı
    unsigned long timeDelta = currentMillis - previousMillis;  // timeDelta = interval yani 20ms fln

    SpeedInput = abs((palsDelta * 3000) / PPR); // 20 ms de gecen encoderi 60000 ms (1dk) gelene cevirdik buda rpm degerini verdi

    lastEncoderCount = currentEncoderCount;
    previousMillis = currentMillis;


    SpeedPID.Compute();
  }



  if (SpeedOutput > 0) {

    analogWrite(RPWM, SpeedOutput);
    analogWrite(LPWM, 0);
  } else if (SpeedOutput < 0) {

    analogWrite(LPWM, abs(SpeedOutput));
    analogWrite(RPWM, 0);
  } else {

    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }

  // Hoca kodu birkaç farklı zamanda çalıştırmak mantıklı dedi ondan böyle
  if (currentMillis - SerialMillis >= 100) {
    Serial.print("SP: ");
    Serial.print(SpeedSetpoint);
    Serial.print(", PV: ");
    Serial.print(SpeedInput);
    Serial.print(", Output: ");
    Serial.println(SpeedOutput);

    SerialMillis = currentMillis;
  }
}

void updateEncoder() {
  int MSB = digitalRead(ENC_A);
  int LSB = digitalRead(ENC_B);
  int currentEncoderState = (MSB << 1) | LSB;
  int sum = (lastEncoderState << 2) | currentEncoderState;

  if (sum == 0b0110 || sum == 0b1001) encoderCount++;
  if (sum == 0b0001 || sum == 0b1110) encoderCount--;

  lastEncoderState = currentEncoderState; //kodun çalışmama sebebi benim mal olup buranın önceki encoder degerini yukarda kullanmammış
}
