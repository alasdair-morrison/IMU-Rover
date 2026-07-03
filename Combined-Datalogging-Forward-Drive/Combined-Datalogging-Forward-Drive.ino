#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTCDue.h>
#include <Adafruit_HUSB238.h>

Adafruit_HUSB238 husb238;
File DataFile;
RTCDue rtc(XTAL);

// Motor driver pins
#define PWM_A 35
#define in1_A 30
#define in2_A 31
#define in1_B 32
#define in2_B 33
#define PWM_B 37

#define PWM_C 39
#define in1_C 26
#define in2_C 27
#define in1_D 28
#define in2_D 29
#define PWM_D 41

#define PWM_E 44
#define in1_E 25
#define in2_E 24
#define in1_F 23
#define in2_F 22
#define PWM_F 45

// Encoder pins
#define Wheel_A_Channel_A 1
#define Wheel_A_Channel_B 2
#define Wheel_B_Channel_A 3 
#define Wheel_B_Channel_B 4  
#define Wheel_C_Channel_A 5  
#define Wheel_C_Channel_B 6  
#define Wheel_D_Channel_A 7  
#define Wheel_D_Channel_B 8  
#define Wheel_E_Channel_A 9  
#define Wheel_E_Channel_B 11      // NO PIN 10 HEYA
#define Wheel_F_Channel_A 12
#define Wheel_F_Channel_B 13

// IMU UART pins used by the board
#define tx 18
#define rx 19

#define MASTER_CLOCK 84000000

const int sdChipSelect = 10;
const int ledPin = 13;
const unsigned long interval = 10;
const int forwardDrivePwm = 180;

uint32_t pwmClockFrequencyHz = 42000000;
int motorPwmChannelA = 0;
int motorPwmChannelB = 1;
int motorPwmChannelC = 2;
int motorPwmChannelD = 3;
int motorPwmChannelE = 5;
int motorPwmChannelF = 6;

volatile long counts[6] = {0, 0, 0, 0, 0, 0};
int MotorPWM[6] = {0, 0, 0, 0, 0, 0};

unsigned long startTime = 0;
bool started = false;
unsigned long previousMillis = 0;

float Roll = 0;
float Pitch = 0;
float Yaw = 0;
float supplyVoltage = 9.0;
bool serialHeaderSent = false;

void InitializeRTC() {
  rtc.begin();
  rtc.setTime(16, 55, 16);
  rtc.setDate(3, 7, 2026);
}

void FormatRTCTimestamp(char* buffer, size_t bufferSize) {
  snprintf(buffer,
           bufferSize,
           "%04u-%02d-%02d %02d:%02d:%02d",
           rtc.getYear(),
           rtc.getMonth(),
           rtc.getDay(),
           rtc.getHours(),
           rtc.getMinutes(),
           rtc.getSeconds());
}

void PrintLogHeader(Print& output) {
  output.println("RTC_Timestamp,Time_ms,Roll,Pitch,Yaw,M1_Count,M2_Count,M3_Count,M4_Count,M5_Count,M6_Count,V1_PWM,V2_PWM,V3_PWM,V4_PWM,V5_PWM,V6_PWM,V1_Voltage,V2_Voltage,V3_Voltage,V4_Voltage,V5_Voltage,V6_Voltage");
}

void PrintLogRow(Print& output, const char* rtcTimestamp, unsigned long timeStamp, const long* encoderCounts) {
  output.print(rtcTimestamp);
  output.print(",");
  output.print(timeStamp);
  output.print(",");
  output.print(Roll);
  output.print(",");
  output.print(Pitch);
  output.print(",");
  output.print(Yaw);
  output.print(",");

  for (int i = 0; i < 6; i++) {
    output.print(encoderCounts[i]);
    output.print(",");
  }

  for (int i = 0; i < 6; i++) {
    output.print(MotorPWM[i]);
    output.print(",");
  }

  for (int i = 0; i < 6; i++) {
    float motorVoltage = (MotorPWM[i] / 255.0) * supplyVoltage;
    output.print(motorVoltage);
    if (i < 5) {
      output.print(",");
    }
  }

  output.println();
}

void SetPin(uint8_t pin)
{
  PIO_Configure(g_APinDescription[pin].pPort,
                PIO_PERIPH_B,
                g_APinDescription[pin].ulPin,
                g_APinDescription[pin].ulPinConfiguration);
}

void setPWM(int channel) {
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(pwmClockFrequencyHz, 0, MASTER_CLOCK);
  PWMC_ConfigureChannelExt(PWM,
                           channel,
                           PWM_CMR_CPRE_CLKA,
                           0,
                           0,
                           0,
                           PWM_CMR_DTE,
                           0,
                           0);
  PWMC_SetPeriod(PWM, channel, 1200);
  PWMC_SetDutyCycle(PWM, channel, 600);
  PWMC_SetDeadTime(PWM, channel, 42, 42);
  PWMC_EnableChannel(PWM, channel);
}

void ApplyMotorPWM() {
  int pwmChannels[6] = {motorPwmChannelA, motorPwmChannelB, motorPwmChannelC, motorPwmChannelD, motorPwmChannelE, motorPwmChannelF};
  for (int i = 0; i < 6; i++) {
    int duty = (int)map(MotorPWM[i], 0, 255, 0, 1200);
    PWMC_SetDutyCycle(PWM, pwmChannels[i], duty);
  }
}

void ISR_MOTOR_1() {
  if (digitalRead(Wheel_A_Channel_A) == digitalRead(Wheel_A_Channel_B)) {
    counts[0]--;
  } else {
    counts[0]++;
  }
}

void ISR_MOTOR_2() {
  if (digitalRead(Wheel_B_Channel_A) == digitalRead(Wheel_B_Channel_B)) {
    counts[1]--;
  } else {
    counts[1]++;
  }
}

void ISR_MOTOR_3() {
  if (digitalRead(Wheel_C_Channel_A) == digitalRead(Wheel_C_Channel_B)) {
    counts[2]--;
  } else {
    counts[2]++;
  }
}

void ISR_MOTOR_4() {
  if (digitalRead(Wheel_D_Channel_A) == digitalRead(Wheel_D_Channel_B)) {
    counts[3]--;
  } else {
    counts[3]++;
  }
}

void ISR_MOTOR_5() {
  if (digitalRead(Wheel_E_Channel_A) == digitalRead(Wheel_E_Channel_B)) {
    counts[4]--;
  } else {
    counts[4]++;
  }
}

void ISR_MOTOR_6() {
  if (digitalRead(Wheel_F_Channel_A) == digitalRead(Wheel_F_Channel_B)) {
    counts[5]--;
  } else {
    counts[5]++;
  }
}

void ReadIMU() {
  if (Serial1.available() >= 11) {
    if (Serial1.read() == 0x55) {
      byte type = Serial1.read();
      if (type == 0x53) {
        int16_t rRaw = (Serial1.read() | (Serial1.read() << 8));
        int16_t pRaw = (Serial1.read() | (Serial1.read() << 8));
        int16_t yRaw = (Serial1.read() | (Serial1.read() << 8));

        for (int i = 0; i < 3; i++) {
          Serial1.read();
        }

        Roll = (float)rRaw / 32768.0 * 180.0;
        Pitch = (float)pRaw / 32768.0 * 180.0;
        Yaw = (float)yRaw / 32768.0 * 180.0;

        if (abs(Roll) > 180 || abs(Pitch) > 180 || abs(Yaw) > 180) {
          return;
        }
      }
    }
  }
}

void EnsureMotorPower() {
  HUSB238_VoltageSetting currentVoltage = husb238.getPDSrcVoltage();
  if (currentVoltage != PD_9V) {
    husb238.selectPD(PD_SRC_9V);
    husb238.requestPD();
    delay(200);
    supplyVoltage = 9.0;
  }
}

void LogData(unsigned long TimeStamp) {
  long Safe_Encoder_Counts[6];
  char rtcTimestamp[32];
  FormatRTCTimestamp(rtcTimestamp, sizeof(rtcTimestamp));

  noInterrupts();
  for (int i = 0; i < 6; i++) {
    Safe_Encoder_Counts[i] = counts[i];
  }
  interrupts();

  if (DataFile) {
    digitalWrite(ledPin, HIGH);
    PrintLogRow(DataFile, rtcTimestamp, TimeStamp, Safe_Encoder_Counts);
    digitalWrite(ledPin, LOW);
  }

  if (Serial) {
    if (!serialHeaderSent) {
      PrintLogHeader(Serial);
      serialHeaderSent = true;
    }
    PrintLogRow(Serial, rtcTimestamp, TimeStamp, Safe_Encoder_Counts);
  } else {
    serialHeaderSent = false;
  }

  static unsigned long lastClose = 0;
  if (TimeStamp - lastClose >= 1000) {
    if (DataFile) {
      DataFile.flush();
    }
    lastClose = TimeStamp;
  }

  static int serialPrintCounter = 0;
  serialPrintCounter++;
  if (serialPrintCounter >= 50) {
    serialPrintCounter = 0;
    Serial.println();
    Serial.print("   Time [ms]:  ");
    Serial.print(TimeStamp);
    Serial.println();

    Serial.print("  Encoder Counts:  ");
    Serial.print("   A: ");
    Serial.print(Safe_Encoder_Counts[0]);
    Serial.print("   |    B: ");
    Serial.print(Safe_Encoder_Counts[1]);
    Serial.print("   |   C: ");
    Serial.print(Safe_Encoder_Counts[2]);
    Serial.print("   |   D: ");
    Serial.print(Safe_Encoder_Counts[3]);
    Serial.print("   |   E: ");
    Serial.print(Safe_Encoder_Counts[4]);
    Serial.print("   |   F: ");
    Serial.print(Safe_Encoder_Counts[5]);
    Serial.println();

    Serial.print("  IMU Euler Angles:  ");
    Serial.print(" R: ");
    Serial.print(Roll);
    Serial.print("   |    P: ");
    Serial.print(Pitch);
    Serial.print("   |   Y: ");
    Serial.print(Yaw);
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  InitializeRTC();

  pinMode(ledPin, OUTPUT);

  pinMode(Wheel_A_Channel_A, INPUT_PULLUP);
  pinMode(Wheel_A_Channel_B, INPUT_PULLUP);
  pinMode(Wheel_B_Channel_A, INPUT_PULLUP);
  pinMode(Wheel_B_Channel_B, INPUT_PULLUP);
  pinMode(Wheel_C_Channel_A, INPUT_PULLUP);
  pinMode(Wheel_C_Channel_B, INPUT_PULLUP);
  pinMode(Wheel_D_Channel_A, INPUT_PULLUP);
  pinMode(Wheel_D_Channel_B, INPUT_PULLUP);
  pinMode(Wheel_E_Channel_A, INPUT_PULLUP);
  pinMode(Wheel_E_Channel_B, INPUT_PULLUP);
  pinMode(Wheel_F_Channel_A, INPUT_PULLUP);
  pinMode(Wheel_F_Channel_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Wheel_A_Channel_A), ISR_MOTOR_1, RISING);
  attachInterrupt(digitalPinToInterrupt(Wheel_B_Channel_A), ISR_MOTOR_2, RISING);
  attachInterrupt(digitalPinToInterrupt(Wheel_C_Channel_A), ISR_MOTOR_3, RISING);
  attachInterrupt(digitalPinToInterrupt(Wheel_D_Channel_A), ISR_MOTOR_4, RISING);
  attachInterrupt(digitalPinToInterrupt(Wheel_E_Channel_A), ISR_MOTOR_5, RISING);
  attachInterrupt(digitalPinToInterrupt(Wheel_F_Channel_A), ISR_MOTOR_6, RISING);

  pinMode(in1_A, OUTPUT);
  pinMode(in2_A, OUTPUT);
  pinMode(in1_B, OUTPUT);
  pinMode(in2_B, OUTPUT);
  pinMode(in1_C, OUTPUT);
  pinMode(in2_C, OUTPUT);
  pinMode(in1_D, OUTPUT);
  pinMode(in2_D, OUTPUT);
  pinMode(in1_E, OUTPUT);
  pinMode(in2_E, OUTPUT);
  pinMode(in1_F, OUTPUT);
  pinMode(in2_F, OUTPUT);

  digitalWrite(in1_A, HIGH);
  digitalWrite(in2_A, LOW);
  digitalWrite(in1_B, HIGH);
  digitalWrite(in2_B, LOW);
  digitalWrite(in1_C, HIGH);
  digitalWrite(in2_C, LOW);
  digitalWrite(in1_D, HIGH);
  digitalWrite(in2_D, LOW);
  digitalWrite(in1_E, HIGH);
  digitalWrite(in2_E, LOW);
  digitalWrite(in1_F, HIGH);
  digitalWrite(in2_F, LOW);

  SetPin(PWM_A);
  SetPin(PWM_B);
  SetPin(PWM_C);
  SetPin(PWM_D);
  SetPin(PWM_E);
  SetPin(PWM_F);

  setPWM(motorPwmChannelA);
  setPWM(motorPwmChannelB);
  setPWM(motorPwmChannelC);
  setPWM(motorPwmChannelD);
  setPWM(motorPwmChannelE);
  setPWM(motorPwmChannelF);

  if (!husb238.begin(HUSB238_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("HUSB238 Init Failed. Check wiring.");
    while (1) {
      delay(10);
    }
  }
  husb238.selectPD(PD_SRC_9V);
  husb238.requestPD();
  delay(500);

  if (!SD.begin(sdChipSelect)) {
    while (1) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
  }

  byte imuWakeCommand[] = {0xFF, 0xAA, 0x03, 0x03, 0x00};
  Serial1.write(imuWakeCommand, 5);

  DataFile = SD.open("LightWheels_Rover_Combined.csv", FILE_WRITE);
  if (!DataFile) {
    digitalWrite(ledPin, HIGH);
    return;
  }

  PrintLogHeader(DataFile);
}

void loop() {
  unsigned long currentMillis = millis();

  if (!started) {
    startTime = currentMillis;
    started = true;
  }

  EnsureMotorPower();

  for (int i = 0; i < 6; i++) {
    MotorPWM[i] = forwardDrivePwm;
  }

  ApplyMotorPWM();
  ReadIMU();

  if (currentMillis - previousMillis >= interval) {
    previousMillis += interval;
    LogData(currentMillis);
  }
}