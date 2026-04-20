#include "SD.h"
#include <Wire.h>
#include "RTClib.h"
#include "RTCDue.h"

//Front Wheels
#define PWM_A 35 //PWM Out Motor A
#define in1_A 28 //Enable 1 Motor A
#define in2_A 29 //Enable 2 Motor A
#define in1_B 30 // Enable 1 Motor B
#define in2_B 31 // Enable 2 Motor B
#define PWM_B 37 // PWM Out Motor B
//Middle Wheels
#define PWM_C 39 // PWM Out Motor C
#define in1_C 24 //Enable 1 Motor C
#define in2_C 25 // Enable 2 Motor C
#define in1_D 26 // Enable 1 Motor D
#define in2_D 27 // Enable 2 Motor D
#define PWM_D 41 // PWM Out Motor D
//Rear Wheels
#define PWM_E 44 // PWM Out Motor E
#define in1_E 23 // Enable 1 Motor E
#define in2_E 22 // Enable 2 Motor E
#define in1_F 21 // Enable 1 Motor F
#define in2_F 20 // Enable 2 Motor F
#define PWM_F 45 // PWM Out Motor F


// ENCODER PINS ;0
#define Wheel_A_Channel_A 50  
#define Wheel_A_Channel_B 51
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

// IMU PINS ;)
#define tx 18
#define rx 19

// 1 XD: ENCODER HAKI (6 MOTORS)
// use volatile so brain processor sees every tiny movement instantly by checking the pins hardware since physcial changes happen
volatile long counts[6] = {0, 0, 0, 0, 0, 0};  // Stores pulse counts for 6 encoders 

// 2 XD: MOTOR HAKI
int PWM_Channel[6] = {0,1,2,3,5,6}; // chA to chF
// Add Global Variable
unsigned long startTime = 0;
bool started = false;

// Stores input commands (0–255) :) Used for logging later (input to output comparison)
int MotorPWM[6] = {0, 0, 0, 0, 0, 0};  // Current SPEED being sent to each motor from a range of 0 to 255

// CONVERT MOTOR PWM TO HARDWARE PWM
void ApplyMotorPWM(){
    for (int i =0; i<6; i++){
        //Convert 0-255 to 0-1200 (BUT INVERTED!!!!!!!!!!!!)
        int DUTY = (int) map(MotorPWM[i], 0, 255, 1200, 0);  // cuz map returns long

        PWMC_SetDutyCycle(PWM, PWM_Channel[i], DUTY);
    }

#define MASTER_CLOCK 84000000

uint32_t clock_a = 42000000; // Sampling frequency in Hz
int chA = 0;
int chB = 1;
int chC = 2;
int chD = 3; 
int chE = 5;
int chF = 6;

void SetPin(uint8_t pin)
{
  PIO_Configure(g_APinDescription[pin].pPort,
                PIO_PERIPH_B, 
                g_APinDescription[pin].ulPin,
                g_APinDescription[pin].ulPinConfiguration);

}

void setPWM(int channel) {
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(clock_a, 0, MASTER_CLOCK); // clock_b = 0
  PWMC_ConfigureChannelExt(PWM,
                           channel, // Channel: variable        
                           PWM_CMR_CPRE_CLKA, // Prescaler: use CLOCK_A
                           0, // Aligment: period is left aligned
                           0, // Polarity: output waveform starts at a low level
                           0, // Counter event: occurs at the end of the period
                           PWM_CMR_DTE, // Dead time generator is enabled
                           0, // Dead time PWMH output is not inverted    
                           0);  // Dead time PWML output is not inverted
  PWMC_SetPeriod(PWM, channel, 1200); // Channel: variable, Period: 1/(1200/42 Mhz) = ~35 kHz
  PWMC_SetDutyCycle(PWM, channel, 600); // Channel: variable, Duty cycle: 50 %
  PWMC_SetDeadTime(PWM, channel, 42, 42); // Channel: variable, Rising and falling edge dead time: 42/42 Mhz = 1 us
  PWMC_EnableChannel(PWM, channel); // Channel: variable
}

#define LOG_INTERVAL  1000 // mills between entries (1000 = 1 second)
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

RTCDue rtc(XTAL);

const int chipSelect = 10;

// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  while(1);
}

void ReadIMU() {
    // Wait for/Check if we have a full 11-byte packet
    if (Serial1.available() >= 11){
        if (Serial1.read() == 0x55){ // Check header
            byte type = Serial1.read();
            if (type == 0x53) { // Means Euler angles
                // Extract raw data, reads two bytes from Serial1 and combines them into a single 16‑bit integer (int16_t)
                int16_t rRaw = (Serial1.read() | (Serial1.read() << 8));
                // Serial1.read() returns one byte (0–255). The first read() becomes the low byte.
                int16_t pRaw = (Serial1.read() | (Serial1.read() << 8));
                //The second read() is shifted left 8 bits (<< 8), making it the high byte.the | (bitwise OR) combines them into a single 16‑bit value.
                int16_t yRaw = (Serial1.read() | (Serial1.read() << 8)); // This is little‑endian byte order (low byte first, high byte second).

                // Discard temperature and checksum, the last 3 bytes i think 
                for(int i=0; i<3; i++) Serial1.read();

                // Convert to human degrees
                Roll = (float)rRaw / 32768.0 * 180.0;
                Pitch = (float)pRaw / 32768.0 * 180.0;
                Yaw = (float)yRaw / 32768.0 * 180.0;
                
                //Reject invalid data
                if (abs(Roll) > 180 || abs(Pitch) > 180 || abs(Yaw) > 180){
                  return;
                }
              } 
          }
      }
  }

void LogData(unsigned long TimeStamp) {
    long Safe_Encoder_Counts[6];
    // Prevents ISR from modifying data mid-copy
    noInterrupts();
    for(int i = 0; i < 6; i++){
        Safe_Encoder_Counts[i] = counts[i];
    }
    interrupts();

    if (DataFile) {
        digitalWrite(ledPin, HIGH);  // GREEN LIGHT: LOGGING ACTIVE!
        //LOG DATA WOHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
        DataFile.print(TimeStamp); DataFile.print(",");
        DataFile.print(Roll); DataFile.print(",");
        DataFile.print(Pitch); DataFile.print(",");
        DataFile.print(Yaw); DataFile.print(",");

        // Loop for encoder values: Log all 6 encoder counts
        for(int i = 0; i < 6; i++){
            DataFile.print(Safe_Encoder_Counts[i]);
            DataFile.print(",");             // a comma to seperate data
        }

        // Loop for PWM values: Log THE PWM / Voltage INPUTS FOR ALL 6 motors
        float supplyVoltage = 7.0;
        for(int i = 0; i < 6; i++){
            DataFile.print(MotorPWM[i]);
            DataFile.print(",");
        }
        for(int i = 0; i < 6; i++){
            float motorVoltage = (MotorPWM[i] / 255.0) * supplyVoltage;
            DataFile.print(motorVoltage);
            if(i < 5) DataFile.print(",");
        }

        // Finish row
        DataFile.println();
        digitalWrite(ledPin, LOW);   // LIGHT OFF: logged!
    } 
    
    // SD card flushing
    static unsigned long lastClose = 0;
    if (TimeStamp - lastClose >= 1000){  // every 1 sec
        if (DataFile){
          DataFile.flush();  // Prevents data loss if power cuts
        }
        lastClose = TimeStamp;
    }

    static int chill_serial_bro = 0;
    chill_serial_bro++;

  if(chill_serial_bro >= 10){               // every 100 ms to prevent spam printing lol 
    chill_serial_bro = 0;
    // Real-Time
    Serial.print("  |  Time: ");
    Serial.print(TimeStamp);
    // ENCODER COUNTS
    /*Serial.print("M1: ");
    Serial.print(Safe_Encoder_Counts[0]);
    Serial.print("   |   M2: ");
    Serial.print(Safe_Encoder_Counts[1]);
    Serial.print("   |   M3: ");
    Serial.print(Safe_Encoder_Counts[2]);
    Serial.print("   |   M4: ");
    Serial.print(Safe_Encoder_Counts[3]);
    Serial.print("   |   M5: ");
    Serial.print(Safe_Encoder_Counts[4]);
    Serial.print("   |   M6: ");
    Serial.print(Safe_Encoder_Counts[5]);
    Serial.println();       // new line*/
    // IMU ANGLES
    Serial.print("   |   R: ");
    Serial.print(Roll);
    Serial.print("   |   P: ");
    Serial.print(Pitch);
    Serial.print("   |   Y: ");
    Serial.print(Yaw);  
    Serial.println();       // new line*/


      // CALCULATE VOLTAGE_IN FROM PWM
  float supplyVoltage = 7.0;
  for(int i = 0; i < 6; i++){
      float motorVoltage = (MotorPWM[i] / 255.0) * supplyVoltage;
      // PRINT PWM
      Serial.print("   |   PWM: ");
      Serial.print(MotorPWM[i]);
      Serial.print(" ");
      // VOLTAGE_IN MOTORS
      Serial.print("    |   Motor_Voltage: ");
      Serial.print(motorVoltage);
   }
  }
}


// TURN TEST: Spin in place for 10 seconds
if (currentMillis - startTime < 10000){
    // Left side forward
    MotorPWM[0] = 120;
    MotorPWM[2] = 120;
    MotorPWM[4] = 120;

    // Right side backward (invert direction)
    MotorPWM[1] = 120;
    MotorPWM[3] = 120;
    MotorPWM[5] = 120;

    // 🔥 Flip direction pins for right side
    digitalWrite(30, LOW); digitalWrite(31, HIGH);
    digitalWrite(26, LOW); digitalWrite(27, HIGH);
    digitalWrite(25, LOW); digitalWrite(24, HIGH);
}
else {
    for (int i = 0; i < 6; i++){
        MotorPWM[i] = 0;
    }
}

// Interrupt ISR Functions so it makes it count pulses at lightfoot speed mode , //Each encoder signal connected to a pin, These pins will trigger interrupts
void ISR_MOTOR_1() {   // i'm measuring position (pulse accumulation)
  if (digitalRead(Wheel_A_Channel_A) == digitalRead(Wheel_A_Channel_B)){
    counts[0]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[0]++;
  }
}
void ISR_MOTOR_2() {
  if (digitalRead(Wheel_B_Channel_A) == digitalRead(Wheel_B_Channel_B)){
    counts[1]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[1]++;
  }
}
void ISR_MOTOR_3() {
  if (digitalRead(Wheel_C_Channel_A) == digitalRead(Wheel_C_Channel_B)){
    counts[2]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[2]++;
  }
}
void ISR_MOTOR_4() {
  if (digitalRead(Wheel_D_Channel_A) == digitalRead(Wheel_D_Channel_B)){
    counts[3]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[3]++;
  }
}
void ISR_MOTOR_5() {
  if (digitalRead(Wheel_E_Channel_A) == digitalRead(Wheel_E_Channel_B)){
    counts[4]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[4]++;
  }
}
void ISR_MOTOR_6() {
  if (digitalRead(Wheel_F_Channel_A) == digitalRead(Wheel_F_Channel_B)){
    counts[5]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[5]++;
  }
}

// 2 ;) IMU & LOGGING GEAR
const int chipselect =  10;
const int ledPin = 13; // Status LED
// Timing system, Logs every 10 ms → 100 Hz
unsigned long previousMillis = 0;
const long interval = 10; // 10 ms = 100 Hz frequency babyyy
// IMU orientation values
float Roll = 0, Pitch = 0, Yaw = 0;

void setup() {
  //Datalogger Setup
    Serial.begin(9600);
  Serial.println();
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START
  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);
  Wire.begin();  
  rtc.begin();
  rtc.setTime(0, 16, 15); // 15:16:00
  rtc.setDate(19, 3, 2026); // 19th March 2026
  
  logfile.println("RTC Initialized");
#if ECHO_TO_SERIAL
  Serial.println("RTC Initialized");
#endif
  logfile.println("millis,time,light,temp");    
#if ECHO_TO_SERIAL
  Serial.println("millis,time,light,temp");
#endif
#if ECHO_TO_SERIAL// attempt to write out the header to the file
  logfile.flush(); // This saves the data to the SD card
#endif
  //Front Wheel Setup
  pinMode(in1_A, OUTPUT);
  pinMode(in2_A, OUTPUT);
  pinMode(in1_B, OUTPUT);
  pinMode(in2_B, OUTPUT);
  // Set initial rotation direction for forwards
  digitalWrite(in1_A, HIGH);
  digitalWrite(in2_A, LOW);
  digitalWrite(in1_B, HIGH);
  digitalWrite(in2_B, LOW);

  //Middle Wheel Setup
  pinMode(in1_C, OUTPUT);
  pinMode(in2_C, OUTPUT);
  pinMode(in1_D, OUTPUT);
  pinMode(in2_D, OUTPUT);
  // Set initial rotation direction for forwards
  digitalWrite(in1_C, HIGH);
  digitalWrite(in2_C, LOW);
  digitalWrite(in1_D, HIGH);
  digitalWrite(in2_D, LOW);

  //Rear Wheel Setup
  pinMode(in1_E, OUTPUT);
  pinMode(in2_E, OUTPUT);
  pinMode(in1_F, OUTPUT);
  pinMode(in2_F, OUTPUT);
  // Set initial rotation direction for forwards
  digitalWrite(in1_E, HIGH);
  digitalWrite(in2_E, LOW);
  digitalWrite(in1_F, HIGH);
  digitalWrite(in2_F, LOW);

  SetPin(PWM_A); // PWMH0
  SetPin(PWM_B); // PWMH1
  SetPin(PWM_C); // PWMH2
  SetPin(PWM_D); // PWMH3
  SetPin(PWM_E); // PWMH5
  SetPin(PWM_F); // PWMH6
  
  setPWM(chA);
  setPWM(chB);
  setPWM(chC);
  setPWM(chD);
  setPWM(chE);
  setPWM(chF);

  pinMode(whA_B, INPUT);
  pinMode(whA_B, INPUT);
  pinMode(whA_A, INPUT);
  pinMode(whB_B, INPUT);
  pinMode(whB_A, INPUT);
  pinMode(whC_B, INPUT);
  pinMode(whC_A, INPUT);
  pinMode(whD_B, INPUT);
  pinMode(whD_A, INPUT);
  pinMode(whE_B, INPUT);
  pinMode(whE_A, INPUT);
  pinMode(whF_B, INPUT);
  pinMode(whF_A, INPUT);

  Serial.begin(9600);  // Laptop
  Serial1.begin(9600); // IMU (WT901C)
  // LED setup
  pinMode(ledPin, OUTPUT);

  // Initialise Encoders with pullups + Sets pins as inputs, Enables internal pull-up resistors, Prevents floating signals
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

  //Links pin to ISR, RISING = trigger on rising   edges_
  attachInterrupt(digitalPinToInterrupt(Wheel_A_Channel_A), ISR_MOTOR_1, RISING); // CHANGE IS doubling resolution  BUT increasing noise sensitivity
  attachInterrupt(digitalPinToInterrupt(Wheel_B_Channel_A), ISR_MOTOR_2, RISING);
  attachInterrupt(digitalPinToInterrupt(Wheel_C_Channel_A), ISR_MOTOR_3, RISING);
  attachInterrupt(digitalPinToInterrupt(Wheel_D_Channel_A), ISR_MOTOR_4, RISING);
  attachInterrupt(digitalPinToInterrupt(Wheel_E_Channel_A), ISR_MOTOR_5, RISING);
  attachInterrupt(digitalPinToInterrupt(Wheel_F_Channel_A), ISR_MOTOR_6, RISING);
}

void loop() {
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
  // delay(100); 
  // PWMC_SetDutyCycle(PWM, chA, 900); // Channel: 0, Duty cycle: 25 %
  // delay(100);
  // PWMC_SetDutyCycle(PWM, chA, 600); // Channel: 0, Duty cycle: 50 %
  // delay(100);
  // PWMC_SetDutyCycle(PWM, chA, 300); // Channel: 0, Duty cycle: 75 %
  delay(100);
  PWMC_SetDutyCycle(PWM, chA, 0); // Channel: 0, Duty cycle: 100 %
  delay(100);
  PWMC_SetDutyCycle(PWM, chA, 600); //channel: 0, duty cycle: 50%
}
