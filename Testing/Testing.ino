#include "SD.h"
#include <Wire.h>
#include "RTClib.h"

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

// Encoders (SKIP PIN 10)
#define whA_B 13 // Wheel A output B
#define whA_A 12 // Wheel A output A
#define whB_B 11 // Wheel B output B
#define whB_A 9 // wheel B output A
#define whC_B 8 // Wheel C output B
#define whC_A 7 // wheel C output A
#define whD_B 6 // wheel D output B
#define whD_A 5 // wheel D output A
#define whE_B 4 // wheel E output B
#define whE_A 3 // wheel E output A
#define whF_B 2 // wheel F output B
#define whF_A 1 // wheel F output A

//IMU
#define tx 14
#define rx 15

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

RTC_DS1307 myRTC; // define the Real Time Clock object

const int chipSelect = 10;

// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  while(1);
}

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
  if (!myRTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }
  

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
