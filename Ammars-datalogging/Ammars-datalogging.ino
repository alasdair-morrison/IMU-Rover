#include <SPI.h>  // needed for SD card communication
#include <SD.h>  // SD card library
File DataFile; // global file object so all functions can write to it

// Encoders (SKIP PIN 10)
#define whA_A 13 // Wheel A output B
#define whA_B 12 // Wheel A output A
#define whB_A 11 // Wheel B output B
#define whB_B 9 // wheel B output A
#define whC_A 8 // Wheel C output B
#define whC_B 7 // wheel C output A
#define whD_A 6 // wheel D output B
#define whD_B 5 // wheel D output A
#define whE_A 4 // wheel E output B
#define whE_B 3 // wheel E output A
//No 2 or 1 for serial link functionality
#define whF_A 14 // wheel F output B 
#define whF_B 15 // wheel F output A

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

// IMU PINS ;)
#define tx 18
#define rx 19


int chA = 0, chB = 1, chC = 2, chD = 3, chE = 5, chF = 6;



// 1 XD: ENCODER HAKI (6 MOTORS)
// use volatile so brain processor sees every tiny movement instantly by checking the pins hardware since physcial changes happen
volatile long counts[6] = {0, 0, 0, 0, 0, 0};  // Stores pulse counts for 6 encoders 

// 2 XD: MOTOR HAKI
int PWM_Channel[6] = {0,1,2,3,5,6};
// Stores input commands (0–255) :) Used for logging later (input to output comparison)
int MotorPWM[6] = {0, 0, 0, 0, 0, 0};  // Current SPEED being sent to each motor from a range of 0 to 255

// CONVERT MOTOR PWM TO HARDWARE PWM
void ApplyMotorPWM(){
    for (int i =0; i<6; i++){
        //Convert 0-255 to 0-1200 (NOT INVERTED!!!!!!!!!!!!)
        int DUTY = (int) map(MotorPWM[i], 0, 255, 0, 1200);  // cuz map returns long

        PWMC_SetDutyCycle(PWM, PWM_Channel[i], DUTY);
    }
}

/*
// FULL QUADRATURE: uses 2 SIGNLAS SLIGHTLY OUT OF PHASE, PHASE DIFFEECNE GIVES DIRECTION
volatile uint8_t PREVIOUS_STATE[6] = {0};    // stores LAST KNOWN STATE OF EACH ENCODER FOR ALL MOTORS
void UPDATE_ENCODER(int i, int pinA, int pinB){
  uint8_t A = digitalRead(pinA);  // pinA is just 1st encoder channel pin i pass in, pinB 2nd
  uint8_t B = digitalRead(pinB);  //
  uint8_t CURRENT_STATE = ((A<<1) | B);  // SHIFT A LEFT SO IT BECOMES MSB or WITH B BECOMES LSB
  uint8_t combined = (PREVIOUS_STATE[i]<<2) | CURRENT_STATE; // i created a 4 bit transition (prev+current AB) so if previous =01 and current=11 then combined=0111

  // STATE TRANSITION TABLE: A: 0 0 1 1  // B : 0 1 0 1 // CURRENT_STATE = 0b00, 0b01, 0b10, 0b11
  if (combined == 0b0001 || combined == 0b0111 || combined == 0b1110 || combined == 0b1000){
    counts[i]++;  // each encoder uses 1 of 4 states and these patterns are SUSUMEEEEEEEE
  }
  else if (combined == 0b0010 || combined == 0b0100 || combined == 0b1101 || combined == 0b1011){
    counts[i]--;  // SIKE GO BACK BUDDY, these are reverse roataion as in the sequence is reversed
  }
  PREVIOUS_STATE[i] = CURRENT_STATE;   // update MEMORY CUZ MEXT INTERRPUT NEEDS THIS AS THE PREVIOUS, IT KEEPS TRACK OF COINTUOS MOTIONNNN
}*/


// Interrupt ISR Functions so it makes it count pulses at lightfoot speed mode , //Each encoder signal connected to a pin, These pins will trigger interrupts
void ISR_MOTOR_1() {   // i'm measuring position (pulse accumulation)
  if (digitalRead(whA_A) == digitalRead(whA_B)){
    counts[0]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[0]++;
  }
}
void ISR_MOTOR_2() {
  if (digitalRead(whB_A) == digitalRead(whB_B)){
    counts[1]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[1]++;
  }
}
void ISR_MOTOR_3() {
  if (digitalRead(whC_A) == digitalRead(whC_B)){
    counts[2]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[2]++;
  }
}
void ISR_MOTOR_4() {
  if (digitalRead(whD_A) == digitalRead(whD_B)){
    counts[3]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[3]++;
  }
}
void ISR_MOTOR_5() {
  if (digitalRead(whE_A) == digitalRead(whE_B)){
    counts[4]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[4]++;
  }
}
void ISR_MOTOR_6() {
  if (digitalRead(whF_A) == digitalRead(whF_B)){
    counts[5]--; //Each ISR runs when encoder signal changes, Every trigger = +1 count
  } else {
    counts[5]++;
  }
}

/*
// Interrupt ISR Functions so it makes it count pulses at lightfoot speed mode , //Each encoder signal connected to a pin, These pins will trigger interrupts
void ISR_MOTOR_1() {UPDATE_ENCODER(0, Wheel_A_Channel_A, Wheel_A_Channel_B);}
void ISR_MOTOR_2() {UPDATE_ENCODER(1, Wheel_B_Channel_A, Wheel_B_Channel_B);}
void ISR_MOTOR_3() {UPDATE_ENCODER(2, Wheel_C_Channel_A, Wheel_C_Channel_B);}
void ISR_MOTOR_4() {UPDATE_ENCODER(3, Wheel_D_Channel_A, Wheel_D_Channel_B);}
void ISR_MOTOR_5() {UPDATE_ENCODER(4, Wheel_E_Channel_A, Wheel_E_Channel_B);}
void ISR_MOTOR_6() {UPDATE_ENCODER(5, Wheel_F_Channel_A, Wheel_F_Channel_B);} 
*/

 

// Add Global Variable
unsigned long startTime = 0;
bool started = false;

// 2 ;) IMU & LOGGING GEAR
const int chipselect =  10;   // NOT 4
const int ledPin = 13; // Status LED
// Timing system, Logs every 10 ms → 100 Hz
unsigned long previousMillis = 0;
const long interval = 10; // 10 ms = 100 Hz frequency babyyy
// IMU ANGLES WOHOOOOOOOOOOOOOOO XD
float Roll = 0, Pitch = 0, Yaw = 0; //(IMU packet 0x53)
// Linear acceleration (IMU packet 0x51)
float Ax = 0, Ay = 0, Az = 0;
// Angular velocity / gyro (IMU packet 0x52)
float Gx = 0, Gy = 0, Gz = 0;

// CONFIGURES PINS FOR PWM HARDWARE LOW-LEVEL STUFF
void SetPin(uint8_t pin){
  PIO_Configure(g_APinDescription[pin].pPort,
                PIO_PERIPH_B,
                g_APinDescription[pin].ulPin,
                g_APinDescription[pin].ulPinConfiguration);
}
// SETS FREQUENCY, PERIOD, DUTY CYCLE, ENABLES PWM CHANNEL
void setPWM(int channel) {
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(42000000, 0, 84000000);
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


void setup(){
    Serial.begin(115200);  // Laptop BAUD RATE
    Serial1.begin(9600); // IMU (WT901C)
    // LED setup
    pinMode(ledPin, OUTPUT);

    // Initialise Encoders with pullups + Sets pins as inputs, Enables internal pull-up resistors, Prevents floating signals
    pinMode(whA_A, INPUT_PULLUP);    
    pinMode(whA_B, INPUT_PULLUP);    
    pinMode(whB_A, INPUT_PULLUP);    
    pinMode(whB_B, INPUT_PULLUP);    
    pinMode(whC_A, INPUT_PULLUP);    
    pinMode(whC_B, INPUT_PULLUP);    
    pinMode(whD_A, INPUT_PULLUP);    
    pinMode(whD_B, INPUT_PULLUP);    
    pinMode(whE_A, INPUT_PULLUP);    
    pinMode(whE_B, INPUT_PULLUP);    
    pinMode(whF_A, INPUT_PULLUP);    
    pinMode(whF_B, INPUT_PULLUP);    



   /* PREVIOUS_STATE[0] = ((digitalRead(Wheel_A_Channel_A) << 1) | digitalRead(Wheel_A_Channel_B));
    PREVIOUS_STATE[1] = ((digitalRead(Wheel_B_Channel_A) << 1) | digitalRead(Wheel_B_Channel_B));
    PREVIOUS_STATE[2] = ((digitalRead(Wheel_C_Channel_A) << 1) | digitalRead(Wheel_C_Channel_B));
    PREVIOUS_STATE[3] = ((digitalRead(Wheel_D_Channel_A) << 1) | digitalRead(Wheel_D_Channel_B));
    PREVIOUS_STATE[4] = ((digitalRead(Wheel_E_Channel_A) << 1) | digitalRead(Wheel_E_Channel_B));
    PREVIOUS_STATE[5] = ((digitalRead(Wheel_F_Channel_A) << 1) | digitalRead(Wheel_F_Channel_B));
*/



    
    //Links pin to ISR, NOT ANYMORE EVERY RISING = RUN ISR BABYYY // NOW ITS CHANGE (rising + fall)IS doubling resolution  BUT increasing noise sensitivity
    attachInterrupt(digitalPinToInterrupt(whA_A), ISR_MOTOR_1, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(whB_A), ISR_MOTOR_2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(whC_A), ISR_MOTOR_3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(whD_A), ISR_MOTOR_4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(whE_A), ISR_MOTOR_5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(whF_A), ISR_MOTOR_6, CHANGE);
    /*// ADD CHANNEL B FOR FULLL QUADRATURE BABYYY!! ACCURACY 10 BILLION PERCENT!!
    attachInterrupt(digitalPinToInterrupt(Wheel_A_Channel_B), ISR_MOTOR_1, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(Wheel_B_Channel_B), ISR_MOTOR_2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Wheel_C_Channel_B), ISR_MOTOR_3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Wheel_D_Channel_B), ISR_MOTOR_4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Wheel_E_Channel_B), ISR_MOTOR_5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Wheel_F_Channel_B), ISR_MOTOR_6, CHANGE);
 */
    // FRONT LIGHTWHEEELS
    pinMode(30, OUTPUT); pinMode(31, OUTPUT);
    pinMode(32, OUTPUT); pinMode(33, OUTPUT);
    digitalWrite(30, HIGH); digitalWrite(31, LOW);
    digitalWrite(32, HIGH); digitalWrite(33, LOW);
      
    // MIDDLE LIGHTWHEEELS
    pinMode(26, OUTPUT); pinMode(27, OUTPUT);
    pinMode(28, OUTPUT); pinMode(29, OUTPUT);
    digitalWrite(26, HIGH); digitalWrite(27, LOW);
    digitalWrite(28, HIGH); digitalWrite(29, LOW);
        

    // REAR LIGHTWHEEEEELS
    pinMode(22, OUTPUT); pinMode(23, OUTPUT);
    pinMode(24, OUTPUT); pinMode(25, OUTPUT);
    digitalWrite(22, HIGH); digitalWrite(23, LOW);
    digitalWrite(24, HIGH); digitalWrite(25, LOW);

/*
// TURN TEST: Spin inplace for 10 seconds
if(currentMillis - startTime < 10000){
  // LEFT SiDE FPRWARD
  MotorPWM[0] = 120;
  MotorPWM[2] = 120;
  MotorPWM[4] = 120;
  
  // RIGHT SiDE BACKWARD (INVERT DIECTION)
  MotorPWM[1] = 120;
  MotorPWM[3] = 120 ;
  MotorPWM[5] = 120;
  
  // FLIP DIRECTION PINS FOR RIGHT SIDE
  digitalWrite(30, LOW); digitalWrite(31, HIGH);
  digitalWrite(26, LOW); digitalWrite(27, HIGH);
  digitalWrite(25, LOW); digitalWrite(24, HIGH);

}
else{
  for(int i = 0; i< 6; i++){
    MotorPWM[i] = 0;

  }
}   */
 // ENABLES PWM OUTPUT
  SetPin(PWM_A); SetPin(PWM_B); SetPin(PWM_C); SetPin(PWM_D); SetPin(PWM_E); SetPin(PWM_F);
  setPWM(chA); setPWM(chB); setPWM(chC); setPWM(chD); setPWM(chE); setPWM(chF);
 
    // Initialise SD card
    if(!SD.begin(chipselect)){
        while (1){ // Infinite SOS Flash MORSE CODE SO COOOOOOL DR SOTNE, if SD fails to indicate errors
            digitalWrite(ledPin, HIGH); delay(100); digitalWrite(ledPin, LOW); delay(100);
        }   
    }

    // SEND  DATA TO IMU AT THE 100 Hz (The 5-byte NINJA RASENGAN SCROLL)
    byte Rasengan[] = {0xFF, 0xAA, 0x03, 0x03, 0x00};                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   Serial1.write(Rasengan, 5);

    // Create CSV Header for Sarah :)
    DataFile = SD.open("LightWheels_Rover.csv", FILE_WRITE);
    // IF FAIL, Panic signal in case SD fails mid-run or disconnects
    if(!DataFile){ 
      digitalWrite(ledPin, HIGH);
      return;
    }
    // IF SUCCESS, Writes header row
    if (DataFile){ 
        DataFile.println("Time_ms, Roll, Pitch, Yaw, Ax, Ay, Az, Gx, Gy, Gz, Wheel_A, Wheel_B, Wheel_C, Wheel_D, Wheel_E, Wheel_F");
    }   
    // ADD VOLTAGE IN ABOVE HEREEEEEEEEEEEEEEEEEEEEEE
}

void loop(){     // LIKE WHILE TRUE INSIDE MAIN
    unsigned long currentMillis = millis();     //Time tracking

    // Start Timer ONE TIME
    if (!started){
        startTime = currentMillis;
        started = true;
    }

    // SUSUMEEE! FORWARD MOTION CONTROL! TATAKEE! KEEP MOVING FORWARD!
    if (currentMillis - startTime < 10000){
        // 10 SECOND SUSUMEEEEE FORWARD
         MotorPWM[0] =180;   // forward SPEEEEED "could be adjusted from 0-255"
         MotorPWM[1] =180;   // forward SPEEEEED "could be adjusted from 0-255"
         MotorPWM[2] =180;   // forward SPEEEEED "could be adjusted from 0-255"
         MotorPWM[3] =180;   // forward SPEEEEED "could be adjusted from 0-255"
         MotorPWM[4] =180;   // forward SPEEEEED "could be adjusted from 0-255"
         MotorPWM[5] =180;   // forward SPEEEEED "could be adjusted from 0-255"


        //TURN RIGHT
        //MotorPWM[0] =145;   //   SPEEEEED "could be adjusted from 0-255"
    /// //   MotorPWM[2] =145;   //   SPEEEEED "could be adjusted from 0-255"
      //  MotorPWM[4] =145;   //      SPEEEEED "could be adjusted from 0-255"
        
       // MotorPWM[1] =80;   //  SPEEEEED "could be adjusted from 0-255"
       // MotorPWM[3] =80;   //   SPEEEEED "could be adjusted from 0-255"
       // MotorPWM[5] =80;   //   SPEEEEED "could be adjusted from 0-255" 
 

        /*
        // SPIN IN PLACE
        //forward
        MotorPWM[0] =160;   //   SPEEEEED "could be adjusted from 0-255"
        MotorPWM[2] =160;   //   SPEEEEED "could be adjusted from 0-255"
        MotorPWM[4] =160;   //   SPEEEEED "could be adjusted from 0-255"
        // flip direction pins so backward
        MotorPWM[1] =160;   //   SPEEEEED "could be adjusted from 0-255"
        MotorPWM[3] =160;   //   SPEEEEED "could be adjusted from 0-255"
        MotorPWM[5] =160;   //   SPEEEEED "could be adjusted from 0-255"*/

    }
    else {
        // stop motors after 10 secando
        for (int i =0; i<6; i++){
            MotorPWM[i] = 0;
        }
    }
       
    // MOTOR VROOM VROOM
    ApplyMotorPWM();

    // Always listen for IMU data in the background
    ReadIMU();    

    
    // The 100 Hz Heartbeat DRUMS OF LIBERATION: Log every 10 ms
    if (currentMillis - previousMillis >= interval){
        previousMillis += interval;
        LogData(currentMillis);
    }
 }

void ReadIMU() {
    // Wait for/Check if we have a full 11-byte packet
    if (Serial1.available() < 11) return;
    
    if (Serial1.read() != 0x55) return; // Check header
    byte type = Serial1.read();
                // Extract raw data, reads two bytes from Serial1 and combines them into a single 16‑bit integer (int16_t)
                int16_t rRaw = (Serial1.read() | (Serial1.read() << 8));
                // Serial1.read() returns one byte (0–255). The first read() becomes the low byte.
                int16_t pRaw = (Serial1.read() | (Serial1.read() << 8));
                //The second read() is shifted left 8 bits (<< 8), making it the high byte.the | (bitwise OR) combines them into a single 16‑bit value.
                int16_t yRaw = (Serial1.read() | (Serial1.read() << 8)); // This is little‑endian byte order (low byte first, high byte second).

                // Discard temperature and checksum, the last 3 bytes i think 
                for(int i=0; i<3; i++) Serial1.read();

          if (type == 0x51) {                     // WT901 linear accel scale = ±16g
                // Convert to human ANGLES degrees
                Ax = (float)rRaw / 32768.0 * 16.0;
                Ay = (float)pRaw / 32768.0 * 16.0;
                Az = (float)yRaw / 32768.0 * 16.0;      
                }    
          else if (type == 0x52) {                 // WT901 angular velo gyro scale = ±2000°/s
                // Convert to human ANGLES degrees
                Gx = (float)rRaw / 32768.0 * 2000.0;
                Gy = (float)pRaw / 32768.0 * 2000.0;
                Gz = (float)yRaw / 32768.0 * 2000.0;  
                }
          else if (type == 0x53) {                 // Means Euler angles // LITTLE ENDIAN
                // Convert to human ANGLES degrees
                Roll = (float)rRaw / 32768.0 * 180.0;
                Pitch = (float)pRaw / 32768.0 * 180.0;
                Yaw = (float)yRaw / 32768.0 * 180.0;
                }
                
                //Reject invalid data
                if (abs(Roll) > 180 || abs(Pitch) > 180 || abs(Yaw) > 180){
                  return;
                }
}

// VOLTAGE GOING IN MOTORS
float supplyVoltage = 7.0;   // CHANGE 7 V  MAYBE???????

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
        DataFile.print(TimeStamp); DataFile.print(",");   // CLEAN EXCEL DATA!!!!!!!!!!!!
        DataFile.print(Roll); DataFile.print(",");  // IM SO HAPPPPPPPPPPPPPPPPY!
        DataFile.print(Pitch); DataFile.print(",");  // FINALLYYYYYYYYYYYYYYYY
        DataFile.print(Yaw); DataFile.print(",");    // THIS TOOK SO LONG IM HAPPPYPYPYPYPYPYPY!!!!!!
        DataFile.print(Ax); DataFile.print(",");
        DataFile.print(Ay); DataFile.print(",");
        DataFile.print(Az); DataFile.print(",");
        DataFile.print(Gx); DataFile.print(",");
        DataFile.print(Gy); DataFile.print(",");
        DataFile.print(Gz); DataFile.print(",");

        /*// Loop for encoder values: Log all 6 encoder counts
        for(int i = 0; i < 6; i++){
            DataFile.print(Safe_Encoder_Counts[i]);
            DataFile.print(",");             // a comma to seperate data
        } */
        DataFile.print(Safe_Encoder_Counts[0]);
        DataFile.print(",");             // a comma to seperate data        
        DataFile.print(Safe_Encoder_Counts[1]);      
        DataFile.print(",");             // a comma to seperate data          
        DataFile.print(Safe_Encoder_Counts[2]);
        DataFile.print(",");             // a comma to seperate data        
        DataFile.print(Safe_Encoder_Counts[3]);
        DataFile.print(",");             // a comma to seperate data        
        DataFile.print(Safe_Encoder_Counts[4]);
        DataFile.print(",");             // a comma to seperate data        
        DataFile.print(Safe_Encoder_Counts[5]);
        DataFile.print(",");             // a comma to seperate data

        /*// Loop for PWM values: Log THE PWM / Voltage INPUTS FOR ALL 6 motors
        for(int i = 0; i < 6; i++){
            DataFile.print(MotorPWM[i]);
            DataFile.print(",");
        }
        for(int i = 0; i < 6; i++){
            float motorVoltage = (MotorPWM[i] / 255.0) * supplyVoltage;
            DataFile.print(motorVoltage);
            if(i < 5) DataFile.print(",");
        }*/

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
  

  // PRINT BROSKI!! ;0
  static int chill_serial_bro = 0;
  chill_serial_bro++;
  if(chill_serial_bro >= 50){               // every 500 ms to prevent spam printing lol 
    chill_serial_bro = 0;
    // Real-Time
    Serial.println();       // new line 
    Serial.print("   Time [ms]:  ");
    Serial.print(TimeStamp);
    Serial.println();

    // ENCODER COUNTS
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
    Serial.println();       // new line 
    
    // IMU ANGLES 
    Serial.print("  Euler Angles:  ");
    Serial.print("   R: ");
    Serial.print(Roll);
    Serial.print("   |    P: ");
    Serial.print(Pitch);
    Serial.print("   |   Y: ");
    Serial.print(Yaw);  
    Serial.println();       // new line

    // Linear Accel + Gyro
    Serial.print("  Linear Accel:  ");
    Serial.print("   Ax: "); Serial.print(Ax);  
    Serial.print("   Ay: "); Serial.print(Ay); 
    Serial.print("   Az: "); Serial.print(Az); 
        Serial.println();       // new line

    Serial.print("  Gyro:  ");
    Serial.print("             Gx: "); Serial.print(Gx); 
    Serial.print("   Gy: "); Serial.print(Gy); 
    Serial.print("   Gz: "); Serial.print(Gz); 
       Serial.println();       // new line



    // CALCULATE VOLTAGE_IN FROM PWM
  /*  for(int i = 0; i < 6; i++){
        float motorVoltage = (MotorPWM[i] / 255.0) * supplyVoltage;
        // PRINT PWM
        Serial.print("   |   PWM: ");
        Serial.print(MotorPWM[i]);
        Serial.print(" ");
        // VOLTAGE_IN MOTORS
        Serial.print("    |   Motor_Voltage: ");
        Serial.print(motorVoltage);
        Serial.println();*/
   // }
  }
}
