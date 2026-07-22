#include <SPI.h>  // needed for SD card communication
#include <SD.h>  // SD card library
#include <Adafruit_HUSB238.h>
File DataFile; // global file object so all functions can write to it
Adafruit_HUSB238 husb238;

#define tx 18 
#define rx 19

// Encoders (SKIP PIN 10)
#define whA_A 53 // Wheel A output A
#define whA_B 52 // Wheel A output B
#define whB_A 51 // Wheel B output A
#define whB_B 9 // wheel B output B
#define whC_A 8 // Wheel C output A
#define whC_B 7 // wheel C output B
#define whD_A 6 // wheel D output A
#define whD_B 5 // wheel D output B
#define whE_A 4 // wheel E output A
#define whE_B 3 // wheel E output B
//No 2 or 1 for serial link functionality
#define whF_A 14 // wheel F output A
#define whF_B 15 // wheel F output B

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

int chA = 0, chB = 1, chC = 2, chD = 3, chE = 5, chF = 6;

// 1 XD: ENCODER HAKI (6 MOTORS)
// use volatile so brain processor sees every tiny movement instantly by checking the pins hardware since physcial changes happen
volatile long counts[6] = {0, 0, 0, 0, 0, 0};  // Stores pulse counts for 6 encoders 

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

// 2 ;) IMU & LOGGING GEAR
const int chipselect = 10;   // NOT 4
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
  PWMC_SetDutyCycle(PWM, channel, 1200);
  PWMC_SetDeadTime(PWM, channel, 42, 42);
  PWMC_EnableChannel(PWM, channel);
}

// --- CODE 1 MOVEMENT DRIVER LOGIC ---
void forward(float duty) {
  int trueDuty = 1200 - int(1200 * duty);
  PWMC_SetDutyCycle(PWM, chA, trueDuty); PWMC_SetDutyCycle(PWM, chB, trueDuty);
  digitalWrite(in1_A, HIGH); digitalWrite(in2_A, LOW); digitalWrite(in1_B, HIGH); digitalWrite(in2_B, LOW);

  PWMC_SetDutyCycle(PWM, chC, trueDuty); PWMC_SetDutyCycle(PWM, chD, trueDuty);
  digitalWrite(in1_C, HIGH); digitalWrite(in2_C, LOW); digitalWrite(in1_D, HIGH); digitalWrite(in2_D, LOW);

  PWMC_SetDutyCycle(PWM, chE, trueDuty); PWMC_SetDutyCycle(PWM, chF, trueDuty);
  digitalWrite(in1_E, HIGH); digitalWrite(in2_E, LOW); digitalWrite(in1_F, HIGH); digitalWrite(in2_F, LOW);
}

void reverse(float duty) {
  int trueDuty = 1200 - int(1200 * duty);
  PWMC_SetDutyCycle(PWM, chA, trueDuty); PWMC_SetDutyCycle(PWM, chB, trueDuty);
  digitalWrite(in1_A, LOW); digitalWrite(in2_A, HIGH); digitalWrite(in1_B, LOW); digitalWrite(in2_B, HIGH);

  PWMC_SetDutyCycle(PWM, chC, trueDuty); PWMC_SetDutyCycle(PWM, chD, trueDuty);
  digitalWrite(in1_C, LOW); digitalWrite(in2_C, HIGH); digitalWrite(in1_D, LOW); digitalWrite(in2_D, HIGH);

  PWMC_SetDutyCycle(PWM, chE, trueDuty); PWMC_SetDutyCycle(PWM, chF, trueDuty);
  digitalWrite(in1_E, LOW); digitalWrite(in2_E, HIGH); digitalWrite(in1_F, LOW); digitalWrite(in2_F, HIGH);
}

void turnLeft(float duty) {
  int trueDuty = 1200 - int(1200 * duty);

  // LEFT SIDE FORWARD (A, C, E)
  PWMC_SetDutyCycle(PWM, chA, trueDuty);
  digitalWrite(in1_A, HIGH); digitalWrite(in2_A, LOW);

  PWMC_SetDutyCycle(PWM, chC, trueDuty);
  digitalWrite(in1_C, HIGH); digitalWrite(in2_C, LOW);

  PWMC_SetDutyCycle(PWM, chE, trueDuty);
  digitalWrite(in1_E, HIGH); digitalWrite(in2_E, LOW);

  // RIGHT SIDE REVERSE (B, D, F)
  PWMC_SetDutyCycle(PWM, chB, trueDuty);
  digitalWrite(in1_B, LOW); digitalWrite(in2_B, HIGH);

  PWMC_SetDutyCycle(PWM, chD, trueDuty);
  digitalWrite(in1_D, LOW); digitalWrite(in2_D, HIGH);

  PWMC_SetDutyCycle(PWM, chF, trueDuty);
  digitalWrite(in1_F, LOW); digitalWrite(in2_F, HIGH);
}

void turnRight(float duty) {
  int trueDuty = 1200 - int(1200 * duty);

  // LEFT SIDE REVERSE (A, C, E)
  PWMC_SetDutyCycle(PWM, chA, trueDuty);
  digitalWrite(in1_A, LOW); digitalWrite(in2_A, HIGH);

  PWMC_SetDutyCycle(PWM, chC, trueDuty);
  digitalWrite(in1_C, LOW); digitalWrite(in2_C, HIGH);

  PWMC_SetDutyCycle(PWM, chE, trueDuty);
  digitalWrite(in1_E, LOW); digitalWrite(in2_E, HIGH);

  // RIGHT SIDE FORWARD (B, D, F)
  PWMC_SetDutyCycle(PWM, chB, trueDuty);
  digitalWrite(in1_B, HIGH); digitalWrite(in2_B, LOW);

  PWMC_SetDutyCycle(PWM, chD, trueDuty);
  digitalWrite(in1_D, HIGH); digitalWrite(in2_D, LOW);

  PWMC_SetDutyCycle(PWM, chF, trueDuty);
  digitalWrite(in1_F, HIGH); digitalWrite(in2_F, LOW);
}

void stop() {
  int trueDuty = 1200;
  PWMC_SetDutyCycle(PWM, chA, trueDuty); PWMC_SetDutyCycle(PWM, chB, trueDuty);
  digitalWrite(in1_A, HIGH); digitalWrite(in2_A, LOW); digitalWrite(in1_B, HIGH); digitalWrite(in2_B, LOW);

  PWMC_SetDutyCycle(PWM, chC, trueDuty); PWMC_SetDutyCycle(PWM, chD, trueDuty);
  digitalWrite(in1_C, HIGH); digitalWrite(in2_C, LOW); digitalWrite(in1_D, HIGH); digitalWrite(in2_D, LOW);

  PWMC_SetDutyCycle(PWM, chE, trueDuty); PWMC_SetDutyCycle(PWM, chF, trueDuty);
  digitalWrite(in1_E, HIGH); digitalWrite(in2_E, LOW); digitalWrite(in1_F, HIGH); digitalWrite(in2_F, LOW);
}

void setup(){
    Serial.begin(115200);  // Laptop BAUD RATE
    Serial1.begin(9600); // IMU (WT901C)

    // 2. Initialize the I2C communication on the Due's default Wire pins (20 & 21)
    if (!husb238.begin(HUSB238_I2CADDR_DEFAULT, &Wire)) {
      Serial.println("HUSB238 Init Failed. Check wiring.");
      while (1) delay(10); // Halt execution if the chip isn't found to protect your circuit
    }

    // 3. Command the HUSB238 to request 9V
    husb238.selectPD(PD_SRC_9V); // Selects the 9V profile from the internal register
    husb238.requestPD();         // Transmits the request to the battery

    // 4. Give the battery and HUSB238 a moment to finalize the handshake 
    // before your main project starts pulling heavy current.
    delay(500);
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

    // Links pin to ISR, NOT ANYMORE EVERY RISING = RUN ISR BABYYY // NOW ITS CHANGE (rising + fall)IS doubling resolution BUT increasing noise sensitivity
    attachInterrupt(digitalPinToInterrupt(whA_A), ISR_MOTOR_1, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(whB_A), ISR_MOTOR_2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(whC_A), ISR_MOTOR_3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(whD_A), ISR_MOTOR_4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(whE_A), ISR_MOTOR_5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(whF_A), ISR_MOTOR_6, CHANGE);

    // FRONT LIGHTWHEEELS
    pinMode(in1_A, OUTPUT); pinMode(in2_A, OUTPUT);
    pinMode(in1_B, OUTPUT); pinMode(in2_B, OUTPUT);
      
    // MIDDLE LIGHTWHEEELS
    pinMode(in1_C, OUTPUT); pinMode(in2_C, OUTPUT);
    pinMode(in1_D, OUTPUT); pinMode(in2_D, OUTPUT);
        
    // REAR LIGHTWHEEEEELS
    pinMode(in1_E, OUTPUT); pinMode(in2_E, OUTPUT);
    pinMode(in1_F, OUTPUT); pinMode(in2_F, OUTPUT);

    // ENABLES PWM OUTPUT
    SetPin(PWM_A);  // channel 0 - MOTOR A EN
    SetPin(PWM_B);  // channel 1 B
    SetPin(PWM_C);  // channel 2 C
    SetPin(PWM_D);  // channel 3 D
    SetPin(PWM_E);  // channel 5 E  
    SetPin(PWM_F);  // channel 6 F
   
    setPWM(chA);
    setPWM(chB);
    setPWM(chC);
    setPWM(chD);
    setPWM(chE);
    setPWM(chF);
 
    // Initialise SD card
    if(!SD.begin(chipselect)){
        while (1){ // Infinite SOS Flash MORSE CODE SO COOOOOOL DR SOTNE, if SD fails to indicate errors
            digitalWrite(ledPin, HIGH); delay(100); digitalWrite(ledPin, LOW); delay(100);
        }   
    }

    // SEND DATA TO IMU AT THE 100 Hz (The 5-byte NINJA RASENGAN SCROLL)
    byte Rasengan[] = {0xFF, 0xAA, 0x03, 0x03, 0x00};
    Serial1.write(Rasengan, 5);

    // --- AUTO-INCREMENT FILE NAME LOGIC ---
    char fileName[15] = "DW_LOG00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
        fileName[6] = i / 10 + '0';
        fileName[7] = i % 10 + '0';
        if (!SD.exists(fileName)) {
            break; // FOUND A FILE NAME THAT DOESN'T EXIST YET!
        }
    }

    // Create CSV Header for Sarah :)
    DataFile = SD.open(fileName, FILE_WRITE);
    // IF FAIL, Panic signal in case SD fails mid-run or disconnects
    if(!DataFile){ 
      digitalWrite(ledPin, HIGH);
      return;
    }
    // IF SUCCESS, Writes header row
    if (DataFile){ 
        DataFile.println("Time_ms, Roll, Pitch, Yaw, Ax, Ay, Az, Gx, Gy, Gz, Wheel_A, Wheel_B, Wheel_C, Wheel_D, Wheel_E, Wheel_F");
        DataFile.flush();
    }   
}
unsigned long voltageMillis = millis();
// --- MAIN LOOP WITH CODE 1'S MOVEMENT ROUTINE ---
void loop(){     // LIKE WHILE TRUE INSIDE MAIN
    unsigned long currentMillis = millis();     //Time tracking
    HUSB238_VoltageSetting currentVoltage = husb238.getPDSrcVoltage();
    // If the battery tripped and dropped us back to the 5V fallback...
    if (currentVoltage != PD_9V) {
      Serial.println("Voltage drop detected! Renegotiating 9V...");
      husb238.selectPD(PD_SRC_9V);
      husb238.requestPD();
      delay(200); // Brief pause to allow the handshake to re-establish
    }
    if (currentMillis < 100000) {        // Forward for 10 seconds    
        forward(0.75);    
  //  } 
  //  else if (currentMillis < 20000) {   // Turn left for next 10 seconds
  //      turnLeft(0.5);  
  //  }
  //  else if (currentMillis < 30000) {   // Reverse for next 10 seconds
  //      reverse(0.5);    
  //  }     
 //   else {          
  //      stop();                        // Stop forever after 30 seconds
    }

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
    int16_t rRaw = (Serial1.read() | (Serial1.read() << 8));
    int16_t pRaw = (Serial1.read() | (Serial1.read() << 8));
    int16_t yRaw = (Serial1.read() | (Serial1.read() << 8)); 

    // Discard temperature and checksum, the last 3 bytes i think 
    for(int i=0; i<3; i++) Serial1.read();

    if (type == 0x51) {                     // WT901 linear accel scale = ±16g
        Ax = (float)rRaw / 32768.0 * 16.0;
        Ay = (float)pRaw / 32768.0 * 16.0;
        Az = (float)yRaw / 32768.0 * 16.0;      
    }    
    else if (type == 0x52) {                 // WT901 angular velo gyro scale = ±2000°/s
        Gx = (float)rRaw / 32768.0 * 2000.0;
        Gy = (float)pRaw / 32768.0 * 2000.0;
        Gz = (float)yRaw / 32768.0 * 2000.0;  
    }
    else if (type == 0x53) {                 // Means Euler angles // LITTLE ENDIAN
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

        DataFile.print(Safe_Encoder_Counts[0]); DataFile.print(",");            
        DataFile.print(Safe_Encoder_Counts[1]); DataFile.print(",");            
        DataFile.print(Safe_Encoder_Counts[2]); DataFile.print(",");            
        DataFile.print(Safe_Encoder_Counts[3]); DataFile.print(",");            
        DataFile.print(Safe_Encoder_Counts[4]); DataFile.print(",");            
        DataFile.print(Safe_Encoder_Counts[5]); DataFile.print(",");

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
    Serial.print("   A: "); Serial.print(Safe_Encoder_Counts[0]);
    Serial.print("   |    B: "); Serial.print(Safe_Encoder_Counts[1]);
    Serial.print("   |   C: "); Serial.print(Safe_Encoder_Counts[2]);
    Serial.print("   |   D: "); Serial.print(Safe_Encoder_Counts[3]);
    Serial.print("   |   E: "); Serial.print(Safe_Encoder_Counts[4]);
    Serial.print("   |   F: "); Serial.print(Safe_Encoder_Counts[5]);
    Serial.println();       // new line 
    
    // IMU ANGLES 
    Serial.print("  Euler Angles:  ");
    Serial.print("   R: "); Serial.print(Roll);
    Serial.print("   |    P: "); Serial.print(Pitch);
    Serial.print("   |   Y: "); Serial.print(Yaw);  
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
  }
}
