//Front Wheels
#define PWM_A 35 //PWM Out Motor A
#define in1_A 30 //Enable 1 Motor A
#define in2_A 31 //Enable 2 Motor A
#define in1_B 32 // Enable 1 Motor B
#define in2_B 33 // Enable 2 Motor B
#define PWM_B 37 // PWM Out Motor B
//Middle Wheels
#define PWM_C 39 // PWM Out Motor C
#define in1_C 26 //Enable 1 Motor C
#define in2_C 27 // Enable 2 Motor C
#define in1_D 28 // Enable 1 Motor D
#define in2_D 29 // Enable 2 Motor D
#define PWM_D 41 // PWM Out Motor D
//Rear Wheels
#define PWM_E 44 // PWM Out Motor E
#define in1_E 25 // Enable 1 Motor E
#define in2_E 24 // Enable 2 Motor E
#define in1_F 23 // Enable 1 Motor F
#define in2_F 22 // Enable 2 Motor F
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

void setup() {
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
  
  setPWM(chA); // sets up corresponding PWM channel to run at 50% duty cycle
  setPWM(chB);
  setPWM(chC);
  setPWM(chD);
  setPWM(chE);
  setPWM(chF);
}
//***ALL DUTY CYCLES ARE INVERTED DUE TO AVAIALBLE PINS 0 = 100% & 1200 = 0%***
void loop() {
  PWMC_SetDutyCycle(PWM, chA, 0); // Channel: 0, Duty cycle: 100 %
  PWMC_SetDutyCycle(PWM, chB, 0); // Channel: 1, Duty cycle: 100 %
  digitalWrite(in1_A, HIGH);
  digitalWrite(in2_A, LOW);
  digitalWrite(in1_B, HIGH);
  digitalWrite(in2_B, LOW);

  PWMC_SetDutyCycle(PWM, chC, 0); // Channel: 2, Duty cycle: 100 %
  PWMC_SetDutyCycle(PWM, chD, 0); // Channel: 3, Duty cycle: 100 %
  digitalWrite(in1_C, HIGH);
  digitalWrite(in2_C, LOW);
  digitalWrite(in1_D, HIGH);
  digitalWrite(in2_D, LOW);

  PWMC_SetDutyCycle(PWM, chE, 0); // Channel: 5, Duty cycle: 100 %
  PWMC_SetDutyCycle(PWM, chF, 0); // Channel: 6, Duty cycle: 100 %
  digitalWrite(in1_E, HIGH);
  digitalWrite(in2_E, LOW);
  digitalWrite(in1_F, HIGH);
  digitalWrite(in2_F, LOW);
  delay(20); 
}
