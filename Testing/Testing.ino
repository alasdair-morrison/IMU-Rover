//Front Wheels
#define enA_A 6 //PWM Out Motor A
#define in1_A 1 //Enable 1 Motor A
#define in2_A 2 //Enable 2 Motor A
#define in1_B 3 // Enable 1 Motor B
#define in2_B 4 // Enable 2 Motor B
//Middle Wheels
#define in1_C 5 //Enable 1 Motor C
#define in2_C 7 // Enable 2 Motor C
#define in1_D 8 // Enable 1 Motor D
#define in2_D 9 // Enable 2 Motor D
//Rear Wheels
#define in1_E 10 // Enable 1 Motor E
#define in2_E 11 // Enable 2 Motor E
#define in1_F 12 // Enable 1 Motor F
#define in2_F 13 // Enable 2 Motor F

void setup() {
  //Front Wheel Setup
  pinMode(enA_A, OUTPUT);
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
}

void loop() {
  int pwmOutput = 1;
  analogWrite(enA_A, 255 * pwmOutput); // Send PWM signal to L298N Enable pin

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
  delay(20); 
}
