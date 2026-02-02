#define enA_A 6 //PWM Out Motor A
#define in1_A 3 //Enable 1 Motor A
#define in2_A 4 //Enable 2 Motor A

void setup() {
  pinMode(enA_A, OUTPUT);
  pinMode(in1_A, OUTPUT);
  pinMode(in2_A, OUTPUT);
  // Set initial rotation direction for forwards
  digitalWrite(in1_A, LOW);
  digitalWrite(in2_A, HIGH);
}

void loop() {
  int pwmOutput = .5;
  analogWrite(enA_A, 255 * pwmOutput); // Send PWM signal to L298N Enable pin

  digitalWrite(in1_A, HIGH);
  digitalWrite(in2_A, LOW);
  delay(20);
  
  //digitalWrite(in1_A, LOW);
  //digitalWrite(in2_A, HIGH);
  //delay(20); 
}
