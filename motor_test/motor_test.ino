int pwma = 6;
int ain1 = 7;
int ain2 = 8;
int pwmb = 5; 
int bin1 = 2; 
int bin2 = 3; 

void setup() {
  pinMode(pwma, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  
  // No need to connect STBY if you want it always on, 
  // as it has an internal pull-up to Vcc[cite: 27, 82].
}

void loop() {
  // Spin Motor B Forward at full speed
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
  analogWrite(pwmb, 255); 
  delay(2000);

  // Spin Motor B Backward at half speed
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, HIGH);
  analogWrite(pwmb, 127); 
  delay(2000);

  // Brake the motor
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, HIGH);
  delay(1000);

  // Spin Motor A Forward at full speed
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  analogWrite(pwma, 255); 
  delay(2000);

  // Spin Motor B Backward at half speed
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  analogWrite(pwma, 127); 
  delay(2000);

  // Brake the motor
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, HIGH);
  delay(1000);
}