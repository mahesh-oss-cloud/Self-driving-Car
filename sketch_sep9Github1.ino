void setup() {
  // Motor control pins as output 
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

}

void loop() {
  // Move Forward 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200); // Adjust speed with PWM

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200); // Adjust speed with PWM
  delay(2000); // Move forward for 2 seconds

  // Stop
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(1000);
}


}
