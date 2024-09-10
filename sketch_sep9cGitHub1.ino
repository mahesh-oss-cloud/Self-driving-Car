#define TRIG_PIN 12 
#define ECHO_PIN 11
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, OUTPUT);
  Serial.begin(9600);

}
long readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
} digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);o

  long duration = pulseIN(ECHO_PIN, HIGH);
  return duration * 0.034 / 2; // Calculate distance in cm 

void loop() {
  // put your main code here, to run repeatedly:
  long distance = readDistance();
  Serial.print("Distance: ");
  Serial.println(distance);


  if (distance < 10) { // If obstacle is closer than 5 cm
    // Stop or avoid obstacle here
  }

  delay(100);
}


}
