#define IN1 5
#define IN2 6

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void fire() {
  // Forward pulse – crank pushes sear lever, releases plunger
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(200);                // Tune: 150–300 ms for reliable release
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(50);

  // Reverse pulse – crank returns to hard stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(200);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}