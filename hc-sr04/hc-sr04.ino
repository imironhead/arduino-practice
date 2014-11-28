#define PIN_TRIG  (6)
#define PIN_ECHO  (7)


void setup() {
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_TRIG, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // warm up
  digitalWrite(PIN_TRIG, LOW);

  delayMicroseconds(2);

  // raise to HIGH and stay at least 10 micro seconds
  digitalWrite(PIN_TRIG, HIGH);

  delayMicroseconds(10);

  // drop to low to trigger the sensor
  digitalWrite(PIN_TRIG, LOW);

  // http://arduino.cc/en/Reference/pulseIn
  // measure the duration of a high pulse in PIN_ECHO
  // timeout us 3000 ms
  // the sound spend duration to reflect from something
  long duration = pulseIn(PIN_ECHO, HIGH, 3000);

  // calculate the distance with sound speed and duration
  long distance_in_cm = (duration / 2) / 29;

  Serial.print(distance_in_cm);
  Serial.println(" cm");

  delay(1000);
}
