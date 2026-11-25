// RoboGuard bench signal. Send 's' to stop pulses and 'r' to resume.
constexpr uint8_t heartbeatPin = 25;
bool running = true;
bool high = false;

void setup() {
  pinMode(heartbeatPin, OUTPUT);
  digitalWrite(heartbeatPin, LOW);
  Serial.begin(115200);
}

void loop() {
  while (Serial.available()) {
    int command = Serial.read();
    if (command == 's') running = false;
    if (command == 'r') running = true;
  }
  high = running && !high;
  digitalWrite(heartbeatPin, high ? HIGH : LOW);
  delay(100);
}
