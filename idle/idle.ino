// just a simple script that puts the board in an idle state
// created by succo (St. Genis Pouilly 2025/11/06)

void setup() {

  // built-in LED is kept off
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

}

void loop() {

  // during the loop, just wait w/o doing anything
  delay(1000);

}
