// master, motor
#define P0_ENA 4
#define P0_PUL 3
#define P0_DIR 2

// slave, motor
#define P1_ENA 7
#define P1_PUL 6
#define P1_DIR 5

// master, stoppers
#define P0_SBAS 10
#define P0_SSIG_L 9
#define P0_SSIG_R 8

// slave, stoppers
#define P1_SBAS 13
#define P1_SSIG_L 12
#define P1_SSIG_R 11

////////////////////////////////////////////////////////////////////////////
// dependencies ////////////////////////////////////////////////////////////

// get booleans from stoppers (HIGH = not pressed; LOW = pressed)
bool get_stoppers(bool & b_S0l, bool & b_S0r, bool & b_S1l, bool & b_S1r) {

  b_S0l = digitalRead(P0_SSIG_L) == LOW;
  b_S0r = digitalRead(P0_SSIG_R) == LOW;
  b_S1l = digitalRead(P1_SSIG_L) == LOW;
  b_S1r = digitalRead(P1_SSIG_R) == LOW;

  return (b_S0l||b_S0r||b_S1l||b_S1r);
  
}

// dependencies ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void setup() {

  // LED on board on
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // all motor pins are outputs, to be programmed
  pinMode(P0_ENA, OUTPUT);
  pinMode(P0_PUL, OUTPUT);
  pinMode(P0_DIR, OUTPUT);
  pinMode(P1_ENA, OUTPUT);
  pinMode(P1_PUL, OUTPUT);
  pinMode(P1_DIR, OUTPUT);

  // stopper base pins are outputs, to be set permanently up
  pinMode(P0_SBAS, OUTPUT);
  pinMode(P0_SBAS, OUTPUT);
  digitalWrite(P0_SBAS, HIGH);
  digitalWrite(P1_SBAS, HIGH);

  // stopper signal pins are inputs
  pinMode(P0_SSIG_L, INPUT_PULLUP);
  pinMode(P0_SSIG_R, INPUT_PULLUP);
  pinMode(P1_SSIG_L, INPUT_PULLUP);
  pinMode(P1_SSIG_R, INPUT_PULLUP);

}

void loop() {

  // at the start of each loop, check whether any stopper is pressed
  bool b_S0l=0, b_S0r=0, b_S1l=0, b_S1r=0, b_Sor=0;
  b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r);

  for (int i=0; i<500; i++) { // forward motion of both motors at once
    if(b_Sor){continue;}
    digitalWrite(P0_DIR, LOW);
    digitalWrite(P1_DIR, LOW);
    digitalWrite(P0_ENA, HIGH);
    digitalWrite(P1_ENA, HIGH);
    digitalWrite(P0_PUL, HIGH);
    digitalWrite(P1_PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(P0_PUL, LOW);
    digitalWrite(P1_PUL, LOW);
    delayMicroseconds(50);
  }
  for (int i=0; i<500; i++) { // backward motion of both motors at once
    if(b_Sor){continue;}
    digitalWrite(P0_DIR, HIGH);
    digitalWrite(P1_DIR, HIGH);
    digitalWrite(P0_ENA, HIGH);
    digitalWrite(P1_ENA, HIGH);
    digitalWrite(P0_PUL, HIGH);
    digitalWrite(P1_PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(P0_PUL, LOW);
    digitalWrite(P1_PUL, LOW);
    delayMicroseconds(50);
  }

}
