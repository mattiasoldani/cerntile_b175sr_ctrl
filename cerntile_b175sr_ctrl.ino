// master, motor
#define P0_ENA 1
#define P0_PUL 1
#define P0_DIR 1

// slave, motor
#define P1_ENA 1
#define P1_PUL 1
#define P1_DIR 1

// master, stoppers
#define P0_SBAS 13
#define P0_SSIG_L 1
#define P0_SSIG_R 1

// slave, stoppers
#define P1_SBAS 10
#define P1_SSIG_L 1
#define P1_SSIG_R 1

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

}
