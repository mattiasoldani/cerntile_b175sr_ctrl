// master, motor
#define P0_ENA 4
#define P0_DIR 3
#define P0_PUL 2

// slave, motor
#define P1_ENA 7
#define P1_DIR 6
#define P1_PUL 5

// master, stoppers
#define P0_SSIG_L 8
#define P0_SSIG_R 9

// slave, stoppers
#define P1_SSIG_L 10
#define P1_SSIG_R 11

// test #0 parameters
// - pushing any stopper turns off the built-in LED
// - motors are not moved at all
#define B_TEST00 0

// test #1 program parameters
// - motors work one at a time, toward one direction at a time, overall tracing a rectangle
// - path length (in number of steps) of each motor can be tuned: TEST01_NSTEPS_0/1
// - if stopper corresponding to direction of motion is pressed, motion stops until stopper is not pressed anymore
#define B_TEST01 1
#define TEST01_NSTEPS_0 3000
#define TEST01_NSTEPS_1 5000
#define TEST01_DELAY_HIGH_0 10
#define TEST01_DELAY_LOW_0 1000
#define TEST01_DELAY_HIGH_1 100
#define TEST01_DELAY_LOW_1 100

// get to zero at the beginning?
#define B_ZERO 1

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

// move both motors to starting (L) position
void get_to_zero() {

  bool b_S0l=0, b_S0r=0, b_S1l=0, b_S1r=0;
  bool b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r);

  bool b_temp_0 = false, b_temp_1 = false;
  bool b_end_0 = false, b_end_1 = false;

  while (!b_end_1) { // backward motion (R to L) of motor 1 until 0
    if (b_S1l) { // to avoid stopper false-positives, requiring two successive press checks
      if (!b_temp_1) {b_temp_1 = true;} else {b_end_1 = true;}
      continue;
    } else {b_temp_1 = false;}
    digitalWrite(P1_DIR, LOW);
    digitalWrite(P1_ENA, HIGH);
    digitalWrite(P1_PUL, HIGH);
    delayMicroseconds(20);
    digitalWrite(P1_PUL, LOW);
    delayMicroseconds(100);
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r);
  }
  while (!b_end_0) { // backward motion (R to L) of motor 0 until 0
    if (b_S0l) { // to avoid stopper false-positives, requiring two successive press checks
      if (!b_temp_0) {b_temp_0 = true;} else {b_end_0 = true;}
      continue;
    } else {b_temp_0 = false;}
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r);
    digitalWrite(P0_DIR, LOW);
    digitalWrite(P0_ENA, HIGH);
    digitalWrite(P0_PUL, HIGH);
    delayMicroseconds(20);
    digitalWrite(P0_PUL, LOW);
    delayMicroseconds(100);
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r);
  }

  return;

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

  // stopper signal pins are inputs
  pinMode(P0_SSIG_L, INPUT_PULLUP);
  pinMode(P0_SSIG_R, INPUT_PULLUP);
  pinMode(P1_SSIG_L, INPUT_PULLUP);
  pinMode(P1_SSIG_R, INPUT_PULLUP);

  if (B_ZERO) get_to_zero();

}

void loop() {

  // at the start of each loop, check whether any stopper is pressed
  bool b_S0l=0, b_S0r=0, b_S1l=0, b_S1r=0;
  bool b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r);

  if (B_TEST00) {
    if (b_Sor) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  if (B_TEST01) {
    for (int i=0; i<TEST01_NSTEPS_0; i++) { // forward motion (L to R) of motor 0
      b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r);
      if(b_S0r){
        delayMicroseconds(TEST01_DELAY_HIGH_0+TEST01_DELAY_LOW_0);
        continue;
      }
      digitalWrite(P0_DIR, HIGH);
      digitalWrite(P0_ENA, HIGH);
      digitalWrite(P0_PUL, HIGH);
      delayMicroseconds(TEST01_DELAY_HIGH_0);
      digitalWrite(P0_PUL, LOW);
      delayMicroseconds(TEST01_DELAY_LOW_0);
    }
    for (int i=0; i<TEST01_NSTEPS_1; i++) { // forward motion (L to R) of motor 1
      b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r);
      if(b_S1r){
        delayMicroseconds(TEST01_DELAY_HIGH_1+TEST01_DELAY_LOW_1);
        continue;
      }
      digitalWrite(P1_DIR, HIGH);
      digitalWrite(P1_ENA, HIGH);
      digitalWrite(P1_PUL, HIGH);
      delayMicroseconds(TEST01_DELAY_HIGH_1);
      digitalWrite(P1_PUL, LOW);
      delayMicroseconds(TEST01_DELAY_LOW_1);
    }
    for (int i=0; i<TEST01_NSTEPS_0; i++) { // backward motion (R to L) of motor 0
      b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r);
      if(b_S0l){
        delayMicroseconds(TEST01_DELAY_HIGH_0+TEST01_DELAY_LOW_0);
        continue;
      }
      digitalWrite(P0_DIR, LOW);
      digitalWrite(P0_ENA, HIGH);
      digitalWrite(P0_PUL, HIGH);
      delayMicroseconds(TEST01_DELAY_HIGH_0);
      digitalWrite(P0_PUL, LOW);
      delayMicroseconds(TEST01_DELAY_LOW_0);
    }
    for (int i=0; i<TEST01_NSTEPS_1; i++) { // backward motion (R to L) of motor 1
      b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r);
      if(b_S1l){
        delayMicroseconds(TEST01_DELAY_HIGH_1+TEST01_DELAY_LOW_1);
        continue;
      }
      digitalWrite(P1_DIR, LOW);
      digitalWrite(P1_ENA, HIGH);
      digitalWrite(P1_PUL, HIGH);
      delayMicroseconds(TEST01_DELAY_HIGH_1);
      digitalWrite(P1_PUL, LOW);
      delayMicroseconds(TEST01_DELAY_LOW_1);
    }
  }

}
