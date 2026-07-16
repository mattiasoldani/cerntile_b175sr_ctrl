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

// number of loops (with no movement) with pressed stopper before breaking loop, for bounce suppression
#define N_LOOP_CHECK 10

// physical length of the two axes
#define L0 26
#define L1 74

// stopper-to-LED boolean: pushing any stopper turns off the built-in LED
#define B_SLED 0

// printout boolean: if true, some info are printed through the serial output
#define B_PRINT 1

// time gaps between motor pulse activation/deactivation
// the higher the delay, the slower the motion
#define DELAY_0 50
#define DELAY_1 25

// pit-stop position in cm
#define X0_PIT 5
#define X1_PIT 63

// test program parameters
// - motors work one at a time, toward one direction at a time, overall tracing a rectangle
// - path length (in number of steps) of each motor can be tuned: TEST_NSTEPS_0/1
// - if stopper corresponding to direction of motion is pressed, motion stops until stopper is not pressed anymore
#define B_TEST 0
#define TEST_NSTEPS_0 10000
#define TEST_NSTEPS_1 15000
#define TEST_DELAY_0 100
#define TEST_DELAY_1 100

////////////////////////////////////////////////////////////////////////////
// dependencies ////////////////////////////////////////////////////////////

bool b_exec = 1, b_finished = 0;

bool b_S0l, b_S0r, b_S1l, b_S1r, b_Sor;

double speed_0_lr;
double speed_0_rl;
double speed_1_lr;
double speed_1_rl;

double x0, x1;

// get booleans from stoppers
// (HIGH = not pressed; LOW = pressed)
// --> returns boolean representing the OR between all stoppers
bool get_stoppers(bool & b_S0l, bool & b_S0r, bool & b_S1l, bool & b_S1r, bool b_led = 0) {

  b_S0l = digitalRead(P0_SSIG_L) == LOW;
  b_S0r = digitalRead(P0_SSIG_R) == LOW;
  b_S1l = digitalRead(P1_SSIG_L) == LOW;
  b_S1r = digitalRead(P1_SSIG_R) == LOW;

  bool bSor = b_S0l||b_S0r||b_S1l||b_S1r;

  if (b_led) {
    if (bSor) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  return (bSor);
  
}

// exert movement step
// --> returns motion time in s
double move_step(int iax, int idir, int delh, int dell) {

      int pdir = iax ? P1_DIR : P0_DIR;
      int pena = iax ? P1_ENA : P0_ENA;
      int ppul = iax ? P1_PUL : P0_PUL;

      digitalWrite(pdir, idir);
      digitalWrite(pena, HIGH);
      digitalWrite(ppul, HIGH);
      double t0 = micros();
      delayMicroseconds(delh);
      digitalWrite(ppul, LOW);
      double t1 = micros();
      delayMicroseconds(dell);

      return (t1 - t0) * 1e-6;

}

// move both motors to starting (L) position
void get_to_zero() {

  b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);

  int i_temp_0 = 0, i_temp_1 = 0;

  for (int i=0; i<1e9; i++) { // backward motion (R to L) of motor 1, all the way to L end
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);
    if (b_S1l) { // to avoid stopper false-positives, requiring two successive press checks
      if (i_temp_1<N_LOOP_CHECK) {i_temp_1++;} else {break;}
      continue;
    } else {i_temp_1 = 0;}
    move_step(1, 0, 100, 100);
  }
  for (int i=0; i<1e9; i++) { // backward motion (R to L) of motor 1, all the way to L end
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);
    if (b_S0l) { // to avoid stopper false-positives, requiring two successive press checks
      if (i_temp_0<N_LOOP_CHECK) {i_temp_0++;} else {break;}
      continue;
    } else {i_temp_0 = 0;}
    move_step(0, 0, 100, 100);
  }

  return;

}

// calibration, i.e. compute motion speed
// --> returns speed in cm/s
double calibrate_speed(int iax, int idir) {

  b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);

  int delh = iax ? DELAY_1 : DELAY_0;
  int dell = iax ? DELAY_1 : DELAY_0;

  double length = (idir ? 1 : -1)*(iax ? L1 : L0);
  double time=0;

  bool b_temp=0;
  int i_temp=0;

  for (int i=0; i<1e9; i++) { // forward motion (L to R) of motor 1, all the way to L end
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);
    if (iax) {b_temp = idir ? b_S1r : b_S1l;}
    else {b_temp = idir ? b_S0r : b_S0l;}
    if (b_temp) { // to avoid stopper false-positives, requiring two successive press checks
      if (i_temp<N_LOOP_CHECK) {i_temp++;} else {break;}
      continue;
    } else {i_temp = 0;}
    time += move_step(iax, idir, delh, dell);
  }

  double speed = length / time;

  if (B_PRINT) {
    if (!iax && idir) Serial.print("measured speed along axis 0 (L to R) [cm/s] = ");
    if (!iax && !idir) Serial.print("measured speed along axis 0 (R to L) [cm/s] = ");
    if (iax && idir) Serial.print("measured speed along axis 1 (L to R) [cm/s] = ");
    if (iax && !idir) Serial.print("measured speed along axis 1 (R to L) [cm/s] = ");
    Serial.print(speed);
    Serial.println();
  }

  return speed;

}

// test mode
void test_mode() {

  Serial.print("simple test mode...");
  Serial.println();

  for (int i=0; i<TEST_NSTEPS_0; i++) { // forward motion (L to R) of motor 0
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);
    if(b_S0r){
      delayMicroseconds(2*TEST_DELAY_0);
      continue;
    }
    move_step(0, 1, TEST_DELAY_0, TEST_DELAY_0);
  }

  for (int i=0; i<TEST_NSTEPS_1; i++) { // forward motion (L to R) of motor 1
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);
    if(b_S1r){
      delayMicroseconds(2*TEST_DELAY_1);
      continue;
    }
    move_step(1, 1, TEST_DELAY_1, TEST_DELAY_1);
  }

  for (int i=0; i<TEST_NSTEPS_0; i++) { // backward motion (R to L) of motor 0
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);
    if(b_S0l){
      delayMicroseconds(2*TEST_DELAY_0);
      continue;
    }
    move_step(0, 0, TEST_DELAY_0, TEST_DELAY_0);
  }

  for (int i=0; i<TEST_NSTEPS_1; i++) { // backward motion (R to L) of motor 1
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);
    if(b_S1l){
      delayMicroseconds(2*TEST_DELAY_1);
      continue;
    }
    move_step(1, 0, TEST_DELAY_1, TEST_DELAY_1);
  }

}

// move one motor to desired (absolute) position in cm
void move_to_axis(int iax, double xnew) {

  int delh = iax ? DELAY_1 : DELAY_0;
  int dell = iax ? DELAY_1 : DELAY_0;

  double x = iax ? x1 : x0;

  double speed;
  if (iax) {speed = (xnew >= x) ? speed_1_lr : speed_1_rl;}
  else {speed = (xnew >= x) ? speed_0_lr : speed_0_rl;}

  int idir = (speed >= 0) ? 1 : 0;

  double timegoal = (xnew - x) / speed;
  double time=0;

  bool b_temp=0;
  int i_temp=0;

  while (time<timegoal) {
    b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);
    if (iax) {b_temp = idir ? b_S1r : b_S1l;}
    else {b_temp = idir ? b_S0r : b_S0l;}
    if (b_temp) { // to avoid stopper false-positives, requiring two successive press checks
      if (i_temp<N_LOOP_CHECK) {
        i_temp++;
      } else {
        if (iax) {x1 = idir ? L1 : 0;}
        else {x0 = idir ? L0 : 0;}
        return;
      }
      continue;
    } else {i_temp = 0;}
    time += move_step(iax, idir, delh, dell);
  }

  if (iax) {x1 = xnew;} else {x0 = xnew;}

}

// move to a new 2D (absolute) position (in cm) and wait a set amount of time (in s)
void move_to_full(double x0new, double x1new, double waittime) {

  move_to_axis(0, x0new);
  move_to_axis(1, x1new);

  if (B_PRINT) {
    Serial.print("current positions [cm]: x0 = ");
    Serial.print(x0);
    Serial.print(" , x1 = ");
    Serial.print(x1);
    Serial.println();
  }

  delay(waittime*1e3);

}

// move to pit-stop position
void get_to_pit_stop() {

  move_to_full(X0_PIT, X1_PIT, 1);

}

// dependencies ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void setup() {

  // serial port for printouts
  if (B_PRINT) Serial.begin(9600);

  // LED on board on
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // all motor pins are outputs, to be programmed (all initialised to LOW)
  pinMode(P0_ENA, OUTPUT);
  pinMode(P0_PUL, OUTPUT);
  pinMode(P0_DIR, OUTPUT);
  pinMode(P1_ENA, OUTPUT);
  pinMode(P1_PUL, OUTPUT);
  pinMode(P1_DIR, OUTPUT);
  digitalWrite(P0_DIR, LOW);
  digitalWrite(P0_ENA, LOW);
  digitalWrite(P0_PUL, LOW);
  digitalWrite(P1_DIR, LOW);
  digitalWrite(P1_ENA, LOW);
  digitalWrite(P1_PUL, LOW);

  // stopper signal pins are inputs
  pinMode(P0_SSIG_L, INPUT_PULLUP);
  pinMode(P0_SSIG_R, INPUT_PULLUP);
  pinMode(P1_SSIG_L, INPUT_PULLUP);
  pinMode(P1_SSIG_R, INPUT_PULLUP);

  // move to position (L, L) before calibration procedure
  get_to_zero();

  // calibration procedure (must be done after getting to zero)
  speed_0_lr = calibrate_speed(0, 1); // axis 0, forward
  speed_0_rl = calibrate_speed(0, 0); // axis 0, backward
  speed_1_lr = calibrate_speed(1, 1); // axis 1, forward
  speed_1_rl = calibrate_speed(1, 0); // axis 1, backward

  // position at the end of setup must be (L, L)
  get_to_zero();
  x0=0;
  x1=0;

}

void loop() {

  // at the start of each loop, check whether any stopper is pressed
  b_S0l=0, b_S0r=0, b_S1l=0, b_S1r=0;
  b_Sor = get_stoppers(b_S0l, b_S0r, b_S1l, b_S1r, B_SLED);

  if (B_TEST) {test_mode();} else 
  {

    if (b_exec) {

////////////////////////////////////////////////////////////////////////////
// main program ////////////////////////////////////////////////////////////

      move_to_full(2, 2, 1);
      move_to_full(10, 10, 1);
      move_to_full(30, -5, 1);
      move_to_full(30, 80, 1);

// main program ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

      b_exec = 0;

    } else {

      if (b_finished) {
        get_to_pit_stop();
        b_finished = 1;
      } else {delay(1000);}
      
    }

  }

}
