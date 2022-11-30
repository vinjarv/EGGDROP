//*************************************
// DC Motor PID position control example
// By Øystein Bjelland, IIR, NTNU
// Based on this example: https://curiores.com/dc-motor-control/
//**************************************

#include <util/atomic.h>
#include "PID.h"

//**************************************

#define ENCA 2  //Encoder pinA
#define ENCB 3  //Encoder pinB
#define ENCX 8  //Encoder pinX
#define PWM 5   //Motor PWM pin
#define DIR2 6  //Motor controller pin2
#define DIR1 7  //Motor controller pin1

// Select experiment or normal control
#define MODE_EXPERIMENT false

#if MODE_EXPERIMENT
  #include "testdata.h"
#else
// Define empty data
  const int testdata[0] PROGMEM;
#endif


volatile long posi = 0;  // position variable. https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
const float PULSES_TO_MM = TWO_PI * (4.0+0.4) / 512.0; // 4mm shaft radius, 512 pulses (encoder in 512 ppr mode, max 2048)
int state = 0;

unsigned long millis_prev;
unsigned long t0, t_start;
int d;
unsigned long data_iter;

// Objects
//      Kp   Ki  Kd
PID pid{-30, -0, 1}; 


//******************************************
//FUNCTIONS FOR GENERATING MOTIONS

// Smoothstep function - interpolate between edge0 and edge1 in a smooth S-curve
// https://en.wikipedia.org/wiki/Smoothstep
float smoothstep(float edge0, float edge1, float x)
{
  if (x<edge0)
    return 0;
  if (x>edge1)
    return 1;
  x = (x - edge0) / (edge1 - edge0);
  return x * x * (3 - 2*x);
}

// Returns pos (mm) from a continous curve to maximize speed
// -0.9g down, 3g up, bottom 5mm, v_bottom -75mm works
float getPosFromCurve(float t)
{
  // a0, a1, v3 are manually selected first
  // t1 and t2 are generated in MATLAB from the equations of motion
  
  // Starting position, accelerating down
  float a0 = -1.35*9.81e3;
  float v0 = 0;
  // Start height
  float s0 = 205;

  // Drop finished, accelerating up
  float t1 = 0.18423977861673067071868744681262;
  float a1 = 10*9.81e3;
  float v1 = v0 + a0*t1;
  float s1 = s0 + v0*t1 + 0.5*a0*t1*t1;

  // Braking done, lower at constant vel.
  float t2 = 0.023343318125491056449163478102578;
  float s2 = s1 + v1*t2 + 0.5*a1*t2*t2;

  // End velocity target
  float v3 = -0.150;
  
  if (t < 0)
    t = 0;

  // Piecewise function
  if (0 <= t and t < t1)
  {
    return s0 + v0*t + 0.5*a0*t*t;
  }
  else if (t1 <= t and t < (t1+t2))
  {
    return s1 + v1*(t-t1) + 0.5*a1*(t-t1)*(t-t1);
  }
  else
  {
    return s2 + v3*(t-(t1+t2));
  }
  
}


void setup() 
{
  Serial.begin(115200);

  // ENCODER
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);  //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

  // DC MOTOR
  pinMode(PWM, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  
  pid.set_output_bounds(-255, 255);
}

// Main function for running a system identification experiment
void main_experiment(const long& pos) 
{
  // Wait for start signal
  if (Serial.available() > 0) {
    Serial.readStringUntil('\n');
    // Start experiment cycle
    state = 1;
    millis_prev = millis();
    d = 0;
    data_iter = 0;
  }

  if (state == 1) {
    // Sample and output at 5ms = 200Hz
    if (millis() >= millis_prev + 5) {
      millis_prev = millis();
      Serial.print(d);
      Serial.print(',');
      Serial.print(pos);
      Serial.print(',');
      Serial.print(millis());
      Serial.println();
      d = pgm_read_word_near(testdata + data_iter);
      setMotor(d);
      data_iter++;
    }

  } else {
    setMotor(0);
  }
  
  // Stop experiment when end of array is reached
  unsigned long data_len = sizeof(testdata) / sizeof(int);
  if (data_iter >= data_len) {
    state = 0;
    data_iter = 0;
    Serial.println("");  // Send newline to signal stop
  }
}

// Main function for the eggdrop
void main_drop(const long& pos) 
{
  const float pos_mm = pos*PULSES_TO_MM;
  float pos_r = 0;

  // Positions
  const float pos_top = 205;
  const float pos_bottom = -0.005;
  
  switch (state) {

    // Move platform to 0 manually
    case 0:
      if (Serial.available() > 0){
        Serial.readStringUntil('\n');
        Serial.println("Zeroed, moving to start pos.");
        zeroEncoder();
        pos_r = 0;
        t0 = millis();
        state = 10;
      }
      break;

    // Run to 200mm
    case 10:
      {
        float t = (millis()-t0)*1e-3;
        float s = smoothstep(0, 1.2, t);
        if (s >= 1){
          // Top reached
          Serial.println("Top reached, pos = " + String(pos_mm));
          state = 20;
        }
        pos_r = pos_top * s;
      }
      break;

    // Wait for start
    case 20:
      {
        pos_r = pos_top;
        if (Serial.available() > 0){
          Serial.readStringUntil('\n');
          Serial.println("Drop started");
          t_start = millis();
          t0 = millis();
          state = 30;
        }
      }
      break;

    // Run drop profile
    case 30:
    {
      float t = (millis()-t0)*1e-3;
      float s = getPosFromCurve(t);
      if (s <= pos_bottom){
        Serial.println("Drop finished");
        Serial.print("Final time: ");
        Serial.println(millis() - t_start);
        Serial.println("ms");
        state = 0;
      }
      pos_r = s;
      Serial.println(s);
    }
    break;
  }

  float y = pid.run(pos_r, pos_mm);
  setMotor(y);
}

// Entrypoint
void loop() {
  // Read current encoder position
  long pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // Select main code at compile time
  #if MODE_EXPERIMENT
    main_experiment(pos);
  #else
    main_drop(pos);
  #endif
}


//******************************************
//FUNCTIONS FOR MOTOR AND ENCODER

// Set applied motor voltage 
// (range -255 to 255)
void setMotor(int val) {
  analogWrite(PWM, abs(val));
  digitalWrite(DIR1, val>0);
  digitalWrite(DIR2, val<0);
}

//Read encoder on interrupt - not called manually
void readEncoder() {
  if(PINE & 1 << 5){ // Fastest version - read port E, pin 5
    posi++;
  } else {
    posi--;
  }
}

// Reset encoder pos.
void zeroEncoder() 
{
  posi = 0;
}
