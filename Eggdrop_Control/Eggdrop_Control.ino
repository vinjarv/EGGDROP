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
#define PWM 5   //Motor PWM pin
#define DIR2 6  //Motor controller pin2
#define DIR1 7  //Motor controller pin1

// Select experiment or normal control
#define MODE_EXPERIMENT true

#if MODE_EXPERIMENT
  #include "testdata.h"
#else
// Define empty data
const int testdata[0];
#endif


volatile double posi = 0;  // position variable. https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
const float PULSES_TO_MM = TWO_PI * 4.0 / 1024.0; // 4mm shaft radius, 1024 pulses (only pos. flank is detected, encoder in 2048 ppr mode)
int state = 0;

// Experiment variables
int millis_prev;
int data;
unsigned int data_iter;

// Objects
PID pid{1.0, 0.01, 0.0};

void setup() {

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

void main_experiment(const int& pos) {

  // Wait for start signal
  if (Serial.available() > 0) {
    Serial.readStringUntil('\n');
    // Start experiment cycle
    state = 1;
    millis_prev = millis();
    data = 0;
    data_iter = 0;
  }

  if (state == 1) {
    // Sample and output at 2ms = 500Hz
    if (millis() >= millis_prev + 2) {
      millis_prev = millis();
      Serial.print(data);
      Serial.print(',');
      Serial.print(pos);
      Serial.print(',');
      Serial.print(millis());
      Serial.println();
      data = testdata[data_iter];
      setMotor(data);
      data_iter++;
    }

  } else {
    setMotor(0);
  }

  // Stop experiment when end of array is reached
  int data_len = sizeof(testdata) / sizeof(int);
  if (data_iter == data_len) {
    state = 0;
    Serial.println("");  // Send newline to signal stop
  }
}

void main_drop(const int& pos) {
  float pos_mm = pos*PULSES_TO_MM;
  float y = pid.run(0, pos_mm);

  switch (state) {

    // Move platform to 0 manually
    case 0:
      break;

    // Run to 200mm
    case 10:
      break;

    // Wait for start
    case 20:
      break;

    // Drop under constant acceleration
    case 30:
      break;

    // Lower speed to safe value
    case 40:
      break;

    // Stop slightly below 0
    case 50:
      break;
  }

  // Acceleration regulator
  // Velocity reg.
}

void loop() {
  // Read current encoder position
  double pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

#if MODE_EXPERIMENT
  main_experiment(pos);
#else
  main_drop(pos);
#endif
}


//******************************************
//FUNCTIONS FOR MOTOR AND ENCODER

//MOTOR
// Set applied voltage (range -255 to 255)
void setMotor(int val) {
  byte dir = 0 - static_cast<byte>(val < 0) + static_cast<byte>(val > 0);  // Get direction
  analogWrite(PWM, abs(val));

  switch (dir) {
    case 1:
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, LOW);
      break;

    case -1:
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, HIGH);
      break;

    default:
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, LOW);
      break;
  }
}

//READ ENCODER ON INTERRUPT
void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}
