#include <HardWire.h>
#include <VL53L0X.h>
#include <I2C_MPU6886.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <util/atomic.h>
#include "PID.h"

// ---------- Objects ----------

VL53L0X range_sensor{};
I2C_MPU6886 imu{I2C_MPU6886_DEFAULT_ADDRESS, Wire};

IPAddress ip(192, 168, 10, 240);
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
EthernetUDP udp_server{};
char packet_buffer[UDP_TX_PACKET_MAX_SIZE];

PID pid_motor {-120, 0, 4}; // Scaled up 4x from eggdrop project
PID pid_position {0, 400, -0.1};

// ---------- IO ----------

#define ENCA 2  //Encoder pinA
#define ENCB 3  //Encoder pinB
#define ENCX 8  //Encoder pinX
#define PWM 5   //Motor PWM pin
#define DIR2 6  //Motor controller pin2
#define DIR1 7  //Motor controller pin1

// ---------- Variables ----------

volatile long posi = 0;  // Position variable. https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
float pos_m = 0.200; // Position in m
float r_pos_m = 0.200f; // Position setpoint
const float PULSES_TO_RAD = TWO_PI / 512.0f; // Pulses per revolution
float pos0; // Initial position when motor run command is received
float t0; // Time in milliseconds when motor run command is received

// ---------- Program ----------

void setup()
{
  // ENCODER
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);  //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  // DC MOTOR
  pinMode(PWM, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pid_motor.set_output_bounds(-255, 255);
  //pid_position.set_output_bounds(-10, 10);
  
  Serial.begin(115200);
  Wire.begin();
  imu.begin();  
  Ethernet.begin(mac, ip);
  
  if(!initialize())
     while(true)
       delay(10);

  range_sensor.startContinuous();
  udp_server.begin(8888);
  Serial.println("Setup complete");
}

// Initialize range sensor and ethernet shield
bool initialize()
{
  range_sensor.setTimeout(500);
  if(!range_sensor.init())
  {
    Serial.println("Failed to detect and initialize range sensor!");
    return false;
  }
  if(Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    Serial.println("Ethernet shield was not found.");
    return false;
  }
  else if(Ethernet.hardwareStatus() == EthernetW5500) 
    Serial.println("Found W5500 ethernet shield");

  if(Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Ethernet::LinkOff: is the cable connected?");
    return false;
  }
  else if(Ethernet.linkStatus() == LinkON)
    Serial.println("Ethernet::LinkOn");
  return true;
}


void loop()
{
  // Read current encoder position
  long pos_counts = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_counts = posi;
  }
  const float pos_rad = PULSES_TO_RAD * pos_counts;

  // If data is ready from external PC, update current position
  if ( udp_server.parsePacket() ) {
    pos_m = update_kf();
  }

  // Check if run command is sent
  // When motor run command is received, store current time and position
  if (Serial.available() > 0 and t0 == 0) {
    String serial_data = Serial.readStringUntil('\n');
    if (serial_data == "run") {
      Serial.println("Running");
      t0 = millis();
      pos0 = pos_m;
    }
    // If serial data is a number inside a valid range, treat it as a requested change in setpoint
    float serial_num = serial_data.toFloat();
    if (serial_num > 0.050 and serial_num < 0.250) {
      r_pos_m = serial_num;
    }
  }

  // If running, get position setpoint from a function f(p0, t-t0)
  if (t0 != 0) {
    r_pos_m = motion_function(pos0, (millis()-t0)*1e-3);
  }
  
  // Run PIDs to control motor
  float r_pos_rad = pid_position.run(r_pos_m, pos_m);
  float u_motor = pid_motor.run(r_pos_rad, pos_rad);
  setMotor(u_motor);
//  Serial.println("----");
//  Serial.println("SP: " + String(r_pos_m, 5));
//  Serial.println("PV: " + String(pos_m, 5));
//  Serial.println("Pos. error: " + String(r_pos_m - pos_m, 5));
//  Serial.println("Angle output: " + String(r_pos_rad));
}

// ---------- Functions ----------

// Send sensor data to external program, receive estimated position
float update_kf()
{
  float accel[3];
  //float gyro[3];
  //float t;
  float d;
  imu.getAccel(&accel[0], &accel[1], &accel[2]);
  //imu.getGyro(&gyro[0], &gyro[1], &gyro[2]);
  //imu.getTemp(&t);
  d = range_sensor.readRangeContinuousMillimeters();

  String sensor_values;
  sensor_values.concat(String(accel[0], 6) + ',');
  sensor_values.concat(String(accel[1], 6) + ',');
  sensor_values.concat(String(accel[2], 6) + ',');
  sensor_values.concat(d);

  udp_server.read(packet_buffer, UDP_TX_PACKET_MAX_SIZE);
  float estimate = String(packet_buffer).toFloat();
  Serial.print("Est: ");Serial.print(estimate*1e3, 2);Serial.println("mm");
  Serial.println("Accel Z: " + String(accel[2] * 9.81, 10));
  
  udp_server.beginPacket(udp_server.remoteIP(), udp_server.remotePort());
  udp_server.write(sensor_values.c_str(), sensor_values.length());
  udp_server.endPacket();
  
  return estimate;
}

float smoothstep(float edge0, float edge1, float x)
{
  if (x<edge0)
    return 0;
  if (x>edge1)
    return 1;
  x = (x - edge0) / (edge1 - edge0);
  return x * x * (3 - 2*x);
}

// Predefined motion - move up and down 5x and stabilize at 0.100m
float motion_function(float pos_start, float t)
{
  // Interpolate from start to sine - 2s
  if (t > 0 and t <= 2) {
    float interp = smoothstep(0, 2, t);
    return pos_start * (1 - interp) + 0.050 * interp;
  }
  // Sine wave of 0.1m at 1Hz - 5s
  if (t > 2 and t <= 7) {
    float sine_val = sin( TWO_PI * 1 * (t-2) + HALF_PI );
    if (sine_val !=0)
      return 0.100 + 0.050 * sine_val / abs(sine_val);
    else
      return 0.100;
  }
  // Interpolate from bottom of sine wave to 0.1m - 1s
  if (t > 7) {
    float interp = smoothstep(7, 9, t);
    return 0.050 * (1 - interp) + 0.100 * interp;
  }
}


// ---------- Functions for motor and encoder ----------

// Set applied voltage (range -255 to 255)
void setMotor(int val) 
{
  analogWrite(PWM, abs(val));
  
//  if(val > 0){
//      digitalWrite(DIR1, HIGH);
//      digitalWrite(DIR2, LOW);
//  } else if(val < 0){
//      digitalWrite(DIR1, LOW);
//      digitalWrite(DIR2, HIGH);
//  } else{
//      digitalWrite(DIR1, LOW);
//      digitalWrite(DIR2, LOW);
//  }
  digitalWrite(DIR1, val>0);
  digitalWrite(DIR2, val<0);
}

// Read encoder on interrupt
void readEncoder() 
{
  //int b = digitalRead(ENCB);
  //if (b > 0) {
  //if (bitRead(PINE, 5)){ // Port E5, "pin" 3
  if(PINE & 1 << 5){ // Fastest version
    posi++;
  } else {
    posi--;
  }
}

// Reset global encoder count
void zeroEncoder() 
{
  posi = 0;
}
