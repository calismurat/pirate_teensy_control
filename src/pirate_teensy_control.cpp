/**
 *
 * @author Murat Calis
 * @copyright 2015 Murat Calis, mc@pirate-robotics.net
 * @date 25.07.2015
 * @name pirate_teensy_control.cpp
 * @brief Teensy Controller for the IR, PIR, RELAY and MD25 Motors
 *
 */

// Do not remove the include below
#include "pirate_teensy_control.h"

// ROS
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>             // Encoders left/right
#include <std_msgs/Float32MultiArray.h> // Encoders ensemble
#include <std_msgs/UInt8.h>             // Battery volts / encoder values / sw version
#include <std_msgs/UInt32.h>            // IR-Code from Input IR
#include <std_msgs/Bool.h>              // PIR-State, Power on/off
#include <std_msgs/Int8.h>              // Motor-Speed -127-0 backward / 0-127 forward / 0 stop
#include <geometry_msgs/Twist.h>        // Twist velocity commands
#include <tf/transform_broadcaster.h>   // TF broadcaster
#include <tf/tf.h>                      // TF
#include <nav_msgs/Odometry.h>          // Odometry data (not used!)
#include <std_msgs/String.h>            // Debug strings

// ARDUINO
#include <IRremote.h>
#include <Wire.h>

// ANALOG/DIGITAL PIN
#define LED_DEFAULT_PIN 13
#define IR_RECEIVER_PIN 2
#define RELAY_SONGLE_PIN 3
#define PIR_PIN 4

// MD25 Motors
#define CMD                 16      // DEFAULT (byte)0x00 Values of 0 being sent using write have to be cast as a byte to stop them being misinterperted as NULL. This is a must but with arduino 1
#define MD25ADDRESS         0x58    // 0x58 7-Bit address of the MD25 Default (0xB0)
#define SOFTWAREREG         0x0D    // Byte to read the software version
#define SPEED1              0       // Byte to send speed to first motor
#define SPEED2              1       // Byte to send speed to second motor
#define ENCODERONE          0x02    // 0x02 Byte to read motor encoder 1
#define ENCODERTWO          0x06    // 0x06 Byte to read motor encoder 2
#define VOLTREAD            0x0A    // 0x0A Byte to read battery volts
#define RESETENCODERS       32      // DEFAULT 0x20
#define MODE_REGISTER       15      // MD25 Operation Mode Register. We use Mode-Value 1: motors -127 +128

// ODOMETRY
#define MAX_CMD_VELOCITY 1.1        // HARDWARE LIMITER: Velocity in m/s
#define MAX_MD25_SPEED 30           // HARDWARE LIMITER: Maximum speed in Byte-range of MD25 speed register
#define WHEEL_ROTATION_TICKS 360;   // Encoder ticks per rotation.

// ARDUINO
#define LOOP_FREQ 100               // Used in loop() as delay
#define DEBUG_MSG 0                 // Used in logToString on/off - 1/0

// ROS Node for all sub/pub
ros::NodeHandle nh;

// P-IR msg
std_msgs::UInt32 int_ir_code_rx;
std_msgs::Bool bol_pir_state;

// MD25 values
std_msgs::Int32 int_encoder1;
std_msgs::Int32 int_encoder2;
std_msgs::Float32MultiArray flo_encoders;
std_msgs::UInt8 int_md25_voltage;
std_msgs::String str_debug;

// ROS Publisher
ros::Publisher pirate_ir_rx("pirate_ir_rx", &int_ir_code_rx);
ros::Publisher pirate_pir_state("pirate_pir_state", &bol_pir_state);
ros::Publisher enc1("pirate_md25_encoder1", &int_encoder1);
ros::Publisher enc2("pirate_md25_encoder2", &int_encoder2);
ros::Publisher encoders("pirate_md25_encoders", &flo_encoders);
ros::Publisher pirate_md25_voltage("pirate_md25_voltage", &int_md25_voltage);
ros::Publisher pirate_debug_string("pirate_debug_string", &str_debug);

// ROS Subscriber
ros::Subscriber<std_msgs::Bool> PowerSub("pirate_power", &pirate_power_cb );
ros::Subscriber<std_msgs::Bool> EncoderResetSub("pirate_md25_encoder_reset", &pirate_md25_encoder_reset );
ros::Subscriber<std_msgs::Bool> DrivingLightsSub("pirate_driving_lights", &pirate_driving_lights );
ros::Subscriber<std_msgs::Int8> MD25_Motor1_Sub("pirate_md25_motor1_speed", &pirate_md25_motor1_speed_cb );
ros::Subscriber<std_msgs::Int8> MD25_Motor2_Sub("pirate_md25_motor2_speed", &pirate_md25_motor2_speed_cb );
ros::Subscriber<std_msgs::Int8> MD25_Motors_Sub("pirate_md25_motors_speed", &pirate_md25_motors_speed_cb );
ros::Subscriber<geometry_msgs::Twist> Twist_Sub("cmd_vel", &cmd_vel_cb );

// IR Setup
IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;

// PIR Setup
int PIR_stat = LOW;
int PIR_val = 0;

// Odometry Setup
float wheels_track = 0.204;          // Track = distance between both wheels starting from the middle of rubber tread (26mm).
float wheel_diameter = 0.1;          // 100mm
int wheel_ticks = 360;               // Ticks per revolution

int result_vel_right = 0;            // Final motor speed vars must comply with motor1/2() argument type "integer"
int result_vel_left = 0;

long last_encoder_left = 0;          // previous encoder left
long last_encoder_right = 0;         // previous encoder right

long x = 0;
long y = 0;
double th = 0;

unsigned long last_time = millis();  // used in time delta calculation

// Arduino Setup
void setup()
{
    nh.initNode();

    nh.subscribe(PowerSub);
    nh.subscribe(EncoderResetSub);
    nh.subscribe(DrivingLightsSub);
    nh.subscribe(MD25_Motor1_Sub);
    nh.subscribe(MD25_Motor2_Sub);
    nh.subscribe(MD25_Motors_Sub);
    nh.subscribe(Twist_Sub);

    nh.advertise(enc1);
    nh.advertise(enc2);
    nh.advertise(encoders);
    nh.advertise(pirate_md25_voltage);
    nh.advertise(pirate_ir_rx);
    nh.advertise(pirate_pir_state);
    nh.advertise(pirate_debug_string);

    // IR
    irrecv.enableIRIn();

    // Relay
    pinMode(RELAY_SONGLE_PIN, OUTPUT);
    digitalWrite(RELAY_SONGLE_PIN, LOW);

    // PIR
    pinMode(PIR_PIN, INPUT);

    // I2C for MD25
    Wire.begin();

    // RESET MD25 Encoders for the beginning
    encodeReset();

    // PREALLOCATE ENCODER DATATYPE MEMORY
    flo_encoders.data = (float *)malloc(sizeof(float)*2);
    flo_encoders.data_length = 2;

    // MODE = -127(max-reverse) 0(stop) +127(max-forward)
    setMode(1);
}


// The loop function is called in an endless loop
void loop()
{
    // IR receiver signal
    if (irrecv.decode(&results))
    {
      // Publish IR-Code to ROS
      int_ir_code_rx.data = results.value;
      pirate_ir_rx.publish( &int_ir_code_rx);

      // IR results are retrieved from a really old CD-Player, use your own values here.
      if (results.value == 3191950770 || results.value == 2534555274)
      {
        // Turn ON RELAY
        digitalWrite(RELAY_SONGLE_PIN, HIGH);
        delay(100);
      }
      else if ( results.value == 3191967090 || results.value == 2488819018)
      {
        // Turn OFF RELAY
        digitalWrite(RELAY_SONGLE_PIN, LOW);
        delay(100);
      }
      irrecv.resume();   // Receive the next value
    }

    // PIR
    PIR_val = digitalRead(PIR_PIN);

    if (PIR_val == HIGH)
    {
      // PIR ON
      if (PIR_stat == LOW)
      {
        PIR_stat = HIGH;

        // PIR 2 ROS
        bol_pir_state.data = true;
        pirate_pir_state.publish( &bol_pir_state);
      }
    }
    else
    {
      // PIR OFF
      if (PIR_stat == HIGH){
        PIR_stat = LOW;

        // PIR 2 ROS
        bol_pir_state.data = false;
        pirate_pir_state.publish( &bol_pir_state);
      }
    }

  // 1 = right, 2 = left
  flo_encoders.data[0] = -1 * encoder1();
  flo_encoders.data[1] = -1 * encoder2();
  encoders.publish( &flo_encoders );

  // Publish voltage
  int_md25_voltage.data = volts();
  pirate_md25_voltage.publish( &int_md25_voltage );

  // ROS loop once, arduino delay
  nh.spinOnce();
  delay(LOOP_FREQ);
}

// @TODO: Work in progress!
// Turn ON/OF driving lights
void dlightSwitch( const bool mode ){
  if ( mode == true )
  {
    // Turn ON LIGHTS
    digitalWrite(RELAY_SONGLE_PIN, HIGH);
    delay(100);
  }
  else if ( mode == false )
  {
    // Turn OFF LIGHTS
    digitalWrite(RELAY_SONGLE_PIN, LOW);
    delay(100);
  }
}


// Set MD25 Encoders Reset
void encodeReset(){
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(RESETENCODERS);
  Wire.endTransmission();
}


// Get MD25 Encoder1 Value
long encoder1(){
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ENCODERONE);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);
  long position;
  position = Wire.read();
  position <<= 8;
  position += Wire.read();
  position <<= 8;
  position += Wire.read();
  position <<= 8;
  position  +=Wire.read();

  delay(1);

  return(position);
}


// Get MD25 Encoder2 Value
long encoder2(){
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ENCODERTWO);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);
  long position;
  position = Wire.read();
  position <<= 8;
  position += Wire.read();
  position <<= 8;
  position += Wire.read();
  position <<= 8;
  position  +=Wire.read();

  delay(1);

  return(position);
}


// Set MD25 MODE
void setMode( int mode ){
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(MODE_REGISTER);
    Wire.write(mode);
    Wire.endTransmission();

    // STOP wheels (security), historical: we used to read currentmode and got turning wheels, so immediately stopped here. This doesn't hurt you.
    motor1(0);
    motor2(0);
}


// Set MD25 Motor1 speed
void motor1( int speed ){
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED1);
    Wire.write(speed);
    Wire.endTransmission();
}


// Set MD25 Motor2 speed
void motor2( int speed ){
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED2);
    Wire.write(speed);
    Wire.endTransmission();
}


// Set MD25 Motors speed
void motors( int speed ){
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED1);
    Wire.write(speed);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED2);
    Wire.write(speed);
    Wire.endTransmission();
}


// Get MD25 Voltage
const int volts(){
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(VOLTREAD);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 1);
  int batteryVolts = Wire.read();

  delay(1);

  return batteryVolts;
}


// Switch the relay on/off
void pirate_power_cb( const std_msgs::Bool &power_msg){
  if( power_msg.data == true ){
    digitalWrite(RELAY_SONGLE_PIN, HIGH);
  }else if( power_msg.data == false ){
    digitalWrite(RELAY_SONGLE_PIN, LOW);
  }
  delay(100);
}


// Set the encoders back to 0
void pirate_md25_encoder_reset( const std_msgs::Bool &encoder_reset_msg){
  if( encoder_reset_msg.data == true ){
    encodeReset();
    delay(1);
  }
}


// Turn on/off driving lights front/back
void pirate_driving_lights( const std_msgs::Bool &driving_lights_msg){
  dlightSwitch( driving_lights_msg.data );
  delay(1);
}


// ROS motor1 speed callback
void pirate_md25_motor1_speed_cb( const std_msgs::Int8 &motor1_msg){
  if( motor1_msg.data >= -127 ){
    motor1( motor1_msg.data );
    delay(1);
  }
}


// ROS motor2 speed callback
void pirate_md25_motor2_speed_cb( const std_msgs::Int8 &motor2_msg){
  if( motor2_msg.data >= -127 ){
    motor2( motor2_msg.data );
    delay(1);
  }
}


// ROS all motors speed callback
void pirate_md25_motors_speed_cb( const std_msgs::Int8 &motors_msg){
  if( motors_msg.data >= -127 ){
    motors( motors_msg.data );
    delay(1);
  }
}


// ROS cmd_vel callback function
void cmd_vel_cb( const geometry_msgs::Twist &cmdvel_msg){
  // Debug
  String msg;

  // Precision calculated motor speed vars
  float vel_right;
  float vel_left;

  // Twist vars
  double vel_x = cmdvel_msg.linear.x;
  double vel_th = cmdvel_msg.angular.z;

  // LOG DEBUG
  msg += "cmdvel_msg.linear.x: ";
  msg += vel_x;
  msg += " cmdvel_msg.angular.z: ";
  msg += vel_th;

  // Turn left/right
  if( vel_x == 0 ){
    vel_right = vel_th * wheels_track / 2.0;
    vel_left = -vel_right;
  }
  // Forward/Backward x
  else if( vel_th == 0 ){
    vel_left = vel_right = vel_x;
  }
  // Move arcs
  else{
    vel_left = vel_x - vel_th * wheels_track / 2.0;
    vel_right = vel_x + vel_th * wheels_track / 2.0;
  }

  // Convert to MD25 motor speed values
  msg += " velocity right: ";
  vel_right = twistToMD25( vel_right, MAX_CMD_VELOCITY );
  msg += vel_right;

  msg += " velocity left: ";
  vel_left = twistToMD25( vel_left, MAX_CMD_VELOCITY );
  msg += vel_left;

  // Convert to integer
  result_vel_left = vel_left;
  result_vel_right = vel_right;

  logToString( msg );

  // Invert motor speed values, maybe I plugged them on the wrong side?
  motor2( -1 * result_vel_left );
  motor1( -1 * result_vel_right );
}


// Convert x,y, theta to byte range value between -127 - +128
float twistToMD25( float value, float max ){
  float result;

  // MIN-MAX hard limiting
  value = constrain( value, -1.0 * max, max );

  // MIN-MAX MD25 motor-speed mapping in Mode-1 = -127 to +128
  if( value > 0.2 || value < -0.2 ){
    result = mapMax( value, max, MAX_MD25_SPEED );
  }else{
    result = (127 / max) * value;
  }

  // MIN signed byte hard limiting
  if( result < ( -1 * MAX_MD25_SPEED ) ){
    result = ( -1 * MAX_MD25_SPEED );
  }
  return result;
}


// Simple min-max conversion
float mapMax( float value, float in_max, float out_max ){
  return (out_max / in_max) * value;
}


// ROS string debug msg
void logToString( const String msg ){
  if( DEBUG_MSG ){
    int str_length = msg.length() + 1;
    char msg_buffer[ str_length ];
    msg.toCharArray( msg_buffer, str_length );

    str_debug.data = msg_buffer;
    pirate_debug_string.publish(&str_debug);
  }
}
