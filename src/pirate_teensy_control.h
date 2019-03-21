// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _pirate_teensy_control_H_
#define _pirate_teensy_control_H_
#include "Arduino.h"

//add your includes for the project pirate_teensy_control here
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

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();

#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project pirate_teensy_control here
void encodeReset();
void changeAddress();
void dlightSwitch( const bool mode);
long encoder1();
long encoder2();
void setMode( const int mode );
void motor1( const int speed );
void motor2( const int speed );
void motors( const byte speed );
const int volts();
void pirate_power_cb( const std_msgs::Bool &power_msg );
void pirate_md25_encoder_reset( const std_msgs::Bool &encoder_reset_msg );
void pirate_driving_lights( const std_msgs::Bool &driving_lights_msg);
void pirate_md25_motor1_speed_cb( const std_msgs::Int8 &motor1_msg );
void pirate_md25_motor2_speed_cb( const std_msgs::Int8 &motor2_msg );
void pirate_md25_motors_speed_cb( const std_msgs::Int8 &motors_msg );
void cmd_vel_cb( const geometry_msgs::Twist &CmdVel);

/**
 * @caution: all floats are legal as byte is not signed but we are working within -127 to +128!
 */
float twistToMD25( float value, float max );
void logToString( const String msg );
float mapMax( float value, float in_max, float out_max );

//Do not add code below this line
#endif /* _pirate_teensy_control_H_ */
