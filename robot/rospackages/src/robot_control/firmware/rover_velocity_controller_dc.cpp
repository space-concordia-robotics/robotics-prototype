#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <geometry_msgs/Twist.h>
#include <ros.h>

#define DC_PIN_FRONT_RIGHT 9
#define DC_PIN_MIDDLE_RIGHT 10
#define DC_PIN_BACK_RIGHT 11

#define DC_PIN_FRONT_LEFT 3
#define DC_PIN_MIDDLE_LEFT 5
#define DC_PIN_BACK_LEFT 6

#define DC_SIGNAL_MIN 127
#define DC_SIGNAL_MIN 0
#define DC_SIGNAL_MAX 255

float MapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void DcCallback(const geometry_msgs::Twist &twist_msg);

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> dc_sub("rover/cmd_vel", DcCallback);

void setup() {
  nh.initNode();
  nh.subscribe(dc_sub);

  //   Serial.begin(9600);
  //   Serial.println("SETUP COMPLETE");
}

void loop() {
  nh.spinOnce();
  delay(1);
}

float MapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DcCallback(const geometry_msgs::Twist &twist_msg) {
  float throttle = twist_msg.linear.x;
  float steering = twist_msg.angular.z;

  // Initialize Left & Right Velocity variables to Rest
  float vel_signal_right = DC_MID_SIGNAL;
  float vel_signal_left = DC_MID_SIGNAL;
  float deg = 0;

  // If statement for CASE 1: steering toward the RIGHT
  if (steering <= 0) {
    deg = MapFloat(steering, -1, 0, -1, 1);
    vel_signal_right = MapFloat(throttle * deg, -1, 1, DC_SIGNAL_MIN, DC_SIGNAL_MAX);
    vel_signal_left = MapFloat((-1) * throttle, -1, 1, DC_SIGNAL_MIN, DC_SIGNAL_MAX);
  }

  // If statement for CASE 2: steering toward the LEFT
  if (steering > 0) {
    deg = MapFloat(steering, 0, 1, 1, -1);
    vel_signal_right = MapFloat(throttle, -1, 1, DC_SIGNAL_MIN, DC_SIGNAL_MAX);
    vel_signal_left = MapFloat((-1) * throttle * deg, -1, 1, DC_SIGNAL_MIN, DC_SIGNAL_MAX);
  }

  // Print Velocity Values
  //   Serial.print("Throttle: ");
  //   Serial.print(throttle);
  //   Serial.print("\tSteering: ");
  //   Serial.println(steering);
  //   Serial.print(steering);
  //   Serial.print(">");
  //   Serial.print(throttle);
  //   Serial.print("o");
  //   Serial.print("Right Side: ");
  //   Serial.print(vel_signal_right);
  //   Serial.print("\tLeft Side: ");
  //   Serial.println(vel_signal_left);

  // Write velocities for the Wheels on the RIGHT side
  analogWrite(DC_PIN_FRONT_RIGHT, vel_signal_right);
  analogWrite(DC_PIN_MIDDLE_RIGHT, vel_signal_right);
  analogWrite(DC_PIN_BACK_RIGHT, vel_signal_right);

  // Write velocities for the Wheels on the Left side
  analogWrite(DC_PIN_FRONT_LEFT, vel_signal_left);
  analogWrite(DC_PIN_MIDDLE_LEFT, vel_signal_left);
  analogWrite(DC_PIN_BACK_LEFT, vel_signal_left);
}