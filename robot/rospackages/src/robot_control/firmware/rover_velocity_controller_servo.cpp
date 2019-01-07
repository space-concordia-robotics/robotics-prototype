#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <geometry_msgs/Twist.h>
#include <ros.h>

#define SERVO_PIN_FRONT_RIGHT 9
#define SERVO_PIN_MIDDLE_RIGHT 10
#define SERVO_PIN_BACK_RIGHT 11

#define SERVO_PIN_FRONT_LEFT 3
#define SERVO_PIN_MIDDLE_LEFT 5
#define SERVO_PIN_BACK_LEFT 6

#define SERVO_SIGNAL_MID 90
#define SERVO_SIGNAL_MIN 0
#define SERVO_SIGNAL_MAX 180

Servo servo_front_right;
Servo servo_middle_right;
Servo servo_back_right;
Servo servo_front_left;
Servo servo_middle_left;
Servo servo_back_left;

float MapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void ServoCallback(const geometry_msgs::Twist &twist_msg);

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> servo_sub("rover/cmd_vel", ServoCallback);

void setup() {
  nh.initNode();
  nh.subscribe(servo_sub);

  // Attach Servos to pins:
  // Right side Servos
  servo_front_right.attach(SERVO_PIN_FRONT_RIGHT);
  servo_middle_right.attach(SERVO_PIN_MIDDLE_RIGHT);
  servo_back_right.attach(SERVO_PIN_BACK_RIGHT);

  // Left side Servos
  servo_front_left.attach(SERVO_PIN_FRONT_LEFT);
  servo_middle_left.attach(SERVO_PIN_MIDDLE_LEFT);
  servo_back_left.attach(SERVO_PIN_BACK_LEFT);

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

void ServoCallback(const geometry_msgs::Twist &twist_msg) {
  float throttle = twist_msg.linear.x;
  float steering = twist_msg.angular.z;

  // Initialize Left & Right Velocity variables to Rest
  float vel_signal_right = SERVO_SIGNAL_MID;
  float vel_signal_left = SERVO_SIGNAL_MID;
  float deg = 0;

  // If statement for CASE 1: steering toward the RIGHT
  if (steering <= 0) {
    deg = MapFloat(steering, -1, 0, -1, 1);
    vel_signal_right = MapFloat(throttle * deg, -1, 1, SERVO_SIGNAL_MIN, SERVO_SIGNAL_MAX);
    vel_signal_left = MapFloat((-1) * throttle, -1, 1, SERVO_SIGNAL_MIN, SERVO_SIGNAL_MAX);
  }

  // If statement for CASE 2: steering toward the LEFT
  if (steering > 0) {
    deg = MapFloat(steering, 0, 1, 1, -1);
    vel_signal_right = MapFloat(throttle, -1, 1, SERVO_SIGNAL_MIN, SERVO_SIGNAL_MAX);
    vel_signal_left = MapFloat((-1) * throttle * deg, -1, 1, SERVO_SIGNAL_MIN, SERVO_SIGNAL_MAX);
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
  servo_middle_right.write(vel_signal_right);
  servo_back_right.write(vel_signal_right);
  servo_front_right.write(vel_signal_right);

  // Write velocities for the Wheels on the Left side
  servo_front_left.write(vel_signal_left);
  servo_middle_left.write(vel_signal_left);
  servo_back_left.write(vel_signal_left);
}