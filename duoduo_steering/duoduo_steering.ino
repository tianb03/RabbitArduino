#include <Servo.h>
#include <ros.h>
#include <dependant_api/robotcmd_motor.h>
#include <dependant_api/arduino_motor.h>

ros::NodeHandle nh;
dependant_api::arduino_motor motor_feedback;
ros::Publisher feedback("/arduino/motor", &motor_feedback);

Servo servo_horizontal;
Servo servo_vertical;

int vertical = 150;
int horizontal = 90;
int pos_vertical = 150;
int pos_horizontal = 90;

void servoCallback(const dependant_api::robotcmd_motor& cloud_terrace)
{
  motor_feedback.id = cloud_terrace.id;
  vertical = cloud_terrace.ver_angle;
  horizontal = cloud_terrace.hor_angle;

  if (vertical >= 130)
    vertical = 130;

  vertical = 180 - vertical;
}

ros::Subscriber<dependant_api::robotcmd_motor> servo("/robotcmd/motor", servoCallback);

void setup()
{
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(servo);
  nh.advertise(feedback);
  servo_vertical.attach(10);
  servo_horizontal.attach(9);
}

void loop()
{
  nh.spinOnce();
  if (horizontal >= 170)
    horizontal = 170;
  if (horizontal <= 10)
    horizontal = 10;
  if (horizontal > pos_horizontal)
    pos_horizontal += 1;
  else if (horizontal < pos_horizontal)
    pos_horizontal -= 1;

  if (vertical >= 170)
    vertical = 170;
  if (vertical <= 50)
    vertical = 50;
  if (vertical > pos_vertical)
    pos_vertical += 1;
  else if (vertical < pos_vertical)
    pos_vertical -= 1;

  servo_vertical.write(pos_vertical);
  servo_horizontal.write(pos_horizontal);

  if (horizontal == pos_horizontal)
    motor_feedback.ret_hor = 0;
  else
    motor_feedback.ret_hor = 1;

  if (vertical == pos_vertical)
    motor_feedback.ret_ver = 0;
  else
    motor_feedback.ret_ver = 1;
  feedback.publish(&motor_feedback);
  delay (20);
}
