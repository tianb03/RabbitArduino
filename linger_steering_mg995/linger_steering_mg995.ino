#include <Servo.h>
#include <ros.h>
#include <dependant_api/linger_steering.h>

ros::NodeHandle nh;
Servo servo_head;
Servo servo_left_arm;
Servo servo_right_arm;
int pos_head = 90;
int pos_left_arm = 90;
int pos_right_arm = 90;
int head = 90;
int left_arm = 90;
int right_arm = 90;

void servoCallback(const dependant_api::linger_steering& linger)
{
  head = linger.head;
  left_arm = linger.left_arm;
  right_arm = linger.right_arm;
}

ros::Subscriber<dependant_api::linger_steering> servo("linger_steering", servoCallback);

void setup()
{
  nh.initNode();
  nh.subscribe(servo);
  servo_head.attach(9);
  servo_left_arm.attach(8);
  servo_right_arm.attach(7);
}

void loop()
{
  nh.spinOnce();

  if (head > pos_head && pos_head < 170 && head - pos_head != 1)
    pos_head += 2;
  else if (head < pos_head && pos_head > 10 && head - pos_head != -1)
    pos_head -= 2;

  if (left_arm > pos_left_arm && pos_left_arm < 170 && left_arm - pos_left_arm != 1)
    pos_left_arm += 2;
  else if (left_arm < pos_left_arm && pos_left_arm > 10 && left_arm - pos_left_arm != -1)
    pos_left_arm -= 2;

  if (right_arm > pos_right_arm && pos_right_arm < 170 && right_arm - pos_right_arm != 1)
    pos_right_arm += 2;
  else if (right_arm < pos_right_arm && pos_right_arm > 10 && right_arm - pos_right_arm != -1)
    pos_right_arm -= 2;

  servo_head.write(pos_head);
  servo_left_arm.write(pos_left_arm);
  servo_right_arm.write(pos_right_arm);
}
