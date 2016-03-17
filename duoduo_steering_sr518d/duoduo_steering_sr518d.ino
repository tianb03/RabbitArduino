#include <Servo.h>
#include <ros.h>
#include <dependant_api/robotcmd_motor.h>
#include <dependant_api/arduino_motor.h>
#include "SpringRC.h"

ros::NodeHandle nh;
dependant_api::arduino_motor motor_feedback;
ros::Publisher feedback("/arduino/motor", &motor_feedback);

int vertical = 150;
int horizontal = 90;
long int pos_vertical = 150;
long int pos_horizontal = 90;

void servoCallback(const dependant_api::robotcmd_motor& cloud_terrace)
{
  motor_feedback.id = cloud_terrace.id;
  vertical = cloud_terrace.ver_angle;
  horizontal = cloud_terrace.hor_angle;

  if (vertical >= 130)
    vertical = 130;

  vertical = 180 - vertical;
  horizontal = 180 - horizontal;
}

ros::Subscriber<dependant_api::robotcmd_motor> servo("/robotcmd/motor", servoCallback);

void setup()
{
  nh.initNode();
  nh.subscribe(servo);
  nh.advertise(feedback);
  SR518.begin(57000, 2);  //配置串口速率为1mbps，设置数字引脚2为串口收发的开关

  //设置柔性边界和斜率
  SR518.setCompliance(0, 1, 1, 8, 8);
  SR518.setCompliance(1, 1, 1, 8, 8);

  //设置运动范围
  SR518.setLimit(0, translateAnalog(0), translateAnalog(180));  //一级保险
  SR518.setLimit(1, translateAnalog(50), translateAnalog(180));
}

void loop()
{
  nh.spinOnce();
  if (horizontal >= 170)  //二级保险
    horizontal = 170;
  if (horizontal <= 10)
    horizontal = 10;
  if (vertical >= 170)
    vertical = 170;
  if (vertical <= 50)
    vertical = 50;

  SR518.moveSpeed(0, translateAnalog(horizontal), 300);
  SR518.moveSpeed(1, translateAnalog(vertical), 300);
  Serial.print(translateAnalog(horizontal));
  Serial.print(",");
  Serial.print(translateAnalog(vertical));
  Serial.print("; ");

  pos_horizontal = translateAngle(SR518.readPosition(0));
  pos_vertical = translateAngle(SR518.readPosition(1));
  Serial.print(pos_horizontal);
  Serial.print(",");
  Serial.println(pos_vertical);

  if (horizontal + 2 >= pos_horizontal && horizontal - 2 <= pos_horizontal)
    motor_feedback.ret_hor = 0;
  else
    motor_feedback.ret_hor = 1;

  if (vertical + 10 >= pos_vertical && vertical - 2 <= pos_vertical)  //此处的数字10不通用
    motor_feedback.ret_ver = 0;
  else
    motor_feedback.ret_ver = 1;
  feedback.publish(&motor_feedback);
}

long int translateAnalog(long int angle)  //将舵机的控制范围由0~300度换算成-60~240度
{
  return (angle + 60) * 1023 / 300;
}

long int translateAngle(long int analog)  //将舵机反馈值转为角度
{
  return analog * 300 / 1023 - 60;
}
