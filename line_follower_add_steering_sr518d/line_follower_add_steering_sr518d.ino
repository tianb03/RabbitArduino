#include <ros.h>
#include <std_msgs/String.h>
#include <dependant_api/robotcmd_motor.h>
#include <dependant_api/arduino_motor.h>
#include <dependant_api/Int16Array.h>
#include "SpringRC.h"

dependant_api::Int16Array raw;
ros::Publisher ir_line_scanner_pub("ir_raw_data", &raw);
int sensor_number = 11;
int print_value;

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
  nh.advertise(ir_line_scanner_pub);
  SR518.begin(57000, 2, 3);  //配置串口速率为1mbps，设数字引脚2,3为收发和使能端口，2,3电平始终相等

  //设置柔性边界和斜率
  SR518.setCompliance(0, 1, 1, 8, 8);
  SR518.setCompliance(1, 1, 1, 8, 8);

  //设置运动范围
  SR518.setLimit(0, translateAnalog(-15), translateAnalog(195));  //一级保险
  SR518.setLimit(1, translateAnalog(50), translateAnalog(180));
}

void loop()
{
  int i, j, k, raw_data[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  raw_data[0] = analogRead(0);
  raw_data[1] = analogRead(1);
  raw_data[2] = analogRead(2);
  raw_data[3] = analogRead(3);
  raw_data[4] = analogRead(4);
  raw_data[5] = analogRead(5);
  raw_data[6] = analogRead(6);
  raw_data[7] = analogRead(7);
  raw_data[8] = analogRead(8);
  raw_data[9] = analogRead(9);
  raw_data[10] = analogRead(10);

  for (k = 0; k < sensor_number; k++)
    raw.data[k] = raw_data[k];
  ir_line_scanner_pub.publish(&raw);

//-------------------------------------------------------------------

  nh.spinOnce();
  if (horizontal >= 195)  //二级保险
    horizontal = 195;
  if (horizontal <= -15)
    horizontal = -15;
  if (vertical >= 170)
    vertical = 170;
  if (vertical <= 50)
    vertical = 50;

  SR518.moveSpeed(0, translateAnalog(horizontal), 300);
  SR518.moveSpeed(1, translateAnalog(vertical), 300);

  pos_horizontal = translateAngle(SR518.readPosition(0));
  pos_vertical = translateAngle(SR518.readPosition(1));

  if (horizontal + 2 >= pos_horizontal && horizontal - 2 <= pos_horizontal)
    motor_feedback.ret_hor = 0;
  else
    motor_feedback.ret_hor = 1;

  if (vertical + 10 >= pos_vertical && vertical - 2 <= pos_vertical)  //此处的数字10不通用
    motor_feedback.ret_ver = 0;
  else
    motor_feedback.ret_ver = 1;
  feedback.publish(&motor_feedback);


  /*print_value++;
  if (print_value >= 10)
  {
    for (k = 0; k < sensor_number; k++)
    {
      Serial.print(raw_data[k]);
      Serial.print("\t");
    }
    Serial.println();

    Serial.print("Rudder control value(x,y) : ");
    Serial.print("(");
    Serial.print(translateAnalog(horizontal));
    Serial.print(",");
    Serial.print(translateAnalog(vertical));
    Serial.print(")");
    Serial.print("    ");
    Serial.print("Rudder feedback(x,y) : ");
    Serial.print("(");
    Serial.print(pos_horizontal);
    Serial.print(",");
    Serial.print(pos_vertical);
    Serial.println(")");
    print_value = 0;
  }*/
}

long int translateAnalog(long int angle)  //将舵机的控制范围由0~300度换算成-60~240度
{
  return (angle + 60) * 1023 / 300;
}

long int translateAngle(long int analog)  //将舵机反馈值转为角度
{
  return analog * 300 / 1023 - 60;
}
