#include <ros.h>
#include <std_msgs/String.h>
#include <dependant_api/robotcmd_motor.h>
#include <dependant_api/arduino_motor.h>
#include "SpringRC.h"

std_msgs::String ir_line_scan;
ros::Publisher ir_line_scanner_pub("ir_line_scanner", &ir_line_scan);
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
  unsigned char i, j, k, a[12] = {48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, '\0'};
  int max_value, min_value, max_num, min_num, max_count, min_count, num_value;
  double median_value, median_average, max_average, min_average, num_average, b[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  for (i = 0; i < sensor_number; i++)
  {
    a[i] = 48;
    b[i] = 0;
  }

  nh.spinOnce();

  for (j=0;j<20;j++)
  {
    max_value = 0;
    min_value = 1000;
    max_num = 0;
    min_num = 0;
    max_count = 0;
    min_count = 0;
    num_value = 0;
    median_value = 0.0;
    median_average = 0.0;
    max_average = 0.0;
    min_average = 0.0;
    num_average = 0.0;

    //先做赋值和找到最大、最小值
    b[0] = analogRead(0);
    b[1] = analogRead(1);
    b[2] = analogRead(2);
    b[3] = analogRead(3);
    b[4] = analogRead(4);
    b[5] = analogRead(5);
    b[6] = analogRead(6);
    b[7] = analogRead(7);
    b[8] = analogRead(8);
    b[9] = analogRead(9);
    b[10] = analogRead(10);
    for (k = 0; k < sensor_number; k++)
    {
      if (b[k] > max_value)
        max_value = b[k];
      if (b[k] < min_value)
        min_value = b[k];
    }
    //求最大、最小值的中值
    median_value = (max_value + min_value) / 2;
    //将大于、小于中值的值分出来，分别求出两组值的平均值，再对这两个平均值求中值
    for (k = 0; k < sensor_number; k++)
    {
      if (median_value < b[k])
      {
        max_num += b[k];
        max_count++;
      }
      if (median_value > b[k])
      {
        min_num += b[k];
        min_count++;
      }
      num_value += b[k];
    }
    max_average = max_num / max_count;
    min_average = min_num / min_count;
    median_average = (max_average + min_average) / 2;
    num_average = num_value / sensor_number;

    //根据两组值的平均值差距，判断是否有线或全是线
    if ((max_average - min_average < 300) && (max_average - min_average > 50))
    {
      if (max_average < 700)
        for (k = 0; k < sensor_number; k++)
          a[k] += 1;
    }
    else if (max_average - min_average > 50)
    {
      for (k = 0; k < sensor_number; k++)
        if (median_average >= b[k])
          a[k] += 1;
    }
  }

  ir_line_scan.data = (const char *) a;
  ir_line_scanner_pub.publish(&ir_line_scan);

//-------------------------------------------------------------------

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
      Serial.print(b[k]);
      Serial.print("\t");
    }
    Serial.println();
    Serial.print("max_ave : ");
    Serial.print(max_average);
    Serial.print("    ");
    Serial.print("min_ave : ");
    Serial.print(min_average);
    Serial.print("    ");
    Serial.print("max_ave - min_ave : ");
    Serial.print(max_average - min_average);
    Serial.print("    ");
    Serial.print("median_ave : ");
    Serial.print(median_average);
    Serial.print("    ");
    Serial.print("standard_deviation : ");
    Serial.println(standard_deviation(b, sensor_number, num_average));

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

double standard_deviation(double x[ ], int n, double mean)
{
  double divisor,sum;
  int k;
  for (sum = k = 0; k < n; k++)
    sum += pow(x[k] - mean, 2);
  if (n < 20)
    divisor = n - 1;
  else
    divisor=n;
  return sqrt(sum / divisor);
}
