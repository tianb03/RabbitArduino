#include <ros.h>
#include <Servo.h>
#include <dependant_api/robotcmd_motor.h>
#include <dependant_api/Int16Array.h>

dependant_api::Int16Array raw;
ros::Publisher ir_line_scanner_pub("ir_raw_data", &raw);
int sensor_number = 8;
int print_value;

ros::NodeHandle nh;
Servo servo_vertical;
Servo servo_horizontal;
int pos_vertical = 50;
int pos_horizontal = 90;
int vertical = 50;
int horizontal = 90;

void servoCallback(const dependant_api::robotcmd_motor& cloud_terrace)
{
  //id = cloud_terrace.id;
  vertical = cloud_terrace.ver_angle;
  horizontal = cloud_terrace.hor_angle;
}

ros::Subscriber<dependant_api::robotcmd_motor> servo("steering_duoduo_2", servoCallback);

void setup()
{
  nh.initNode();
  nh.subscribe(servo);
  servo_horizontal.attach(2);
  servo_vertical.attach(3);
  nh.advertise(ir_line_scanner_pub);
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

  if (horizontal > pos_horizontal && pos_horizontal < 170 && horizontal - pos_horizontal != 1)
    pos_horizontal += 2;
  else if (horizontal < pos_horizontal && pos_horizontal > 10 && horizontal - pos_horizontal != -1)
    pos_horizontal -= 2;

  if (vertical > pos_vertical && pos_vertical < 75 && vertical - pos_vertical != 1)
    pos_vertical += 2;
  else if (vertical < pos_vertical && pos_vertical > 30 && vertical - pos_vertical != -1)
    pos_vertical -= 2;

  servo_horizontal.write(pos_horizontal);
  servo_vertical.write(pos_vertical);

  /*print_value++;
  if (print_value >= 10)
  {
    for (k = 0; k < sensor_number; k++)
    {
      Serial.print(raw_data[k]);
      Serial.print("\t");
    }
    Serial.println();
    print_value = 0;
  }*/
}
