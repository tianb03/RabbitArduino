#include <ros.h>
#include <beginner_tutorials/Int16Array.h>

ros::NodeHandle nh;
beginner_tutorials::Int16Array analog;
ros::Publisher ir_line_scanner_pub("ir_analog", &analog);
int sensor_number = 8;
int print_value;

void setup()
{
  nh.initNode();
  nh.advertise(ir_line_scanner_pub);
}

void loop()
{
  int i, j, k, a[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  a[0] = analogRead(0);
  a[1] = analogRead(1);
  a[2] = analogRead(2);
  a[3] = analogRead(3);
  a[4] = analogRead(4);
  a[5] = analogRead(5);
  a[6] = analogRead(6);
  a[7] = analogRead(7);
  a[8] = analogRead(8);
  a[9] = analogRead(9);
  a[10] = analogRead(10);

  for (k = 0; k < sensor_number; k++)
    analog.data[k] = a[k];
  ir_line_scanner_pub.publish(&analog);
  nh.spinOnce();

  delay(18.5);

  /**/print_value++;
  if (print_value >= 20)
  {
    for (k = 0; k < sensor_number; k++)
    {
      Serial.print(a[k]);
      Serial.print("\t");
    }
    Serial.println();
    print_value = 0;
  }
}
