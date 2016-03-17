#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String ir_line_scan;
ros::Publisher ir_line_scanner_pub("ir_line_scanner", &ir_line_scan);
int print_value;

void setup()
{
  nh.initNode();
  nh.advertise(ir_line_scanner_pub);
}

void loop()
{
  unsigned char i, j, k, a[12] = {48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, '\0'};
  int max_value, min_value, max_num, min_num, max_count, min_count, num_value;
  double median_value, median_average, max_average, min_average, num_average, b[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  for (i = 0; i < 11; i++)
  {
    a[i] = 48;
    b[i] = 0;
  }

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
    for (k = 0; k < 8; k++)
    {
      if (b[k] > max_value)
        max_value = b[k];
      if (b[k] < min_value)
        min_value = b[k];
    }
    //求最大、最小值的中值
    median_value = (max_value + min_value) / 2;
    //将大于、小于中值的值分出来，分别求出两组值的平均值，再对这两个平均值求中值
    for (k = 0; k < 8; k++)
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
    num_average = num_value / 8;

    //根据两组值的平均值差距，判断是否有线或全是线
    if ((max_average - min_average < 300) && (max_average - min_average > 50))
    {
      if (max_average < 700)
        for (k = 0; k < 8; k++)
          a[k] += 1;
    }
    else if (max_average - min_average > 50)
    {
      for (k = 0; k < 8; k++)
        if (median_average >= b[k])
          a[k] += 1;
    }
  }

  ir_line_scan.data = (const char *) a;
  ir_line_scanner_pub.publish(&ir_line_scan);
  nh.spinOnce();

  /**/print_value++;
  if (print_value >= 50)
  {
    for (k = 0; k < 8; k++)
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
    Serial.println(standard_deviation(b, 8, num_average));
    print_value = 0;
  }
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
