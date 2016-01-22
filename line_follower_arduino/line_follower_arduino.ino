#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String ir_line_scan;
ros::Publisher ir_line_scanner_pub("ir_line_scanner", &ir_line_scan);

void setup()
{
  Serial.begin(9600); 
  nh.initNode();
  nh.advertise(ir_line_scanner_pub);
}

void loop()
{
//  unsigned char i,j,a[12]="hellooooYOU";
//  unsigned char i,j,a[12]={0,0,0,0,0,0,0,0,0,0,0,'\0'};
  unsigned char i,j,a[12]={48,48,48,48,48,48,48,48,48,48,48,'\0'};
  for(j=0;j<20;j++)
    for(i=2;i<13;i++)
     a[i-2]+=digitalRead(i);
  
  ir_line_scan.data = (const char *) a;
  ir_line_scanner_pub.publish(&ir_line_scan);
  nh.spinOnce();
}
