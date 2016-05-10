//gongwenbo in tarsbot 10.23

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <dependant_api/robotcmd_motor.h>    //quote ros message  

int temp_vertical= 0;        //vertical temporary parameters
int temp_horizontal= 0;   // horizontal temporary parameter
int flag_save_ver_angle;   // protect vertical angle
int flag_h;                          
ros::NodeHandle  nh;
Servo servo_vertical;       //define vertical servo
Servo servo_horizontal;  //define horizontal servo

void servo_cb(const dependant_api::robotcmd_motor& cmd_msg)
{  
   flag_h=cmd_msg.hor_angle;
   flag_save_ver_angle=cmd_msg.ver_angle;
   if (flag_save_ver_angle>130) flag_save_ver_angle=130;  // the rang of vertical angle is 0~130 

  /*
  if (cmd_msg.hor_angle>temp_horizontal)
    {
       while(temp_horizontal<cmd_msg.hor_angle)
        {
         servo_horizontal.write(temp_horizontal);
         delay (10);
         temp_horizontal++;
        }
     }
   else if (cmd_msg.hor_angle<temp_horizontal)   
   {
    while(temp_horizontal>cmd_msg.hor_angle)
         {
            servo_horizontal.write(temp_horizontal);
            delay (10);
            temp_horizontal--;
         }
    }
   
   else if (cmd_msg.ver_angle>temp_vertical)
    {
       while(temp_vertical<cmd_msg.ver_angle)
        {
         servo_vertical.write(temp_vertical);
         delay (10);
         temp_vertical++;
        }
     }
   else if (cmd_msg.ver_angle>temp_vertical)
   {
    while(temp_vertical>cmd_msg.ver_angle)
         {
            servo_vertical.write(temp_vertical);
            delay (10);
            temp_vertical--;
         }
    }
  
   temp_horizontal=cmd_msg.hor_angle ;
   temp_vertical=cmd_msg.ver_angle ;
  */
  // servo.write(cmd_msg.angle); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<dependant_api::robotcmd_motor> sub("/robotcmd/motor", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo_vertical.attach(10);     //attach vertical servo to pin 9
  servo_horizontal.attach(9);   //attach horiziontal servo to pin 10
}

void loop(){
   nh.spinOnce();
   
   if (flag_save_ver_angle>temp_vertical)  temp_vertical++;       //----------------deceletate vertical servo speed--------------------//
   else if(flag_save_ver_angle<temp_vertical) temp_vertical--;
   else temp_vertical=flag_save_ver_angle;                                 //----------------deceletate vertical servo speed--------------------//
   
   if (flag_h>temp_horizontal)  temp_horizontal++;                    //----------------deceletate horizontal servo speed--------------------//
   else if(flag_h<temp_horizontal) temp_horizontal--;
   else temp_horizontal=flag_h;                                                   //----------------deceletate horizontal servo speed--------------------//
   
   servo_vertical.write(180-temp_vertical);                                  //adjust servo orientation
   delay (10);                                                                                  //the frequence of scanning
   servo_horizontal.write(temp_horizontal);
   delay (10);

}
