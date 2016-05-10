/*
gongwenbo in tarsbot 11.4 at yangling

Pin description:
horizontal angle port is 9
vertical angle port is 10
*/

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <dependant_api/robotcmd_motor.h>    //publish ros message 
#include <dependant_api/arduino_motor.h>      //subscribe ros message

dependant_api::arduino_motor call_a_msg;      //define subscribe ros message
int temp_vertical= 0;         //vertical temporary parameters
int temp_horizontal= 0;   // horizontal temporary parameter
int flag_save_ver_angle=20;   // initial vertical angle
int flag_h=90;                         // initial horizontal angle          
ros::NodeHandle  nh;
ros::Publisher chatter("/arduino/motor", &call_a_msg);

Servo servo_vertical;       //define vertical servo
Servo servo_horizontal;  //define horizontal servo

void servo_cb(const dependant_api::robotcmd_motor& cmd_msg)
{  
  
   call_a_msg.id=cmd_msg.id;
   flag_h=cmd_msg.hor_angle;
   flag_save_ver_angle=cmd_msg.ver_angle;
   if (flag_save_ver_angle>130) flag_save_ver_angle=130;  // the rang of vertical angle is 0~130 
 
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<dependant_api::robotcmd_motor> sub("/robotcmd/motor", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  
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
  
   call_a_msg.ret_hor= flag_h;                                                        //-------------------------feed back servo angle-------------------//
   call_a_msg.ret_ver=180-flag_save_ver_angle;
   
   if (call_a_msg.ret_hor==temp_horizontal) call_a_msg.ret_hor=0;
   else call_a_msg.ret_hor=1;
   if(call_a_msg.ret_ver=(180-temp_vertical)) call_a_msg.ret_ver=0;
   else call_a_msg.ret_ver=1; 
   
   chatter.publish( &call_a_msg );                                               //-------------------------feed back servo angle-------------------//
   
   
   

}
