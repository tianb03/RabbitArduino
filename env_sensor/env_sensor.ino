/*---------------------------------------------------I/O explain 
MQ_2             analog_pin -A3   digital_pin -1 
HC_SR501      digital_pin-3
BH1750         ADD -cathode     SDA -A4      SCL-A5
DHT11           digital_pin-2
KY-038           analog_pin-A0
---------------------------------------------------I/Oexplain
tarsbot  at 2015.12.4
*/

#include <Wire.h>
#include "sensor.h"
#include <ros.h>
#include <dependant_api/all_sensor.h>


#define Sensor_yw  A3    // --------------------------------------smoke
#define Sensor_yw_DO  2
unsigned int SensorValue = 0;  //-------------------------------smoke

int Sensor_man= 3;  //------------------------------------------man

int sensorPin = A0;//----------------------------------------------noise


//#include <std_msgs/Int16.h>

dependant_api::all_sensor  data_msg;
//std_msgs::Int16 data_msg;
ros::Publisher chatter("/robot/env_sensor", &data_msg);
ros::NodeHandle  nh;

BH1750 illuminationMeter(2);


void setup(){
  //Serial.begin(9600);
  nh.initNode();
  nh.advertise(chatter);
  illuminationMeter.begin();
 
  pinMode(Sensor_yw_DO,INPUT);
  pinMode(Sensor_yw,INPUT);
  
   pinMode(Sensor_man, INPUT);//-------------------------------------man
  
}


void loop() 
{ 
  illuminationMeter.DHT11_Read();				//读取温湿度值
  data_msg.humidity= illuminationMeter.HUMI_Buffer_Int;
  data_msg.temperature= illuminationMeter.TEM_Buffer_Int;
  
  SensorValue = analogRead(Sensor_yw);
  data_msg.smoke=SensorValue;
  
  uint16_t  lux = illuminationMeter.readLightLevel();
  data_msg.illumination=lux;
  
  data_msg.human=digitalRead(Sensor_man);
  
  data_msg.noise=analogRead (sensorPin);
  
  chatter.publish( &data_msg );
  nh.spinOnce();
  delay(300);
  
  
  
  
  
}
