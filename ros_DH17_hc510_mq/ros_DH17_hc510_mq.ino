/*---------------------------------------------------I/O explain 
MQ_2   analog -A3   digital -1 
HC_SR501  3
BH1750   ADD -cathode     SDA -A4      SCL-A5
DHT11  2
KY-038  A2
---------------------------------------------------I/Oexplain
*/

#include <Wire.h>
#include "sensor.h"
#include <ros.h>
#include <dependant_api/all_sensor.h>


#define Sensor_yw  A3    // --------------------------------------yan_wu
#define Sensor_yw_DO  2
unsigned int SensorValue = 0;  //-------------------------------yan_wu

int Sensor_man= 3;  //------------------------------------------man

int sensorPin = A2;//----------------------------------------------microphone


//#include <std_msgs/Int16.h>

dependant_api::all_sensor  data_msg;
//std_msgs::Int16 data_msg;
ros::Publisher chatter("/robot/env_sensor", &data_msg);
ros::NodeHandle  nh;

BH1750 lightMeter(2);


void setup(){
  //Serial.begin(9600);
  nh.initNode();
  nh.advertise(chatter);
  lightMeter.begin();
 
  pinMode(Sensor_yw_DO,INPUT);
  pinMode(Sensor_yw,INPUT);
  
   pinMode(Sensor_man, INPUT);//-------------------------------------man
  
}


void loop() 
{ 
  lightMeter.DHT11_Read();				//读取温湿度值
  data_msg.huminity= lightMeter.HUMI_Buffer_Int;
  data_msg.temperature= lightMeter.TEM_Buffer_Int;
  
  SensorValue = analogRead(Sensor_yw);
  data_msg.yan_wu=SensorValue;
  
  uint16_t  lux = lightMeter.readLightLevel();
  data_msg.light=lux;
  
  data_msg.test_man=digitalRead(Sensor_man);
  
  data_msg.microphone=analogRead (sensorPin);
  
  chatter.publish( &data_msg );
  nh.spinOnce();
  delay(300);
  
  
  
  
  
}
