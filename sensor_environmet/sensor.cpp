#include "sensor.h"
#include <util/delay.h>


unsigned char HUMI_Buffer_Int = 0;
unsigned char TEM_Buffer_Int = 0;


BH1750::BH1750(int pin) 
{
  DHT11_DQ = pin;
}

void BH1750::begin(uint8_t mode) {

  Wire.begin();
  //write8(mode);
  configure(mode);
}


void BH1750::configure(uint8_t mode)
{

  switch (mode) {
  case BH1750_CONTINUOUS_HIGH_RES_MODE:
  case BH1750_CONTINUOUS_HIGH_RES_MODE_2:
  case BH1750_CONTINUOUS_LOW_RES_MODE:
  case BH1750_ONE_TIME_HIGH_RES_MODE:
  case BH1750_ONE_TIME_HIGH_RES_MODE_2:
  case BH1750_ONE_TIME_LOW_RES_MODE:
    // apply a valid mode change
    write8(mode);
    _delay_ms(10);
    break;
  default:
    // Invalid measurement mode
#if BH1750_DEBUG == 1
    Serial.println("Invalid measurement mode");
#endif
    break;
  }
}


uint16_t BH1750::readLightLevel(void) 
{

  uint16_t level;

  Wire.beginTransmission(BH1750_I2CADDR);
  Wire.requestFrom(BH1750_I2CADDR, 2);
#if (ARDUINO >= 100)
  level = Wire.read();
  level <<= 8;
  level |= Wire.read();
#else
  level = Wire.receive();
  level <<= 8;
  level |= Wire.receive();
#endif
  Wire.endTransmission();

#if BH1750_DEBUG == 1
  Serial.print("Raw light level: ");
  Serial.println(level);
#endif

  level = level/1.2; // convert to lux

#if BH1750_DEBUG == 1
  Serial.print("Light level: ");
  Serial.println(level);
#endif
  return level;
}



/*********************************************************************/


void BH1750::write8(uint8_t d)
{
  Wire.beginTransmission(BH1750_I2CADDR);
#if (ARDUINO >= 100)
  Wire.write(d);
#else
  Wire.send(d);
#endif
  Wire.endTransmission();
}



void  BH1750::DHT11_Init()             //------------------------------------DHT11
{
  pinMode(DHT11_DQ,OUTPUT);

  digitalWrite(DHT11_DQ,LOW);  //拉低总线，发开始信号；
  delay(30);  //延时要大于 18ms，以便 DHT11 能检测到开始信号；
  digitalWrite(DHT11_DQ,HIGH);
  delayMicroseconds(40);  //等待 DHT11 响应；
  pinMode(DHT11_DQ,INPUT_PULLUP);
  while(digitalRead(DHT11_DQ) == HIGH);
  delayMicroseconds(80);   //DHT11 发出响应，拉低总线 80us；
  if(digitalRead(DHT11_DQ) == LOW);
  delayMicroseconds(80);   //DHT11 拉高总线 80us 后开始发送数据；
}


int  BH1750::DHT11_Read_Byte()  //----------------------------------- DHT11
{
  unsigned char i,  dat = 0;
  unsigned int j;

  pinMode(DHT11_DQ,INPUT_PULLUP);
  delayMicroseconds(2);
  for( i=0; i<8; i++)
  {
    while(digitalRead(DHT11_DQ) == LOW);   //等待 50us；
    delayMicroseconds(40);   //判断高电平的持续时间，以判定数据是‘0’还是‘1’；
    if(digitalRead(DHT11_DQ) == HIGH)
      dat |= (1<<(7-i));   //高位在前，低位在后；
    while(digitalRead(DHT11_DQ) == HIGH);   //数据‘1’，等待下一位的接收；

  }
  return dat;
}
void BH1750::DHT11_Read()          //-----------------------------------DHT11
{
   DHT11_Init();

  HUMI_Buffer_Int = DHT11_Read_Byte();   		//读取湿度的整数值
  DHT11_Read_Byte();							//读取湿度的小数值
  TEM_Buffer_Int = DHT11_Read_Byte();			//读取温度的整数值
  DHT11_Read_Byte();							//读取温度的小数值
  DHT11_Read_Byte();							//读取校验和
  delayMicroseconds(50);						//DHT11拉低总线50us

  pinMode(DHT11_DQ,OUTPUT);
  digitalWrite(DHT11_DQ,HIGH);				//释放总线	

}










