#include "ESP8266.h"

#define SSID        "testtwo"
#define PASSWORD    "123456123456"
#define HOST_NAME   "192.168.4.1"
#define HOST_PORT   (8080)

int joy_x = 0;  //pin analogic de les X
int joy_y = 1;  //pin analógic de les Y
long int val_x = 0;
long int val_y = 0;
int count_a = 0;
int count_b = 0;

ESP8266 wifi_client(Serial1,115200);
uint8_t mux_id = 0;

void setup()
{
  Serial.begin(9600);
  while (!wifi_client.setOprToStation())  //不知为何，使用模式3不能正确与服务建立TCP链接
  {
    Serial.print("Client WiFi settings...\r\n");
  }
  Serial.print("Client WiFi set successfully\r\n");

  while (!wifi_client.joinAP(SSID, PASSWORD))
  {
    Serial.print("Client joins WiFi...\r\n");
  }
  Serial.print("Client to join WiFi success\r\n");
  Serial.print("IP: ");       
  Serial.println(wifi_client.getLocalIP().c_str());
  
  while (!wifi_client.enableMUX())
  {
    Serial.print("The client is set up in multiple connections...\r\n");
  }
  Serial.print("Client set multiple connection success\r\n");
  
  while (!wifi_client.createTCP(mux_id, HOST_NAME, HOST_PORT))
  {
    Serial.print("create tcp ");
    Serial.print(mux_id);
    Serial.println(" err");
    delay(300);
  }
  Serial.print("create tcp ");
  Serial.print(mux_id);
  Serial.println(" ok");

  Serial.print("setup end\r\n");
}

void loop()
{
  uint8_t buffer[2] = {0,0};

  val_x = analogRead(joy_x);
  val_y = -analogRead(joy_y);
  
  val_x -= 510;  //val_x: -510 ~ 513
  val_y += 517;  //val_y: -506 ~ 517
  count_a = (val_y + val_x) / 4;
  count_b = (val_y - val_x) / 4;
  if (count_a > 127)  count_a = 127;
  if (count_a < -127) count_a = -127;
  if (count_b > 127)  count_b = 127;
  if (count_b < -127) count_b = -127;
  if ((-2 <= count_a) && (count_a <= 2) && (-2 <= count_b) && (count_b <= 2))
  {
    count_a = 0;
    count_b = 0;
  }
  
  buffer[0] = count_a + 128;
  buffer[1] = count_b + 128;
  
  if (wifi_client.send(mux_id, buffer, 2))
  {
    Serial.print(buffer[0]);
    Serial.print(",");
    Serial.print(buffer[1]);
    Serial.print("  ");
    Serial.println("send ok");
  }
  else
  {
    Serial.println("send err");
  }
  
  /*用于接收客户端数据。仅示例，变量不可用
  len = wifi_client.recv(mux_id, buffer, sizeof(buffer), 10000);
  if (len > 0)
  {
    Serial.print("Received:[");
    for(uint32_t i = 0; i < len; i++)
    {
      Serial.print((char)buffer[i]);
    }
    Serial.print("]\r\n");
  }
  */
  
  /*检查网络联通状态示例
  if (wifi_client.getLocalIP().indexOf("0.0.0.0") == -1)
  {
    Serial.println(wifi_client.getLocalIP().c_str());
  }
  */
}
