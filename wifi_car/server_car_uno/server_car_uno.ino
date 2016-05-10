#include "ESP8266.h"

#define SSID        "testtwo"
#define PASSWORD    "123456123456"
#define HOST_PORT   (8080)

#define HALL_A_PIN 2
#define HALL_B_PIN 3
#define IN1        7
#define IN2        8
#define ENA        9
#define ENB        10
#define IN3        11
#define IN4        12

ESP8266 wifi_server(Serial,115200);

void setup()
{
  Serial.begin(115200);  //不能没有这句
  //Serial.print("setup begin\r\n");

  //Serial.print("FW Version: ");
  //Serial.println(wifi_client.getVersion().c_str());

  while (!wifi_server.setOprToStationSoftAP())
  {
    //Serial.print("Server WiFi settings...\r\n");
  }
  //Serial.print("Server WiFi set successfully\r\n");

  while (!wifi_server.setSoftAPParam(SSID, PASSWORD, 1, 3))
  {
    //Serial.print("Server opens WiFi...\r\n");
  }
  //Serial.print("Server opens WiFi success\r\n");
  //Serial.print("IP: ");       
  //Serial.println(wifi_server.getLocalIP().c_str());
  
  while (!wifi_server.enableMUX())
  {
    //Serial.print("The server is set up in multiple connections...\r\n");
  }
  //Serial.print("Server set multiple connection success\r\n");
  
  while (!wifi_server.startTCPServer(HOST_PORT))
  {
    //Serial.print("Open TCP service...\r\n");
  }
  //Serial.print("Open TCP service success\r\n");
  
  //Serial.print("setup end\r\n");
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(HALL_A_PIN, INPUT);
  pinMode(HALL_B_PIN, INPUT);
}

void loop()
{
  uint8_t cache[2] = {0,0};
  static uint8_t mux_id = 0;
  int a_speed = 0, b_speed = 0;

  uint32_t len = wifi_server.recv(mux_id, cache, sizeof(cache), 10000);
  if (len > 0)
  {
    a_speed = 2 * ((int)cache[0] - 128);
    //Serial.print(a_speed);
    //Serial.print(",");
    b_speed = 2 * ((int)cache[1] - 128);
    //Serial.println(b_speed);
  }
  
  if (a_speed > 0 && b_speed > 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, a_speed);
    analogWrite(ENB, b_speed);
  }
  else if (a_speed > 0 && b_speed < 0)
  {
    b_speed = abs(b_speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, a_speed);
    analogWrite(ENB, b_speed);
  }
  else if (a_speed < 0 && b_speed < 0)
  {
    a_speed = abs(a_speed);
    b_speed = abs(b_speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, a_speed);
    analogWrite(ENB, b_speed);
  }
  else if (a_speed < 0 && b_speed > 0)
  {
    a_speed = abs(a_speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, a_speed);
    analogWrite(ENB, b_speed);
  }
  else if (a_speed == 0 && b_speed == 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  a_speed = 0;
  b_speed = 0;
  
  /*用于向客户端发送数据。仅示例，变量不可用
  char *hello = "Hello, this is server!";
  if (wifi_server.send(mux_id, (const uint8_t*)hello, strlen(hello)))
  {
    Serial.println("send ok");
  } else {
    Serial.println("send err");
  }
  */
}
