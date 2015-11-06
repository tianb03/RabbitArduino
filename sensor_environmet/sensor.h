#ifndef BH1750_h
#define BH1750_h

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include "Wire.h"

#define BH1750_DEBUG 0

#define BH1750_I2CADDR 0x23

// No active state
#define BH1750_POWER_DOWN 0x00

// Wating for measurment command
#define BH1750_POWER_ON 0x01

// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_RESET 0x07

// Start measurement at 1lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGH_RES_MODE  0x10

// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2  0x11

// Start measurement at 4lx resolution. Measurement time is approx 16ms.
#define BH1750_CONTINUOUS_LOW_RES_MODE  0x13

// Start measurement at 1lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_HIGH_RES_MODE  0x20

// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_HIGH_RES_MODE_2  0x21

// Start measurement at 1lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_LOW_RES_MODE  0x23

class BH1750
{
 public:
    BH1750(int pin);
    void begin(uint8_t mode = BH1750_CONTINUOUS_HIGH_RES_MODE);
    void configure(uint8_t mode);
    uint16_t readLightLevel(void);
  
  
  	//DHT11(int pin);     ------------------------------------------DHT11
	void DHT11_Init();
	int DHT11_Read_Byte();
	void DHT11_Read();

	unsigned char HUMI_Buffer_Int;
	unsigned char TEM_Buffer_Int;  //-----------------------DHT11




 private:
 
  void write8(uint8_t data);
  int DHT11_DQ;  // ----------------------------------------------DHT11

};

#endif
