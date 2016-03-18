/*
   SpringRC.cpp - SR518+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics. All rights reserved.
 Created by Savage on 27/01/11.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 Modified HardwareSerial( Nicholas Zambetti & David A. Mellis ) Library 
 to listen at 1MBps USART Transmissions by Josue Gutierrez - Savage Electronics
 
 Contact: quarryman89 at Hotmail dot com 
 
 */

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include "Arduino.h"
#include "wiring_private.h"

#include "SpringRC.h"

#define RX_BUFFER_SIZE 128

struct Serial_buffer {
  volatile unsigned char buffer[RX_BUFFER_SIZE];
  volatile unsigned char head;
  volatile unsigned char tail;
};

Serial_buffer _rx_buffer = {{0}, 0, 0};

ISR(USART1_RX_vect)            
{
  //#if defined(__AVR_ATmega8__)
  //	_rx_buffer.buffer[(_rx_buffer.head++)] = UDR;	
  //#else
  _rx_buffer.buffer[(_rx_buffer.head++)] = UDR1;
  //#endif
}

// Constructors ////////////////////////////////////////////////////////////////

SR518Class::SR518Class(volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
volatile uint8_t *udr, uint8_t rxen, uint8_t txen, 
uint8_t rxcie, uint8_t udre, uint8_t u2x)
{
  _ubrrh = ubrrh;
  _ubrrl = ubrrl;
  _ucsra = ucsra;
  _ucsrb = ucsrb;
  _udr = udr;
  _rxen = rxen;
  _txen = txen;
  _rxcie = rxcie;
  _udre = udre;
  _u2x = u2x;
}

// Private Methods //////////////////////////////////////////////////////////////

int SR518Class::read_error(void)
{
  Time_Counter = 0;
  while(usart_available() < 5 & Time_Counter < TIME_OUT){  // Wait for Data
    Time_Counter++;
    delay(1);
    if( usart_peek() != 255 ){
      usart_read();
    }
  }

  while (usart_available() > 0){
    Incoming_Byte = usart_read();
    if ( Incoming_Byte == 255 & usart_peek() == 255 ){
      usart_read();                                    // Start Bytes
      usart_read();                                    // SR518 ID
      usart_read();                                    // Length
      Error_Byte = usart_read();                       // Error
      return (Error_Byte);
    }
  }
  return (-1);											 // No SR Response
}

int SR518Class::usart_available(void)
{
  return (_rx_buffer.head - _rx_buffer.tail);
}

int SR518Class::usart_peek(void)
{
  if ((_rx_buffer.head - _rx_buffer.tail) == 0) {
    return -1;
  } 
  else {
    return _rx_buffer.buffer[_rx_buffer.tail];
  }
}

int SR518Class::usart_read(void)
{
  if ((_rx_buffer.head - _rx_buffer.tail) == 0) {
    return -1;
  } 
  else {
    unsigned char c = _rx_buffer.buffer[_rx_buffer.tail];
    _rx_buffer.tail++;
    if (_rx_buffer.tail == _rx_buffer.head)
      _rx_buffer.head = _rx_buffer.tail =  0;
    return c;
  }
}

void SR518Class::usart_write(uint8_t c)
{
  while (!((*_ucsra) & (1 << _udre)));
  *_udr = c;
}



// Public Methods //////////////////////////////////////////////////////////////

void SR518Class::begin(long baud, unsigned char D_Pin, unsigned char R_Pin)
{
  uint16_t baud_setting;

  pinMode(D_Pin,OUTPUT);
  pinMode(R_Pin,OUTPUT);
  Direction_Pin = D_Pin;
  Enable_Pin = R_Pin;

  *_ucsra = 0;
  baud_setting = (F_CPU / 8 / baud - 1) / 2;
  *_ubrrh = baud_setting >> 8;
  *_ubrrl = baud_setting;

  sbi(*_ucsrb, _rxen);
  sbi(*_ucsrb, _txen);
  sbi(*_ucsrb, _rxcie);
  //sbi(*_ucsra, _u2x);

}	

void SR518Class::end()
{
  cbi(*_ucsrb, _rxen);
  cbi(*_ucsrb, _txen);
  cbi(*_ucsrb, _rxcie);  
  //cbi(*_ucsra, _u2x);
}	

int SR518Class::ping(unsigned char ID)
{
  TChecksum = (ID + SR_READ_DATA + SR_PING);  
  while ( TChecksum >= 255){
    TChecksum -= 255;
  }
  Checksum = 255 - TChecksum;          

  digitalWrite(Direction_Pin,HIGH);
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);                     
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_READ_DATA);
  usart_write(SR_PING);    
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);
  digitalWrite(Enable_Pin,LOW);

  return (read_error());              
}

int SR518Class::setID(unsigned char ID, unsigned char New_ID)
{    
  TChecksum = (ID + SR_ID_LENGTH + SR_WRITE_DATA + SR_ID + New_ID);  
  while ( TChecksum >= 255){
    TChecksum -= 255;
  }
  Checksum = 255 - TChecksum;         

  digitalWrite(Direction_Pin,HIGH);     // Set Tx Mode
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);                // Send Instructions over Serial
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_ID_LENGTH);
  usart_write(SR_WRITE_DATA);
  usart_write(SR_ID);
  usart_write(New_ID);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);      // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  return (read_error());                // Return the read error
}

int SR518Class::setBD(unsigned char ID, unsigned char Baud_Rate)
{    
  TChecksum = (ID + SR_BD_LENGTH + SR_WRITE_DATA + SR_BAUD_RATE + Baud_Rate); 
  while ( TChecksum >= 255){
    TChecksum -= 255;
  }
  Checksum = 255 - TChecksum;         

  digitalWrite(Direction_Pin,HIGH);      // Set Tx Mode
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);                 // Send Instructions over Serial
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_BD_LENGTH);
  usart_write(SR_WRITE_DATA);
  usart_write(SR_BAUD_RATE);
  usart_write(Baud_Rate);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);      // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  return (read_error());                // Return the read error
}

int SR518Class::regWrite(unsigned char ID, unsigned char Reg_addr, unsigned char Reg_val)
{   
  unsigned char REG_WRITE_LENGTH = 0x04;
  TChecksum = (ID + REG_WRITE_LENGTH + SR_WRITE_DATA + Reg_addr + Reg_val); 
  while ( TChecksum >= 255){
    TChecksum -= 255;
  }
  Checksum = 255 - TChecksum;         

  digitalWrite(Direction_Pin,HIGH);      // Set Tx Mode
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);                 // Send Instructions over Serial
  usart_write(SR_START);
  usart_write(ID);
  usart_write(REG_WRITE_LENGTH);
  usart_write(SR_WRITE_DATA);
  usart_write(Reg_addr);
  usart_write(Reg_val);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);      // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  return (read_error());                // Return the read error
}

int SR518Class::regRead(unsigned char ID, unsigned char Reg_addr)
{	
  unsigned char REG_READ_LENGTH = 0x04;
  TChecksum = (ID + REG_READ_LENGTH  + SR_READ_DATA + Reg_addr + SR_BYTE_READ);
  while ( TChecksum >= 255){
    TChecksum -= 255;     
  }
  Checksum = 255 - TChecksum;

  digitalWrite(Direction_Pin,HIGH); 
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);
  usart_write(SR_START);
  usart_write(ID);
  usart_write(REG_READ_LENGTH);
  usart_write(SR_READ_DATA);
  usart_write(Reg_addr);
  usart_write(SR_BYTE_READ);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);     // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  unsigned char regRead_Byte = 0;
  Time_Counter = 0;
  while(usart_available() < 6 & Time_Counter < TIME_OUT){
    Time_Counter++;
    delay(1);
    if( usart_peek() != 255 ){
      usart_read();
    }   
  }

  while (usart_available() > 0){
    Incoming_Byte = usart_read();
    if ( Incoming_Byte == 255 & usart_peek() == 255 ){
      usart_read();                            // Start Bytes
      usart_read();                            // SR518 ID
      usart_read();                            // Length
      if( (Error_Byte = usart_read()) != 0 )   // Error
        return (Error_Byte*(-1));
      regRead_Byte = usart_read();         // Temperature
    }
  }
  return (regRead_Byte);               // Returns the read temperature
}


int SR518Class::move(unsigned char ID, long Position)
{
  char Position_H,Position_L;
  Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
  Position_L = Position;

  TChecksum = (ID + SR_GOAL_LENGTH + SR_WRITE_DATA + SR_GOAL_POSITION_L + Position_L + Position_H);
  while ( TChecksum >= 255){            
    TChecksum -= 255;     
  }
  Checksum = 255 - TChecksum;

  digitalWrite(Direction_Pin,HIGH);      // Set Tx Mode
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);                 // Send Instructions over Serial
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_GOAL_LENGTH);
  usart_write(SR_WRITE_DATA);
  usart_write(SR_GOAL_POSITION_L);
  usart_write(Position_L);
  usart_write(Position_H);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);       // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  return (read_error());                 // Return the read error
}

int SR518Class::moveSpeed(unsigned char ID, long Position, long Speed)
{
  char Position_H,Position_L,Speed_H,Speed_L;
  Position_H = Position >> 8;    
  Position_L = Position;                // 16 bits - 2 x 8 bits variables
  Speed_H = Speed >> 8;
  Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables

  TChecksum = (ID + SR_GOAL_SP_LENGTH + SR_WRITE_DATA + SR_GOAL_POSITION_L);
  TChecksum += (Position_L + Position_H + Speed_L + Speed_H);
  while ( TChecksum >= 255){            
    TChecksum -= 255;     
  }
  Checksum = 255 - TChecksum;

  digitalWrite(Direction_Pin,HIGH);     // Set Tx Mode
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);                // Send Instructions over Serial
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_GOAL_SP_LENGTH);
  usart_write(SR_WRITE_DATA);
  usart_write(SR_GOAL_POSITION_L);
  usart_write(Position_L);
  usart_write(Position_H);
  usart_write(Speed_L);
  usart_write(Speed_H);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);     // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  return (read_error());               // Return the read error
}

int SR518Class::setLimit(unsigned char ID, long Position1, long Position2)
{
  char Position1_H,Position1_L,Position2_H,Position2_L;
  Position1_H = Position1 >> 8;    
  Position1_L = Position1;                // 16 bits - 2 x 8 bits variables
  Position2_H = Position2 >> 8;    
  Position2_L = Position2;                // 16 bits - 2 x 8 bits variables

  TChecksum = (ID + SR_LIMIT_LENGTH + SR_WRITE_DATA + SR_CW_ANGLE_LIMIT_L);
  TChecksum += (Position1_L + Position1_H + Position2_L + Position2_H);
  while ( TChecksum >= 255){            
    TChecksum -= 255;     
  }
  Checksum = 255 - TChecksum;

  digitalWrite(Direction_Pin,HIGH);     // Set Tx Mode
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);                // Send Instructions over Serial
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_LIMIT_LENGTH);
  usart_write(SR_WRITE_DATA);
  usart_write(SR_CW_ANGLE_LIMIT_L);
  usart_write(Position1_L);
  usart_write(Position1_H);
  usart_write(Position2_L);
  usart_write(Position2_H);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);     // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  return (read_error());               // Return the read error
}

int SR518Class::setCompliance(unsigned char ID, unsigned char Margin_CW, unsigned char Margin_CCW, unsigned char Slope_CW, unsigned char Slope_CCW)
{
  //unsigned char Margin_CW, Margin_CCW, Slope_CW, Slope_CCW;

  TChecksum = (ID + SR_COMPLIENCE_LENGTH + SR_WRITE_DATA + SR_CW_COMPLIANCE_MARGIN);
  TChecksum += (Margin_CW + Margin_CCW + Slope_CW + Slope_CCW);
  while ( TChecksum >= 255){            
    TChecksum -= 255;     
  }
  Checksum = 255 - TChecksum;

  digitalWrite(Direction_Pin,HIGH);     // Set Tx Mode
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);                // Send Instructions over Serial
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_COMPLIENCE_LENGTH);
  usart_write(SR_WRITE_DATA);
  usart_write(SR_CW_COMPLIANCE_MARGIN);
  usart_write(Margin_CW);
  usart_write(Margin_CCW);
  usart_write(Slope_CW);
  usart_write(Slope_CCW);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);     // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  return (read_error());               // Return the read error
}

int SR518Class::torqueStatus( unsigned char ID, bool Status)
{
  TChecksum = (ID + SR_TORQUE_LENGTH + SR_WRITE_DATA + SR_TORQUE_ENABLE + Status);
  while ( TChecksum >= 255){
    TChecksum -= 255;     
  }
  Checksum = 255 - TChecksum;

  digitalWrite(Direction_Pin,HIGH);   // Set Tx Mode
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);              // Send Instructions over Serial
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_TORQUE_LENGTH);
  usart_write(SR_WRITE_DATA);
  usart_write(SR_TORQUE_ENABLE);
  usart_write(Status);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);    // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  return (read_error());              // Return the read error
}

int SR518Class::ledStatus(unsigned char ID, bool Status)
{    
  TChecksum = (ID + SR_LED_LENGTH + SR_WRITE_DATA + SR_LED + Status);
  while ( TChecksum >= 255){
    TChecksum -= 255;     
  }
  Checksum = 255 - TChecksum;

  digitalWrite(Direction_Pin,HIGH);   // Set Tx Mode
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);              // Send Instructions over Serial
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_LED_LENGTH);
  usart_write(SR_WRITE_DATA);
  usart_write(SR_LED);
  usart_write(Status);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);    // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  return (read_error());              // Return the read error
}

int SR518Class::readTemperature(unsigned char ID)
{	
  TChecksum = (ID + SR_TEM_LENGTH  + SR_READ_DATA + SR_PRESENT_TEMPERATURE + SR_BYTE_READ);
  while ( TChecksum >= 255){
    TChecksum -= 255;     
  }
  Checksum = 255 - TChecksum;

  digitalWrite(Direction_Pin,HIGH); 
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_TEM_LENGTH);
  usart_write(SR_READ_DATA);
  usart_write(SR_PRESENT_TEMPERATURE);
  usart_write(SR_BYTE_READ);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);     // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  Temperature_Byte = 0;
  Time_Counter = 0;
  while(usart_available() < 6 & Time_Counter < TIME_OUT){
    Time_Counter++;
    delay(1);
    if( usart_peek() != 255 ){
      usart_read();
    }   
  }

  while (usart_available() > 0){
    Incoming_Byte = usart_read();
    if ( Incoming_Byte == 255 & usart_peek() == 255 ){
      usart_read();                            // Start Bytes
      usart_read();                            // SR518 ID
      usart_read();                            // Length
      if( (Error_Byte = usart_read()) != 0 )   // Error
        return (Error_Byte*(-1));
      Temperature_Byte = usart_read();         // Temperature
    }
  }
  return (Temperature_Byte);               // Returns the read temperature
}

int SR518Class::readPosition(unsigned char ID)
{	
  TChecksum = (ID + SR_POS_LENGTH  + SR_READ_DATA + SR_PRESENT_POSITION_L + SR_BYTE_READ_POS);
  while ( TChecksum >= 255){
    TChecksum -= 255;     
  }
  Checksum = 255 - TChecksum;

  digitalWrite(Direction_Pin,HIGH); 
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_POS_LENGTH);
  usart_write(SR_READ_DATA);
  usart_write(SR_PRESENT_POSITION_L);
  usart_write(SR_BYTE_READ_POS);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);     // Set Rx Mode
  digitalWrite(Enable_Pin,LOW);

  Position_Long_Byte = 0;
  Time_Counter = 0;
  while(usart_available() < 7 & Time_Counter < TIME_OUT){
    Time_Counter++;
    delay(1);
    if( usart_peek() != 255 ){
      usart_read();
    }   
  }

  while (usart_available() > 0){
    Incoming_Byte = usart_read();
    if ( Incoming_Byte == 255 & usart_peek() == 255 ){
      usart_read();                            // Start Bytes
      usart_read();                            // SR518 ID
      usart_read();                            // Length
      if( (Error_Byte = usart_read()) != 0 )   // Error
        return (Error_Byte*(-1));

      Position_Low_Byte = usart_read();            // Position Bytes
      Position_High_Byte = usart_read();
      Position_Long_Byte = Position_High_Byte << 8; 
      Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;
    }
  }
  return (Position_Long_Byte);     // Returns the read position
}

int SR518Class::readVoltage(unsigned char ID)
{    
  TChecksum = (ID + SR_VOLT_LENGTH  + SR_READ_DATA + SR_PRESENT_VOLTAGE + SR_BYTE_READ);
  while ( TChecksum >= 255){
    TChecksum -= 255;     
  }
  Checksum = 255 - TChecksum;

  digitalWrite(Direction_Pin,HIGH); 
  digitalWrite(Enable_Pin,HIGH);
  usart_write(SR_START);
  usart_write(SR_START);
  usart_write(ID);
  usart_write(SR_VOLT_LENGTH);
  usart_write(SR_READ_DATA);
  usart_write(SR_PRESENT_VOLTAGE);
  usart_write(SR_BYTE_READ);
  usart_write(Checksum);
  delayMicroseconds(TX_DELAY_TIME);
  digitalWrite(Direction_Pin,LOW);     // Set Rx Mode 
  digitalWrite(Enable_Pin,LOW);

  Voltage_Byte = 0;
  Time_Counter = 0;
  while(usart_available() < 6 & Time_Counter < TIME_OUT){
    Time_Counter++;
    delay(1);
    if( usart_peek() != 255 ){
      usart_read();
    }   
  }

  while (usart_available() > 0){
    Incoming_Byte = usart_read();
    if ( Incoming_Byte == 255 & usart_peek() == 255 ){
      usart_read();                            // Start Bytes
      usart_read();                            // SR518 ID
      usart_read();                            // Length
      if( (Error_Byte = usart_read()) != 0 )   // Error
        return (Error_Byte*(-1));
      Voltage_Byte = usart_read();             // Voltage
    }
  }
  return (Voltage_Byte);               // Returns the read Voltage
}


// Preinstantiate Objects //////////////////////////////////////////////////////

//#if defined(__AVR_ATmega8__)
//SR518Class SR518(&UBRRH, &UBRRL, &UCSRA, &UCSRB, &UDR, RXEN, TXEN, RXCIE, UDRE);
//#else
SR518Class SR518(&UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UDR1, RXEN1, TXEN1, RXCIE1, UDRE1, U2X1);
//#endif

