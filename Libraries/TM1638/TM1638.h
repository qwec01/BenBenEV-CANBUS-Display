
#ifndef _TM1638_
#define _TM1638_

#include "Arduino.h"
#include <avr/pgmspace.h>

class TM1638Display
{
  private:
        
    byte mCLK, mDIO, mSTB;
    
  public:
    TM1638Display(byte DIO, byte CLK, byte STB);
    void rst();
    void Refresh(float InputNum[], byte dot[], byte brightness = 7, bool OnOff = 1, bool mirror = 0);
    void SendCommand(byte value);
    void SendAddr(byte addr);
    void SendData(byte data);
    void SendNum(byte a, bool mirror, bool dot);
	void ShowTime(unsigned int miniute, byte brightness,bool dot);
    static const byte c1[12] PROGMEM;
    static const byte c2[12] PROGMEM;
//    void Refresh(byte a, byte b, byte c, byte d, byte brightness = 7, bool OnOff = 1, bool dota = 0, bool dotb = 0, bool dotc = 0, bool dotd = 0);

};

#endif
