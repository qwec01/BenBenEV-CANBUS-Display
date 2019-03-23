#include "Arduino.h"
#include "TM1638.h"
#include <avr/pgmspace.h>
#define mirror 1
#if !mirror
//正常
//                                            0 ,  1...........3............5..........7,           9 , empty, -
const byte TM1638Display::c1[12] PROGMEM = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x00, 0x40};
//const byte TM1638Display::c2[12] PROGMEM = {0x08, 0x00, 0x10, 0x10, 0x18, 0x18, 0x18, 0x00, 0x1f, 0x18, 0x00, 0x10}; //带小数点加0x20
#else
//镜像
//                                            0 ,  1...........3............5..........7,           9 , empty, -
const byte TM1638Display::c1[12] PROGMEM = {0x3F, 0x30, 0x6D, 0x79, 0x72, 0x5B, 0x5F, 0x31, 0x7F, 0x7B, 0x00, 0x40};
//const byte TM1638Display::c2[12] PROGMEM = {0x08, 0x08, 0x18, 0x18, 0x18, 0x10, 0x10, 0x08, 0x1f, 0x18, 0x00, 0x10}; //带小数点加0x20
#endif
TM1638Display::TM1638Display(byte DIO, byte CLK, byte STB)
{
  mCLK = CLK;
  mDIO = DIO;
  mSTB = STB;
  pinMode(mCLK, OUTPUT);
  pinMode(mDIO, OUTPUT);
  pinMode(mSTB, OUTPUT);
}
void TM1638Display::SendCommand(byte value)
{
  digitalWrite(mSTB, HIGH);
  digitalWrite(mSTB, LOW);
  shiftOut(mDIO, mCLK, LSBFIRST, value);
  digitalWrite(mSTB, HIGH);
}
void TM1638Display::SendAddr(byte addr)
{
  digitalWrite(mSTB, LOW);
  shiftOut(mDIO, mCLK, LSBFIRST, addr);
}
void TM1638Display::SendData(byte data)
{
  shiftOut(mDIO, mCLK, LSBFIRST, data);
}

void TM1638Display::rst()
{
  //  SendCommand(0);//显示模式设置，4位8段
  SendCommand(0x40);//数据命令设置，写数据到显存，地址自动增加
  SendAddr(0xC0);//第一个显存地址
  for (byte i = 0; i <= 15; i++)
  {
    SendData(0);
  }
}
void TM1638Display::SendNum(byte a)
{
  SendData(pgm_read_word_near(c1 + a));
  SendData(0);
}
void TM1638Display::Refresh(float InputNum[2], byte dot[2], byte brightness, bool OnOff)
{
  //  SendCommand(0);//显示模式设置，4位8段
  //SendCommand(0x40);//数据命令设置，写数据到显存，地址自动增加
  SendAddr(0xC0);//第一个数显存地址
  byte digit;
  for (byte i = 0; i <= 1; i++)
  {
    unsigned int Num = abs(InputNum[i]) * pow(10, dot[i]);
    if (Num >= 2000 && InputNum[i] < 0) break; //第2个数没有，过

#if !mirror //-----------------------------------------------------------正常
    if (Num >= 1000)
    {
      digit = pgm_read_word_near(c1 + Num / 1000);
      if (dot[i] == 3)
        digit += 0x80;
      if (InputNum[i] < 0)
        digit += 0x40;
      SendData(digit);
      SendData(0);
    }
    else if (InputNum[i] < 0)
    {
      SendData(0x40);
      SendData(0);
    }
    else
      SendNum(10);
    if (Num >= 100)
    {
      digit = pgm_read_word_near(c1 + (Num / 100) % 10);
      if (dot[i] == 2)
        digit += 0x80;
      SendData(digit);
      SendData(0);
    }
    else
      SendNum(10);
    if (Num >= 10)
    {
      digit = pgm_read_word_near(c1 + (Num / 10) % 10);
      if (dot[i] == 1)
        digit += 0x80;
      SendData(digit);
      SendData(0);
    }
    else
      SendNum(10);
    if (Num >= 1)
    {
      digit = pgm_read_word_near(c1 + Num % 10);
      if (dot[i] == 0)
        digit += 0x80;
      SendData(digit);
      SendData(0);
    }
    else
      SendNum(10);
#else //--------------------------------------------------------------------镜像
    if (1)  //最低位
    {
//      Serial.println(Num);
      digit = pgm_read_word_near(c1 + Num % 10);
      if (dot[i] == 1)
        digit += 0x80;
      SendData(digit);
      SendData(0);
    }
    else
      SendNum(10);
    if (dot[i]>=0)  //倒数第2低位
    {
      digit = pgm_read_word_near(c1 + (Num / 10) % 10);
      if (dot[i] == 2)
        digit += 0x80;
      SendData(digit);
      SendData(0);
    }
    else
      SendNum(10);
    if (dot[i]>=1) //次高位
    {
      digit = pgm_read_word_near(c1 + (Num / 100) % 10);
      if (dot[i] == 3)
        digit += 0x80;
      SendData(digit);
      SendData(0);
    }
    else
      SendNum(10);

    if (Num >= 1000)  //最高位
    {
      //      byte digit1;
      digit = pgm_read_word_near(c1 + Num / 1000);
      //    SendData(pgm_read_word_near(c1 + Num / 1000));
      //    digit = pgm_read_word_near(c2 + Num / 1000);
      if (dot[i] == 4)
        digit += 0x80;
      if (InputNum[i] < 0)
        digit += 0x40;
      SendData(digit);
      SendData(0);
    }
    else if (InputNum[i] < 0)
    {
      SendNum(11);
    }
    else
      SendNum(10);
#endif
  }
  if (brightness > 7) brightness = 7;
  if (brightness < 1) brightness = 1;
  SendCommand(0x80 + (OnOff << 3) + brightness);//显示开/关+亮度
}


//-------------------------------------------老refresh
//void TM1638Display::Refresh(byte a, byte b, byte c, byte d, byte brightness, bool OnOff, bool dota, bool dotb, bool dotc, bool dotd)
//{
//  SendCommand(0);//显示模式设置，4位8段
//  SendCommand(0x40);//数据命令设置，写数据到显存，地址自动增加
//  SendAddr(0xc0);//第一个显存地址
//  //------------------------↓这段需要720us
//  SendData(pgm_read_word_near(c1 + a));
//  SendData(pgm_read_word_near(c2 + a) + dota * 0x20);
//  SendData(pgm_read_word_near(c1 + b));
//  SendData(pgm_read_word_near(c2 + b) + dotb * 0x20);
//  SendData(pgm_read_word_near(c1 + c));
//  SendData(pgm_read_word_near(c2 + c) + dotc * 0x20);
//  SendData(pgm_read_word_near(c1 + d));
//  SendData(pgm_read_word_near(c2 + d) + dotd * 0x20);
//  //------------------------↑这段需要720us
//  if (brightness > 7) brightness = 7;
//  SendCommand(0x80 + (OnOff << 3) + brightness);//显示开/关+亮度
//}
