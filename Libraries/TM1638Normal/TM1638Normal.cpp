#include "Arduino.h"
#include "TM1638Normal.h"
#include <avr/pgmspace.h>

//--------------------------------------------数码管显示代码
//                                            0 ,  1...........3............5..........7,           9 , empty, -
const byte TM1638Display::c1[12] PROGMEM = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x00, 0x40};

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
  SendCommand(0x8F);
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
    unsigned long Num = abs(InputNum[i]) * pow(10, dot[i]);
    if (Num >= 2000 && InputNum[i] < 0) break; //第2个数没有，过
	//
	if (Num > 9999)
	{
		for (byte j=0;j<=3;j++)
		{			
			SendData(0x40);
			SendData(0x00);
		}
		continue;		
	}

    if (dot[i]>=2 || Num >= 1000)
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
    if (dot[i]>=1 || Num >= 100)
    {
      digit = pgm_read_word_near(c1 + (Num / 100) % 10);
      if (dot[i] == 2)
        digit += 0x80;
      SendData(digit);
      SendData(0);
    }
    else
      SendNum(10);
    if (dot[i]>=0 || Num >= 10)
    {
      digit = pgm_read_word_near(c1 + (Num / 10) % 10);
      if (dot[i] == 1)
        digit += 0x80;
      SendData(digit);
      SendData(0);
    }
    else
      SendNum(10);
    if (1)
    {
      digit = pgm_read_word_near(c1 + Num % 10);
      if (dot[i] == 0)
        digit += 0x80;
      SendData(digit);
      SendData(0);
    }
    else
      SendNum(10);
    }
  if (brightness > 7) brightness = 7;
  if (brightness < 1) brightness = 1;
  SendCommand(0x80 + (OnOff << 3) + brightness);//显示开/关+亮度
}