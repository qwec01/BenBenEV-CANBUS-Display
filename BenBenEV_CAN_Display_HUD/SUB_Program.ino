//----------------------------------------------------------------------------------------//
//                                    ↓CAN报文处理子程序↓                                  //
//----------------------------------------------------------------------------------------//
static void RCV0 (const CANMessage & inMessage) {   //ID=307和327、615和625
  //  Serial.println(inMessage.id);
  if (inMessage.id == 0x307)  //SOC
  {
    SOC = (inMessage.data[1] << 2) + (inMessage.data[2] >> 6);
  }

  if (inMessage.id == 0x615) //单体电压最值
  {
    MinVolt = (inMessage.data[0] << 8) + inMessage.data[1];
    MinVoltNum = inMessage.data[2];
    MaxVolt = (inMessage.data[4] << 8) + inMessage.data[5];
    MaxVoltNum = inMessage.data[6];
  }
  if (inMessage.id == 0x625) //温度最值
  {
    if ((MinBatProbTemp != inMessage.data[0] - 40) || (MaxBatProbTemp != inMessage.data[3] - 40))
    {
      MinBatProbTemp = inMessage.data[0] - 40;
      MaxBatProbTemp = inMessage.data[3] - 40;
      fBatTemp = 1;
    }
    if ((MinBatProbTempNum != inMessage.data[1]) || (MaxBatProbTempNum != inMessage.data[4]))
    {
      MinBatProbTempNum = inMessage.data[1];
      MaxBatProbTempNum = inMessage.data[4];
      fBatNum = 1;
    }
  }
  if (inMessage.id == 0x675)    //电池温度1-8
  {
    for (i = 0; i <= 7; i++)
      BTemp[i] = inMessage.data[i] - 40;
  }
  if (inMessage.id == 0x685)    //电池温度9-12
  {
    for (i = 0; i <= 3; i++)
      BTemp[i + 8] = inMessage.data[i] - 40;
  }
}

static void RCV1 (const CANMessage & inMessage)   //ID=231、675、685
{
  //8us
  //  Serial.println(inMessage.id);
  if (inMessage.id == 0x231) //DCDC电流
  {
    if (dcdcCurrent != (inMessage.data[3] << 4) + (inMessage.data[4] >> 4))
    {
      dcdcCurrent = (inMessage.data[3] << 4) + (inMessage.data[4] >> 4);
      if ((flag & 0x40) == 0)
        flag += 0x40;
    }
  }
  if (inMessage.id >= 0x640 && inMessage.id <= 0x656) //所有单体电压
  {
    byte pos;
    pos = (inMessage.id - 0x641);
    memcpy(BVoltagebuf[pos], inMessage.data, 8);
  }
    if (inMessage.id == 0x675)    //电池温度1-8，单体电压界面
    {
      for (i = 0; i <= 7; i++)
        BTemp[i] = inMessage.data[i] - 40;
    }
  if (inMessage.id == 0x349)
  {
    ChgVin = (inMessage.data[0] << 1) + (inMessage.data[1] >> 7); //市电电压，1倍
    ChgIin = ((inMessage.data[1]&0x1F) << 3) + (inMessage.data[2] >> 5); //市电电流，10倍
  }
}

static void RCV2 (const CANMessage & inMessage)   //ID=3C1, 153，231
{

  if (inMessage.id == 0x3C1) //温度
  {
    Temp[4] = inMessage.data[0] - 40; //蒸发器
    Temp[5] = inMessage.data[1] - 40; //制热液
    Temp[6] = inMessage.data[2] - 40; //冷却液
    Temp[7] = inMessage.data[7] - 40; //回风口
  }

  if (inMessage.id == 0x153) //电机转速转矩
  {

    if (motorspd != (inMessage.data[0] << 8) + inMessage.data[1] - 12000)
    {
      //4-8us
      motorspd = (inMessage.data[0] << 8) + inMessage.data[1] - 12000;
      fspd = 1;

    }
    if (MotorTorque != (inMessage.data[2] << 4) + (inMessage.data[3] >> 4) - 1997)
    {
      //计算牵引力时24-28us，不计算8us
      MotorTorque = (inMessage.data[2] << 4) + (inMessage.data[3] >> 4) - 1997; //10倍，轮上转矩=T*7.930428
      if ((flag & 0x10) == 0)
        flag += 0x10;

    }

  }

  if (inMessage.id == 0x231) //DCDC电流
  {
    if (dcdcCurrent != (inMessage.data[3] << 4) + (inMessage.data[4] >> 4))
    {
      dcdcCurrent = (inMessage.data[3] << 4) + (inMessage.data[4] >> 4);
      if ((flag & 0x40) == 0)
        flag += 0x40;
    }
  }

}

static void RCV3 (const CANMessage & inMessage)   //ID=2A2, 523，325
{
  if (inMessage.id == 0x2A2)  //温度
  {
    for (i = 0; i <= 3; i++)
      Temp[i] = inMessage.data[i + 3] - 50;
  }
  //  if (inMessage.id == 0x523)  //续航
  //  {
  //
  //  }
  if (inMessage.id == 0x325)
  {
    hour = ((inMessage.data[2] << 8) + inMessage.data[3]) / 3600;
    minute = (((inMessage.data[2] << 8) + inMessage.data[3]) - hour * 3600) / 60;
  }
  if (inMessage.id == 0x685)    //电池温度9-12
  {
    for (byte i = 0; i <= 3; i++)
      BTemp[i + 8] = inMessage.data[i] - 40;
  }
}

static void RCV4 (const CANMessage & inMessage)   //ID=2B1,185
{
  if (inMessage.id == 0x2B1) //电压电流续航
  {
    km0 = inMessage.data[6];
  }
  if (inMessage.id == 0x185) //电压电流SOC
  {
    //8-12us
    Voltage =  (inMessage.data[0] << 5) + (inMessage.data[1] >> 3); //10倍
    Current =  ((inMessage.data[1] & 0x07) << 11) + (inMessage.data[2] << 3) + ((inMessage.data[3] & 0xE0) >> 5) - 6000; //10倍
    if (SOC_BMS != ((inMessage.data[3] & 0x1F) << 5) + (inMessage.data[4] >> 3))
    {
      SOC_BMS = ((inMessage.data[3] & 0x1F) << 5) + (inMessage.data[4] >> 3); //10倍
    }
    count185++;

  }

}

static void RCV5 (const CANMessage & inMessage)   //ID=191, 380
{
  if (inMessage.id == 0x380)
  {
    for (i = 0; i <= 2; i++)
      odo[i] = inMessage.data[i + 4];
  }
  if (HVACstat != inMessage.data[0])
  {
    HVACstat = inMessage.data[0];
    if ((flag & 0x80) == 0)
      flag += 0x80;
  }

}
//----------------------------------------------------------------------------------------//
//                                    ↑CAN报文处理子程序↑                                  //
//----------------------------------------------------------------------------------------//

//----------------------------------------------------------------------------------------//
//                                       ↓自定义子程序↓                                    //
//----------------------------------------------------------------------------------------//
void comma(char x)
{
  Serial.print(x);
}
void LABL(byte m, int x1, int y1, int x2, String str, byte c, byte ali)
{
  Serial.print("LABL(");
  dispatch();
  Serial.print(m); comma(',');
  dispatch();
  Serial.print(x1); comma(',');
  Serial.print(y1); comma(',');
  dispatch();
  Serial.print(x2); comma(',');
  Serial.print('\'');
  Serial.print(str);
  Serial.print('\''); comma(',');
  dispatch();
  Serial.print(c); comma(',');
  Serial.print(ali); Serial.print(");");
  dispatch();
}

void SPG_TPN(byte x, byte y)
{
  Serial.print("SPG(");
  Serial.print(x);
  Serial.print(");");
  Serial.print("TPN(");
  Serial.print(y);
  Serial.println(");");
}

void BOXF(int x1, int y1, int x2, int y2, byte c)
{
  //BOXF(x1,y1,x2,y2,c);
  //用颜色 c 画一个实心方框，左上角(x1,y1),右下角(x2,y2)
  Serial.print("BOXF(");
  Serial.print(x1); comma(',');
  dispatch();
  Serial.print(y1); comma(',');
  Serial.print(x2); comma(',');
  dispatch();
  Serial.print(y2); comma(',');
  Serial.print(c);
  dispatch();
  Serial.print(");");
  dispatch();

}

void CELS(byte l, byte h, unsigned int x, byte color)
{
  Serial.print("CELS(24,");
  dispatch();
  Serial.print(l); comma(',');
  Serial.print(h); Serial.print(",'");
  dispatch();
  Serial.print(x); Serial.print("',15,");
  Serial.print(color);
  Serial.print(",1);");
  dispatch();
}
//--------------------------------------亮度滤波程序
byte brightFilter()
{
  byte x;
  BrightFilterBuf[10] = analogRead(0);
  for (x = 0; x < 10; x++)
  {
    BrightFilterBuf[x] = BrightFilterBuf[x + 1]; // 所有数据左移，低位仍掉
    brightorg += BrightFilterBuf[x];
  }
  brightorg = brightorg / 10;
  bright = brightorg / 10.0 * 1.754;
  //  if (brightorg > 420)
  //    bright = -0.0575 * (float)brightorg + 53.997;
  //  if (brightorg <= 420)
  //    bright = -0.2171 * (float)brightorg + 121.8;
  if (bright < 3) bright = 3;
  if (bright > 100) bright = 100;
  return bright;
}
//------------------------刷新串口屏，以防超过屏的处理能力
void execute()
{
  if (ex >= 25)
  {
    Serial.println();
    ex = 0;
#if !debug
    while (!Serial.find("OK"));
    dispatch();
#else
    delay(75);
    dispatch();
#endif
  }

}

//--------------------------取出CAN数据
void dispatch()
{
  while (SEC.dispatchReceivedMessage());
  while (PRI.dispatchReceivedMessage());

}

void setZero()
{
  Voltaget = 0;
  SOCt = 0;
  powert = 0;
  consumet = 0;
  hour = 0; minute = 0; hourt = 0; minutet = 0;
  spd = 0;
  MaxBatProbTemp = -40;
  MinBatProbTemp = -40;
  for (i = 0; i <= 7; i++)
  {
    Temp[i] = 0;
    Tempt[i] = 0;
  }
  for (i = 0; i <= 11; i++)
  {
    BTemp[i] = 0;
  }
}
//----------------------------------------------------------------------------------------//
//                                      ↑自定义子程序↑                                     //
//----------------------------------------------------------------------------------------//
