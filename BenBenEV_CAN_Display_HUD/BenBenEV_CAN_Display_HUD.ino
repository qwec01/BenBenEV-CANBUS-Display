/*
   ACAN2515全局变量使用61字节，默认接收缓存32条512字节，默认发送缓存16条256字节，一个CAN总线使用849字节
   PRI CAN接收缓存32*16=512字节，发送缓存0，变量61字节，共578字节
   SEC CAN接收缓存4*16=64字节，发送缓存0，变量61字节，共125字节
*/
#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <ACAN2515.h>
#define SEC_CS 8
#define PRI_CS 9
#define SEC_INT 2  //副CAN线，仪表板通信，包含HVAC温度etc
#define PRI_INT 3  //主CAN线，包括单体电压、电池温度、电机控制etc
#define debug 0     //不接串口屏时设为1，否则为0
#define mirror 1    //HUD是否镜像显示
#define DIO 4
#define CLK 6
#define STB 7

//flag bit:     7         6         5         4         3     2     1     0
//         HVACstat  dcdcCurrent  rstkmall  TrckFrce    Ri

char MaxBatProbTemp, MinBatProbTemp;
float energy;
int power, powert = -1, spd = 0, spdt, consume, consumet, Current;
unsigned int SOC, SOCt, SOC100, SOC_BMS, SOCt_BMS, SOC100_BMS, consumeavg, Efficiency, count185t;
unsigned int meter_per_SOC, meter_per_SOC_BMS, used_SOC, used_SOC_BMS;
byte charging = 0, hour, minute, hourt, minutet,  i, ex, spgnow, spg, bn, color = 15;
byte MaxVoltNum, MinVoltNum,  MaxBatProbTempNum, MinBatProbTempNum, HVACstat;
byte  km0, BVoltagebuf[22][8],  odo[3], eepromWrote;
byte fBatTemp = 0, fBatNum = 0, fspd = 0, flag = 0, dispatched = 0;
char Temp[8], Tempt[8], BTemp[12];
unsigned int Voltagebox, powerbox, TractionForceBox, refreshinterval = 333, BVoltage[90], Ri,  MaxVolt, MinVolt;
unsigned int kmremaining, kmall, BMSkmremaining, BMSkmall;
unsigned int dcdcCurrent;
int motorspd = 0, Voltage, Voltaget, Voltagei, MotorTorque, brightorg;
byte bright;
unsigned long runtime, lastruntime, refresh, brightruntime, ODO, ODOt, ODObegin, ODO100, t, count185 = 0, ODObeginForKmallCalc;
int BrightFilterBuf[11] = {1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024}, TractionForce;
static uint32_t XTAL = 8UL * 1000UL * 1000UL; //晶振16M
#if !mirror
//正常
//                             0 ,  1....................................,           9 , empty, -
const byte c1[12] PROGMEM = {0x1f, 0x06, 0x1b, 0x0f, 0x06, 0x0d, 0x1d, 0x07, 0xff, 0x0f, 0x00, 0x00};
const byte c2[12] PROGMEM = {0x08, 0x00, 0x10, 0x10, 0x18, 0x18, 0x18, 0x00, 0x1f, 0x18, 0x00, 0x10}; //带小数点加0x20
#else
//镜像
//                             0 ,  1....................................,           9 , empty, -
const byte c1[12] PROGMEM = {0x1f, 0x10, 0x0d, 0x19, 0x12, 0x1b, 0x1f, 0x11, 0xff, 0x1b, 0x00, 0x00};
const byte c2[12] PROGMEM = {0x08, 0x08, 0x18, 0x18, 0x18, 0x10, 0x10, 0x08, 0x1f, 0x18, 0x00, 0x10}; //带小数点加0x20
#endif
ACAN2515 PRI (PRI_CS, SPI, PRI_INT);
ACAN2515 SEC (SEC_CS, SPI, SEC_INT);


//------------------------------------------------------------------MCP2515定义
ACAN2515Mask rxm1 = standard2515Mask (0x7FF , 0, 0);
ACAN2515Settings settings (XTAL, 500UL * 1000UL);

void RefreshHUD(byte a, byte b, byte c, byte d, byte brightness, bool OnOff, bool dota = 0, bool dotb = 0, bool dotc = 0, bool dotd = 0)
{
  SendCommand(0);
  SendCommand(0x40);
  SendAddr(0xc0);
  //------------------------↓这段需要720us
  SendData(pgm_read_word_near(c1 + a));
  SendData(pgm_read_word_near(c2 + a) + dota * 0x20);
  SendData(pgm_read_word_near(c1 + b));
  SendData(pgm_read_word_near(c2 + b) + dotb * 0x20);
  SendData(pgm_read_word_near(c1 + c));
  SendData(pgm_read_word_near(c2 + c) + dotc * 0x20);
  SendData(pgm_read_word_near(c1 + d));
  SendData(pgm_read_word_near(c2 + d) + dotd * 0x20);
  //------------------------↑这段需要720us
  if (brightness > 7) brightness = 7;
  SendCommand(0x80 + (OnOff << 3) + brightness);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.setTimeout(20);
  SPI.begin();
  pinMode(CLK, OUTPUT);
  pinMode(DIO, OUTPUT);
  pinMode(STB, OUTPUT);
  //------------------------------------------------↓初始化SEC CAN线-----------------------
  {
    Serial.println(F("Configuring SEC CANBUS"));
    settings.mRequestedMode = ACAN2515Settings::NormalMode;
    settings.mReceiveBufferSize = 8;
    settings.mTransmitBuffer0Size = 0;
    ACAN2515Mask rxm0 = standard2515Mask (0x7DF , 0, 0);
    const ACAN2515AcceptanceFilter filters [] = {
      {standard2515Filter (0x307 , 0, 0), RCV0},//ID307&327，压缩机&SOC 10+10/s
      {standard2515Filter (0x7ff , 0, 0), RCV1},
      {standard2515Filter (0x3c1 , 0, 0), RCV2},//温度 10/s
      {standard2515Filter (0x2a2 , 0, 0), RCV3},//温度 10/s
      {standard2515Filter (0x2b1 , 0, 0), RCV4},//电压电流充电状态10/s
      {standard2515Filter (0x380 , 0, 0), RCV5} //ODO 1/s
    };
    unsigned int errorCode = SEC.begin (settings, [] { SEC.isr (); }, rxm0, rxm1, filters, 6); //isr中断服务程序在源码的ACAN2515.cpp里
#if debug
    if (errorCode == 0)
      Serial.println(F("SEC Succ"));
    else
    {
      Serial.print(F("SEC Init Fail, ErrCode=0x"));
      Serial.println(errorCode, HEX);
    }
#endif
  }
  //------------------------------------------------↑初始化SEC CAN线

  //------------------------------------------------↓初始化PRI CAN线
  {
    Serial.println(F("Configuring PRI CANBUS"));
    settings.mReceiveBufferSize = 26;
    settings.mTransmitBuffer0Size = 0;
    ACAN2515Mask rxm0 = standard2515Mask (0x7CF , 0, 0);
    const ACAN2515AcceptanceFilter filters [] = {
      {standard2515Filter (0x615 , 0, 0), RCV0},//ID 615&625 最高最低温度&最高最低单体电压 10+10/s
      {standard2515Filter (0x231 , 0, 0), RCV1},//DCDC电压电流 10/s
      {standard2515Filter (0x153 , 0, 0), RCV2},//电机速度、扭矩 100/s
      {standard2515Filter (0x7ff , 0, 0), RCV3},//523,续航
      {standard2515Filter (0x185 , 0, 0), RCV4},//电压电流SOC 100/s
      {standard2515Filter (0x191 , 0, 0), RCV5} //HVAC状态 100/s
    };
    //一秒330帧数据，3ms一帧  //一个中断140us
    unsigned int errorCode = PRI.begin (settings, [] { PRI.isr (); }, rxm0, rxm1, filters, 6); //664us, isr中断服务程序在源码的ACAN2515.cpp里
#if debug
    if (errorCode == 0)
      Serial.println(F("PRI Succ"));
    else
    {
      Serial.print(F("PRI Init Fail, ErrCode=0x"));
      Serial.println(errorCode, HEX);
    }
#endif
  }
  //------------------------------------------------↑初始化PRI CAN线------------------------
  EEPROM.get(0, ODObegin);  //清零时的总里程
  EEPROM.get(4, energy);    //清零后的总耗电量
  EEPROM.get(12, ODO100);   //重算续航时的总里程
  EEPROM.get(16, SOC100);   //重算续航时的SOC
  EEPROM.get(18, SOC100_BMS);
  EEPROM.get(24, ODObeginForKmallCalc); //4 bytes
  EEPROM.get(28, used_SOC);
  EEPROM.get(30, used_SOC_BMS);
  delay(50);
  while (Serial.read() >= 0);
  SPG_TPN(1, 2);
#if !debug
  while (!Serial.find("OK"));
#else
  delay(300);
#endif
  //--------------------------------------------显示EEPROM值
  Serial.print(F("DS24(0,180,'Used SOC_BMS: ")); Serial.print(used_SOC_BMS); Serial.print("',15,0);");
  Serial.print(F("DS24(0,210,'Used SOC:     ")); Serial.print(used_SOC); Serial.print("',15,0);");
  Serial.print(F("DS24(0,240,'energy:       ")); Serial.print(energy); Serial.println("',15,0);");
  for (byte i = 0; i <= 1; i++)
  {
    RefreshHUD(8, 8, 8, 8, 7, 1, 1, 1, 1);
    delay(500);
    RefreshHUD(0, 0, 0, 0, 0, 0);
    delay(500);
  }
  delay(1000);
  while (Serial.read() >= 0);
  SPG_TPN(2, 2);
  spg = 2;
  spgnow = 2;
#if !debug
  while (!Serial.find("OK"));
#else
  delay(500);
#endif
  flag += 0xF0;
  for (i = 0; i <= 7; i++)
    Tempt[i] = 127;
}


void loop()
{
  dispatch();
  runtime = millis();
  if (spg == 2 || spg == 4) refreshinterval = 333;
  if (spg == 3) refreshinterval = 1000;
  if (spg == 5) refreshinterval = 5000;
  if (runtime - brightruntime >= 200)
  {
    bright = brightFilter();
    if (motorspd > -3000)
      spd = abs(motorspd) / 7.3170; //←换胎的话修改这个数值
    if (spd < 1000)
#if !mirror
      RefreshHUD(10, spd / 100, (spd % 100) / 10, spd % 10, bright / 14.28, 1, 0, 0, 1, 0);
#else
      RefreshHUD(spd % 10, (spd % 100) / 10, spd / 100, 10, bright / 14.28, 1, 1, 0, 0, 0);
#endif
    if (spd >= 1000)
#if !mirror
      RefreshHUD(spd / 1000, (spd % 1000) / 100, (spd % 100) / 10, spd % 10, bright / 14.28, 1, 0, 0, 1, 0);
#else
      RefreshHUD(spd % 10, (spd % 100) / 10, (spd % 1000) / 100, spd / 1000,  bright / 14.28, 1, 1, 0, 0, 0);
#endif
    brightruntime = runtime;
  }
  if (Serial.available() > 0)//---------------------------------按钮
  {
    if (Serial.read() == '[')
    {
      bn = Serial.parseInt();
      if (bn != 0)
      {
                if (bn < 10 && bn != 4 && bn != 6)
                {
                  spg = bn;
                  changeFilt();
                  //printUSART();
                  lastruntime = millis();
                  runtime = lastruntime;
                }
        if (bn == 4)  //能耗清零
        {
          ODObegin = ODO;
          energy = 0;
          EEPROM.put(0, ODObegin);
          EEPROM.put(4, energy);
        }
        if (bn == 6)  //重算续航
        {
          SOC100 = SOC;
          ODO100 = ODO;
          ODObeginForKmallCalc = ODO;
          SOC100_BMS = SOC_BMS;
          used_SOC = 0;
          used_SOC_BMS = 0;
          EEPROM.put(12, ODO);  //4 bytes
          EEPROM.put(16, SOC);
          EEPROM.put(18, SOC_BMS);
          kmall = 180;
          kmremaining = 180 * (SOC / 100.0);
          BMSkmall = 180;
          BMSkmremaining = 180 * (SOC_BMS / 100.0);
          EEPROM.put(24, ODObeginForKmallCalc);
          EEPROM.put(28, used_SOC);
          EEPROM.put(30, used_SOC_BMS);
          if ((flag & 0x20) == 0)
            flag += 0x20;
        }
      }
    }
    while (Serial.read() >= 0);
  }
  if (runtime - lastruntime > refreshinterval)
  {
    printUSART();
    lastruntime = runtime;
  }

}
