
//----------------------------------------------------------------------------------------//
//                                        ↓更改滤波选项↓                                   //
//----------------------------------------------------------------------------------------//
/*------------如果FLASH不足，将“setFiltersOnTheFly”改为"begin"，前者多占200字节----------------*/
void changeFilt()
{
  if (spg == 2)
  {
    //Serial.println("Configuring PRI CANBUS to Running State");
    settings.mReceiveBufferSize = 26;
    settings.mTransmitBuffer0Size = 0;
    ACAN2515Mask rxm0 = standard2515Mask (0x7CF , 0, 0);
    const ACAN2515AcceptanceFilter filters [] = {
      {standard2515Filter (0x615 , 0, 0), RCV0},
      {standard2515Filter (0x231 , 0, 0), RCV1},
      {standard2515Filter (0x153 , 0, 0), RCV2},
      {standard2515Filter (0x523 , 0, 0), RCV3},
      {standard2515Filter (0x185 , 0, 0), RCV4},
      {standard2515Filter (0x191 , 0, 0), RCV5}
    };

    unsigned int errorCode = PRI.setFiltersOnTheFly (rxm0, rxm1, filters, 6);
  }
  if (spg == 3)
  {
    //Serial.println("Configuring PRI CANBUS to Chargin State");
    settings.mReceiveBufferSize = 26;
    settings.mTransmitBuffer0Size = 0;
    ACAN2515Mask rxm0 = standard2515Mask (0x70F , 0, 0);
    const ACAN2515AcceptanceFilter filters [] = {
      {standard2515Filter (0x615 , 0, 0), RCV0},
      {standard2515Filter (0x349 , 0, 0), RCV1},
      {standard2515Filter (0x231 , 0, 0), RCV2},
      {standard2515Filter (0x325 , 0, 0), RCV3},
      {standard2515Filter (0x185 , 0, 0), RCV4},
      {standard2515Filter (0x191 , 0, 0), RCV5}
    };
    unsigned int errorCode = PRI.setFiltersOnTheFly (rxm0, rxm1, filters, 6);
  }
  if (spg == 5)
  {
    settings.mReceiveBufferSize = 26;
    settings.mTransmitBuffer0Size = 0;
    ACAN2515Mask rxm0 = standard2515Mask (0x7C0 , 0, 0);
    const ACAN2515AcceptanceFilter filters [] = {
      {standard2515Filter (0x615 , 0, 0), RCV0},//温度电压最值,含675
      {standard2515Filter (0x640 , 0, 0), RCV1},//单体电压
      {standard2515Filter (0x675 , 0, 0), RCV2},//前8个温度
      {standard2515Filter (0x685 , 0, 0), RCV3},//后4个温度
      {standard2515Filter (0x7FF , 0, 0), RCV4},//市电电压
      {standard2515Filter (0x7FF , 0, 0), RCV5}
    };
    unsigned int errorCode = PRI.setFiltersOnTheFly (rxm0, rxm1, filters, 6); 
  }
}
//----------------------------------------------------------------------------------------//
//                                      ↑更改滤波选项↑                                  //
//----------------------------------------------------------------------------------------//
