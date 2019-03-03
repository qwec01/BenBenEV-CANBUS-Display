void SendCommand(byte value)
{

  digitalWrite(STB, HIGH);
  digitalWrite(STB, LOW);
  digitalWrite(CLK, LOW);
  shiftOut(DIO, CLK, LSBFIRST, value);
  digitalWrite(STB, HIGH);
}
void SendAddr(byte addr)
{
  digitalWrite(STB, LOW);
  digitalWrite(CLK, LOW);
  shiftOut(DIO, CLK, LSBFIRST, addr);
  //  digitalWrite(STB, HIGH);
}
void SendData(byte data)
{
  digitalWrite(CLK, LOW);
  shiftOut(DIO, CLK, LSBFIRST, data);
}
