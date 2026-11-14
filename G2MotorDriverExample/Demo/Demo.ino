#include "DualG2HighPowerMotorShield.h"

// Uncomment the version corresponding with the version of your shield.
//DualG2HighPowerMotorShield24v14 md;
// DualG2HighPowerMotorShield18v18 md;
 DualG2HighPowerMotorShield24v18 md;
// DualG2HighPowerMotorShield18v22 md;

// Setup serial monitor control variables

void stopIfFault()
{
  if (md.getM1Fault())
  {
    md.disableDrivers();
	delay(1);
    Serial.println("M1 fault");
    while (1);
  }
  if (md.getM2Fault())
  {
    md.disableDrivers();
	delay(1);
    Serial.println("M2 fault");
    while (1);
  }
}

// MOTOR 1
void setM1(int i)
{
  md.setM1Speed(i);
  stopIfFault();
  if (i%200 == 100)
  {
    Serial.print("M1 current: ");
    Serial.println(md.getM1CurrentMilliamps());
	}
  delay(10);
}

// MOTOR 2
void setM2(int i)
{
  md.setM2Speed(i);
  stopIfFault();
  if (i%200 == 100)
  {
    Serial.print("M2 current: ");
    Serial.println(md.getM2CurrentMilliamps());
	}
  delay(10);
}


void setup()
{
  Serial.begin(115200);
  delay(3000); // give time for serail to start, otherwise it resets sketch for some reason

  Serial.println("SETUP START");
  //Serial.println("Dual G2 High Power Motor Shield");
  md.init();
  delay(2);
  md.calibrateCurrentOffsets();
 
  delay(2); // The drivers require a maximum of 1ms to elapse when brought out of sleep mode.

  // Uncomment to flip a motor's direction:
  //md.flipM1(true);
  //md.flipM2(true);
}

void loop()
{
  md.enableDrivers();
  delay(2);

  for (int i = 0; i <= 400; i++)
  {  setM1(i);  }
  for (int i = 400; i >= -400; i--)
  {  setM1(i);  }
  for (int i = -400; i <= 0; i++)
  {  setM1(i);  }


  for (int i = 0; i <= 400; i++)
  {  setM2(i);  }
  for (int i = 400; i >= -400; i--)
  {  setM2(i);  }
  for (int i = -400; i <= 0; i++)
  {  setM2(i);  }

  md.disableDrivers(); // Put the MOSFET drivers into sleep mode.
  delay(500);

}
