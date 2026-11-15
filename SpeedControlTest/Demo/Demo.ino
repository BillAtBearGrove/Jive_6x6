// Basic code to test speed control for a motor
// start/stop and Input speed and step rate from serial monitor
// Run M1 only for simplicity

#include "DualG2HighPowerMotorShield.h"

// Setup serial monitor control variables
int speed = 0; // limited to -100 to 100 percent speed
int speedNow = 0; // limited to -100 to 100 percent speed
int step = 5; // increment steps 1 to 100
int del = 5; // ms between increment/decrement 0 to 1000
bool run = false; // "s" will toggle run mode 
int target = 0; // target speed that we will step to when we start
String input = "";

int maxOut = 400;

// Uncomment the version corresponding with the version of your shield.
//DualG2HighPowerMotorShield24v14 md;
// DualG2HighPowerMotorShield18v18 md;
 DualG2HighPowerMotorShield24v18 md;
// DualG2HighPowerMotorShield18v22 md;


/* ////////////////////////////////////////////////////////// */
/* ////////////////////////////////////////////////////////// */

void setup()
{
  Serial.begin(115200);
  delay(3000); // give time for serial monitor to start, otherwise it resets sketch for some reason

  Serial.println("SETUP START");
  Serial.println("Dual G2 High Power Motor Shield");

  Serial.println();
  Serial.println("s = Start/Stop; speed = -100 to 100; step = 1 to 100 per step");
  Serial.println("Commands:");
  Serial.println("target=20 where target = -100 to 100 as percent full speed");
  Serial.println("step=1 where step = 1 to 100 per ramp loop");
  Serial.println("del=1 where del = 1 to 100 ms per ramp loop");
  Serial.println("<enter> starts and stops motor");
  Serial.println();

  md.init();
  delay(2); // The drivers require a maximum of 1ms to elapse when brought out of sleep mode.
  md.calibrateCurrentOffsets();
  delay(2);

  // Uncomment to flip a motor's direction:
  //md.flipM1(true);
  //md.flipM2(true);

  md.enableDrivers();
  delay(2);
}

void loop()
{
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "") {
        run = !run;
    }
    else if (cmd == "m") {
      measure();
    }
    else if (cmd.indexOf('=') == -1) {
        // Plain number entered
        target = constrain(cmd.toInt(), -100, 100);

        Serial.print("changing target to ");
        Serial.println(target);
    }
    else {
        int pos = cmd.indexOf('=');

        String var = cmd.substring(0, pos);
        String val = cmd.substring(pos + 1);

        if (var == "target") {
            target = constrain(val.toInt(), -100, 100);
            Serial.print("changing target to ");
            Serial.println(target);
        }
        else if (var == "step") {
            step = constrain(val.toInt(), 1, 100);
            Serial.print("changing step to ");
            Serial.println(step);
        }
        else if (var == "del") {
            del = constrain(val.toInt(), 0, 1000);
            Serial.print("changing delay to ");
            Serial.println(del);
        }
        else {
            Serial.println("Unknown variable");
        }
    }
  
    Serial.print("  run    = ");  Serial.println(run);
    Serial.print("  target = "); Serial.println(target);
    Serial.print("  speed  = "); Serial.println(speed);
    Serial.print("  step   = ");  Serial.println(step);
    Serial.print("  del    = ");  Serial.println(del);
    Serial.println();
  
    speed = run? target:0; // set speed to zero if not run (steps down)

    while (speedNow < speed) {
      speedNow = constrain(speedNow + step, -100, 100);
      long temp = (long)maxOut * speedNow / 100;
      int setPoint = constrain(temp, -maxOut, maxOut);
      setM1(setPoint);
      delay(del);
    }

    while (speedNow > speed) {
      speedNow = constrain(speedNow - step, -100, 100);
      long temp = (long)maxOut * speedNow / 100;
      int setPoint = constrain(temp, -maxOut, maxOut);
      setM1(setPoint);
      delay(del);
    }

    /* Serial.println();Serial.print("  speedNow (setPoint) = "); Serial.println(speedNow);
    float avgCurr = 0;
    for (int i=0; i<100; i++){
      avgCurr +=md.getM1CurrentMilliamps();
    }
    Serial.println();Serial.print("  avgCurr = "); Serial.println(avgCurr/100);
    */
    measure();
  }
}

/* ////////////////////////////////////////////////////////// */
/* ////////////////////////////////////////////////////////// */
void measure()
{
  Serial.println();Serial.print("  speedNow (setPoint) = "); Serial.println(speedNow);
  float avgCurr = 0;
  for (int i=0; i<100; i++){
    avgCurr +=md.getM1CurrentMilliamps();
  }
  Serial.println();Serial.print("  avgCurr = "); Serial.println(avgCurr/100);
}

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
