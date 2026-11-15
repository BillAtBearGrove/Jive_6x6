// Basic code to test speed control for a motor
// start/stop and Input speed and step rate from serial monitor
// Run M1 only for simplicity

#include "DualG2HighPowerMotorShield.h"
#include "PID.h"

// Setup serial monitor control variables
int target = 0; // target speed that we will step to when we start
int speed = 0; // commanded speed target (desired after ramp up/down)
int speedNow = 0; // instantaneous commanded speed (may be different than speed while ramping up/down)
int step = 1; // increment steps 1 to 100
int del = 5; // ms between loops 0 to 1000 ms
bool run = false; // "s" will toggle run mode 
String input = "";

double resistance = 0.311;
double Ke = 0.1458; // this should be the motor KE reflected to the output shaft (Ke motor * Gear Ratio)
double rpmMeas = 0;
double rpmTarget = 0;
double voltageSupply = 24;
double voltageSent = 0;
int setPoint = 0;

double Kp = 0.01; //10; //0.015;3
double Ki = 0; //0.001; //0.002;
double Kd = 0; //1; //0.02;
double iForget = 0.005; // 0.005; // 0-1 = % to forget the Integrator each step
double PID_min = -1;
double PID_max = 1;
double PID_rateLim = 0.001; //0.005; //0.001;


// PID setup classes
  PID motorPID;

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

 // PID setup
    motorPID.setLimits(PID_min, PID_max);
    motorPID.setConstants(Kp, Ki, Kd, iForget);
    motorPID.setRateLimit(PID_rateLim);
 
}

void loop()
{
  stopIfFault();
  if (Serial.available()) {
    readSerial();
  }  
  speed = run? target:0; // set speed to zero if not run (steps down)

  if (speedNow < speed) {
    speedNow = constrain(speedNow + step, -100, 100);
  }
  if (speedNow > speed) {
    speedNow = constrain(speedNow - step, -100, 100);
  }
  
  long temp = (long)maxOut * speedNow / 100;
  setPoint = constrain(temp, -maxOut, maxOut);
  setM1(setPoint);

  delay(del);
}

/* ////////////////////////////////////////////////////////// */
/* ////////////////////////////////////////////////////////// */
void measure()
{
  Serial.println();
  Serial.print("  speedNow = "); Serial.println(speedNow);
  float avgCurr = 0;
  int num = 1000; // number of samples
  for (int i=0; i<num; i++){
    avgCurr +=md.getM1CurrentMilliamps();
  }
  avgCurr = avgCurr/num/1000; // current in amps
  Serial.print("  avgCurr = "); Serial.println(avgCurr);
  float temp = abs(((float)voltageSupply*(float)speedNow) / 100);
  Serial.print("  temp = "); Serial.print(temp);
  Serial.print("  i = "); Serial.print(avgCurr, 4);
  Serial.print("  R = "); Serial.print(resistance, 4);
  Serial.print("  Ke = "); Serial.print(Ke, 4);
  float rpmEstimate = (temp - (avgCurr * resistance) )/Ke; // rpm = (v-(i*r))/Ke
  Serial.print("  rpmEstimate = "); Serial.println(rpmEstimate);
  Serial.println();
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

void readSerial()
{
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

}