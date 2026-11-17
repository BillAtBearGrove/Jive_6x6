// Basic code to test speed control for a motor
// start/stop and Input speed from serial monitor
// Run M1 only for simplicity
/*
  This SW set can used with InvacareTDX_1165865MotorData_ImpericalSpeedStudy_withController.xlsx (or similar) to 
  calculate the effective Ke, R, and Vo for the motor to get the right rpm speed calculation
  And then can use Kp, Ki, Kd to tune the PID for accurate following.
*/

#include "DualG2HighPowerMotorShield.h"
#include "PID.h"

// Setup serial monitor control variables
float rpmSetpoint = 0; // commanded rpmTarget_L if running, else 0
float rpmStep = 0.5; // max increment steps for rpm to change 1 to 100 per loop
int outputStep = 10; // max increment steps for motor output 1 to 100
int del = 0; // ms between loops 0 to 1000 ms
bool run = false; // "s" will toggle run mode 
String input = "";

// Motor Data taken from excel solver based on no load and stall measured data 
float Ke = 0.13; //0.1458; // this should be the motor KE reflected to the output shaft (Ke motor * Gear Ratio)
float resistance = 0.428; //0.6; //0.311;
float Vo = 1.88; // estimate voltage drop used in speed calc

float rpmTarget_L = 0;  // operator/requested RPM
float rpmCmd_L = 0;     // commanded RPM after rate limiting
float rpmEstimate_L = 0; // calculated rpm = (v-(i*r))/Ke where v = 24v supply * cmdM1/400
float rpmMax = 150; // maximum commandable rpm
float voltageSupply = 24; // assumes 24V battery voltage
float voltageSent = 0; // calculated voltage sent to motor = vSupply * PIDcmd
float amps_L = 0;
int cmdM1 = 0;

bool echo = false;

float Kp = 0.1; // suggested 0-1
float Ki = 0.01; // suggested 0-0.1
float Kd = 0.4; // suggested 0-0.1
float iForget = 0.005; // 0.005; // 0-1 = % to forget the Integrator each steps
float PID_min = -1;
float PID_max = 1;
float PID_rateLim = 0.001; //0.005; //0.001;
float PIDout_L = 0; // -1 to 1 as percent duty cycle to send to M1 motor


// PID setup classes
  PID motorPID_L;

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
  delay(2000); // give time for serial monitor to start, otherwise it resets sketch for some reason

  Serial.println("SETUP START");
  Serial.println("Dual G2 High Power Motor Shield");

  Serial.println();
  Serial.println("s = Start/Stop; rpmTarget = -100 to 100; step = 1 to 100 per step");
  Serial.println("Commands:");
  Serial.println("20 (or any number -100 to 100) sets rpmTarget as percent full speed");
  Serial.println("rpmStep=1 where rpmStep = 1 to 100 per ramp loop");
  Serial.println("outputStep=1 where outputStep = 1 to 100 per ramp loop");
  Serial.println("del=1 where del = 1 to 100 ms per ramp loop");
  Serial.println("<enter> starts and stops motor");
  Serial.println();
  Serial.println("Kp=0.1, Ki=0.01, Kd=1; or iForget=0.005 sets Kp, Ki, Kd and iForget terms");
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
    motorPID_L.setLimits(PID_min, PID_max);
    motorPID_L.setConstants(Kp, Ki, Kd, iForget);
    motorPID_L.setRateLimit(PID_rateLim);
 
}

void loop()
{
  stopIfFault();
  if (Serial.available()) {
    readSerial();
  }  
  rpmSetpoint = run? rpmTarget_L:0; // set rpmSetpoint to zero if not run (steps down)

  if (rpmCmd_L < rpmSetpoint) {
    rpmCmd_L = constrain(rpmCmd_L + rpmStep, -rpmMax, rpmSetpoint);
  }
  if (rpmCmd_L > rpmSetpoint) {
    rpmCmd_L = constrain(rpmCmd_L - rpmStep, rpmSetpoint, rpmMax);
  }

  float FeedForward_L = rpmCmd_L/rpmMax; // basic feedfoward term to use for now
  rpmEstimate_L = calcRpm_L();
  PIDout_L = motorPID_L.calc( rpmCmd_L - rpmEstimate_L, FeedForward_L ); // error controller PID(error, FeedForward)
  
  long temp = (long)maxOut * PIDout_L; // use PID to control output
  int cmdM1Prev = cmdM1;
  cmdM1 = constrain(temp, max(-maxOut, cmdM1Prev - outputStep), min(maxOut, cmdM1Prev + outputStep)); // temporary rate limted changes to output
  setM1(cmdM1);

  delay(del);
}

/* ////////////////////////////////////////////////////////// */
/* ////////////////////////////////////////////////////////// */
float calcRpm_L()
{
  float firstOrdFilt = 0.01; // filter to apply to measured current
  amps_L += ( (md.getM1CurrentMilliamps()/1000) - amps_L) * firstOrdFilt; // measure current and filter

  float temp = abs(((float)voltageSupply*(float)cmdM1) / 400);
  float rpmEstimate_L = constrain( (temp - Vo - (amps_L * resistance) )/Ke , 0, rpmMax * 1.1); // rpm = (v -vo-(i*r))/Ke

  if (cmdM1<0){
    rpmEstimate_L = -rpmEstimate_L; // make negative if drive backwards
  }
  if( echo ){
    //Serial.print(" temp = "); Serial.print(temp);
    //Serial.print(" amps_L * resistance = "); Serial.print(amps_L * resistance);
  }

  return rpmEstimate_L;
}

void measure()
{
  Serial.println();
  Serial.print(" rpmCmd_L = "); Serial.print(rpmCmd_L);
  Serial.print(" cmdM1 = "); Serial.println(cmdM1);
  Serial.print("    V = "); Serial.print( (float)(24*cmdM1)/400, 2);
  Serial.print(" I = "); Serial.print(amps_L, 4);
  Serial.print(" rpmEstimate_L = "); Serial.println(rpmEstimate_L);
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
  /*
  if (i%200 == 100)
  {
    Serial.print("M1 current: ");
    Serial.println(md.getM1CurrentMilliamps());
	}
  delay(10);
  */
}

// MOTOR 2
void setM2(int i)
{
  md.setM2Speed(i);
  stopIfFault();
  /*
  if (i%200 == 100)
  {
    Serial.print("M2 current: ");
    Serial.println(md.getM2CurrentMilliamps());
	}
  delay(10);
  */
}

void readSerial()
{
      String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "") {
        run = !run;
        Serial.print("changing run to ");  Serial.println(run);
    }
    else if (cmd == "m") {
      Serial.println();
      Serial.print("measuring... "); 
      measure();
    }
    else if (cmd == "echo") {
      echo = !echo;
    }
    else if (cmd.indexOf('=') == -1) {
        // Plain number entered
        float temp = constrain(cmd.toFloat(), -rpmMax, rpmMax);
        Serial.print("changing rpmTarget_L "); Serial.print(rpmTarget_L);
        Serial.print(" to "); Serial.println(temp);
        rpmTarget_L = temp;
    }
    else {
        int pos = cmd.indexOf('=');

        String var = cmd.substring(0, pos);
        String val = cmd.substring(pos + 1);

        if (var == "rpmTarget_L") {
            float temp = constrain(val.toFloat(), -rpmMax, rpmMax);
            Serial.print("changing rpmTarget_L "); Serial.print(rpmTarget_L);
            Serial.print(" to "); Serial.println(temp);
            rpmTarget_L = temp;
        }
        else if (var == "rpmStep") {
            float temp = constrain(val.toFloat(), 0.01, 100);
            Serial.print("changing rpmStep "); Serial.print(rpmStep);
            Serial.print(" to "); Serial.println(temp);
            rpmStep = temp;
        }
        else if (var == "outputStep") {
            float temp = constrain(val.toInt(), 1, 100);
            Serial.print("changing outputStep "); Serial.print(outputStep);
            Serial.print(" to "); Serial.println(temp);
            outputStep = temp;
        }
        else if (var == "del") {
            float temp = constrain(val.toInt(), 0, 1000);
            Serial.print("changing del "); Serial.print(del);
            Serial.print(" to "); Serial.println(temp);
            del = temp;
        }
        else if (var == "Kp") {
            float temp = constrain(val.toFloat(), 0, 10);
            Serial.print("changing Kp "); Serial.print(Kp,4);
            Serial.print(" to "); Serial.println(temp);
            Kp = temp;
        }
        else if (var == "Ki") {
            float temp = constrain(val.toFloat(), 0, 1);
            Serial.print("changing Ki "); Serial.print(Ki);
            Serial.print(" to "); Serial.println(temp);
            Ki = temp;
        }
        else if (var == "Kd") {
            float temp = constrain(val.toFloat(), 0, 1);
            Serial.print("changing Kd "); Serial.print(Kd,4);
            Serial.print(" to "); Serial.println(temp);
            Kd = temp;
        }
        else if (var == "iForget") {
            float temp = constrain(val.toFloat(), 0, 1);
            Serial.print("changing iForget "); Serial.print(iForget,4);
            Serial.print(" to "); Serial.println(temp);
            iForget = temp;
        }
        else {
            Serial.println("Unknown variable");
        }
      
      motorPID_L.setConstants(Kp, Ki, Kd, iForget); // send pid variables to PID object
    }
  /*
    Serial.println();
    Serial.print("  run = ");  Serial.print(run);
    Serial.print(" rpmTarget_L = "); Serial.print(rpmTarget_L);
    Serial.println();

    Serial.print("  rpmStep = ");  Serial.print(rpmStep);
    Serial.print(" outputStep = ");  Serial.print(outputStep);
    Serial.print(" delay = ");  Serial.print(del);
    Serial.println();

    Serial.print("  Kp = "); Serial.print(Kp);
    Serial.print(" Ki = "); Serial.print(Ki);
    Serial.print(" Kd = "); Serial.print(Kd);
    Serial.print(" iForget = "); Serial.print(iForget);
    Serial.println();
    Serial.println();
  */
}