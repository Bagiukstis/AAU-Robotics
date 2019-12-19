#include "dynamixel.h"

Dynamixel ROBOT(13);

void setup() {
  Serial.begin(57600);
  ROBOT.begin(57600);

 //ROBOT.setTorqueEnable(0xFE, (bool) true);
 //ROBOT.setPositionGain(0xFE, (unsigned int) 100);
 //ROBOT.setGoalPosition(0xFE, (unsigned long) 2048);
 //ROBOT.FullReboot(0xFE, 1300); //Sets robot to pos, reboots and sets it back to the same pos

 //ROBOT.Temperature(0x07);

 //ROBOT.Ping(0xFE); //To see available ID's, ping to all and then look for available ID's using function StatusPrint.
 //ROBOT.StatusPrint();
 //ROBOT.StatusPrint();
 //ROBOT.StatusPrint();
 //ROBOT.StatusPrint();
 //ROBOT.StatusPrint();

}

void loop() {
//ROBOT.SamplingFreq(); //To measure temperature in 10Hz samplingfreq, temperature for specific motor must be states in void setup.
}
