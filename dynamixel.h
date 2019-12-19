#ifndef DYNAMIXEL_P2_DYNAMIXEL_P2_H
#define DYNAMIXEL_P2_DYNAMIXEL_P2_H

#include <Arduino.h>

using namespace std;

class Dynamixel{
private:
    Stream *_serialport;
    int _flow_control_pin = 13;

    struct status_packet_info{
      unsigned char id;
      unsigned char error;
      unsigned char parameters[4];
    };
//Thats where we are sending data to packet array
void Header(unsigned char *pkg, unsigned char id);
void putInt8t(unsigned char *pkg ,unsigned char value, unsigned int pos);
void putInt16t(unsigned char *pkg, unsigned int value, unsigned int pos);
void putInt32t(unsigned char *pkg, unsigned long value, unsigned int pos);
void CheckPKG(unsigned char *pkg, unsigned char l);
void TransmitPacket(unsigned char *pkg); //Function sending packages
void CreateCRC(unsigned char *pkg, unsigned short blk_size);

status_packet_info ReceiveStatusPacket(); //Function reading what is inside the packet
unsigned short update_crc (unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

public:

  Dynamixel(int flow_control_pin); //Flow control pin to Dynamixel.
  void begin(long baud_rate); //Begin the communication with the Dynamixel

  //Handy functions written in Protocol 2
  void setTorqueEnable(unsigned char id, bool value);
  void setPositionGain(unsigned char id, unsigned int value);
  void setGoalPosition(unsigned char id, unsigned long value);
  void OperationMode(unsigned char id, unsigned char value);
  void Temperature(unsigned char id);
  unsigned char ReceiveTemperature();
  void CheckPKG();
  void Reboot(unsigned char id);
  void SamplingFreq();
  void Ping(unsigned char id);
  void StatusPrint();

  //Functions using other functions to perform a task
  void FullReboot(unsigned char id, unsigned long value);
};


#endif
