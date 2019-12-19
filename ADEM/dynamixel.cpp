//Actuators, Drivers and Electronic Modules exam cpp code for Arduino.

#include "dynamixel.h"
unsigned char pkg[30]; //Packet array of 30 positions. A bit too much but just to be sure:)

//http://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/
//http://emanual.robotis.com/docs/en/dxl/protocol2/

Dynamixel::Dynamixel(int flow_control_pin) { //Flow control pin 13 is OUTPUT (Sending signals)
    _flow_control_pin = flow_control_pin;
    pinMode(_flow_control_pin, OUTPUT);
}
//Command packets!
void Dynamixel::OperationMode(unsigned char id, unsigned char value){
  Dynamixel::Header(pkg,id);
  Dynamixel::putInt16t(pkg, 6,5);
  Dynamixel::putInt8t(pkg, 0x03,7);
  Dynamixel::putInt16t(pkg, 11, 8);
  Dynamixel::putInt8t(pkg, value, 10);
  Dynamixel::CreateCRC(pkg, 11);
  Dynamixel::TransmitPacket(pkg);
}
void Dynamixel::setPositionGain(unsigned char id, unsigned int value){
  Dynamixel::Header(pkg, id);
  Dynamixel::putInt16t(pkg, 7, 5);
  Dynamixel::putInt8t(pkg, 0x03,7);
  Dynamixel::putInt16t(pkg, 84, 8);
  Dynamixel::putInt16t(pkg, value, 10);
  Dynamixel::CreateCRC(pkg, 12);
  Dynamixel::TransmitPacket(pkg);
}
void Dynamixel::setTorqueEnable(unsigned char id, bool state){
  Dynamixel::Header(pkg, id);
  Dynamixel::putInt16t(pkg, 6,5);
  Dynamixel::putInt8t(pkg, 0x03,7);
  Dynamixel::putInt16t(pkg, 64, 8);
  Dynamixel::putInt16t(pkg, state, 10);
  Dynamixel::CreateCRC(pkg, 11);
  Dynamixel::TransmitPacket(pkg);
}

void Dynamixel::setGoalPosition(unsigned char id, unsigned long value){
  Dynamixel::Header(pkg, id);
  Dynamixel::putInt16t(pkg, 9,5);
  Dynamixel::putInt8t(pkg, 0x03,7);
  Dynamixel::putInt16t(pkg, 116,8);
  Dynamixel::putInt32t(pkg, value, 10);
  Dynamixel::CreateCRC(pkg, 14);
  Dynamixel::TransmitPacket(pkg);
}

void Dynamixel::Temperature(unsigned char id){
//According to robotis, 1 unit = 1 degree of celcius.
  Dynamixel::Header(pkg, id);
  Dynamixel::putInt16t(pkg, 7,5);
  Dynamixel::putInt8t(pkg, 0x02, 7);
  Dynamixel::putInt16t(pkg, 146, 8);
  Dynamixel::putInt16t(pkg, 1, 10);
  Dynamixel::CreateCRC(pkg, 12);
  Dynamixel::CheckPKG(pkg, 14);
  Dynamixel::TransmitPacket(pkg);
  if(id>0x05){
    Serial.println(" ");
    Serial.println("ID not found");
    Serial.println("Error 0x08");
  }
  else if(Dynamixel::ReceiveTemperature() > 40){
    Serial.println("Temperature is above the limit (40C), rebooting....");
    Dynamixel::Reboot(id);
  }
}

void Dynamixel::Reboot(unsigned char id){
  Dynamixel::Header(pkg, id);
  Dynamixel::putInt16t(pkg,3 ,5);
  Dynamixel::putInt8t(pkg, 0x08, 7);
  Dynamixel::CreateCRC(pkg, 8);
  Dynamixel::TransmitPacket(pkg);
}

void Dynamixel::Ping(unsigned char id){
  Dynamixel::Header(pkg, id);
  Dynamixel::putInt16t(pkg, 3,5);
  Dynamixel::putInt8t(pkg, 0x01,7);
  Dynamixel::CreateCRC(pkg, 8);
  Dynamixel::CheckPKG(pkg,10);
  Dynamixel::TransmitPacket(pkg);
  Serial.println(" ");
  Dynamixel::StatusPrint();
}

unsigned char Dynamixel::ReceiveTemperature(){
int message [50];
int i = 0;
int temp = 0;
delay(100);
Serial.println(" ");
Serial.print("Temperature > ");
while (Serial1.available()){ //while dynamixel available, readings are put to message array.
  message[i] = Serial1.read();
  i++;
}
temp = message[9]; //POS 9 - Parameter for temp :)
Serial.println(temp);
return temp;
}

void Dynamixel::StatusPrint(){
  status_packet_info status = Dynamixel::ReceiveStatusPacket();
  Serial.println("Received>");
  Serial.println(" ");
  Serial.println("ID");
  Serial.println(status.id);
  Serial.println("Error");
  Serial.println(status.error);
}
void Dynamixel::SamplingFreq(){
int init_time = 0;
int f_time = 1000/10;
init_time = millis();
Dynamixel::ReceiveTemperature();
while(millis()-init_time < f_time) continue;
Serial.println(millis());
}

void Dynamixel::FullReboot(unsigned char id, unsigned long value){
 setTorqueEnable(0xFE, (bool) true);
 setPositionGain(0xFE, (unsigned int) 100);
 setGoalPosition(id, value);
 delay(2000);
 Reboot(id);
 delay(2000);
 setTorqueEnable(0xFE, (bool) true);
 setPositionGain(0xFE, (unsigned int) 100);
 setGoalPosition(id, value);
}
void Dynamixel::TransmitPacket(unsigned char *pkg) {
    digitalWrite(_flow_control_pin, HIGH); //Writing to Arduino - ON
    unsigned short bytes_in_packet = (pkg[6] << 8) + pkg[5] + 7; // +7 is Header + Length field

    for (int i = 0; i < bytes_in_packet; i++) {
        _serialport->write(pkg[i]);
    }
    _serialport->flush(); //Waiting for all messages to go through.
    digitalWrite(_flow_control_pin, LOW); // Writing to Arduino - OFF
}
void Dynamixel::CheckPKG(unsigned char *pkg, unsigned char l){
  Serial.println("Transmited ping packet ->");
  for (size_t i = 0; i < l; i++) {
    Serial.print(pkg[i], HEX); //Printing packet info. :)
    Serial.print(" ");
  }
}
void Dynamixel::CreateCRC(unsigned char *pkg, unsigned short blk_size) {
    unsigned short cal_crc = update_crc(0, pkg, blk_size);
    pkg[blk_size] = (cal_crc & 0x00FF);
    pkg[blk_size + 1] = (cal_crc >> 8) & 0x00FF;
}
//CRC stuff
unsigned short Dynamixel::update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    unsigned short i, j;
    unsigned short crc_table[256] = {
            0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
            0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
            0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
            0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
            0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
            0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
            0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
            0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
            0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
            0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
            0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
            0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
            0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
            0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
            0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
            0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
            0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
            0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
            0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
            0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
            0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
            0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
            0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
            0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
            0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
            0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
            0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };
    for (j = 0; j < data_blk_size; j++) {
        i = ((unsigned short) (crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
//Defining packets!
void Dynamixel::Header(unsigned char *pkg, unsigned char id){
  pkg[0] = 0xff;
  pkg[1] = 0xff;
  pkg[2] = 0xfd;
  pkg[3] = 0x00;
  pkg[4] = id;
}
void Dynamixel::putInt8t(unsigned char *pkg, unsigned char value, unsigned int pos){
  pkg[pos]= value;
}
void Dynamixel::putInt16t(unsigned char *pkg, unsigned int value, unsigned int pos){
  pkg[pos] = value & 0x00ff;
  pkg[pos+1]= value>>8;
}
void Dynamixel::putInt32t(unsigned char *pkg, unsigned long value, unsigned int pos){
  for(int i=0; i<4; i++){
    pkg[pos+i] = value & 0x000000ff;
    value = value >> 8;
  }
}
void Dynamixel::begin(long baud_rate = 57600) { // The boud rate for Dynamixel motors (they can also use other boud rates, 57600 is standard)
#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
    Serial1.begin(baud_rate);  // Set up Serial for Leonardo and Mega
    _serialport = &Serial1;
#else
    Serial.begin(baud_rate);   // Set up Serial for all others (Uno, etc)
    _serialport = &Serial;
#endif
}
Dynamixel::status_packet_info Dynamixel::ReceiveStatusPacket() { //Status packet - printing ID's and Errors
    unsigned long start_time = micros();
    status_packet_info status;
    status.error = 0x00;

    while (micros()<start_time+5000){
        if (_serialport->available() >= 7) {
            //Serial.println("Scanning for header...");
            // Get rid of the header
            for (int i = 0; i < 2; ++i) {
                if (_serialport->peek() == 0xFF) {
                    _serialport->read();
                } else {
                    status.error = 0x08; // 0x08 is not defined in the protocol. Consider it an unknown error.
                    return status;
                }
            }

            if (_serialport->peek() == 0xFD) {
                _serialport->read();
            } else {
                status.error = 0x08;
                return status;
            }

            if (_serialport->peek() == 0x00) {
                _serialport->read();
            } else {
                status.error = 0x08;
                return status;
            }

            // Looking for ID and for packet length:)
            unsigned char id = _serialport->read();
            unsigned char l1 = _serialport->read();
            unsigned char l2 = _serialport->read();
            unsigned short packet_length = l1 + (l2 << 8);
            status.id = id;

            // Recreate RX-packet
            unsigned char rx_packet[packet_length+7];
            rx_packet[0] = 0xFF;
            rx_packet[1] = 0xFF;
            rx_packet[2] = 0xFD;
            rx_packet[3] = 0x00;
            rx_packet[4] = id;
            rx_packet[5] = l1;
            rx_packet[6] = l2;

            while (_serialport->available() < packet_length) {
                // Serial.println(_serialport->available());
            }

            // Populate the rest of the packet with instr, err, params, crc
            for (int j = 0; j < packet_length; ++j) {
                rx_packet[j + 7] = _serialport->read();
            }

            // Set error in return value
            status.error = rx_packet[8];
            if (status.error != 0x00) return status;

            // Extract parameters
            for (int k = 0; k < packet_length - 4; ++k) {
                status.parameters[k] = rx_packet[9 + k];
            }

            unsigned short calc_crc = update_crc(0, rx_packet, packet_length+5);
            if ((rx_packet[packet_length+5] != (calc_crc & 0xFF)) | (rx_packet[packet_length+6] != ((calc_crc >> 8) & 0xFF))){
                status.error = 0x03; // CRC error
            }
            return status;
        }
    }
    status.error = 0x09; // TIMEOUT ERROR
    return status;
}
