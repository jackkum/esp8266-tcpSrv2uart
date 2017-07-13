#ifndef __PACK_H_
#define __PACK_H_

#include "c_types.h"

#define PACKET_ERROR_NO_ERROR         0x00
#define PACKET_ERROR_CRC_ERROR        0x01
#define PACKET_ERROR_UNKNOW_FUNCTION  0x02
#define PACKET_ERROR_CH_ERROR         0x03
#define PACKET_ERROR_ADDR             0xFF

#define PACK_COMMAND_GET_DATA         0x01
#define PACK_COMMAND_SET_DATA         0x02

#define BACKTIMER_VALUE         10      //Timeout in secs
#define INITIAL_CRC_VALUE       0xFFFF  //initial value for CRC calculations

#define MAX_DATA_LENGTH               130

#pragma pack(push)
#pragma pack(1)
typedef struct {
  uint32  address:24;
  uint8   command;
  uint8   length;
} Header;

typedef struct {
  Header   header;
  uint8  data[MAX_DATA_LENGTH]; 
} Packet;

typedef struct {
  uint8 error;
  uint32 baudrate:24;
  uint8 bits;
  uint8 parity;
  uint8 stop;
  //uint16 adc;
} Data;
#pragma pack(pop)

#define PACKET_HEADER_SIZE   sizeof(Header)
#define PACKET_SIZE          sizeof(Packet)

Packet *getPacket(char *data);
uint8 checkPacket(Packet *pack, uint32 address);
uint8 checkCrc(Packet *pack);
void setCrc(Packet *pack);
uint16 getCrc(Packet *pack);

uint16 crc16_update(uint16 crc, uint8 a);


#endif /*__PACK_H_*/