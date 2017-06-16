
#include "pack.h"

Packet * getPacket(char *data)
{
    return &((Packet *) data)[0]; // pointer to the start of packet
}

uint8 checkPacket(Packet *pack, uint32 address)
{
  if(pack->header.address != address){
    return PACKET_ERROR_ADDR;
  }

  return PACKET_ERROR_NO_ERROR;
}

uint8 checkCrc(Packet *pack) {
  uint8 b1, b2;
  b1 = (uint8) *(((uint8 *) pack) + pack->header.length - 1);
  b2 = (uint8) *(((uint8 *) pack) + pack->header.length - 2);
  
  uint16 crc = (uint16)((b1<<8) | b2);

  return crc == getCrc(pack) ? PACKET_ERROR_NO_ERROR : PACKET_ERROR_CRC_ERROR;
}

void setCrc(Packet *pack)
{
  uint16 crc = getCrc(pack);
  uint8 len  = pack->header.length;

  *(((uint8*) pack)+len-2) = (uint8) (crc & 0x00FF);
  *(((uint8*) pack)+len-1) = (uint8) ((crc & 0xFF00)>>8);  
}

uint16 getCrc(Packet *pack) {
  uint8 cnt  = 0;
  uint16 crc = INITIAL_CRC_VALUE;

  while (cnt < (pack->header.length-2)) {
    crc = crc16_update(crc, *(((uint8*) pack)+cnt));
    cnt++;
  };

  return crc;
}

uint16 crc16_update(uint16 crc, uint8 a) {
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
      if (crc & 1)
          crc = (crc >> 1) ^ 0xA001;
      else
          crc = (crc >> 1);
  }

  return crc;
}
