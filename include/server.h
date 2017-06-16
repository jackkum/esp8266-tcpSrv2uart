#ifndef __SERVER_H_
#define __SERVER_H_

#include "c_types.h"
#include "user_interface.h"
#include "espconn.h"

#define os_realloc pvPortRealloc

typedef struct espconn Socket;
typedef void (*OnRecvCallback)(char *, uint16);

typedef struct {
  uint16 port;
  Socket socket;
  esp_tcp tcp;
  Socket *client;
  OnRecvCallback onRecvCallback;
} SERVER;

SERVER *createServer(uint16 port, OnRecvCallback onRecvCallback);
SERVER *findServerByClient(Socket *client);
void writeClient(SERVER * self, char * buffer, uint8 len);

#endif /*__SERVER_H_*/