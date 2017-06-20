#ifndef __SERVER_H_
#define __SERVER_H_

#include "c_types.h"
#include "user_interface.h"
#include "espconn.h"
#include "driver/uart.h"

#define os_realloc pvPortRealloc
#define RS484_DIR BIT5

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

static void ICACHE_FLASH_ATTR onRecv(void *arg, char *data, uint16 len);
static void ICACHE_FLASH_ATTR onReconnect(void *arg, sint8 err);
static void ICACHE_FLASH_ATTR onDisconnect(void *arg);
static void ICACHE_FLASH_ATTR onSent(void *arg);
static void ICACHE_FLASH_ATTR onConnected(void *arg);

#endif /*__SERVER_H_*/