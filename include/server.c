
#include "server.h"

void ICACHE_FLASH_ATTR onRecv(void *arg, char *data, uint16 len);
void ICACHE_FLASH_ATTR onReconnect(void *arg, sint8 err);
void ICACHE_FLASH_ATTR onDisconnect(void *arg);
void ICACHE_FLASH_ATTR onSent(void *arg);
void ICACHE_FLASH_ATTR onConnected(void *arg);

/**
 * Pointers to servers
 */
SERVER **servers;

/**
 * Counter of servers
 */
volatile uint16 serverCount = 0;

/**
 * @brief      Creates a server.
 *
 * @param[in]  port            The port
 * @param[in]  onRecvCallback  On receive callback
 *
 * @return     Pointer to the server
 */
SERVER *createServer(uint16 port, OnRecvCallback onRecvCallback)
{
  // increment count of servers
  serverCount++;

  // add memory
  servers = (SERVER **)os_realloc(servers, serverCount * sizeof(SERVER));

  // get created server
  SERVER *srv = (SERVER*) &servers[serverCount-1];

  // fill params of the server
  srv->socket.type       = ESPCONN_TCP;
  srv->socket.state      = ESPCONN_NONE;
  srv->tcp.local_port    = port;
  srv->socket.proto.tcp  = &srv->tcp;
  srv->port              = port;
  srv->onRecvCallback    = onRecvCallback;

  // register connection event
  espconn_regist_connectcb(&srv->socket, onConnected);
  // accept connection
  espconn_accept(&srv->socket);
  // register timeout
  espconn_regist_time(&srv->socket, 180, 0);

  // return a pointer to the server
  return srv;
}

/**
 * @brief      Find server by client socket
 *
 * @param      client  The socket client
 *
 * @return     Pointer to the server
 */
SERVER *findServerByClient(Socket *client)
{
  uint16 i = 0;
  for(i = 0; i < serverCount; i++){
    SERVER *srv = (SERVER *)&servers[i];

    if(srv->port == client->proto.tcp->remote_port){
      return srv;
    }
  }

  return NULL;
}

/**
 * @brief      Write to the socket
 *
 * @param      self    The object
 * @param      buffer  The buffer
 * @param[in]  len     The length
 */
void writeClient(SERVER * self, char * buffer, uint8 len)
{
  if(self->client == NULL){
    return;
  }

  espconn_sent(self->client, buffer, len);
}

/**
 * @brief      Callback for read from client socket
 *
 * @param      arg   The client
 * @param      data  The data
 * @param[in]  len   The length
 */
void ICACHE_FLASH_ATTR onRecv(void *arg, char *data, uint16 len)
{
  Socket *client = (Socket *)arg;

  SERVER *srv = findServerByClient(client);

  if(srv == NULL){
    uart0_sendStr("Server not found by port number\n");
    return;
  }

  if(srv->onRecvCallback == NULL){
    return;
  }

  srv->onRecvCallback(data, len);
}

/**
 * @brief      On disconnect callback
 *
 * @param      arg   The argument
 */
void ICACHE_FLASH_ATTR onDisconnect(void *arg)
{
  uart0_sendStr("TCP client closed\n");

  Socket *client = (Socket *)arg;

  SERVER *srv = findServerByClient(client);

  if(srv == NULL){
    uart0_sendStr("Server not found by port number\n");
    return;
  }

  if (srv->client != NULL){
    if (srv->client->state == ESPCONN_NONE || srv->client->state >= ESPCONN_CLOSE){
      srv->client = NULL;
    }
  }
}

/**
 * @brief      On connect callback
 *
 * @param      arg   The argument
 */
void ICACHE_FLASH_ATTR onConnected(void *arg)
{
  uart0_sendStr("Register callbacks\n");
  Socket *client = (Socket *)arg;

  SERVER *srv = findServerByClient(client);

  if(srv == 0){
    uart0_sendStr("Server not found by port number\n");
    return;
  }

  srv->client = client;

  espconn_regist_recvcb(client,   onRecv);
  espconn_regist_reconcb(client,  onReconnect);
  espconn_regist_disconcb(client, onDisconnect);
  espconn_regist_sentcb(client,   onSent);
}



void ICACHE_FLASH_ATTR onSent(void *arg)
{
}

void ICACHE_FLASH_ATTR onReconnect(void *arg, sint8 err)
{
}