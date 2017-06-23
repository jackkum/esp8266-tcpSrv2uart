
#include "server.h"

/**
 * Pointers to servers
 */
static SERVER servers[2];

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
  //servers = (SERVER **)os_realloc(servers, serverCount * sizeof(SERVER));

  // get created server
  SERVER *srv = (SERVER*) &servers[serverCount-1];

  // fill params of the server
  srv->socket.type       = ESPCONN_TCP;
  srv->socket.state      = ESPCONN_NONE;
  srv->tcp.local_port    = port;
  srv->socket.proto.tcp  = &(srv->tcp);
  srv->port              = port;
  srv->onRecvCallback    = onRecvCallback;

  // register connection event
  espconn_regist_connectcb(&(srv->socket), onConnected);

  // accept connection
  espconn_accept(&(srv->socket));

  // register timeout
  //espconn_regist_time(&srv->socket, 180, 0);

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

    if(srv->port == client->proto.tcp->local_port){
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
    //__debug("No more client socket", RS484_DIR);
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
static void ICACHE_FLASH_ATTR onRecv(void *arg, char *data, uint16 len)
{

  //__debug("Client recv", RS484_DIR);

  Socket *client = (Socket *)arg;

  SERVER *srv = findServerByClient(client);

  if(srv == NULL){
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
static void ICACHE_FLASH_ATTR onDisconnect(void *arg)
{
  //__debug("Client disconnected", RS484_DIR);

  Socket *client = (Socket *)arg;

  SERVER *srv = findServerByClient(client);

  if(srv == NULL){
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
static void ICACHE_FLASH_ATTR onConnected(void *arg)
{

  Socket *client = (Socket *)arg;

  SERVER *srv = findServerByClient(client);

  if(srv == NULL){
    //__debug("No server found", RS484_DIR);
    return;
  }

  //__debug("Client connected", RS484_DIR);
  srv->client = client;

  espconn_regist_recvcb(client,   onRecv);
  espconn_regist_reconcb(client,  onReconnect);
  espconn_regist_disconcb(client, onDisconnect);
  espconn_regist_sentcb(client,   onSent);
}



static void ICACHE_FLASH_ATTR onSent(void *arg)
{
}

static void ICACHE_FLASH_ATTR onReconnect(void *arg, sint8 err)
{
}