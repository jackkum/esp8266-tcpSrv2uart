#define PLATFORM_DEBUG  true

#include "user_config.h"
#include "ets_sys.h"
#include "osapi.h"
#include "os_type.h"
#include "user_interface.h"
#include "driver/uart.h"
#include "pack.h"
#include "server.h"
#include "wifi.h"
#include "espconn.h"
#include "mem.h"
#include "gpio.h"

#define TASK_QUEUE_LEN 4
#define RS484_DIR BIT5
#define LED_RED BIT14
#define LED_GREEN BIT16

static void recvTask(os_event_t *events);
/**
 * @brief      Callbaks for receive
 *
 * @param      data  The data
 * @param[in]  len   The length
 */
static void ICACHE_FLASH_ATTR onProxyData(char * data, uint16 len);
static void ICACHE_FLASH_ATTR onCommandData(char * data, uint16 len);

// UartDev is defined and initialized in rom code.
extern UartDevice UartDev;

/**
 * Servers
 */
SERVER *bridge, *config;

void delay_ms(uint16 ms)
{
  while(ms--){
    os_delay_us(1000);
  }
}

/**
 * Entrance point
 */
void ICACHE_FLASH_ATTR user_init()
{
  // init gpio
  gpio_init();

  // default initialize uart
  uart_init(BIT_RATE_9600, BIT_RATE_9600);

  // config direction of IO
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
  //PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO16_U, FUNC_GPIO16);

  // pull up to the ground 
  gpio_output_set(0, RS484_DIR, RS484_DIR, 0);
  gpio_output_set(LED_RED, 0, LED_RED, 0);
  //gpio_output_set(LED_GREEN, 0, 0, LED_GREEN);

  // delay
  delay_ms(100);
 
  // init wifi
  WiFiInit();

  __debug("\r\nStarted ESP Bridge", RS484_DIR);

  // os task for recv bytes from uart
  os_event_t *recvTaskQueue = (os_event_t *) os_malloc(sizeof(os_event_t) * TASK_QUEUE_LEN);
  system_os_task(recvTask, USER_TASK_PRIO_0, recvTaskQueue, TASK_QUEUE_LEN);

  // let's get it started
  __debug("Create server: 8888", RS484_DIR);
  bridge = createServer(8888, &onCommandData);

  // server for configurations
  //__debug("Create server: 9999", RS484_DIR);
  //config = createServer(9999, &onCommandData);

  __debug("Initialization complete", RS484_DIR);
}

/**
 * @brief      Callback task
 *
 * @param      events  The events
 */
static void ICACHE_FLASH_ATTR recvTask(os_event_t *events)
{
  char sig_rx[1];
  switch (events->sig) {
    case 0:
      sig_rx[0] = (char) events->par;
      writeClient(bridge, sig_rx, 1);
      break;

    default:
      break;
  }
}

/**
 * @brief      Callback for the transparent server
 *
 * @param      data  The data
 * @param[in]  len   The length
 */
static void ICACHE_FLASH_ATTR onProxyData(char * data, uint16 len) {

  // higth level
  gpio_output_set(RS484_DIR, 0, RS484_DIR, 0);
  gpio_output_set(0, LED_RED, LED_RED, 0);
  // wait
  os_delay_us(1);
  // send buffer
  uart0_tx_buffer(data, len);
  // wait
  os_delay_us(2000);
  // low level
  gpio_output_set(0, RS484_DIR, RS484_DIR, 0);
  gpio_output_set(LED_RED, 0, LED_RED, 0);
}

/**
 * @brief      Callback for the configuration server
 *
 * @param      data  The data
 * @param[in]  len   The length
 */
static void ICACHE_FLASH_ATTR onCommandData(char * data, uint16 len) {

  // cast to packet
  Packet *pack = getPacket(data);
  // init data from packet
  Data *d = (Data *) &pack->data;
  // init answer
  Packet answer;
  // init data of answer
  Data aData;

  // fill headers
  answer.header.address = DEVICE_ADDR;
  answer.header.command = pack->header.command;
  answer.header.length  = 7 + sizeof(Data);

  // request is for me
  if(checkPacket(pack, DEVICE_ADDR) == PACKET_ERROR_NO_ERROR){

    // validate crc of packet
    uint8 error = checkCrc(pack);

    // put error to the answer
    aData.error = error;

    // no errors
    if(error == PACKET_ERROR_NO_ERROR){
      // check command
      switch(pack->header.command){

        // command to set params of uart
        case PACK_COMMAND_SET_DATA:
          UartDev.baut_rate = (UartBautRate) d->baudrate;
          UartDev.data_bits = (UartBitsNum4Char) d->bits;
          UartDev.parity    = (UartParityMode) d->parity;
          UartDev.stop_bits = (UartStopBitsNum) d->stop;

          uart_config(UART0);
          uart_config(UART1);

        break;
      }

      // set current params to the answer
      aData.baudrate = (uint32) UartDev.baut_rate;
      aData.bits     = (uint8) UartDev.data_bits;
      aData.parity   = (uint8) UartDev.parity;
      aData.stop     = (uint8) UartDev.stop_bits;
    }

    // copy params to the answer
    os_memcpy(&answer.data, (uint8 *)&aData, sizeof(Data));
    
    // recalculate crc of packet
    setCrc(&answer);

    // answer to the client
    writeClient(bridge, (uint8 *)&answer, answer.header.length);
  } else {
    onProxyData(data, len);
  }
}