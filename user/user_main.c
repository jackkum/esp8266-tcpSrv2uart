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
#define LED_RED BIT12
#define LED_GREEN BIT13

static void recvTask(os_event_t *events);
static void ICACHE_FLASH_ATTR onProxyDataToSocket();

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

volatile char rx_buffer[255];
volatile uint8 rx_pos = 0;

static os_timer_t rx_timer, led_timer;

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

void ledGreen()
{
  gpio_output_set(LED_RED, 0,   LED_RED,   0);
  gpio_output_set(0, LED_GREEN, LED_GREEN, 0);
}

void ledRed()
{
  gpio_output_set(0, LED_RED,   LED_RED,   0);
  gpio_output_set(LED_GREEN, 0, LED_GREEN, 0);
}

void ledOff()
{
  gpio_output_set(0, LED_RED,   LED_RED,   0);
  gpio_output_set(0, LED_GREEN, LED_GREEN, 0);
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

  /*
  // feature implementation of saving params
  uint8 data[sizeof(Data)];
  system_rtc_mem_read(64, &data, sizeof(Data)) ;
  Data *d = (Data *) &data;
  UartDev.baut_rate = (UartBautRate) d->baudrate;
  UartDev.data_bits = (UartBitsNum4Char) d->bits;
  UartDev.parity    = (UartParityMode) d->parity;
  UartDev.stop_bits = (UartStopBitsNum) d->stop;

  uart_config(UART0);
  uart_config(UART1);
  */

  // config direction of IO
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U,  FUNC_GPIO12);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U,  FUNC_GPIO13);
  

  // pull up to the ground 
  gpio_output_set(0, RS484_DIR, RS484_DIR, 0);
  
  // turn off the lends
  ledOff();

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
  switch (events->sig) {
    case 0:
      
      // disable timer
      os_timer_disarm(&rx_timer);

      // led
      ledRed();

      // get next byte
      rx_buffer[rx_pos++] = (uint8) events->par;
      
      if(rx_pos >= 255){
        // buffet overflow, just transfer buffer
        onProxyDataToSocket();
      } else {
        // set timer 200ms for wait whole packet
        os_timer_setfn(&rx_timer, (os_timer_func_t *)onProxyDataToSocket, NULL);
        os_timer_arm(&rx_timer, 200, 0);
      }
      
      break;

    default:
      break;
  }
}

static void ICACHE_FLASH_ATTR onProxyDataToSocket() {
  os_timer_disarm(&rx_timer);
  writeClient(bridge, (char *)&rx_buffer[0], rx_pos);
  rx_pos = 0;

  ledOff();
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

  // wait
  os_delay_us(1);
  // send buffer
  uart0_tx_buffer(data, len);
  // wait
  os_delay_us(2000);
  // low level
  gpio_output_set(0, RS484_DIR, RS484_DIR, 0);
}

static void ICACHE_FLASH_ATTR onLedOfTimer() {
  ledOff();
}

/**
 * @brief      Callback for the configuration server
 *
 * @param      data  The data
 * @param[in]  len   The length
 */
static void ICACHE_FLASH_ATTR onCommandData(char * data, uint16 len) {

  ledGreen();
  os_timer_disarm(&led_timer);
  os_timer_setfn(&led_timer, (os_timer_func_t *)onLedOfTimer, NULL);
  os_timer_arm(&led_timer, 300, 0);

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

          switch(d->bits){
            case 5: UartDev.data_bits = FIVE_BITS;  break;
            case 6: UartDev.data_bits = SIX_BITS;   break;
            case 7: UartDev.data_bits = SEVEN_BITS; break;
            case 8: UartDev.data_bits = EIGHT_BITS; break;
          }

          switch(d->parity){
            case 'N': UartDev.parity = NONE_BITS;  break;
            case 'O': UartDev.parity = ODD_BITS;   break;
            case 'E': UartDev.parity = EVEN_BITS;  break;
          }

          switch(d->stop){
            case 1: UartDev.stop_bits = ONE_STOP_BIT;  break;
            case 2: UartDev.stop_bits = TWO_STOP_BIT;  break;
          }

          uart_config(UART0);
          uart_config(UART1);

          // write params
          //system_rtc_mem_write(64, &pack->data, sizeof(Data));

        break;

        default:
          aData.error = 0xFF; // unknown command
      }

      // set current params to the answer
      aData.baudrate = (uint32) UartDev.baut_rate;
      switch(UartDev.data_bits){
        case FIVE_BITS:  d->bits = 5; break;
        case SIX_BITS:   d->bits = 6; break;
        case SEVEN_BITS: d->bits = 7; break;
        case EIGHT_BITS: d->bits = 8; break;
        default: d->bits = UartDev.data_bits;
      }

      switch(UartDev.parity){
        case NONE_BITS: d->parity = 'N'; break;
        //case ODD_BITS:  d->parity = 'O'; break;
        case EVEN_BITS: d->parity = 'E'; break;
        default: d->parity = 'N';
      }

      switch(UartDev.stop_bits){
        case ONE_STOP_BIT: d->stop = 1; break;
        case TWO_STOP_BIT: d->stop = 2; break;
        default: d->stop = 0;
      }
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