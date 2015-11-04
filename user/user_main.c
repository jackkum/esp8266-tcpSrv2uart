#define PLATFORM_DEBUG	true

#include "ets_sys.h"
#include "osapi.h"
#include "os_type.h"
#include "user_interface.h"
#include "driver/uart.h"
#include "espconn.h"
#include "mem.h"
#include "gpio.h"
#include "user_config.h"

#define TASK_QUEUE_LEN 4

typedef enum {
	WIFI_CONNECTING,
	WIFI_CONNECTING_ERROR,
	WIFI_CONNECTED,
	TCP_DISCONNECTED,
	TCP_CONNECTING,
	TCP_CONNECTING_ERROR,
	TCP_CONNECTED,
	TCP_SENDING_DATA_ERROR,
	TCP_SENT_DATA
} tConnState;

static void recvTask(os_event_t *events);
static void ICACHE_FLASH_ATTR config_recv_cb(void *arg, char *data, unsigned short len);
static void ICACHE_FLASH_ATTR config_recon_cb(void *arg, sint8 err);
static void ICACHE_FLASH_ATTR config_discon_cb(void *arg);
static void ICACHE_FLASH_ATTR config_sent_cb(void *arg);
static void ICACHE_FLASH_ATTR config_connected_cb(void *arg);

static char macaddr[6];
static ETSTimer WiFiLinker;
static tConnState connState = WIFI_CONNECTING;

static struct espconn config_conn;
static esp_tcp config_tcp_conn;

struct espconn *client;

uint8_t controlServerStatus = 0;

void delay_ms(uint16_t ms)
{
	while(ms--){
		os_delay_us(1000);
	}
}

void ICACHE_FLASH_ATTR config_conn_init()
{
	//uart0_sendStr("TCP server accepting on 8888\r\n");

	config_conn.type           = ESPCONN_TCP;
	config_conn.state          = ESPCONN_NONE;
	config_tcp_conn.local_port = 8888;
	config_conn.proto.tcp      = &config_tcp_conn;

	espconn_regist_connectcb(&config_conn, config_connected_cb);
	espconn_accept(&config_conn);

	os_event_t *recvTaskQueue = (os_event_t *)os_malloc(sizeof(os_event_t) * TASK_QUEUE_LEN);
	system_os_task(recvTask, USER_TASK_PRIO_0, recvTaskQueue, TASK_QUEUE_LEN);
}

static void ICACHE_FLASH_ATTR config_recv_cb(void *arg, char *data, unsigned short len)
{
	uart0_tx_buffer(data, len);
}

static void ICACHE_FLASH_ATTR config_recon_cb(void *arg, sint8 err)
{
}

static void ICACHE_FLASH_ATTR config_discon_cb(void *arg)
{
	//uart0_sendStr("TCP client closed...\r\n");
	if (client != NULL){
		if (client->state == ESPCONN_NONE || client->state >= ESPCONN_CLOSE){
			client = NULL;
		}
	}
}

static void ICACHE_FLASH_ATTR config_sent_cb(void *arg)
{

}

static void ICACHE_FLASH_ATTR config_connected_cb(void *arg)
{
	//uart0_sendStr("TCP client opened...\r\n");

	if (client != NULL){
		espconn_disconnect((struct espconn *)arg);
		return;
	}

	client = (struct espconn *)arg;

	espconn_regist_recvcb  (client, config_recv_cb);
	espconn_regist_reconcb (client, config_recon_cb);
	espconn_regist_disconcb(client, config_discon_cb);
	espconn_regist_sentcb  (client, config_sent_cb);

	//char *transmission = "OK\r\n\r\nOK!\n";
	//sint8 d = espconn_sent(conn,transmission,strlen(transmission));
}

static void ICACHE_FLASH_ATTR recvTask(os_event_t *events)
{
	char sig_rx[1];
	switch (events->sig) {
		case 0:
			if(client != NULL){
				sig_rx[0] = (char)events->par;
				espconn_sent(client, sig_rx, 1);
			}
			break;
		default:
			break;
	}
}

static void ICACHE_FLASH_ATTR wifi_check_ip(void *arg)
{
	struct ip_info ipConfig;

	os_timer_disarm(&WiFiLinker);

	wifi_get_ip_info(STATION_IF, &ipConfig);

	if (wifi_station_get_connect_status() == STATION_GOT_IP && ipConfig.ip.addr != 0){
		//uart0_sendStr("WiFi connected\r\n");

		os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
		os_timer_arm(&WiFiLinker, 5000, 0);

		if(controlServerStatus == 0) {
			controlServerStatus++;
			config_conn_init();
		}
    } else {

		if(wifi_station_get_connect_status() == STATION_WRONG_PASSWORD)
		{
			//uart0_sendStr("WiFi connecting error, wrong password\r\n");
		}
		else if(wifi_station_get_connect_status() == STATION_NO_AP_FOUND)
		{
			//uart0_sendStr("WiFi connecting error, ap not found\r\n");
		}
		else if(wifi_station_get_connect_status() == STATION_CONNECT_FAIL)
		{
			//uart0_sendStr("WiFi connecting fail\r\n");
		}
		else
		{
			//uart0_sendStr("WiFi connecting...\r\n");
		}

		os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
		os_timer_arm(&WiFiLinker, 1000, 0);

		controlServerStatus = 0;
    }
}

//Init function
void ICACHE_FLASH_ATTR user_init()
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);

	delay_ms(100);

	//uart0_sendStr("\r\nESP8266 platform starting...\r\n");

	struct station_config stationConfig;

	if(wifi_get_opmode() != STATION_MODE){
		wifi_set_opmode(STATION_MODE);
	}

	if(wifi_get_opmode() == STATION_MODE){
		wifi_station_get_config(&stationConfig);
		os_memset(stationConfig.ssid, 0, sizeof(stationConfig.ssid));
		os_memset(stationConfig.password, 0, sizeof(stationConfig.password));
		os_sprintf(stationConfig.ssid, "%s", SSID);
		os_sprintf(stationConfig.password, "%s", SSID_PASSWORD);
		wifi_station_set_config(&stationConfig);
		wifi_get_macaddr(SOFTAP_IF, macaddr);
		//uart0_sendStr(macaddr);
		//uart0_sendStr("\r\n");
	}

	os_timer_disarm(&WiFiLinker);
	os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
	os_timer_arm(&WiFiLinker, 1000, 0);

}
