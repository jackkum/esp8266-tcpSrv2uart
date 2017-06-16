
#include "wifi.h"

void WiFiInit(void)
{
  wifi_set_opmode(SSID_MODE);

  WiFiConf config;
 
  wifi_softap_get_config(&config); // Get config first.
  
  // clear
  os_memset(config.ssid, 0, 32);
  os_memset(config.password, 0, 64);

  // set
  os_memcpy(config.ssid,     SSID,          sizeof(SSID));
  os_memcpy(config.password, SSID_PASSWORD, sizeof(SSID_PASSWORD));

  config.authmode       = SSID_AUTH_MODE;
  config.ssid_len       = 0;               // or its actual length
  config.max_connection = SSID_MAX_CON;    // how many stations can connect to ESP8266 softAP at most.

  // Set ESP8266 softap config .
  wifi_softap_set_config(&config); 
}