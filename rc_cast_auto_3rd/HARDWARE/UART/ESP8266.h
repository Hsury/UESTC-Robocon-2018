#ifndef __ESP8266_H
#define __ESP8266_H

#include "Includes.h"

enum ESP8266_WIFI_MODE
{
    STATION = 1, 
    SOFTAP = 2, 
    SOFTAP_STATION = 3
};

enum ESP8266_ENCRYPTION
{
    OPEN = 0, 
    WPA_PSK = 2, 
    WPA2_PSK = 3, 
    WPA_WPA2_PSK = 4
};

void ESP8266_Init(uint32_t Baudrate);
void ESP8266_ExitTransLink(void);
bool ESP8266_UARTConfig(uint32_t Baudrate);
bool ESP8266_WiFiModeConfig(uint8_t Mode);
bool ESP8266_StationConfig(char *SSID, char *Password);
bool ESP8266_StationDHCPConfig(bool isEnable);
bool ESP8266_StationIPConfig(char *IPAddress, char *Gateway, char *Netmask);
bool ESP8266_SoftAPConfig(char *SSID, char *Password, uint8_t Channel, uint8_t Encryption, uint8_t MaxConn, bool HideSSID);
bool ESP8266_UDPTransLinkConfig(char *IPAddress, uint16_t RemotePort, uint16_t LocalPort);
bool ESP8266_TCPTransLinkConfig(char *IPAddress, uint16_t RemotePort);
bool ESP8266_Reset(void);

#endif
