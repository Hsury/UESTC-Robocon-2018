/*

  /\\\        /\\\  /\\\\\\\\\\\\\\\     /\\\\\\\\\\\    /\\\\\\\\\\\\\\\        /\\\\\\\\\         
  \/\\\       \/\\\ \/\\\///////////    /\\\/////////\\\ \///////\\\/////      /\\\////////         
   \/\\\       \/\\\ \/\\\              \//\\\      \///        \/\\\         /\\\/                 
    \/\\\       \/\\\ \/\\\\\\\\\\\       \////\\\               \/\\\        /\\\                  
     \/\\\       \/\\\ \/\\\///////           \////\\\            \/\\\       \/\\\                 
      \/\\\       \/\\\ \/\\\                     \////\\\         \/\\\       \//\\\               
       \//\\\      /\\\  \/\\\              /\\\      \//\\\        \/\\\        \///\\\            
         \///\\\\\\\\\/   \/\\\\\\\\\\\\\\\ \///\\\\\\\\\\\/         \/\\\          \////\\\\\\\\\  
            \/////////     \///////////////    \///////////           \///              \/////////  

                        ===== UESTC Robot Probe For ABU Robocon 2018 =====
                              Copyright (c) 2018 HsuRY <i@hsury.com>

                                        VERSION 2018/06/01

*/

// Use esptool.py --port /dev/ttyUSB0 erase_flash to clear SPIFFS

/*
TODO List:
1. Complete variable logging system
2. Complete selftest system
3. Add Competition/Debug dual mode
*/

#include <Arduino.h>
#include <FreeRTOS/timers.h>
#include <WiFi.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <RemoteDebug.h>
#include <BY8X01-16P.h>
#include <Wire.h>
#include <DS3232RTC.h>
#include <TimeLib.h>
#include <NtpClientLib.h>
#include <ESP8266FtpServer.h>
#include "bitmap.h"

#define HW_NAME "AR"
#define SW_NAME "2018/06/01"

#define USE_STATIC_IP_ADDRESS 0
#define ENABLE_TOUCH_CALIBRATE 0
#define ENABLE_CAN_DEBUG 0
#define UPDATE_RTC_TIME 0
#define NOTIFY_DEVICE_NUM 4

const char* AP_SSID      = "AR-Probe";
const char* AP_PASSWORD  = "duoguanyuenan8";

const char* STA_SSID     = "UESTC-Robocon-WiFi";
const char* STA_PASSWORD = "duoguanyuenan8";

#if USE_STATIC_IP_ADDRESS
IPAddress LOCAL_IP(192, 168, 1, 45);
IPAddress GATEWAY(192, 168, 1, 1);
IPAddress SUBNET(255, 255, 255, 0);
#endif

const char * UDP_ADDRESS = "192.168.1.255";
const int UDP_PORT = 2019;

struct ProbeConfig
{
    uint8_t volume = 5;
    bool hideSSID = false;
    bool rawUART = false;
} config;

#define CAN_RX_PIN GPIO_NUM_4
#define CAN_TX_PIN GPIO_NUM_5
#define HSPI_SCLK_PIN GPIO_NUM_14
#define UART2_TX_PIN GPIO_NUM_16
#define UART2_RX_PIN GPIO_NUM_17
#define I2C_SDA_PIN GPIO_NUM_22
#define I2C_SCL_PIN GPIO_NUM_23
#define SD_CARD_CS_PIN GPIO_NUM_25
#define HSPI_MOSI_PIN GPIO_NUM_26
#define HSPI_MISO_PIN GPIO_NUM_27
#define UART1_RX_PIN GPIO_NUM_32
#define UART1_TX_PIN GPIO_NUM_33
#define AUDIO_BUSY_PIN GPIO_NUM_34

/*
TFT Relevant Pin List

TFT_MISO  21
TFT_MOSI  19
TFT_SCLK  18
TFT_CS    13 // Chip select control pin
TFT_DC    15 // Data Command control pin
TFT_RST   2 // Reset pin (could connect to RST pin)
TOUCH_CS  12 // Chip select pin (T_CS) of touch screen

P.S. Check them in lib/TFT_eSPI/User_Setup.h
P.P.S. TFT and Touchscreen are using VSPI, transactions (To work with other devices on the bus) are automatically enabled by TFT_eSPI for an ESP32 (to use HAL mutex)
*/

#define Gyroscope_ID 0x11
#define Encoder_ID 0x12
#define DT35_X_ID 0x26
#define DT35_Y_ID 0x24
#define Keyboard_ID 0x30
#define MR_ID 0x71
#define Cradle_ID 0x72
#define Hint_Tone_ID 0x74
#define Online_ID 0xA0
#define Custom_ID 0xA1
#define Timer_ID 0xA2

/*
CAN Device Table

Index     ID        Device       Variable       Description
----------------------------------------------------------------------
  0      0x10       Base         Base           Base Controller
  1      0x20       Cradle       Cradle         Cradle Controller
  2      0x30       Gyro         Gyro           Gyro
  3      0x31       Encoder      Encoder        Encoder
  4      0x40       Elmo_BLF     Elmo[0]        Base Left Front Elmo
  5      0x41       Elmo_BLR     Elmo[1]        Base Left Rear Elmo
  6      0x42       Elmo_BRR     Elmo[2]        Base Right Rear Elmo
  7      0x43       Elmo_BRF     Elmo[3]        Base Right Front Elmo
  8      0x50       DT35_H       DT35[0]        Head
  9      0x51       DT35_F       DT35[1]        Left Front
  10     0x52       DT35_R       DT35[2]        Left Rear
*/

CAN_device_t CAN_cfg;
CAN_frame_t CAN_rx_frame;
CAN_frame_t CAN_tx_frame;
uint32_t packNum;

WiFiUDP udp;

SPIClass SPI2(HSPI); // In order to make full use of the chip and not to meet FreeRTOS task conflict with TFT, enable HSPI (HSPI = SPI2; VSPI = SPI3, Default)
HardwareSerial Serial1(1);  // UART1/Serial1 pins 9, 10
HardwareSerial Serial2(2);  // UART2/Serial2 pins 16, 17

DS3232RTC RTC(false); // I2C need to be instanced manually
BY8X0116P audioController(Serial2, AUDIO_BUSY_PIN);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite heartbeatSprite = TFT_eSprite(&tft);
RemoteDebug Debug;
FtpServer ftpSrv;

File externalLogFile;
File CANLogFile;
File UARTLogFile;
File UARTTextFile;

TaskHandle_t WiFiStationTaskHandle;
TaskHandle_t AudioTaskHandle;
TaskHandle_t TFTTaskHandle;
TaskHandle_t CANRecvTaskHandle;
TaskHandle_t UARTRecvTaskHandle;
TaskHandle_t TestTaskHandle;
TaskHandle_t RobotSelftestTaskHandle;
TaskHandle_t HeartbeatAnimeTaskHandle;

TimerHandle_t PingTimerHandle;
TimerHandle_t RSSITimerHandle;

SemaphoreHandle_t RTCUpdateSemaphoreHandle;
SemaphoreHandle_t HeartbeatAnimeUpdateSemaphoreHandle;
SemaphoreHandle_t RSSIUpdateSemaphoreHandle;
SemaphoreHandle_t OnlineUpdateSemaphoreHandle;

xQueueHandle AudioFIFO;
xQueueHandle SerialFIFO;

char BootTimePrefix[21]; // xxxx_xx_xx_xx_xx_xx_\0

// To learn more about Arduino String Class, see also https://hackingmajenkoblog.wordpress.com/2016/02/04/the-evils-of-arduino-strings/
String LogHistory[13 * 20]; // Record for 20 pages at most
uint16_t LogPtr = 0;

uint32_t DeviceNotify[NOTIFY_DEVICE_NUM];
uint32_t Online;
float Custom;
uint32_t Timer[8];
float Gyroscope;
float Encoder[2]; // X-Y coordinate
float DT35[2]; // X-Y coordinate

uint8_t TxRemote = 0x00;
uint8_t RxRemote = 0x00;

bool isRTC = false;
bool isWiFiConnected = false;
bool isUpdating = false;
bool isSDCardInserted = false;
bool isRecording = false;

// NOTE: If we create a 16-bit Sprite, 320 x 240 x 2 bytes RAM is occupied, not a good choice

void WiFiStationTask(void * pvParameters);
void AudioTask(void * pvParameters);
void TFTTask(void * pvParameters);
void CANRecvTask(void * pvParameters);
void UARTRecvTask(void * pvParameters);
void TestTask(void * pvParameters);
void RobotSelftestTask(void * pvParameters);
void HeartbeatAnimeTask(void * pvParameters);

void PingTimerCallback(TimerHandle_t xTimer);
void RSSITimerCallback(TimerHandle_t xTimer);

void processCmdRemoteDebug();
void AddToPlaylist(uint8_t index);
void TouchscreenCalibrate();
void readConfig();
void writeConfig();

void RemoteHandle(uint8_t Tx = 0x00);
void Log(const char * format, ...);
uint16_t Rainbow(uint8_t Value);

void setup() {
    Serial.begin(921600);
    Serial.println();

    Serial1.begin(115200, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
    pinMode(UART1_RX_PIN, INPUT_PULLUP); // Enable internal pull-up resistance to avoid messy code
    Serial2.begin(9600, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

    SPI2.begin(HSPI_SCLK_PIN, HSPI_MISO_PIN, HSPI_MOSI_PIN);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // Specify I2C Pins here instead of in RTC library

    Log("Hardware: %s", HW_NAME);
    Log("Software: %s", SW_NAME);

    setSyncProvider(RTC.get); // Try to sync with RTC
    if (timeStatus() == timeSet) // Check RTC sync status
    {
        isRTC = true;
        Log("RTC module initialized");
    }
    else Log("RTC module not initialized");
    Log("Boot time: %4d/%02d/%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
    sprintf(BootTimePrefix, "%4d_%02d_%02d_%02d_%02d_%02d_", year(), month(), day(), hour(), minute(), second());

    if (SD.begin(SD_CARD_CS_PIN, SPI2)) // Detect SD Card
    {
        uint8_t cardType = SD.cardType();
        if (cardType != CARD_NONE)
        {
            isSDCardInserted = true;
            Log("SD Card mounted");
            if (cardType == CARD_MMC) Log(" - Type: MMC");
            else if (cardType == CARD_SD) Log(" - Type: SDSC");
            else if (cardType == CARD_SDHC) Log(" - Type: SDHC");
            else if (cardType == CARD_UNKNOWN) Log(" - Type: UNKNOWN");
            uint64_t cardSize = SD.cardSize() / (1024 * 1024);
            Log(" - Size: %llu MB", cardSize);
            Log(" - Total: %llu MB", SD.totalBytes() / (1024 * 1024));
            Log(" - Used: %llu MB", SD.usedBytes() / (1024 * 1024));
            externalLogFile = SD.open("/" + String(BootTimePrefix) + "Syslog.csv", FILE_APPEND);
            if (externalLogFile) externalLogFile.println("Time,Millis,Log,");
            CANLogFile = SD.open("/" + String(BootTimePrefix) + "CAN.csv", FILE_APPEND);
            if (CANLogFile) CANLogFile.println("Time,Millis,Pack No.,ID,Length,Byte[0],Byte[1],Byte[2],Byte[3],Byte[4],Byte[5],Byte[6],Byte[7],");
            UARTLogFile = SD.open("/" + String(BootTimePrefix) + "UART.csv", FILE_APPEND);
            if (UARTLogFile) UARTLogFile.println("Time,Millis,Hex,Char,");
            UARTTextFile = SD.open("/" + String(BootTimePrefix) + "UART.txt", FILE_APPEND);
        }
    }
    // NOTE: ESP32 has MMC Controller, not using here mainly because it needs more pins and the pins cannot be remapped

    if (SPIFFS.begin(true)) Log("SPIFFS mounted"); // Format SPIFFS on fail is enabled
    readConfig(); // Load config

    WiFi.softAP(AP_SSID, AP_PASSWORD, 1, config.hideSSID); // Begin AP mode
    WiFi.softAPsetHostname(HW_NAME);
    IPAddress APIP = WiFi.softAPIP();
    Log("AP started");
    Log(" - SSID: %s", AP_SSID);
    Log(" - Password: %s", AP_PASSWORD);
    Log(" - IP: %u.%u.%u.%u", APIP[0], APIP[1], APIP[2], APIP[3]);

    ArduinoOTA
        .onStart([]() {
            isUpdating = true;
            if (ArduinoOTA.getCommand() == U_FLASH)
                Log("Start updating sketch");
            else // U_SPIFFS
                Log("Start updating filesystem");
            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        })
        .onEnd([]() {
            isUpdating = false;
            Log("OTA end");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Log("OTA Progress: %u%%", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            isUpdating = false;
            if (error == OTA_AUTH_ERROR) Log("OTA Error[%u]: Auth Failed", error);
            else if (error == OTA_BEGIN_ERROR) Log("OTA Error[%u]: Begin Failed", error);
            else if (error == OTA_CONNECT_ERROR) Log("OTA Error[%u]: Connect Failed", error);
            else if (error == OTA_RECEIVE_ERROR) Log("OTA Error[%u]: Receive Failed", error);
            else if (error == OTA_END_ERROR) Log("OTA Error[%u]: End Failed", error);
        });
    ArduinoOTA.setMdnsEnabled(false); // If WiFi is not in STA connected status, mDNS fails to start, so disable here
    ArduinoOTA.begin(); // Allow OTA process in AP Mode, default port is 3232
    Log("OTA service started");

    if (isSDCardInserted)
    {
        ftpSrv.begin("uestc", "robocon"); // Enable a simple file-only FTP server if SD Card was inserted
        Log("FTP service started");
        Log(" - Username: uestc");
        Log(" - Password: robocon");
    }

    Debug.begin(HW_NAME); // Initiaze the telnet server
    Debug.setResetCmdEnabled(true); // Enable the reset command
    String helpCmd = "demo - Try different levels of debug message\r\n";
	helpCmd.concat("count - Drive the speaker to count from 1 to 4\r\n");
    helpCmd.concat("r1 - Send TZ1 1st retry command to cradle\r\n");
    helpCmd.concat("r2 - Send TZ1 2nd retry command to cradle\r\n");
    helpCmd.concat("r3 - Send TZ2 1st retry command to cradle");
	Debug.setHelpProjectsCmds(helpCmd);
	Debug.setCallBackProjectCmds(&processCmdRemoteDebug);
    Log("Telnet service started");

    audioController.init(); // Initialize BY8301-16P module
    audioController.setVolume(config.volume);
    audioController.stop();
    //audioController.printModuleInfo();
    Log("Audio service started");

    tft.init();
    tft.setRotation(1);
    #if ENABLE_TOUCH_CALIBRATE
    TouchscreenCalibrate();
    #endif
    uint16_t calData[5] = {255, 3563, 157, 3553, 5};
    tft.setTouch(calData);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextFont(2);

    heartbeatSprite.createSprite(35, 24);
    heartbeatSprite.fillSprite(TFT_BLACK);

    /*
    pinMode(TFT_PEN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TFT_PEN_PIN), TouchISR, FALLING);
    */

    CAN_cfg.speed = CAN_SPEED_1000KBPS; // Set CAN Bus speed to 1Mbps
    CAN_cfg.tx_pin_id = CAN_TX_PIN; // Set CAN TX Pin
    CAN_cfg.rx_pin_id = CAN_RX_PIN; // Set CAN RX Pin
    CAN_cfg.rx_queue = xQueueCreate(16, sizeof(CAN_frame_t)); // Create CAN RX Queue
    ESP32Can.CANInit(); // Start CAN module

    RTCUpdateSemaphoreHandle = xSemaphoreCreateBinary();
    HeartbeatAnimeUpdateSemaphoreHandle = xSemaphoreCreateBinary();
    RSSIUpdateSemaphoreHandle = xSemaphoreCreateBinary();
    OnlineUpdateSemaphoreHandle = xSemaphoreCreateBinary();

    AudioFIFO = xQueueCreate(32, sizeof(uint8_t)); // Create a FIFO to buffer the playing request
    SerialFIFO = xQueueCreate(2048, sizeof(char)); // Create a FIFO to buffer Serial data

    PingTimerHandle = xTimerCreate("Ping Timer", pdMS_TO_TICKS(500), pdTRUE, (void *)1, PingTimerCallback);
    RSSITimerHandle = xTimerCreate("RSSI Timer", pdMS_TO_TICKS(500), pdTRUE, (void *)2, RSSITimerCallback);

    // NOTE: My principle to arrange the priority of tasks is up to the delay in the task, the longer delay, the first you go
    xTaskCreate(WiFiStationTask, "WiFi Station Config", 4096, NULL, 4, &WiFiStationTaskHandle);
    xTaskCreate(AudioTask, "Audio Control", 4096, NULL, 3, &AudioTaskHandle);
    xTaskCreate(TFTTask, "TFT Update", 4096, NULL, 2, &TFTTaskHandle);
    xTaskCreate(CANRecvTask, "CAN Bus Receive", 4096, NULL, 1, &CANRecvTaskHandle);
    xTaskCreate(UARTRecvTask, "UART Receive", 4096, NULL, 1, &UARTRecvTaskHandle);
    xTaskCreate(TestTask, "Priority Test", 4096, NULL, 1, &TestTaskHandle);
    xTaskCreate(HeartbeatAnimeTask, "Heartbeat Anime Task", 2048, NULL, 1, &HeartbeatAnimeTaskHandle);

    xTimerStart(RSSITimerHandle, 0);

    AddToPlaylist(1); // Play OS started tone
}

void loop() {
    RemoteHandle();
    ArduinoOTA.handle();
    Debug.handle();
    ftpSrv.handleFTP();
    #if UPDATE_RTC_TIME
    if (xSemaphoreTake(RTCUpdateSemaphoreHandle, 0) == pdTRUE)
    {
        RTC.set(now());
        Log("RTC time updated");
    }
    #endif
    //delay(50); // Maybe we can use yield() to take place of it
}

void WiFiStationTask(void * pvParameters)
{
    WiFi.begin(STA_SSID, STA_PASSWORD); // Begin STA mode
    #if USE_STATIC_IP_ADDRESS
    WiFi.config(LOCAL_IP, GATEWAY, SUBNET);
    #endif
    WiFi.setHostname(HW_NAME);
    uint32_t WiFiTimeout = millis() + 3E4; // Set WiFi STA connection timeout to 30 seconds
    while (WiFi.status() != WL_CONNECTED && millis() < WiFiTimeout) delay(500); // RTOS delay function: vTaskDelay(pdMS_TO_TICKS(xms)), it seems that delay() acts the same to it
    if (WiFi.status() == WL_CONNECTED) goto Connected;
    else
    {
        Log("WiFi connected failed, SmartConfig begin");
        WiFi.beginSmartConfig();
        while (!WiFi.smartConfigDone()) delay(500); // SmartConfig packet received
        Log("SmartConfig received");
        WiFiTimeout = millis() + 3E4;
        while (WiFi.status() != WL_CONNECTED && millis() < WiFiTimeout) delay(500);
        if (WiFi.status() == WL_CONNECTED) goto Connected;
        else
        {
            Log("WiFi connected failed, stop retrying");
            WiFi.mode(WIFI_AP);
        }
    }
    vTaskDelete(NULL);

    Connected:
    isWiFiConnected = true;
    IPAddress STAIP = WiFi.localIP();
    Log("WiFi connected");
    Log(" - SSID: %s", STA_SSID);
    Log(" - IP: %u.%u.%u.%u", STAIP[0], STAIP[1], STAIP[2], STAIP[3]);
    udp.begin(UDP_PORT);
    xTimerStart(PingTimerHandle, 0);
    Log("UDP transfer started");
    Log(" - Target IP: %s", UDP_ADDRESS);
    Log(" - Target Port: %d", UDP_PORT);
    ArduinoOTA.end();
    ArduinoOTA.setHostname(HW_NAME); // Equals to the parameter in function MDNS.begin(HW_NAME)
    ArduinoOTA.setMdnsEnabled(true); // Now STA connected, restart OTA here to enable mDNS
    ArduinoOTA.begin();
    MDNS.addService("telnet", "tcp", 23); // Telnet service (RemoteDebug)
    if (!isRTC || UPDATE_RTC_TIME)
    {
        NTP.onNTPSyncEvent ([](NTPSyncEvent_t event)
        {
            if (event == timeSyncd)
            {
                Log("Got NTP time: %s", NTP.getTimeDateString(NTP.getLastNTPSync()).c_str());
                #if UPDATE_RTC_TIME
                if (isRTC) xSemaphoreGive(RTCUpdateSemaphoreHandle);
                #endif
            }
            else if (event == noResponse) Log("Time sync error: NTP server not reachable");
            else if (event == invalidAddress) Log("Time sync error: Invalid NTP server address");
        });
        Log("NTP service started");
        NTP.begin("ntp7.aliyun.com", 8);
        NTP.setInterval(30, 600);
    }
    AddToPlaylist(2); // Play prefix tone
    AddToPlaylist(3); // Play WiFi Station connected tone
    vTaskDelete(NULL);
}

void AudioTask(void * pvParameters)
{
    /*
    Audio File List

    001 启动音乐.mp3
    002 提示音.mp3
    003 网络已连接.wav
    004 开始自检.wav
    005 自检通过.wav
    006 自检未通过.wav
    007 底盘主控掉线.wav
    008 云台主控掉线.wav
    009 陀螺仪与码盘掉线.wav
    010 号电机掉线.wav
    011 号激光掉线.wav
    012 一.wav
    013 二.wav
    014 三.wav
    015 四.wav
    016 已到达TZ1.wav
    017 已到达TZ2.wav
    018 已到达TZ3.wav
    019 我来扔个球.wav
    020 老铁，我准备好啦.wav
    */
    
    while (1)
    {
        uint8_t index;
        xQueueReceive(AudioFIFO, &index, portMAX_DELAY); // Attempt to get the index in blocking mode
        while (audioController.isBusy()) delay(25); // Read the Busy Pin of the audio chip
        audioController.playFileIndex(index);
        Log("Begin to play voice %u", index);
        delay(250); // Note that the library or the audio chip itself does not support high rate command stream, so wait here
    }
    vTaskDelete(NULL);
}

void TFTTask(void * pvParameters)
{
    /*
    Scene List

    0 = Booting
    1 = Main
    2 = Selftest
    3 = Variable
    4 = Settings
    5 = Log
    6 = CAN
    7 = UART
    */

    uint8_t scene = 0;
    uint8_t subScene = 0;
    bool redraw = true; // Whether to redraw static parts of a scene
    uint16_t xPos = 0, yPos = 16; // To store command interface coordinates
    bool pressed;
    uint16_t xTouch = 0, yTouch = 0; // To store the touch coordinates
    char tmp;
    uint8_t trayPos = 1; // To store how many icons are shown in the tray (WiFi is always there)
    bool pause = false; // Whether to pause refresh in a page
    uint8_t LogPage = 1;
    bool LogFollow = true;
    while (1)
    {
        // Display control code is below
        switch (scene)
        {
            case 0: // @Booting
            tft.pushImage((320 - 281) / 2, (240 - 64) / 2, 281, 64, bitmap_uestc); // Display UESTC LOGO
            delay(1000);
            tft.pushImage((320 - 300) / 2, (240 - 116) / 2, 300, 116, bitmap_robocon); // Display Robocon 2018 LOGO
            delay(1000);
            scene = 1;
            redraw = true;
            break;

            case 1: // @Main
            if (redraw)
            {
                tft.pushImage(0, 0, 320, 240, bitmap_main);
                tft.pushImage(0, 0, 168, 24, bitmap_uestc_banner);
                redraw = false;
            }
            if (isWiFiConnected) tft.pushImage(320 - 24, 0, 24, 24, bitmap_debug);
            //else tft.pushImage(320 - 24, 0, 24, 24, bitmap_wifi_disconnected);
            trayPos = 1; // Reset the tray position
            if (isSDCardInserted)
            {
                tft.pushImage(320 - 24 * (trayPos + 1), 0, 24, 24, bitmap_sdcard);
                trayPos++;
            }
            if (isUpdating)
            {
                tft.pushImage(320 - 24 * (trayPos + 1), 0, 24, 24, bitmap_update);
                trayPos++;
            }
            if (Debug.isActive(Debug.ANY))
            {
                tft.pushImage(320 - 24 * (trayPos + 1), 0, 24, 24, bitmap_debug);
                trayPos++;
            }
            tft.fillRect(320 - 24 * (5 + 1), 0, 24 * (5 - trayPos + 1), 24, TFT_BLACK); // Erase icons left, assumes that the maximum number of icons is 5 (exclude WiFi)
            char clockStr[20];
            sprintf(clockStr, "%4d/%02d/%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
            tft.setTextColor(Rainbow(millis() / 39), TFT_BLACK);
            tft.setTextDatum(TC_DATUM);
            tft.drawString(clockStr, 160, 32);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextDatum(TL_DATUM);
            break;

            case 2: // @Selftest
            if (redraw)
            {
                tft.fillScreen(TFT_BLACK);
                tft.pushImage(0, 0, 32, 32, bitmap_home, TFT_BLACK);
                tft.pushImage(44, 3, 168, 24, bitmap_uestc_banner);
                /*
                tft.setTextSize(2);
                tft.setTextDatum(CC_DATUM);
                tft.drawString("ROBOT CTRL PANEL", 176 , 16);
                tft.setTextDatum(TL_DATUM);
                tft.setTextSize(1);
                //tft.pushImage(288, 0, 32, 32, bitmap_refresh, TFT_BLACK);
                */
                tft.loadFont("Chinese");

                tft.setTextDatum(C_BASELINE);
                tft.drawString("电机", 98 , 50);
                tft.drawLine(98, 56, 98, 60, TFT_WHITE);
                tft.drawLine(26, 60, 170, 60, TFT_WHITE);
                tft.drawLine(26, 60, 26, 68, TFT_WHITE);
                tft.drawLine(62, 60, 62, 68, TFT_WHITE);
                tft.drawLine(98, 60, 98, 68, TFT_WHITE);
                tft.drawLine(134, 60, 134, 68, TFT_WHITE);
                tft.drawLine(170, 60, 170, 68, TFT_WHITE);
                tft.drawString("惯导", 214 , 50);
                tft.drawLine(214, 56, 214, 60, TFT_WHITE);
                tft.drawLine(214, 60, 214, 68, TFT_WHITE);
                tft.drawString("激光", 276 , 50);
                tft.drawLine(276, 56, 276, 60, TFT_WHITE);
                tft.drawLine(258, 60, 294, 60, TFT_WHITE);
                tft.drawLine(258, 60, 258, 68, TFT_WHITE);
                tft.drawLine(294, 60, 294, 68, TFT_WHITE);

                tft.setTextDatum(CC_DATUM);
                tft.drawRect(8, 128, 73, 50, TFT_WHITE);
                tft.drawString("预备", 44 , 153);
                tft.drawRect(85, 128, 73, 50, TFT_WHITE);
                tft.drawString("吸地", 121 , 153);
                tft.drawRect(162, 128, 73, 50, TFT_WHITE);
                tft.drawString("复位", 198 , 153);
                tft.drawRect(239, 128, 73, 50, TFT_WHITE);
                tft.drawString("发射", 275 , 153);
                tft.drawRect(8, 182, 73, 50, TFT_WHITE);
                tft.drawString("重试", 44 , 207);
                tft.drawRect(85, 182, 73, 50, TFT_WHITE);
                tft.drawString("到TZ1", 121 , 207);
                tft.drawRect(162, 182, 73, 50, TFT_WHITE);
                tft.drawString("到TZ2", 198 , 207);
                tft.drawRect(239, 182, 73, 50, TFT_WHITE);
                tft.drawString("到TZ3", 275 , 207);
                tft.setTextDatum(TL_DATUM);

                tft.unloadFont();
                xSemaphoreGive(OnlineUpdateSemaphoreHandle);
                redraw = false;
            }
            if (xSemaphoreTake(HeartbeatAnimeUpdateSemaphoreHandle, 0) == pdTRUE) heartbeatSprite.pushSprite(225, 0);
            if (xSemaphoreTake(RSSIUpdateSemaphoreHandle, 0) == pdTRUE)
            {
                tft.setTextColor(Rainbow(WiFi.RSSI() ? WiFi.RSSI() + 100 : 0), TFT_BLACK);
                tft.setTextDatum(TR_DATUM);
                tft.setTextPadding(tft.textWidth("000"));
                tft.setTextSize(2);
                tft.drawString(WiFi.RSSI() ? String(WiFi.RSSI()) : "", 319, 0);
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                tft.setTextDatum(TL_DATUM);
                tft.setTextPadding(0);
                tft.setTextSize(1);
            }
            if (xSemaphoreTake(OnlineUpdateSemaphoreHandle, 0) == pdTRUE)
            {
                if (Online & (1 << 0)) tft.pushImage(8, 74, 32, 32, bitmap_pass); // Elmo1
                else tft.pushImage(8, 74, 32, 32, bitmap_fail);
                if (Online & (1 << 1)) tft.pushImage(44, 74, 32, 32, bitmap_pass); // Elmo2
                else tft.pushImage(44, 74, 32, 32, bitmap_fail);
                if (Online & (1 << 2)) tft.pushImage(80, 74, 32, 32, bitmap_pass); // Elmo3
                else tft.pushImage(80, 74, 32, 32, bitmap_fail);
                /*
                if (Online & (1 << 3)) tft.pushImage(116, 74, 32, 32, bitmap_pass); // Elmo4
                else tft.pushImage(116, 74, 32, 32, bitmap_fail);
                if (Online & (1 << 4)) tft.pushImage(152, 74, 32, 32, bitmap_pass); // Elmo5
                else tft.pushImage(152, 74, 32, 32, bitmap_fail);
                */
                tft.pushImage(116, 74, 32, 32, bitmap_pass);
                tft.pushImage(152, 74, 32, 32, bitmap_pass);
                if (Online & (1 << 3)) tft.pushImage(196, 74, 32, 32, bitmap_pass); // Gyro&Encoder
                else tft.pushImage(196, 74, 32, 32, bitmap_fail);
                if (Online & (1 << 4)) tft.pushImage(240, 74, 32, 32, bitmap_pass); // DT35 X
                else tft.pushImage(240, 74, 32, 32, bitmap_fail);
                if (Online & (1 << 5)) tft.pushImage(276, 74, 32, 32, bitmap_pass); // DT35 Y
                else tft.pushImage(276, 74, 32, 32, bitmap_fail);
            }
            break;

            case 3: // @Variable
            if (redraw)
            {
                tft.fillScreen(TFT_BLACK);
                tft.pushImage(0, 0, 32, 32, bitmap_home, TFT_BLACK);
                tft.pushImage(32, 0, 32, 32, bitmap_prev, TFT_BLACK);
                tft.pushImage(256, 0, 32, 32, bitmap_next, TFT_BLACK);
                tft.pushImage(288, 0, 32, 32, bitmap_record_begin, TFT_BLACK);
                switch (subScene)
                {
                    case 0: // @Variable.Summary
                    tft.setFreeFont(&FreeSans9pt7b);
                    tft.setTextDatum(R_BASELINE);
                    tft.drawString("Gyro:", 144, 60);
                    tft.drawString("Encoder[X]:", 144, 84);
                    tft.drawString("Encoder[Y]:", 144, 108);
                    tft.drawString("DT35[X]:", 144, 132);
                    tft.drawString("DT35[Y]:", 144, 156);
                    tft.drawString("Custom:", 144, 180);
                    tft.setTextFont(2);
                    tft.setTextDatum(TL_DATUM);
                    break;

                    case 1: // @Variable.Timer
                    tft.setFreeFont(&FreeSans9pt7b);
                    tft.setTextDatum(R_BASELINE);
                    tft.drawString("Timer 1:", 144, 60);
                    tft.drawString("Timer 2:", 144, 84);
                    tft.drawString("Timer 3:", 144, 108);
                    tft.drawString("Timer 4:", 144, 132);
                    tft.drawString("Timer 5:", 144, 156);
                    tft.drawString("Timer 6:", 144, 180);
                    tft.drawString("Timer 7:", 144, 204);
                    tft.drawString("Timer 8:", 144, 228);
                    tft.setTextFont(2);
                    tft.setTextDatum(TL_DATUM);
                    break;
                }
                redraw = false;
            }
            tft.setTextDatum(TC_DATUM);
            tft.setTextPadding(192);
            tft.drawString("FSM Code: 0", 160 , 0);
            tft.drawString("Waiting for commands", 160 , 16);
            tft.setTextDatum(TL_DATUM);
            tft.setTextPadding(0);
            if (isRecording) tft.pushImage(288, 0, 32, 32, bitmap_record_stop, TFT_BLACK);
            else tft.pushImage(288, 0, 32, 32, bitmap_record_begin, TFT_BLACK);
            switch (subScene)
            {
                case 0:
                tft.setFreeFont(&FreeSans9pt7b);
                tft.setTextDatum(L_BASELINE);
                tft.setTextPadding(160);
                tft.drawString(String(Gyroscope, 3), 160, 60);
                tft.drawString(String(Encoder[0], 3), 160, 84);
                tft.drawString(String(Encoder[1], 3), 160, 108);
                tft.drawString(String(DT35[0], 3), 160, 132);
                tft.drawString(String(DT35[1], 3), 160, 156);
                tft.drawString(String(Custom, 3), 160, 180);
                tft.setTextFont(2);
                tft.setTextDatum(TL_DATUM);
                tft.setTextPadding(0);
                break;

                case 1:
                tft.setFreeFont(&FreeSans9pt7b);
                tft.setTextDatum(L_BASELINE);
                tft.setTextPadding(160);
                tft.drawString(String(Timer[0]), 160, 60);
                tft.drawString(String(Timer[1]), 160, 84);
                tft.drawString(String(Timer[2]), 160, 108);
                tft.drawString(String(Timer[3]), 160, 132);
                tft.drawString(String(Timer[4]), 160, 156);
                tft.drawString(String(Timer[5]), 160, 180);
                tft.drawString(String(Timer[6]), 160, 204);
                tft.drawString(String(Timer[7]), 160, 228);
                tft.setTextFont(2);
                tft.setTextDatum(TL_DATUM);
                tft.setTextPadding(0);
                break;
            }
            break;

            case 4: // @Settings
            if (redraw)
            {
                tft.fillScreen(TFT_BLACK);
                tft.pushImage(0, 0, 32, 32, bitmap_home, TFT_BLACK);
                if (isSDCardInserted) tft.pushImage(256, 0, 32, 32, bitmap_eject_sdcard, TFT_BLACK);
                tft.pushImage(288, 0, 32, 32, bitmap_power, TFT_BLACK);
                tft.setFreeFont(&FreeSans9pt7b);

                tft.setTextDatum(R_BASELINE);
                tft.drawString("Volume:", 144 , 48); // <Space> does not take up any place
                tft.pushImage(160, 28, 32, 32, bitmap_volume_down, TFT_BLACK);
                tft.setTextDatum(C_BASELINE);
                tft.drawString(String(config.volume), 208 , 48);
                tft.pushImage(224, 28, 32, 32, bitmap_volume_up, TFT_BLACK);

                tft.setTextDatum(R_BASELINE);
                tft.drawString("Hide SSID:", 144 , 84);
                if (config.hideSSID) tft.pushImage(160, 64, 54, 32, bitmap_switch_on, TFT_BLACK);
                else tft.pushImage(160, 64, 54, 32, bitmap_switch_off, TFT_BLACK);

                tft.drawString("Raw UART:", 144 , 120);
                if (config.rawUART) tft.pushImage(160, 100, 54, 32, bitmap_switch_on, TFT_BLACK);
                else tft.pushImage(160, 100, 54, 32, bitmap_switch_off, TFT_BLACK);

                tft.setTextColor(TFT_DARKGREEN, TFT_BLACK);
                tft.setTextFont(2);
                tft.setTextDatum(CC_DATUM);
                tft.drawString("UESTC Robot Probe For ABU Robocon 2018", 160, 200);
                tft.drawString("Copyright (c) 2018 HsuRY <i@hsury.com>", 160, 216);
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                tft.setTextDatum(TL_DATUM);
                redraw = false;
            }
            break;

            case 5: // @Log
            if (redraw)
            {
                tft.fillScreen(TFT_BLACK);
                tft.pushImage(0, 0, 32, 32, bitmap_home, TFT_BLACK);
                tft.pushImage(32, 0, 32, 32, bitmap_prev, TFT_BLACK);
                tft.pushImage(256, 0, 32, 32, bitmap_next, TFT_BLACK);
                tft.pushImage(288, 0, 32, 32, bitmap_clear, TFT_BLACK);
                redraw = false;
            }
            if (LogFollow) LogPage = (LogPtr - 1) / 13 + 1; // [0, 13] => 1; [14, 26] => 2
            tft.setTextDatum(CC_DATUM);
            tft.setTextPadding(192);
            tft.drawString("Page " + String(LogPage) + "/" + String((LogPtr - 1) / 13 + 1) + ", " + String(LogPtr) + " records", 160 , 16);
            tft.setTextDatum(TL_DATUM);
            tft.setTextPadding(320);
            for (uint8_t i = 0; i < 13; i++)
            {
                tft.drawString(LogHistory[i + (LogPage - 1) * 13], 0, 32 + i * 16);
            }
            tft.setTextPadding(0);
            break;
            
            case 6: // @CAN Bus Monitor interface
            if (redraw)
            {
                tft.fillScreen(TFT_BLACK);
                tft.fillRect(0, 0, 320, 16, TFT_RED); // Draw red title bar
                tft.setTextColor(TFT_WHITE, TFT_RED);
                tft.setTextDatum(CC_DATUM); // Text align to Centre-Centre, same as MC_DATUM
                tft.drawString("CAN Bus Monitor", 160 , 8);
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                tft.setTextDatum(TL_DATUM); // Text align to Top-Left
                xPos = 0;
                yPos = 16;
                redraw = false;
            }
            if (!pause && ulTaskNotifyTake(pdTRUE, 0)) // If task is notified, which means that a new CAN frame has come, refresh the screen
            {
                CAN_frame_t CAN_rx_frame_holder = CAN_rx_frame; // Avoid data being updated when displayed
                tft.setCursor(0, yPos);
                tft.printf("%-9u | 0x%-4X |", packNum, CAN_rx_frame_holder.MsgID);
                for (uint8_t i = 0; i < CAN_rx_frame_holder.FIR.B.DLC; i++)
                {
                    tft.printf(" %02X", CAN_rx_frame_holder.data.u8[i]);
                }
                yPos += 16;
                if (yPos >= 240) yPos = 16;
                tft.fillRect(0, yPos, 320, 16, TFT_BLACK);
            }
            break;

            case 7: // @UART Monitor interface
            if (redraw)
            {
                tft.fillScreen(TFT_BLACK);
                tft.fillRect(0, 0, 320, 16, TFT_BLUE); // Draw blue title bar
                tft.setTextColor(TFT_WHITE, TFT_BLUE);
                tft.setTextDatum(CC_DATUM); // Text align to Centre-Centre, same as MC_DATUM
                tft.drawString("UART Monitor", 160 , 8);
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                tft.setTextDatum(TL_DATUM); // Text align to Top-Left
                xPos = 0;
                yPos = 16;
                redraw = false;
            }
            while (!pause && xQueueReceive(SerialFIFO, &tmp, 0) == pdTRUE)
            {
                if (!config.rawUART)
                {
                    if (tmp > 31 && tmp < 128) xPos += tft.drawChar(tmp, xPos, yPos); // Char which can be displayed
                    if (tmp == '\r') xPos = 0; // Home
                    if (tmp == '\n') // New Line
                    {
                        yPos += 16;
                        if (yPos >= 240)
                        {
                            xPos = 0;
                            yPos = 16;
                        }
                        tft.fillRect(0, yPos, 320, 16, TFT_BLACK);
                    }
                }
                else
                {
                    tft.setCursor(xPos, yPos);
                    tft.printf("%02X", tmp);
                    xPos += 20;
                    if (xPos >= 320)
                    {
                        xPos = 0;
                        yPos += 16;
                        if (yPos >= 240) yPos = 16;
                        tft.fillRect(0, yPos, 320, 16, TFT_BLACK);
                    }
                }
            }
            break;
        }
        if (scene != 6) delay(25); // Disable screen refresh interval when CAN bus monitor is turned on
        // Touchscreen control code is below
        if (pressed) delay(100); // Touchscreen pressed seperator
        pressed = tft.getTouch(&xTouch, &yTouch); 
        if (pressed)
        {
            //Log("Touch, x=%u, y=%u", xTouch, yTouch);
            //tft.fillCircle(xTouch, yTouch, 2, TFT_WHITE);
            switch (scene)
            {
                case 1: // @Main
                if (xTouch >= 13 && xTouch <= 152 && yTouch >= 57 && yTouch <= 106) // => Selftest
                {
                    scene = 2;
                    redraw = true;
                    Online = 0;
                    CAN_tx_frame.MsgID = Online_ID;
                    CAN_tx_frame.FIR.B.DLC = 1;
                    CAN_tx_frame.data.u8[0] = 0x00;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                }
                else if (xTouch >= 167 && xTouch <= 306 && yTouch >= 57 && yTouch <= 106) // => Variable
                {
                    scene = 3;
                    //subScene = 0; // Uncomment this line to forget footprint
                    redraw = true;
                }
                else if (xTouch >= 13 && xTouch <= 152 && yTouch >= 117 && yTouch <= 166) // => Settings
                {
                    scene = 4;
                    redraw = true;
                }
                else if (xTouch >= 167 && xTouch <= 306 && yTouch >= 117 && yTouch <= 166) // => Log
                {
                    LogPage = (LogPtr - 1) / 13 + 1;
                    LogFollow = true;
                    scene = 5;
                    redraw = true;
                }
                else if (xTouch >= 13 && xTouch <= 152 && yTouch >= 177 && yTouch <= 226) // => CAN
                {
                    pause = false;
                    scene = 6;
                    redraw = true;
                }
                else if (xTouch >= 167 && xTouch <= 306 && yTouch >= 177 && yTouch <= 226) // => UART
                {
                    pause = false;
                    scene = 7;
                    redraw = true;
                }
                break;

                case 2: // @Selftest
                if (xTouch >= 0 && xTouch <= 32 && yTouch >= 0 && yTouch <= 32) goto Home; // Home
                /*
                else if (xTouch >= 288 && xTouch <= 320 && yTouch >= 0 && yTouch <= 32) // REBOOT
                {
                    CAN_tx_frame.MsgID = 0xAA;
                    CAN_tx_frame.FIR.B.DLC = 5;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                    if (RobotSelftestTaskHandle == NULL)
                    {
                        Log("Robot Selftest task began");
                        xTaskCreate(RobotSelftestTask, "Robot Selftest Control", 4096, NULL, 5, &RobotSelftestTaskHandle);
                    }
                }
                */
                else if (xTouch >= 8 && xTouch <= 81 && yTouch >= 120 && yTouch <= 170) // Button 1
                {
                    CAN_tx_frame.MsgID = Keyboard_ID;
                    CAN_tx_frame.FIR.B.DLC = 1;
                    CAN_tx_frame.data.u8[0] = 0x08;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                    audioController.stop();
                    audioController.playFileIndex(2);
                }
                else if (xTouch >= 85 && xTouch <= 158 && yTouch >= 120 && yTouch <= 170) // Button 2
                {
                    CAN_tx_frame.MsgID = Keyboard_ID;
                    CAN_tx_frame.FIR.B.DLC = 1;
                    CAN_tx_frame.data.u8[0] = 0x06;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                    audioController.stop();
                    audioController.playFileIndex(2);
                }
                else if (xTouch >= 162 && xTouch <= 235 && yTouch >= 120 && yTouch <= 170) // Button 3
                {
                    CAN_tx_frame.MsgID = Keyboard_ID;
                    CAN_tx_frame.FIR.B.DLC = 1;
                    CAN_tx_frame.data.u8[0] = 0x04;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                    audioController.stop();
                    audioController.playFileIndex(2);
                }
                else if (xTouch >= 239 && xTouch <= 312 && yTouch >= 120 && yTouch <= 170) // Button 4
                {
                    CAN_tx_frame.MsgID = Keyboard_ID;
                    CAN_tx_frame.FIR.B.DLC = 1;
                    CAN_tx_frame.data.u8[0] = 0x02;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                    audioController.stop();
                    audioController.playFileIndex(2);
                }
                else if (xTouch >= 8 && xTouch <= 81 && yTouch >= 174 && yTouch <= 224) // Button 5
                {
                    CAN_tx_frame.MsgID = Keyboard_ID;
                    CAN_tx_frame.FIR.B.DLC = 1;
                    CAN_tx_frame.data.u8[0] = 0x01;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                    audioController.stop();
                    audioController.playFileIndex(2);
                }
                else if (xTouch >= 85 && xTouch <= 158 && yTouch >= 174 && yTouch <= 224) // Button 6
                {
                    CAN_tx_frame.MsgID = Keyboard_ID;
                    CAN_tx_frame.FIR.B.DLC = 1;
                    CAN_tx_frame.data.u8[0] = 0x07;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                    audioController.stop();
                    audioController.playFileIndex(2);
                }
                else if (xTouch >= 162 && xTouch <= 235 && yTouch >= 174 && yTouch <= 224) // Button 7
                {
                    CAN_tx_frame.MsgID = Keyboard_ID;
                    CAN_tx_frame.FIR.B.DLC = 1;
                    CAN_tx_frame.data.u8[0] = 0x05;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                    audioController.stop();
                    audioController.playFileIndex(2);
                }
                else if (xTouch >= 239 && xTouch <= 312 && yTouch >= 174 && yTouch <= 224) // Button 8
                {
                    CAN_tx_frame.MsgID = Keyboard_ID;
                    CAN_tx_frame.FIR.B.DLC = 1;
                    CAN_tx_frame.data.u8[0] = 0x03;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                    audioController.stop();
                    audioController.playFileIndex(2);
                }
                break;

                case 3: // @Variable
                if (xTouch >= 0 && xTouch <= 32 && yTouch >= 0 && yTouch <= 32) goto Home; // Home
                else if (xTouch >= 32 && xTouch <= 64 && yTouch >= 0 && yTouch <= 32) // Previous page
                {
                    if (subScene > 0) subScene--;
                    else subScene = 1;
                    redraw = true;
                }
                else if (xTouch >= 256 && xTouch <= 288 && yTouch >= 0 && yTouch <= 32) // Next page
                {
                    if (subScene < 1) subScene++;
                    else subScene = 0;
                    redraw = true;
                }
                else if (xTouch >= 288 && xTouch <= 320 && yTouch >= 0 && yTouch <= 32) // Switch whether to record
                {
                    isRecording = !isRecording;
                }
                break;

                case 4: // @Settings
                if (xTouch >= 0 && xTouch <= 32 && yTouch >= 0 && yTouch <= 32) // Home
                {
                    writeConfig();
                    goto Home;
                }
                else if (xTouch >= 256 && xTouch <= 288 && yTouch >= 0 && yTouch <= 32) // Eject SD Card
                {
                    if (isSDCardInserted)
                    {
                        isSDCardInserted = false;
                        tft.fillRect(256, 0, 32, 32, TFT_BLACK); // Remove SD Card Icon
                        Log("SD Card unmounted");
                        if (externalLogFile) externalLogFile.close();
                        if (CANLogFile) CANLogFile.close();
                        if (UARTLogFile) UARTLogFile.close();
                        if (UARTTextFile) UARTTextFile.close();
                        SD.end();
                    }
                }
                else if (xTouch >= 288 && xTouch <= 320 && yTouch >= 0 && yTouch <= 32) // Reboot
                {
                    writeConfig();
                    if (isSDCardInserted)
                    {
                        if (externalLogFile) externalLogFile.close();
                        if (CANLogFile) CANLogFile.close();
                        if (UARTLogFile) UARTLogFile.close();
                        if (UARTTextFile) UARTTextFile.close();
                        SD.end();
                    }
                    SPIFFS.end();
                    ESP.restart();
                }
                else if (xTouch >= 160 && xTouch <= 192 && yTouch >= 28 && yTouch <= 60) // Volume down
                {
                    if (config.volume > 0) config.volume--;
                    tft.fillRect(192, 28, 32, 32, TFT_BLACK);
                    tft.setFreeFont(&FreeSans9pt7b);
                    tft.setTextDatum(C_BASELINE);
                    tft.drawString(String(config.volume), 208 , 48);
                    tft.setTextFont(2);
                    tft.setTextDatum(TL_DATUM);
                    audioController.setVolume(config.volume);
                }
                else if (xTouch >= 224 && xTouch <= 256 && yTouch >= 28 && yTouch <= 60) // Volume up
                {
                    if (config.volume < 30) config.volume++;
                    tft.fillRect(192, 28, 32, 32, TFT_BLACK);
                    tft.setFreeFont(&FreeSans9pt7b);
                    tft.setTextDatum(C_BASELINE);
                    tft.drawString(String(config.volume), 208 , 48);
                    tft.setTextFont(2);
                    tft.setTextDatum(TL_DATUM);
                    audioController.setVolume(config.volume);
                }
                else if (xTouch >= 160 && xTouch <= 214 && yTouch >= 64 && yTouch <= 96) // Switch whether to hide SSID
                {
                    config.hideSSID = !config.hideSSID;
                    if (config.hideSSID) tft.pushImage(160, 64, 54, 32, bitmap_switch_on, TFT_BLACK);
                    else tft.pushImage(160, 64, 54, 32, bitmap_switch_off, TFT_BLACK);
                }
                else if (xTouch >= 160 && xTouch <= 214 && yTouch >= 100 && yTouch <= 132) // Switch whether to print raw UART data
                {
                    config.rawUART = !config.rawUART;
                    if (config.rawUART) tft.pushImage(160, 100, 54, 32, bitmap_switch_on, TFT_BLACK);
                    else tft.pushImage(160, 100, 54, 32, bitmap_switch_off, TFT_BLACK);
                }
                break; // Do not response to blank zone

                case 5: // @Log
                if (xTouch >= 0 && xTouch <= 32 && yTouch >= 0 && yTouch <= 32) goto Home; // Home
                else if (xTouch >= 32 && xTouch <= 64 && yTouch >= 0 && yTouch <= 32) // Previous page
                {
                    LogFollow = false;
                    if (LogPage > 1) LogPage--;
                    else LogPage = (LogPtr - 1) / 13 + 1;
                }
                else if (xTouch >= 64 && xTouch <= 256 && yTouch >= 0 && yTouch <= 32) // Goto latest page
                {
                    LogFollow = true;
                }
                else if (xTouch >= 256 && xTouch <= 288 && yTouch >= 0 && yTouch <= 32) // Next page
                {
                    LogFollow = false;
                    if (LogPage < (LogPtr - 1) / 13 + 1) LogPage++;
                    else LogPage = 1;
                }
                else if (xTouch >= 288 && xTouch <= 320 && yTouch >= 0 && yTouch <= 32) // Clear, should optimize later
                {
                    LogFollow = true;
                    for (uint16_t i = 0; i < 260; i++) LogHistory[i] = "";
                    LogPtr = 0;
                }
                else break;
                redraw = true;
                break;

                case 6: // @CAN
                if (xTouch >= 0 && xTouch <= 320 && yTouch >= 0 && yTouch <= 16) goto Home; // Tap the title bar to return
                else
                {
                    pause = !pause;
                    if (pause) // Draw pause icon
                    {
                        tft.fillRect(306, 2, 4, 12, TFT_WHITE);
                        tft.fillRect(314, 2, 4, 12, TFT_WHITE);
                    }
                    else tft.fillRect(306, 2, 12, 12, TFT_RED);
                }
                break;

                case 7: // @UART
                if (xTouch >= 0 && xTouch <= 320 && yTouch >= 0 && yTouch <= 16) goto Home; // Tap the title bar to return
                else
                {
                    pause = !pause;
                    if (pause) // Draw pause icon
                    {
                        tft.fillRect(306, 2, 4, 12, TFT_WHITE);
                        tft.fillRect(314, 2, 4, 12, TFT_WHITE);
                    }
                    else tft.fillRect(306, 2, 12, 12, TFT_BLUE);
                }
                break;

                Home:
                default: // => Main
                scene = 1;
                redraw = true;
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void CANRecvTask(void * pvParameters)
{
    while (1)
    {
        //Serial.printf("CAN Rx queue usage: %u\r\n", uxQueueMessagesWaiting(CAN_cfg.rx_queue));
        xQueueReceive(CAN_cfg.rx_queue, &CAN_rx_frame, portMAX_DELAY);
        packNum++;
        if (CAN_rx_frame.FIR.B.FF == CAN_frame_std && CAN_rx_frame.FIR.B.RTR != CAN_RTR) // Software CAN filter
        {
            float FTmp;
            int32_t STmp;
            switch (CAN_rx_frame.MsgID)
            {
                case Gyroscope_ID:
                DeviceNotify[0] = millis();
                memcpy(&FTmp, &CAN_rx_frame.data.u8[0], 4);
                Gyroscope = FTmp >= 0 ? fmod(FTmp, 360) : fmod(FTmp, 360) + 360;
                break;

                case Encoder_ID:
                DeviceNotify[1] = millis();
                memcpy(&STmp, &CAN_rx_frame.data.u8[0], 4);
                Encoder[0] = STmp * PI * 50.7 / 2000 / 1000;
                memcpy(&STmp, &CAN_rx_frame.data.u8[4], 4);
                Encoder[1] = STmp * PI * 50.7 / 2000 / 1000;
                break;

                case DT35_X_ID:
                DeviceNotify[2] = millis();
                memcpy(&STmp, &CAN_rx_frame.data.u8[0], 4);
                DT35[0] = (STmp / 1000.0 + 0.889) * cos(Gyroscope / 180 * PI);
                break;

                case DT35_Y_ID:
                DeviceNotify[3] = millis();
                memcpy(&STmp, &CAN_rx_frame.data.u8[0], 4);
                DT35[1] = (STmp / 1000.0 + 0.0) * cos(Gyroscope / 180 * PI);
                break;

                case MR_ID:
                if (CAN_rx_frame.data.u8[0] == 0xFF) // Packet asking AR to retry giving from MR
                {
                    RemoteHandle(CAN_rx_frame.data.u8[1]);
                }
                break;

                case Cradle_ID:
                if (CAN_rx_frame.data.u8[0] == 0xAA) // Base reports that it has arrived in TZx
                {
                    if (CAN_rx_frame.data.u8[1] == 0x01) // In TZ1
                    {
                        Log("Robot arrived in TZ1\r\n");
                        //AddToPlaylist(2);
                        //AddToPlaylist(16);
                    }
                    else if (CAN_rx_frame.data.u8[1] == 0x02) // In TZ2
                    {
                        Log("Robot arrived in TZ2\r\n");
                        //AddToPlaylist(2);
                        //AddToPlaylist(17);
                    }
                    else if (CAN_rx_frame.data.u8[1] == 0x03) // In TZ3
                    {
                        Log("Robot arrived in TZ3\r\n");
                        //AddToPlaylist(2);
                        //AddToPlaylist(18);
                    }
                }
                else if (CAN_rx_frame.data.u8[0] == 0xEE) // Cradle reports that it is ready to receive another shuttlecock
                {
                    RemoteHandle(CAN_rx_frame.data.u8[1]);
                    Log("Cradle is ready\r\n");
                    AddToPlaylist(2);
                    AddToPlaylist(20);
                }
                break;

                case Hint_Tone_ID:
                if (CAN_rx_frame.data.u8[0] == 0xFF && CAN_rx_frame.data.u8[1] == 0x01) // Play the hint tone before throwing the shuttlecock
                {
                    Log("Ready to throw the shuttlecock\r\n");
                    AddToPlaylist(2);
                    AddToPlaylist(19);
                }
                break;

                case Online_ID:
                memcpy(&Online, &CAN_rx_frame.data.u8[0], 4);
                xSemaphoreGive(OnlineUpdateSemaphoreHandle);
                break;

                case Custom_ID:
                memcpy(&Custom, &CAN_rx_frame.data.u8[0], sizeof(Custom));
                break;

                case Timer_ID:
                if (CAN_rx_frame.data.u8[0] < 8) memcpy(&Timer[CAN_rx_frame.data.u8[0]], &CAN_rx_frame.data.u8[1], 4);
                break;
            }
        }
        if (TFTTaskHandle) xTaskNotifyGive(TFTTaskHandle); // xTaskNotify(TFTTaskHandle, 1, eSetValueWithOverwrite); 
        if (CANLogFile) // Save log file to SD Card
        {
            CANLogFile.printf("%02u:%02u:%02u,%u,%u,0x%X,%d,", hour(), minute(), second(), millis(), packNum, CAN_rx_frame.MsgID, CAN_rx_frame.FIR.B.DLC);
            for (uint8_t i = 0; i < CAN_rx_frame.FIR.B.DLC; i++) CANLogFile.printf("0x%02X,", CAN_rx_frame.data.u8[i]);
            for (uint8_t i = 0; i < 8 - CAN_rx_frame.FIR.B.DLC; i++) CANLogFile.printf(",");
            CANLogFile.printf("\r\n");
        }
        if (ENABLE_CAN_DEBUG)
        {
            Serial.println("-------------------------");
            Serial.printf("Pack Number: %u\r\n", packNum);
            Serial.print("Type: ");
            if (CAN_rx_frame.FIR.B.FF == CAN_frame_std) Serial.println("Standard");
            else Serial.println("Extend");
            Serial.print("RTR: ");
            if (CAN_rx_frame.FIR.B.RTR == CAN_RTR) Serial.println("True");
            else Serial.println("False");
            Serial.printf("Msg ID: 0x%X\r\n", CAN_rx_frame.MsgID);
            Serial.printf("DLC: %d\r\n", CAN_rx_frame.FIR.B.DLC);
            Serial.print("Data:");
            for (uint8_t i = 0; i < CAN_rx_frame.FIR.B.DLC; i++)
            {
                Serial.printf(" %02X", CAN_rx_frame.data.u8[i]);
            }
            Serial.println();
            Serial.println("-------------------------");
        }
    }
    vTaskDelete(NULL);
}

void UARTRecvTask(void * pvParameters)
{
    char tmp; // Variable to temporarily save the char coming from UART
    while (1)
    {
        if (Serial1.available())
        {
            tmp = Serial1.read();
            xQueueSend(SerialFIFO, &tmp, 0); // Insert item into the queue
            if (UARTLogFile) UARTLogFile.printf("%02u:%02u:%02u,%u,0x%02X,%c,\r\n", hour(), minute(), second(), millis(), tmp, tmp); // Save log file to SD Card
            if (UARTTextFile) UARTTextFile.print(tmp); // Save text file to SD Card
        }
    }
    vTaskDelete(NULL);
}

void TestTask(void * pvParameters)
{
    while (1)
    {
        Serial1.printf("STM32 UART Test!\r\n");
        delay(2500);
    }
    vTaskDelete(NULL);
}

void RobotSelftestTask(void * pvParameters)
{
    AddToPlaylist(2);
    AddToPlaylist(4);
    bool isOnline[NOTIFY_DEVICE_NUM + 1]; // Preserve index 0 as global status
    uint32_t SelftestBegin = millis();
    uint32_t SelftestEnd = millis() + 5E3; // Set robot selftest timeout to 5 seconds
    while (millis() < SelftestEnd)
    {
        isOnline[0] = true;
        for (uint8_t i = 1; i <= NOTIFY_DEVICE_NUM; i++)
        {
            isOnline[i] = DeviceNotify[i - 1] > SelftestBegin && DeviceNotify[i - 1] < SelftestEnd;
            isOnline[0] &= isOnline[i]; // If device notification arrived during selftest, mark the device online
        }
        if (isOnline[0]) break;
        delay(50);
    }
    if (isOnline[0])
    {
        AddToPlaylist(2);
        AddToPlaylist(5);
    }
    else // Some devices did not send notification during selftest
    {
        AddToPlaylist(2);
        AddToPlaylist(6);
        for (uint8_t i = 1; i <= NOTIFY_DEVICE_NUM; i++)
        {
            if (!isOnline[i]) Log("Device %u is offline", i); // Later will change to AddToPlaylist()
        }
    }
    delay(2500); // Wait for a short time
    RobotSelftestTaskHandle = NULL;
    vTaskDelete(NULL);
}

void HeartbeatAnimeTask(void * pvParameters)
{
    uint8_t ShadeWidth = 8;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int8_t ShadePos = -ShadeWidth;
        while (ShadePos <= 35)
        {
            heartbeatSprite.fillSprite(TFT_BLACK);
            heartbeatSprite.pushImage(0, 0, 35, 24, bitmap_heartbeat);
            heartbeatSprite.fillRect(0, 0, ShadePos, 24, TFT_BLACK);
            heartbeatSprite.fillRect(ShadePos + ShadeWidth, 0, 35 - ShadePos - ShadeWidth, 24, TFT_BLACK);
            ShadePos += 1;
            xSemaphoreGive(HeartbeatAnimeUpdateSemaphoreHandle);
            delay(25);
        }
        ulTaskNotifyTake(pdTRUE, 0);
    }
    vTaskDelete(NULL);
}

void PingTimerCallback(TimerHandle_t xTimer)
{
    udp.beginPacket(UDP_ADDRESS, UDP_PORT);
    udp.write(0xFF);
    udp.endPacket();
}

void RSSITimerCallback(TimerHandle_t xTimer)
{
    xSemaphoreGive(RSSIUpdateSemaphoreHandle);
}

void processCmdRemoteDebug()
{
    String lastCmd = Debug.getLastCommand();
    if (Debug.isActive(Debug.ANY))
    {
        if (lastCmd == "demo")
        {
            DEBUG_V("VERBOSE\n");
            DEBUG_D("DEBUG\n");
            DEBUG_I("INFO\n");
            DEBUG_W("WARNING\n");
            DEBUG_E("ERROR\n");
        }
        else if (lastCmd == "count")
        {
            AddToPlaylist(12);
            AddToPlaylist(13);
            AddToPlaylist(14);
            AddToPlaylist(15);
        }
        else if (lastCmd == "r1")
        {
            CAN_tx_frame.MsgID = Cradle_ID;
            CAN_tx_frame.FIR.B.DLC = 2;
            CAN_tx_frame.data.u8[0] = 0xFF;
            CAN_tx_frame.data.u8[1] = 0x11;
            ESP32Can.CANWriteFrame(&CAN_tx_frame);
            Log("TZ1 1st retry command sent to cradle");
            audioController.stop();
            audioController.playFileIndex(2);
        }
        else if (lastCmd == "r2")
        {
            CAN_tx_frame.MsgID = Cradle_ID;
            CAN_tx_frame.FIR.B.DLC = 2;
            CAN_tx_frame.data.u8[0] = 0xFF;
            CAN_tx_frame.data.u8[1] = 0x12;
            ESP32Can.CANWriteFrame(&CAN_tx_frame);
            Log("TZ1 2nd retry command sent to cradle");
            audioController.stop();
            audioController.playFileIndex(2);
        }
        else if (lastCmd == "r3")
        {
            CAN_tx_frame.MsgID = Cradle_ID;
            CAN_tx_frame.FIR.B.DLC = 2;
            CAN_tx_frame.data.u8[0] = 0xFF;
            CAN_tx_frame.data.u8[1] = 0x13;
            ESP32Can.CANWriteFrame(&CAN_tx_frame);
            Log("TZ2 1st retry command sent to cradle");
            audioController.stop();
            audioController.playFileIndex(2);
        }
    }
}

void AddToPlaylist(uint8_t index)
{
    // According to ESP32-IDF Documentation, it's not recommended to use xQueueOverwrite() when the capcity of the queue is larger than one
    xQueueSend(AudioFIFO, &index, pdMS_TO_TICKS(1000)); // Insert item into the queue
}

#if ENABLE_TOUCH_CALIBRATE
void TouchscreenCalibrate()
{
    uint16_t calData[5];
    Log("Touchscreen calibration began");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawCentreString("Touch corners as indicated", tft.width() / 2, tft.height() / 2, 2);
    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);
    Log("The calibration code in setup() is shown below:");
    Log("    uint16_t calData[5] = {%u, %u, %u, %u, %u};", calData[0], calData[1], calData[2], calData[3], calData[4]);
    Log("    tft.setTouch(calData);");
    Log("Touchscreen calibration finished");
}
#endif

void readConfig()
{
    StaticJsonBuffer<512> jsonBuffer; // Allocate the memory pool on the stack, see also https://arduinojson.org/assistant to compute the capacity
    if (SPIFFS.exists("/config.json"))
    {
        File internalConfigFile = SPIFFS.open("/config.json");
        JsonObject &root = jsonBuffer.parseObject(internalConfigFile); // Parse internal JSON config file
        if (root.success()) Log("Config loaded from SPIFFS");
        else Log("Config cannot be loaded from SPIFFS");
        config.volume = root["volume"] | 5; // An interesting approach to load default value while JSON parse error
        config.hideSSID = root["hideSSID"] | false;
        config.rawUART = root["rawUART"] | false;
        internalConfigFile.close(); // Close the file (File's destructor doesn't close the file)
    }
    else // Config file not found
    {
        Log("Cannot find config file in SPIFFS");
        writeConfig(); // Save default config to SPIFFS
    }
}

void writeConfig()
{
    StaticJsonBuffer<512> jsonBuffer;
    if (SPIFFS.exists("/config.json")) SPIFFS.remove("/config.json"); // Delete existing file, otherwise the configuration is appended to the file
    File internalConfigFile = SPIFFS.open("/config.json", FILE_WRITE);
    JsonObject &root = jsonBuffer.createObject();
    root["volume"] = config.volume;
    root["hideSSID"] = config.hideSSID;
    root["rawUART"] = config.rawUART;
    if (root.printTo(internalConfigFile)) Log("Config saved to SPIFFS");
    else Log("Config cannot be saved to SPIFFS");
    internalConfigFile.close();
}

void RemoteHandle(uint8_t Tx)
{
    // Tx
    if (Tx && Tx < 0x80) TxRemote = Tx;
    if (TxRemote)
    {
        udp.beginPacket(UDP_ADDRESS, UDP_PORT);
        udp.write(TxRemote);
        udp.endPacket();
    }
    // Rx
    if (isWiFiConnected && udp.parsePacket())
    {
        xTaskNotifyGive(HeartbeatAnimeTaskHandle);
        while (udp.available())
        {
            uint8_t tmp = udp.read();
            if (tmp < 0x80) // Command packet
            {
                if (tmp != RxRemote)
                {
                    if (tmp < 0x40) CAN_tx_frame.MsgID = Cradle_ID; // MR => AR
                    else CAN_tx_frame.MsgID = MR_ID; // AR => MR
                    CAN_tx_frame.FIR.B.DLC = 2;
                    CAN_tx_frame.data.u8[0] = 0xFF;
                    CAN_tx_frame.data.u8[1] = tmp;
                    ESP32Can.CANWriteFrame(&CAN_tx_frame);
                    Log("Got 0x%X from peer", tmp);
                    RxRemote = tmp;
                }
                udp.beginPacket(UDP_ADDRESS, UDP_PORT);
                udp.write(tmp + 0x80);
                udp.endPacket();
            }
            else if (tmp != 0xFF) // ACK packet
            {
                Log("Sent 0x%X to peer", tmp - 0x80);
                if (tmp - 0x80 == TxRemote) TxRemote = 0x00; // If ACK received, stop sending
            }
            //udp.flush(); // Necessary if not read the remaining data
            //break;
        }
    }
}

void Log(const char * format, ...) // Customized printf based logging function
{
    char buf[128]; // Resulting string limitted to 128 chars
    va_list vargs;
    va_start(vargs, format);
    // Cannot use printf here, use vprintf(format, vargs) instead
    vsnprintf(buf, 128, format, vargs);
    Serial.printf(buf);
    Serial.printf("\r\n");
    if (Debug.isActive(Debug.ANY))
    {
        Debug.printf(buf);
        Debug.printf("\r\n");
    }
    if (LogPtr >= 260)
    {
        for (uint16_t i = 0; i < 260; i++) LogHistory[i] = ""; // Free the heap used by previous log
        LogPtr = 0;
    }
    LogHistory[LogPtr++] = String(buf);
    if (externalLogFile) // Save log file to SD Card
    {
        externalLogFile.printf("%02u:%02u:%02u,%u,", hour(), minute(), second(), millis());
        externalLogFile.printf(buf);
        externalLogFile.printf(",\r\n");
    }
    va_end(vargs);
}

uint16_t Rainbow(uint8_t Value)
{
    // Value is expected to be in range 0-127
    // The value is converted to a spectrum colour from 0 = red through to 127 = blue
    
    if (Value > 127) Value = 255 - Value; // Make the color swing

    uint8_t Red   = 0; // Red is the top 5 bits of a 16 bit colour value
    uint8_t Green = 0; // Green is the middle 6 bits
    uint8_t Blue  = 0; // Blue is the bottom 5 bits

    uint8_t Sector = Value >> 5;
    uint8_t Amplit = Value & 0x1F;

    switch (Sector)
    {
        case 0:
        Red   = 0x1F;
        Green = Amplit;
        Blue  = 0;
        break;

        case 1:
        Red   = 0x1F - Amplit;
        Green = 0x1F;
        Blue  = 0;
        break;

        case 2:
        Red   = 0;
        Green = 0x1F;
        Blue  = Amplit;
        break;

        case 3:
        Red   = 0;
        Green = 0x1F - Amplit;
        Blue  = 0x1F;
        break;
    }
    return Red << 11 | Green << 6 | Blue;
}
