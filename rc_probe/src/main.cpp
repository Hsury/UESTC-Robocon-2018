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

                                        VERSION 2018/02/02

*/

// Use esptool.py --port /dev/ttyUSB0 erase_flash to clear SPIFFS

#include <Arduino.h>
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
#include <TimeLib.h>
#include <NtpClientLib.h>
// TODO: Add DS3231 RTC module
#include <ESP8266FtpServer.h>
#include "bitmap.h"

#define HW_NAME "AutoRobot"
#define SW_NAME "20180202"

#define ENABLE_TOUCH_CALIBRATE 0
#define NOTIFY_DEVICE_NUM 5

const char* AP_SSID      = "AR_Probe";
const char* AP_PASSWORD  = "duoguanriben8";

const char* STA_SSID     = "HsuRY";
const char* STA_PASSWORD = "***REMOVED***";

struct ProbeConfig
{
    uint8_t volume = 5;
    boolean hideSSID = false;
    boolean rawUART = false;
} config;

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4
#define UART1_TX_PIN GPIO_NUM_12
#define UART1_RX_PIN GPIO_NUM_14
#define UART2_TX_PIN GPIO_NUM_16
#define UART2_RX_PIN GPIO_NUM_17
#define HSPI_SCLK_PIN GPIO_NUM_27
#define HSPI_MISO_PIN GPIO_NUM_32
#define HSPI_MOSI_PIN GPIO_NUM_33
#define AUDIO_BUSY_PIN GPIO_NUM_22
#define TFT_PEN_PIN GPIO_NUM_25 // Seems useless, just leave it alone as a mark
#define SD_CARD_CS_PIN GPIO_NUM_26

/*
TFT Relevant Pin List

TFT_MISO  19
TFT_MOSI  23
TFT_SCLK  18
TFT_CS    15 // Chip select control pin
TFT_DC    2 // Data Command control pin
TFT_RST   13 // Reset pin (could connect to RST pin)
TOUCH_CS  21 // Chip select pin (T_CS) of touch screen

P.S. Check them in lib/TFT_eSPI/User_Setup.h
P.P.S. TFT and Touchscreen are using VSPI, transactions (To work with other devices on the bus) are automatically enabled by TFT_eSPI for an ESP32 (to use HAL mutex)
*/

#define Gyroscope_ID 0x11
#define Encoder_ID 0x12
#define DT35_H_ID 0x50
#define DT35_F_ID 0x51
#define DT35_R_ID 0x52

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
CAN_frame_t rx_frame;
uint32_t packNum;

SPIClass SPI2(HSPI); // In order to make full use of the chip and not to meet FreeRTOS task conflict with TFT, enable HSPI (HSPI = SPI2; VSPI = SPI3, Default)
HardwareSerial Serial1(1);  // UART1/Serial1 pins 9, 10
HardwareSerial Serial2(2);  // UART2/Serial2 pins 16, 17

BY8X0116P audioController(Serial2, AUDIO_BUSY_PIN);
TFT_eSPI tft = TFT_eSPI();
RemoteDebug Debug;
FtpServer ftpSrv;

TaskHandle_t WiFiStationTaskHandle;
TaskHandle_t AudioTaskHandle;
TaskHandle_t TFTTaskHandle;
TaskHandle_t CANRecvTaskHandle;
TaskHandle_t UARTRecvTaskHandle;
TaskHandle_t TestTaskHandle;
TaskHandle_t RobotSelftestTaskHandle;

xQueueHandle AudioFIFO;
xQueueHandle SerialFIFO;

//To learn more about Arduino String Class, see also https://hackingmajenkoblog.wordpress.com/2016/02/04/the-evils-of-arduino-strings/
String LogHistory[13 * 20]; // Record for 20 pages at most
uint16_t LogPtr = 0;

uint32_t DeviceNotify[NOTIFY_DEVICE_NUM];
float Gyroscope;
int32_t Encoder[2]; // X-Y coordinate
int32_t DT35[3]; // Array to save the data of DT35s

boolean isWiFiConnected = false;
boolean isUpdating = false;
boolean isSDCardInserted = false;
boolean isRecording = false;

// NOTE: If we create a 16-bit Sprite, 320 x 240 x 2 bytes RAM is occupied, not a good choice

void WiFiStationTask(void * pvParameters);
void AudioTask(void * pvParameters);
void TFTTask(void * pvParameters);
void CANRecvTask(void * pvParameters);
void UARTRecvTask(void * pvParameters);
void TestTask(void * pvParameters);
void RobotSelftestTask(void * pvParameters);
void processCmdRemoteDebug();
void AddToPlaylist(uint8_t index);
void TouchscreenCalibrate();
void readConfig();
void writeConfig();

void Log(const char * format, ...);

void setup() {
    Serial.begin(115200);
    Serial.println();

    Serial1.begin(115200, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
    Serial2.begin(9600, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

    SPI2.begin(HSPI_SCLK_PIN, HSPI_MISO_PIN, HSPI_MOSI_PIN);

    CAN_cfg.speed = CAN_SPEED_1000KBPS; // Set CAN Bus speed to 1Mbps
    CAN_cfg.tx_pin_id = CAN_TX_PIN; // Set CAN TX Pin
    CAN_cfg.rx_pin_id = CAN_RX_PIN; // Set CAN RX Pin
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t)); // Create CAN RX Queue
    ESP32Can.CANInit(); // Start CAN module

    if (SPIFFS.begin(true)) Log("SPIFFS mounted"); // Format SPIFFS on fail is enabled
    readConfig(); // Load config

    /*
    pinMode(TFT_PEN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TFT_PEN_PIN), TouchISR, FALLING);
    */

    tft.init();
    tft.setRotation(3);
    #if ENABLE_TOUCH_CALIBRATE
    TouchscreenCalibrate();
    #endif
    uint16_t calData[5] = {231, 3590, 156, 3560, 3};
    tft.setTouch(calData);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextFont(2);

    WiFi.softAP(AP_SSID, AP_PASSWORD, 1, config.hideSSID); // Begin AP mode
    WiFi.softAPsetHostname(HW_NAME);
    IPAddress APIP = WiFi.softAPIP();
    Log("AP started");
    Log(" - SSID: %s", AP_SSID);
    Log(" - Password: %s", AP_PASSWORD);
    Log(" - IP: %u.%u.%u.%u", APIP[0], APIP[1], APIP[2], APIP[3]);

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
            ftpSrv.begin("uestc", "robocon"); // Enable a simple file-only FTP server if SD Card was inserted
            Log("FTP server started");
            Log(" - Username: uestc");
            Log(" - Password: robocon");
        }
    }
    // NOTE: ESP32 has MMC Controller, not using here mainly because it needs more pins and the pins cannot be remapped
    
    audioController.init(); // Initialize BY8301-16P module
    audioController.setVolume(config.volume);
    audioController.stop();
    //audioController.printModuleInfo();

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

    NTP.onNTPSyncEvent ([](NTPSyncEvent_t event)
    {
        if (event == timeSyncd) Log("Got NTP time: %s", NTP.getTimeDateString(NTP.getLastNTPSync()).c_str());
        else if (event == noResponse) Log("Time sync error: NTP server not reachable");
        else if (event == invalidAddress) Log("Time sync error: Invalid NTP server address");
    });

    Debug.begin(HW_NAME); // Initiaze the telnet server
    Debug.setResetCmdEnabled(true); // Enable the reset command
    String helpCmd = "play - Drive the speaker\n";
	helpCmd.concat("uestc - emmmm");
	Debug.setHelpProjectsCmds(helpCmd);
	Debug.setCallBackProjectCmds(&processCmdRemoteDebug);

    AudioFIFO = xQueueCreate(32, sizeof(uint8_t)); // Create a FIFO to buffer the playing request
    SerialFIFO = xQueueCreate(2048, sizeof(char)); // Create a FIFO to buffer Serial data

    // NOTE: My principle to arrange the priority of tasks is up to the delay in the task, the longer delay, the first you go
    xTaskCreate(WiFiStationTask, "WiFi Station Config", 4096, NULL, 4, &WiFiStationTaskHandle);
    xTaskCreate(AudioTask, "Audio Control", 4096, NULL, 3, &AudioTaskHandle);
    xTaskCreate(TFTTask, "TFT Update", 4096, NULL, 2, &TFTTaskHandle);
    xTaskCreate(CANRecvTask, "CAN Bus Receive", 4096, NULL, 1, &CANRecvTaskHandle);
    xTaskCreate(UARTRecvTask, "UART Receive", 4096, NULL, 1, &UARTRecvTaskHandle);
    xTaskCreate(TestTask, "Priority Test", 4096, NULL, 1, &TestTaskHandle);

    AddToPlaylist(1); // Play OS started tone
}

void loop() {
    ArduinoOTA.handle();
    Debug.handle();
    ftpSrv.handleFTP();
    delay(50); // Maybe we can use yield() to take place of it
}

void WiFiStationTask(void * pvParameters)
{
    WiFi.begin(STA_SSID, STA_PASSWORD); // Begin STA mode
    WiFi.setHostname(HW_NAME);
    uint32_t WiFiTimeout = millis() + 3E4; // Set WiFi STA connection timeout to 30 seconds
    while (WiFi.status() != WL_CONNECTED && millis() < WiFiTimeout)
    {
        delay(500); // RTOS delay function: vTaskDelay(pdMS_TO_TICKS(xms)), it seems that delay() acts the same to it
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        isWiFiConnected = true;
        IPAddress STAIP = WiFi.localIP();
        Log("WiFi connected");
        Log(" - SSID: %s", STA_SSID);
        Log(" - IP: %u.%u.%u.%u", STAIP[0], STAIP[1], STAIP[2], STAIP[3]);
        ArduinoOTA.end();
        ArduinoOTA.setHostname(HW_NAME); // Equals to the parameter in function MDNS.begin(HW_NAME)
        ArduinoOTA.setMdnsEnabled(true); // Now STA connected, restart OTA here to enable mDNS
        ArduinoOTA.begin();
        MDNS.addService("telnet", "tcp", 23); // Telnet service (RemoteDebug)
        NTP.begin("time1.aliyun.com", 8);
        NTP.setInterval(10, 60);
        Log("NTP service started");
        AddToPlaylist(2); // Play prefix tone
        AddToPlaylist(3); // Play WiFi Station connected tone
    }
    else
    {
        Log("WiFi connected failed, SmartConfig begin");
        WiFi.beginSmartConfig();
        while (!WiFi.smartConfigDone()) // SmartConfig packet received
        {
            delay(500);
        }
        Log("SmartConfig received");
        WiFiTimeout = millis() + 3E4;
        while (WiFi.status() != WL_CONNECTED && millis() < WiFiTimeout)
        {
            delay(500);
        }
        if (WiFi.status() == WL_CONNECTED)
        {
            isWiFiConnected = true;
            IPAddress STAIP = WiFi.localIP();
            Log("WiFi connected");
            Log(" - SSID: %s", WiFi.SSID());
            Log(" - IP: %u.%u.%u.%u", STAIP[0], STAIP[1], STAIP[2], STAIP[3]);
            ArduinoOTA.end();
            ArduinoOTA.setHostname(HW_NAME); // Equals to the parameter in function MDNS.begin(HW_NAME)
            ArduinoOTA.setMdnsEnabled(true); // Now STA connected, restart OTA here to enable mDNS
            ArduinoOTA.begin();
            MDNS.addService("telnet", "tcp", 23); // Telnet service (RemoteDebug)
            NTP.begin("time1.aliyun.com", 8);
            NTP.setInterval(10, 60);
            Log("NTP service started");
            AddToPlaylist(2); // Play prefix tone
            AddToPlaylist(3); // Play WiFi Station connected tone
        }
        else
        {
            Log("WiFi connected failed, Stop");
            WiFi.mode(WIFI_AP);
        }
    }
    vTaskDelete(NULL);
}

void AudioTask(void * pvParameters)
{
    /*
    Audio File List

    001启动音乐.mp3
    002提示音.mp3
    003NetworkConnected.mp3
    004自检开始.mp3
    005自检通过.mp3
    006自检未通过.mp3
    007底盘主控掉线.mp3
    008云台主控掉线.mp3
    009陀螺仪与码盘掉线.mp3
    010号电机掉线.mp3
    011号激光掉线.mp3
    012一.mp3
    013二.mp3
    014三.mp3
    015四.mp3
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
    uint8_t scene = 0;
    uint16_t xPos = 0, yPos = 16; // To store command interface coordinates
    boolean pressed;
    uint16_t xTouch = 0, yTouch = 0; // To store the touch coordinates
    char tmp;
    uint8_t trayPos = 1; // To store how many icons are shown in the tray (WiFi is always there)
    boolean pause = false; // Whether to pause refresh in a page
    uint8_t LogPage = 1;
    boolean LogFollow = true;
    while (1)
    {
        // Display control code is below
        switch (scene)
        {
            case 0: // @Booting
            tft.pushImage((320 - 281) / 2, (240 - 64) / 2, 281, 64, bitmap_uestc); // Display UESTC LOGO
            delay(2500);
            tft.pushImage((320 - 300) / 2, (240 - 116) / 2, 300, 116, bitmap_robocon); // Display Robocon 2018 LOGO
            delay(2500);
            tft.pushImage(0, 0, 320, 240, bitmap_main);
            tft.setCursor(0, 0);
            tft.printf("HW: %s\r\nSW: %s", HW_NAME, SW_NAME);
            scene = 1;
            break;

            case 1: // @Main
            if (isWiFiConnected) tft.pushImage(320 - 24, 0, 24, 24, bitmap_wifi_connected);
            else tft.pushImage(320 - 24, 0, 24, 24, bitmap_wifi_disconnected);
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
            tft.setCursor(0, 32);
            tft.printf("MS: %u", millis());
            break;

            case 2: // @Selftest
            tft.setCursor(100, 32);
            tft.print("Selftest Page");
            tft.pushImage(100, (240 - 32) / 2, 32, 32, bitmap_pass, TFT_BLACK); // Make black as transparent color
            tft.pushImage(200, (240 - 32) / 2, 32, 32, bitmap_fail, TFT_BLACK); // Make black as transparent color
            break;

            case 3: // @Variable
            tft.setFreeFont(&FreeSans9pt7b);
            tft.setTextDatum(L_BASELINE);
            tft.setTextPadding(160);
            tft.drawString(String(Gyroscope, 6), 160, 64);
            tft.drawString(String(Encoder[0]), 160, 88);
            tft.drawString(String(Encoder[1]), 160, 112);
            tft.drawString(String(DT35[0]), 160, 136);
            tft.drawString(String(DT35[1]), 160, 160);
            tft.drawString(String(DT35[2]), 160, 184);
            tft.setTextFont(2);
            tft.setTextDatum(TL_DATUM);
            tft.setTextPadding(0);
            break;

            case 4: // @Settings
            break;

            case 5: // @Log
            if (ulTaskNotifyTake(pdTRUE, 0) == 1)
            {
                RefreshLog: // Force refresh log label
                if (LogFollow) LogPage = (LogPtr - 1) / 13 + 1; // [0, 13] => 1; [14, 26] => 2
                tft.setTextDatum(CC_DATUM);
                tft.fillRect(64, 0, 192, 32, TFT_BLACK);
                tft.drawString("Page " + String(LogPage) + "/" + String((LogPtr - 1) / 13 + 1) + ", " + String(LogPtr) + " records", 160 , 16);
                tft.setTextDatum(TL_DATUM);
                tft.setCursor(0, 32);
                for (uint8_t i = 0; i < 13; i++)
                {
                    tft.fillRect(0, 32 + i * 16, 320, 16, TFT_BLACK);
                    tft.println(LogHistory[i + (LogPage - 1) * 13]);
                }
            }
            break;
            
            case 6: // @CAN Bus Monitor interface
            if (!pause && ulTaskNotifyTake(pdTRUE, 0) == 2) // If task is notified, which means that a new CAN frame has come, refresh the screen
            {
                tft.setCursor(0, yPos);
                tft.printf("%u, ID: %d, Data: ", packNum, rx_frame.MsgID);
                for (uint8_t i = 0; i < rx_frame.FIR.B.DLC; i++)
                {
                    tft.printf("%02x ", rx_frame.data.u8[i]);
                }
                yPos += 16;
                if (yPos >= 240) yPos = 16;
                tft.fillRect(0, yPos, 320, 16, TFT_BLACK);
            }
            break;

            case 7: // @UART Monitor interface
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
                    tft.printf("%02x", tmp);
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
        // Touchscreen control code is below
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
                    if (RobotSelftestTaskHandle == NULL)
                    {
                        Log("Robot Selftest task began");
                        xTaskCreate(RobotSelftestTask, "Robot Selftest Control", 4096, NULL, 5, &RobotSelftestTaskHandle);
                    }
                    tft.fillScreen(TFT_BLACK);
                    tft.pushImage(0, 0, 32, 32, bitmap_home, TFT_BLACK);
                    tft.pushImage(288, 0, 32, 32, bitmap_refresh, TFT_BLACK);
                    scene = 2;
                }
                else if (xTouch >= 167 && xTouch <= 306 && yTouch >= 57 && yTouch <= 106) // => Variable
                {
                    tft.fillScreen(TFT_BLACK);
                    tft.pushImage(0, 0, 32, 32, bitmap_home, TFT_BLACK);
                    tft.pushImage(32, 0, 32, 32, bitmap_prev, TFT_BLACK);
                    tft.setTextDatum(TC_DATUM);
                    tft.drawString("FSM Code: 12", 160 , 0);
                    tft.drawString("Waiting for launch command", 160 , 16);
                    tft.setTextDatum(TL_DATUM);
                    tft.pushImage(256, 0, 32, 32, bitmap_next, TFT_BLACK);
                    if (isRecording) tft.pushImage(288, 0, 32, 32, bitmap_record_stop, TFT_BLACK);
                    else tft.pushImage(288, 0, 32, 32, bitmap_record_begin, TFT_BLACK);
                    tft.setFreeFont(&FreeSans9pt7b);
                    tft.setTextDatum(R_BASELINE);
                    tft.drawString("Gyro:", 144, 64);
                    tft.drawString("Encoder[X]:", 144, 88);
                    tft.drawString("Encoder[Y]:", 144, 112);
                    tft.drawString("DT35[H]:", 144, 136);
                    tft.drawString("DT35[F]:", 144, 160);
                    tft.drawString("DT35[R]:", 144, 184);
                    tft.setTextFont(2);
                    tft.setTextDatum(TL_DATUM);
                    scene = 3;
                }
                else if (xTouch >= 13 && xTouch <= 152 && yTouch >= 117 && yTouch <= 166) // => Settings
                {
                    tft.fillScreen(TFT_BLACK);
                    tft.pushImage(0, 0, 32, 32, bitmap_home, TFT_BLACK);
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

                    tft.setTextFont(2);
                    tft.setTextDatum(TL_DATUM);
                    scene = 4;
                }
                else if (xTouch >= 167 && xTouch <= 306 && yTouch >= 117 && yTouch <= 166) // => Log
                {
                    tft.fillScreen(TFT_BLACK);
                    tft.pushImage(0, 0, 32, 32, bitmap_home, TFT_BLACK);
                    tft.pushImage(32, 0, 32, 32, bitmap_prev, TFT_BLACK);
                    tft.setTextDatum(CC_DATUM);
                    LogPage = (LogPtr - 1) / 13 + 1;
                    tft.drawString("Page " + String(LogPage) + "/" + String((LogPtr - 1) / 13 + 1) + ", " + String(LogPtr) + " records", 160 , 16);
                    tft.setTextDatum(TL_DATUM);
                    tft.pushImage(256, 0, 32, 32, bitmap_next, TFT_BLACK);
                    tft.pushImage(288, 0, 32, 32, bitmap_clear, TFT_BLACK);
                    LogFollow = true;
                    scene = 5;
                    goto RefreshLog;
                }
                else if (xTouch >= 13 && xTouch <= 152 && yTouch >= 177 && yTouch <= 226) // => CAN
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
                    pause = false;
                    scene = 6;
                }
                else if (xTouch >= 167 && xTouch <= 306 && yTouch >= 177 && yTouch <= 226) // => UART
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
                    pause = false;
                    scene = 7;
                }
                break;

                case 3: // @Variable
                if (xTouch >= 0 && xTouch <= 32 && yTouch >= 0 && yTouch <= 32) goto Home; // Home
                else if (xTouch >= 32 && xTouch <= 64 && yTouch >= 0 && yTouch <= 32); // Previous page
                else if (xTouch >= 256 && xTouch <= 288 && yTouch >= 0 && yTouch <= 32); // Next page
                else if (xTouch >= 288 && xTouch <= 320 && yTouch >= 0 && yTouch <= 32) // Switch whether to record
                {
                    isRecording = !isRecording;
                    if (isRecording) tft.pushImage(288, 0, 32, 32, bitmap_record_stop, TFT_BLACK);
                    else tft.pushImage(288, 0, 32, 32, bitmap_record_begin, TFT_BLACK);
                }
                break;

                case 4: // @Settings
                if (xTouch >= 0 && xTouch <= 32 && yTouch >= 0 && yTouch <= 32) goto Home; // Home
                else if (xTouch >= 288 && xTouch <= 320 && yTouch >= 0 && yTouch <= 32) ESP.restart(); // Reboot
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
                    writeConfig();
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
                    writeConfig();
                }
                else if (xTouch >= 160 && xTouch <= 214 && yTouch >= 64 && yTouch <= 96) // Switch whether to hide SSID
                {
                    config.hideSSID = !config.hideSSID;
                    if (config.hideSSID) tft.pushImage(160, 64, 54, 32, bitmap_switch_on, TFT_BLACK);
                    else tft.pushImage(160, 64, 54, 32, bitmap_switch_off, TFT_BLACK);
                    writeConfig();
                }
                else if (xTouch >= 160 && xTouch <= 214 && yTouch >= 100 && yTouch <= 132) // Switch whether to print raw UART data
                {
                    config.rawUART = !config.rawUART;
                    if (config.rawUART) tft.pushImage(160, 100, 54, 32, bitmap_switch_on, TFT_BLACK);
                    else tft.pushImage(160, 100, 54, 32, bitmap_switch_off, TFT_BLACK);
                    writeConfig();
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
                goto RefreshLog;
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
                tft.pushImage(0, 0, 320, 240, bitmap_main);
                tft.setCursor(0, 0);
                tft.printf("HW: %s\r\nSW: %s", HW_NAME, SW_NAME);
                scene = 1;
                break;
            }
            delay(100);
        }
        delay(25);
    }
    vTaskDelete(NULL);
}

void CANRecvTask(void * pvParameters)
{
    while (1)
    {
        if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
        {
            packNum++;
            if (rx_frame.FIR.B.FF == CAN_frame_std && rx_frame.FIR.B.RTR != CAN_RTR) // Software CAN filter
            {
                switch (rx_frame.MsgID)
                {
                    case Gyroscope_ID:
                    DeviceNotify[0] = millis();
                    memcpy(&Gyroscope, &rx_frame.data.u8[0], 4);
                    break;

                    case Encoder_ID:
                    DeviceNotify[1] = millis();
                    memcpy(&Encoder[0], &rx_frame.data.u8[0], 4);
                    memcpy(&Encoder[1], &rx_frame.data.u8[4], 4);
                    break;

                    case DT35_H_ID:
                    DeviceNotify[2] = millis();
                    memcpy(&DT35[0], &rx_frame.data.u8[0], 4);
                    break;

                    case DT35_F_ID:
                    DeviceNotify[3] = millis();
                    memcpy(&DT35[1], &rx_frame.data.u8[0], 4);
                    break;

                    case DT35_R_ID:
                    DeviceNotify[4] = millis();
                    memcpy(&DT35[2], &rx_frame.data.u8[0], 4);
                    break;
                }
            }
            if (TFTTaskHandle) xTaskNotify(TFTTaskHandle, 2, eSetValueWithOverwrite); //xTaskNotifyGive(TFTTaskHandle);
            if (isSDCardInserted) // Save log file to SD Card
            {
                File CANLogFile = SD.open("/CAN.csv", FILE_APPEND);
                if (CANLogFile)
                {
                    CANLogFile.printf("%s,%u,", NTP.getTimeDateString().c_str(), rx_frame.FIR.B.DLC);
                    for (uint8_t i = 0; i < rx_frame.FIR.B.DLC; i++) CANLogFile.printf("0x%02x,", rx_frame.data.u8[i]);
                    for (uint8_t i = 0; i < 8 - rx_frame.FIR.B.DLC; i++) CANLogFile.printf(",");
                    CANLogFile.printf("\r\n");
                }
                CANLogFile.close();
            }
            Serial.println("-------------------------");
            Serial.printf("Pack Number: %u\r\n", packNum);
            Serial.print("Type: ");
            if (rx_frame.FIR.B.FF == CAN_frame_std) Serial.println("Standard");
            else Serial.println("Extend");
            Serial.print("RTR: ");
            if (rx_frame.FIR.B.RTR == CAN_RTR) Serial.println("True");
            else Serial.println("False");
            Serial.printf("Msg ID: 0x%08x\r\n", rx_frame.MsgID);
            Serial.printf("DLC: %d\r\n", rx_frame.FIR.B.DLC);
            Serial.print("Data: ");
            for (uint8_t i = 0; i < rx_frame.FIR.B.DLC; i++)
            {
                Serial.printf("%02x ", rx_frame.data.u8[i]);
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
        if (Serial.available())
        {
            tmp = Serial.read();
            xQueueSend(SerialFIFO, &tmp, 0); // Insert item into the queue
            if (isSDCardInserted) // Save log file to SD Card
            {
                File UARTLogFile = SD.open("/UART.csv", FILE_APPEND);
                if (UARTLogFile)
                {
                    UARTLogFile.printf("%s,0x%02x,%c,\r\n", NTP.getTimeDateString().c_str(), tmp, tmp);
                }
                UARTLogFile.close();
            }
        }
    }
    vTaskDelete(NULL);
}

void TestTask(void * pvParameters)
{
    while (1)
    {
        Log("[%u] System is running", millis());
        //Log(NTP.getTimeDateString().c_str());
        delay(5000);
    }
    vTaskDelete(NULL);
}

void RobotSelftestTask(void * pvParameters)
{
    AddToPlaylist(2);
    AddToPlaylist(4);
    boolean isOnline[NOTIFY_DEVICE_NUM + 1]; // Preserve index 0 as global status
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

void processCmdRemoteDebug()
{
    String lastCmd = Debug.getLastCommand();
    if (lastCmd == "play")
    {
        if (Debug.isActive(Debug.ANY))
        {
            Log("OK, let's make it!");
            DEBUG_V("* This is a message of debug level VERBOSE\n");
            DEBUG_D("* This is a message of debug level DEBUG\n");
            DEBUG_I("* This is a message of debug level INFO\n");
            DEBUG_W("* This is a message of debug level WARNING\n");
            DEBUG_E("* This is a message of debug level ERROR\n");
            audioController.stop();
            audioController.playFileIndex(1);
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
    if (TFTTaskHandle) xTaskNotify(TFTTaskHandle, 1, eSetValueWithOverwrite); // xTaskNotifyGive(TFTTaskHandle);
    if (isSDCardInserted) // Save log file to SD Card
    {
        File externalLogFile = SD.open("/Syslog.csv", FILE_APPEND);
        if (externalLogFile)
        {
            externalLogFile.printf(NTP.getTimeDateString().c_str());
            externalLogFile.printf(",");
            externalLogFile.printf(buf);
            externalLogFile.printf(",\r\n");
        }
        externalLogFile.close();
    }
    va_end(vargs);
}
