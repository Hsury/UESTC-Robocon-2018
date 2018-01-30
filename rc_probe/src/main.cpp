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

                                        VERSION 2018/01/30

*/

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
#include "bitmap.h"

#define HW_NAME "AutoRobot"
#define SW_NAME "20180130"

#define ENABLE_TOUCH_CALIBRATE 0
#define NOTIFY_DEVICE_NUM 3

const char* AP_SSID      = "AR_Probe";
const char* AP_PASSWORD  = "duoguanriben8";

const char* STA_SSID     = "HsuRY";
const char* STA_PASSWORD = "***REMOVED***";

struct ProbeConfig
{
    uint8_t volume = 5;
    boolean hideSSID = false;
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

TaskHandle_t WiFiStationTaskHandle;
TaskHandle_t AudioTaskHandle;
TaskHandle_t TFTTaskHandle;
TaskHandle_t CANRecvTaskHandle;
TaskHandle_t UARTRecvTaskHandle;
TaskHandle_t TestTaskHandle;
TaskHandle_t RobotSelftestTaskHandle;

xQueueHandle AudioFIFO;
xQueueHandle SerialFIFO;

uint32_t DeviceNotify[NOTIFY_DEVICE_NUM];
uint32_t DT35[3]; // Array to save the data of DT35s
boolean isWiFiConnected = false;
boolean isUpdating = false;
boolean isSDCardInserted = false;

/*
NOTE: If we create a 16-bit Sprite, 320 x 240 x 2 bytes RAM is occupied, not a good choice
String UARTHistory[14];
String CANHistory[14];
*/

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

    if (SPIFFS.begin(true)) Serial.println("SPIFFS mounted"); // Format SPIFFS on fail is enabled
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
    Serial.println("AP started");
    Serial.printf(" - SSID: %s\r\n", AP_SSID);
    Serial.printf(" - Password: %s\r\n", AP_PASSWORD);
    Serial.print(" - IP: ");
    Serial.println(APIP);
    
    if (SD.begin(SD_CARD_CS_PIN, SPI2)) // Detect SD Card
    {
        uint8_t cardType = SD.cardType();
        if (cardType != CARD_NONE)
        {
            isSDCardInserted = true;
            Serial.println("SD Card mounted");
            Serial.print(" - Type: ");
            if (cardType == CARD_MMC) Serial.println("MMC");
            else if (cardType == CARD_SD) Serial.println("SDSC");
            else if (cardType == CARD_SDHC) Serial.println("SDHC");
            else if (cardType == CARD_UNKNOWN) Serial.println("UNKNOWN");
            uint64_t cardSize = SD.cardSize() / (1024 * 1024);
            Serial.printf(" - Size: %llu MB\r\n", cardSize);
            Serial.printf(" - Total: %llu MB\r\n", SD.totalBytes() / (1024 * 1024));
            Serial.printf(" - Used: %llu MB\r\n", SD.usedBytes() / (1024 * 1024));
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
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_SPIFFS
                type = "filesystem";
            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() {
            isUpdating = false;
            Serial.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            isUpdating = false;
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });
    ArduinoOTA.setMdnsEnabled(false); // If WiFi is not in STA connected status, mDNS fails to start, so disable here
    ArduinoOTA.begin(); // Allow OTA process in AP Mode, default port is 3232

    Debug.begin(HW_NAME); // Initiaze the telnet server
    Debug.setResetCmdEnabled(true); // Enable the reset command
    String helpCmd = "play - Drive the speaker\n";
	helpCmd.concat("uestc - emmmm");
	Debug.setHelpProjectsCmds(helpCmd);
	Debug.setCallBackProjectCmds(&processCmdRemoteDebug);

    AudioFIFO = xQueueCreate(32, sizeof(uint8_t)); // Create a FIFO to buffer the playing request
    SerialFIFO = xQueueCreate(2048, sizeof(char)); // Create a FIFO to buffer Serial data

    // NOTE: My principle to arrange the priority of tasks is up to the delay in the task, the longer delay, the first you go
    xTaskCreate(WiFiStationTask, "WiFi Station Config", 2048, NULL, 4, &WiFiStationTaskHandle);
    xTaskCreate(AudioTask, "Audio Control", 2048, NULL, 3, &AudioTaskHandle);
    xTaskCreate(TFTTask, "TFT Update", 4096, NULL, 2, &TFTTaskHandle);
    xTaskCreate(CANRecvTask, "CAN Bus Receive", 2048, NULL, 1, &CANRecvTaskHandle);
    xTaskCreate(UARTRecvTask, "UART Receive", 2048, NULL, 1, &UARTRecvTaskHandle);
    xTaskCreate(TestTask, "Priority Test", 1024, NULL, 1, &TestTaskHandle);

    AddToPlaylist(1); // Play OS started tone
}

void loop() {
    ArduinoOTA.handle();
    Debug.handle();
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
        Serial.println("WiFi connected");
        Serial.printf(" - SSID: %s\r\n", STA_SSID);
        Serial.print(" - IP: ");
        Serial.println(STAIP);
        ArduinoOTA.end();
        ArduinoOTA.setHostname(HW_NAME); // Equals to the parameter in function MDNS.begin(HW_NAME)
        ArduinoOTA.setMdnsEnabled(true); // Now STA connected, restart OTA here to enable mDNS
        ArduinoOTA.begin();
        MDNS.addService("telnet", "tcp", 23); // Telnet service (RemoteDebug)
        AddToPlaylist(2); // Play prefix tone
        AddToPlaylist(3); // Play WiFi Station connected tone
    }
    else
    {
        Serial.println("WiFi connected failed, SmartConfig begin");
        WiFi.beginSmartConfig();
        while (!WiFi.smartConfigDone()) // SmartConfig packet received
        {
            delay(500);
        }
        Serial.println("SmartConfig received");
        WiFiTimeout = millis() + 3E4;
        while (WiFi.status() != WL_CONNECTED && millis() < WiFiTimeout)
        {
            delay(500);
        }
        if (WiFi.status() == WL_CONNECTED)
        {
            isWiFiConnected = true;
            IPAddress STAIP = WiFi.localIP();
            Serial.println("WiFi connected");
            Serial.printf(" - SSID: %s\r\n", WiFi.SSID());
            Serial.print(" - IP: ");
            Serial.println(STAIP);
            ArduinoOTA.end();
            ArduinoOTA.setHostname(HW_NAME); // Equals to the parameter in function MDNS.begin(HW_NAME)
            ArduinoOTA.setMdnsEnabled(true); // Now STA connected, restart OTA here to enable mDNS
            ArduinoOTA.begin();
            MDNS.addService("telnet", "tcp", 23); // Telnet service (RemoteDebug)
            AddToPlaylist(2); // Play prefix tone
            AddToPlaylist(3); // Play WiFi Station connected tone
        }
        else
        {
            Serial.println("WiFi connected failed, Stop");
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
        Serial.printf("Begin to play voice %u\r\n", index);
        DEBUG_I("Begin to play voice %u\r\n", index);
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
    uint8_t line = 1;
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
            tft.setCursor(100, 32);
            tft.print("Variable Page");
            tft.setCursor(0, 64);
            tft.println("DT35 Debug Tool");
            tft.setCursor(0, 100);
            tft.fillRect(0, 100, 200, 48, TFT_BLACK);
            tft.printf("Head: %u\r\n", DT35[0]);
            tft.printf("Front: %u\r\n", DT35[1]);
            tft.printf("Rear: %u\r\n", DT35[2]);
            tft.pushImage(200, 120, 48, 48, bitmap_prev, TFT_BLACK);
            tft.pushImage(260, 120, 48, 48, bitmap_next, TFT_BLACK);
            break;

            case 4: // @Settings
            
            break;

            case 5: // @Log
            tft.setCursor(100, 32);
            tft.print("Log Page");
            break;
            
            case 6: // @CAN Bus Monitor interface
            if (ulTaskNotifyTake(pdTRUE, 0)) // If task is notified, which means that a new CAN frame has come, refresh the screen
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
            while (xQueueReceive(SerialFIFO, &tmp, 0) == pdTRUE)
            {
                if (tmp > 31 && tmp < 128) xPos += tft.drawChar(tmp, xPos, yPos); // Char which can be displayed
                if (tmp == '\r') xPos = 0; // Return
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
            break;
        }
        // Touchscreen control code is below
        pressed = tft.getTouch(&xTouch, &yTouch); 
        if (pressed)
        {
            Serial.printf("Touch, x=%u, y=%u\r\n", xTouch, yTouch);
            //tft.fillCircle(x, y, 2, TFT_WHITE);
            switch (scene)
            {
                case 1: // @Main
                if (xTouch >= 13 && xTouch <= 152 && yTouch >= 57 && yTouch <= 106) // => Selftest
                {
                    if (RobotSelftestTaskHandle == NULL)
                    {
                        Serial.println("Robot Selftest task began");
                        xTaskCreate(RobotSelftestTask, "Robot Selftest Control", 2048, NULL, 5, &RobotSelftestTaskHandle);
                    }
                    tft.fillScreen(TFT_BLACK);
                    scene = 2;
                }
                else if (xTouch >= 167 && xTouch <= 306 && yTouch >= 57 && yTouch <= 106) // => Variable
                {
                    tft.fillScreen(TFT_BLACK);
                    scene = 3;
                }
                else if (xTouch >= 13 && xTouch <= 152 && yTouch >= 117 && yTouch <= 166) // => Settings
                {
                    tft.fillScreen(TFT_BLACK);
                    tft.pushImage(4, 4, 32, 32, bitmap_home, TFT_BLACK);
                    tft.pushImage(284, 4, 32, 32, bitmap_power, TFT_BLACK);
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

                    tft.setTextFont(2);
                    tft.setTextDatum(TL_DATUM);
                    scene = 4;
                }
                else if (xTouch >= 167 && xTouch <= 306 && yTouch >= 117 && yTouch <= 166) // => Log
                {
                    tft.fillScreen(TFT_BLACK);
                    scene = 5;
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
                    scene = 7;
                }
                break;

                case 4: // @Settings
                if (xTouch >= 160 && xTouch <= 192 && yTouch >= 28 && yTouch <= 60) // Volume down
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
                    break;
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
                    break;
                }
                else if (xTouch >= 160 && xTouch <= 214 && yTouch >= 64 && yTouch <= 96) // Switch whether to hide SSID
                {
                    config.hideSSID = !config.hideSSID;
                    if (config.hideSSID) tft.pushImage(160, 64, 54, 32, bitmap_switch_on, TFT_BLACK);
                    else tft.pushImage(160, 64, 54, 32, bitmap_switch_off, TFT_BLACK);
                    writeConfig();
                    break;
                }
                else if (xTouch >= 284 && xTouch <= 316 && yTouch >= 4 && yTouch <= 36) ESP.restart(); // Reboot
                else if (xTouch >= 4 && xTouch <= 36 && yTouch >= 4 && yTouch <= 36); // Home
                else break; // Do not response to blank zone

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
    Serial.println("Waiting for CAN Bus Data...");
    while (1)
    {
        if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
        {
            packNum++;
            if (rx_frame.FIR.B.FF == CAN_frame_std && rx_frame.FIR.B.RTR != CAN_RTR) // Software CAN filter
            {
                switch (rx_frame.MsgID)
                {
                    case DT35_H_ID:
                    DeviceNotify[0] = millis();
                    memcpy(&DT35[0], &rx_frame.data.u8[0], 4);
                    Debug.printf("DT35_H: %u\r\n", DT35[0]);
                    break;

                    case DT35_F_ID:
                    DeviceNotify[1] = millis();
                    memcpy(&DT35[1], &rx_frame.data.u8[0], 4);
                    Debug.printf("DT35_F: %u\r\n", DT35[1]);
                    break;

                    case DT35_R_ID:
                    DeviceNotify[2] = millis();
                    memcpy(&DT35[2], &rx_frame.data.u8[0], 4);
                    Debug.printf("DT35_R: %u\r\n", DT35[2]);
                    break;
                }
            }
            xTaskNotifyGive(TFTTaskHandle); // TEST now, print CAN Bus information on the screen
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
        }
    }
    vTaskDelete(NULL);
}

void TestTask(void * pvParameters)
{
    while (1)
    {
        Serial.print("Prio Test, Timestamp = ");
        Serial.println(millis());
        delay(1000);
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
            if (!isOnline[i]) Serial.printf("Device %u is offline\r\n", i); // Later will change to AddToPlaylist()
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
            Debug.println("OK, let's make it!");
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
    Serial.println("Touchscreen calibration began");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawCentreString("Touch corners as indicated", tft.width() / 2, tft.height() / 2, 2);
    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);
    Serial.println("The calibration code in setup() is shown below:");
    Serial.print("    uint16_t calData[5] = {");
    for (uint8_t i = 0; i < 5; i++)
    {
        Serial.print(calData[i]);
        if (i < 4) Serial.print(", ");
    }
    Serial.println("};");
    Serial.println("    tft.setTouch(calData);");
    Serial.println("Touchscreen calibration finished");
}
#endif

void readConfig()
{
    StaticJsonBuffer<512> jsonBuffer; // Allocate the memory pool on the stack, see also https://arduinojson.org/assistant to compute the capacity
    if (SPIFFS.exists("/config.json"))
    {
        File internalConfigFile = SPIFFS.open("/config.json");
        JsonObject &root = jsonBuffer.parseObject(internalConfigFile); // Parse internal JSON config file
        if (root.success()) Serial.println("Config loaded from SPIFFS");
        else Serial.println("Config cannot be loaded from SPIFFS");
        config.volume = root["volume"] | 5; // An interesting approach to load default value while JSON parse error
        config.hideSSID = root["hideSSID"] | false;
        internalConfigFile.close(); // Close the file (File's destructor doesn't close the file)
    }
    else // Config file not found
    {
        Serial.println("Cannot find config file in SPIFFS");
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
    if (root.printTo(internalConfigFile)) Serial.println("Config saved to SPIFFS");
    else Serial.println("Config cannot be saved to SPIFFS");
    internalConfigFile.close();
}
