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

                                        VERSION 2018/01/26

*/

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <RemoteDebug.h>
#include <BY8X01-16P.h>

#define ENABLE_TOUCH_CALIBRATE 0

#define VOLUME 10
#define HOST_NAME "AR"
#define NOTIFY_DEVICE_NUM 16

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4
#define UART1_TX_PIN GPIO_NUM_12
#define UART1_RX_PIN GPIO_NUM_14
#define UART2_TX_PIN GPIO_NUM_16
#define UART2_RX_PIN GPIO_NUM_17
#define AUDIO_BUSY_PIN GPIO_NUM_22
#define TFT_PEN_PIN GPIO_NUM_25 // Seems useless, just leave it alone

/*
TFT Relevant Pin List
Ps. Check them in lib/TFT_eSPI/User_Setup.h

TFT_MISO  19
TFT_MOSI  23
TFT_SCLK  18
TFT_CS    15 // Chip select control pin
TFT_DC    2 // Data Command control pin
TFT_RST   13 // Reset pin (could connect to RST pin)
TOUCH_CS  21 // Chip select pin (T_CS) of touch screen
*/

const char* AP_SSID      = "AR_Probe";
const char* AP_PASSWORD  = "duoguanriben8";

const char* STA_SSID     = "HsuRY";
const char* STA_PASSWORD = "***REMOVED***";

CAN_device_t CAN_cfg;
CAN_frame_t rx_frame;
uint16_t packNum;
uint16_t DeviceNotify[NOTIFY_DEVICE_NUM];

TaskHandle_t WiFiStationTaskHandle;
TaskHandle_t TFTUpdateTaskHandle;
TaskHandle_t CANRecvTaskHandle;
TaskHandle_t AudioTaskHandle;
TaskHandle_t TouchscreenTaskHandle;
TaskHandle_t TestTaskHandle;
TaskHandle_t RobotSelftestTaskHandle;

xQueueHandle AudioFIFO;

HardwareSerial Serial1(1);  // UART1/Serial1 pins 9, 10
HardwareSerial Serial2(2);  // UART2/Serial2 pins 16, 17

BY8X0116P audioController(Serial2, AUDIO_BUSY_PIN);
TFT_eSPI tft = TFT_eSPI();
RemoteDebug Debug;

void WiFiStationTask(void * pvParameters);
void TFTUpdateTask(void * pvParameters);
void CANRecvTask(void * pvParameters);
void AudioTask(void * pvParameters);
void TouchscreenTask(void * pvParameters);
void TestTask(void * pvParameters);
void RobotSelftestTask(void * pvParameters);

void processCmdRemoteDebug();

void AddToPlaylist(uint8_t index);

void TouchscreenCalibrate();

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println();

    Serial1.begin(115200, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
    Serial2.begin(9600, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

    tft.init();
    tft.setRotation(3);
    #if ENABLE_TOUCH_CALIBRATE
    TouchscreenCalibrate();
    #endif
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextFont(2);
    tft.println("SYSTEM STARTED");

    /*
    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TFT_PEN_PIN), TouchISR, FALLING);
    */

    WiFi.softAP(AP_SSID, AP_PASSWORD); // Begin AP mode  // ssid_hidden = 1
    WiFi.softAPsetHostname(HOST_NAME);
    IPAddress APIP = WiFi.softAPIP();
    Serial.println("AP Started");
    Serial.printf(" - SSID: %s\r\n", AP_SSID);
    Serial.printf(" - PASSWORD: %s\r\n", AP_PASSWORD);
    Serial.print(" - IP: ");
    Serial.println(APIP);
    
    CAN_cfg.speed = CAN_SPEED_1000KBPS; // Set CAN Bus speed to 1Mbps
    CAN_cfg.tx_pin_id = CAN_TX_PIN; // Set CAN TX Pin
    CAN_cfg.rx_pin_id = CAN_RX_PIN; // Set CAN RX Pin
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t)); // Create CAN RX Queue
    //start CAN Module
    ESP32Can.CANInit();

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_SPIFFS
                type = "filesystem";
            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() {
            Serial.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });
    ArduinoOTA.setMdnsEnabled(false); // If WiFi is not in STA connected status, mDNS fails to start, so disable here
    ArduinoOTA.begin(); // Allow OTA process in AP Mode, default port is 3232

    Debug.begin(HOST_NAME); // Initiaze the telnet server
    //Debug.setResetCmdEnabled(true); // Enable the reset command
    String helpCmd = "play - Drive the speaker\n";
	helpCmd.concat("uestc - emmmm");
	Debug.setHelpProjectsCmds(helpCmd);
	Debug.setCallBackProjectCmds(&processCmdRemoteDebug);

    audioController.init(); // Initialize BY8301-16P module
    audioController.setVolume(VOLUME);
    audioController.stop();
    //audioController.printModuleInfo();

    AudioFIFO = xQueueCreate(32, sizeof(uint8_t)); // Create a FIFO to buffer the playing request

    xTaskCreate(WiFiStationTask, "WiFi Station Config", 2048, NULL, 1, &WiFiStationTaskHandle);
    xTaskCreate(TFTUpdateTask, "TFT Update", 2048, NULL, 2, &TFTUpdateTaskHandle);
    xTaskCreate(CANRecvTask, "CAN Bus Receive", 2048, NULL, 3, &CANRecvTaskHandle);
    xTaskCreate(AudioTask, "Audio Control", 2048, NULL, 4, &AudioTaskHandle);
    xTaskCreate(TouchscreenTask, "Touchscreen Control", 2048, NULL, 5, &TouchscreenTaskHandle);
    xTaskCreate(TestTask, "Priority Test", 1024, NULL, 1, &TestTaskHandle);

    AddToPlaylist(1); // Play OS started tone
}

void loop() {
    // put your main code here, to run repeatedly:
    ArduinoOTA.handle();
    Debug.handle();
    delay(10); // Maybe we can use yield() to take place of it
}

void WiFiStationTask(void * pvParameters)
{
    WiFi.begin(STA_SSID, STA_PASSWORD); // Begin STA mode
    WiFi.setHostname(HOST_NAME);
    uint16_t WiFiTimeout = millis() + 3E4; // Set WiFi STA connection timeout to 30 seconds
    while (WiFi.status() != WL_CONNECTED && millis() < WiFiTimeout)
    {
        delay(500); // RTOS delay function: vTaskDelay(pdMS_TO_TICKS(xms)), it seems that delay() acts the same to it
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        IPAddress STAIP = WiFi.localIP();
        Serial.println("WiFi connected");
        Serial.printf(" - SSID: %s\r\n", STA_SSID);
        Serial.print(" - IP: ");
        Serial.println(STAIP);
        ArduinoOTA.end();
        ArduinoOTA.setHostname(HOST_NAME); // Equals to the parameter in function MDNS.begin(HOST_NAME)
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
            IPAddress STAIP = WiFi.localIP();
            Serial.println("WiFi connected");
            Serial.printf(" - SSID: %s\r\n", WiFi.SSID());
            Serial.print(" - IP: ");
            Serial.println(STAIP);
            ArduinoOTA.end();
            ArduinoOTA.setHostname(HOST_NAME); // Equals to the parameter in function MDNS.begin(HOST_NAME)
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

void TFTUpdateTask(void * pvParameters)
{
    int line = 1;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        tft.printf("%u, ID: %d, Data: ", packNum, rx_frame.MsgID);
        for (uint8_t i = 0; i < rx_frame.FIR.B.DLC; i++)
        {
            tft.printf("%02x ", rx_frame.data.u8[i]);
        }
        tft.println();
        line++;
        if (line >= 15)
        {
            tft.fillScreen(TFT_BLACK);
            tft.setCursor(0, 0);
            line = 0;
        }
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
                    case 0x12:
                    DeviceNotify[0] = millis();
                    break;

                    case 0x13:
                    DeviceNotify[1] = millis();
                    break;
                }
            }
            xTaskNotifyGive(TFTUpdateTaskHandle); // TEST now, print CAN Bus information on the screen
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
        //delay(1);
    }
    vTaskDelete(NULL);
}

void AudioTask(void * pvParameters)
{
    /*
    Audio File List

    001启动音乐.mp3
    002提示音.mp3
    003设备已联网.mp3
    004机器自检已启动.mp3
    005自检通过.mp3
    006自检未通过.mp3
    007底盘主控掉线.mp3
    008云台主控掉线.mp3
    009陀螺仪与码盘掉线.mp3
    010一.mp3
    011二.mp3
    012三.mp3
    013四.mp3
    014号电机掉线.mp3
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

void TouchscreenTask(void * pvParameters)
{
    uint16_t x = 0, y = 0; // To store the touch coordinates
    boolean pressed;
    while (1)
    {
        pressed = tft.getTouch(&x, &y); 
        if (pressed)
        {
            Serial.printf("Touch, x=%u, y=%u\r\n", x, y);
            tft.fillCircle(x, y, 2, TFT_WHITE);
            if (RobotSelftestTaskHandle == NULL)
            {
                Serial.println("Robot Selftest task began");
                xTaskCreate(RobotSelftestTask, "Robot Selftest Control", 2048, NULL, 6, &RobotSelftestTaskHandle);
            }
        }
        delay(25); // Assumes that the pressing duration must be longer than this
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
    uint16_t SelftestBegin = millis();
    uint16_t SelftestEnd = millis() + 1E4; // Set robot selftest timeout to 10 seconds
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
    delay(5000); // Wait for a short time
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
