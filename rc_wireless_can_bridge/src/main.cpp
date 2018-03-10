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

                    ===== UESTC Wireless CAN Brigde For ABU Robocon 2018 =====
                              Copyright (c) 2018 HsuRY <i@hsury.com>

                                        VERSION 2018/02/27

*/

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32_CAN.h>
#include <rom/crc.h>
#include <WiFi.h>

#define AUTO
//#define MANUAL

const char*     SSID = "UESTC_RC_2018";
const char* PASSWORD = "2018_RC_UESTC";
IPAddress    GATEWAY = (192, 168, 39, 1);
uint16_t        PORT = 2333;

#define BUZZER_PIN GPIO_NUM_32
#define CAN_TX_PIN GPIO_NUM_4
#define CAN_RX_PIN GPIO_NUM_5
#define CAN_FILTER_LOWER 0x20
#define CAN_FILTER_UPPER 0x30

#ifdef AUTO
WiFiServer server(PORT);
#endif
WiFiClient client;

extern CAN_device_t CAN_cfg;
CAN_FRAME CAN_Tx_Frame;
CAN_FRAME CAN_Rx_Frame;

uint32_t LastPackArrivedTS = 0;
uint32_t LastPackSentTS = 0;
bool NetworkStatus = false;

#ifdef MANUAL
TaskHandle_t WiFiGuardTaskHandle;
void WiFiGuardTask(void * pvParameters);
#endif
TaskHandle_t BeepTaskHandle;
void BeepTask(void * pvParameters);

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("System started");
    CAN_cfg.tx_pin_id = CAN_TX_PIN;
    CAN_cfg.rx_pin_id = CAN_RX_PIN;
    CAN.init(CAN_BPS_1000K);
    CAN.watchForRange(CAN_FILTER_LOWER, CAN_FILTER_UPPER);
    xTaskCreate(BeepTask, "Beep Task", 2048, NULL, 5, &BeepTaskHandle);
    #ifdef AUTO
    Serial.println("Identity: Auto");
    WiFi.softAP(SSID, PASSWORD); // , 1, true
    WiFi.softAPConfig(GATEWAY, GATEWAY, IPAddress(255, 255, 255, 0));
    xTaskNotify(BeepTaskHandle, 2, eSetValueWithOverwrite);
    NetworkStatus = true;
    server.begin();
    #elif defined MANUAL
    Serial.println("Identity: Manual");
    WiFi.begin(SSID, PASSWORD);
    NetworkStatus = false;
    xTaskNotify(BeepTaskHandle, 1, eSetValueWithOverwrite);
    xTaskCreate(WiFiGuardTask, "WiFi Guard Task", 2048, NULL, 4, &WiFiGuardTaskHandle);
    #endif
}

void loop() {
    StaticJsonBuffer<512> jsonBuffer;
    if (NetworkStatus)
    {
        if (LastPackArrivedTS == 0 || millis() - LastPackArrivedTS > 2500) // 网络可以，但是超时或者刚连上，重新建立连接
        {
            if (LastPackArrivedTS)
            {
                Serial.println("Partner is offline");
                xTaskNotify(BeepTaskHandle, 2, eSetValueWithOverwrite);
                LastPackArrivedTS = 0;
            }
            client.stop();
            #ifdef AUTO
            client = server.available();
            #elif defined MANUAL
            client.connect(GATEWAY, PORT);
            #endif
            LastPackSentTS = 0;
        }
        #ifdef AUTO
        if (client && client.connected())
        #elif defined MANUAL
        if (client.connected())
        #endif
        {
            if (LastPackSentTS == 0) // 刚建立连接，从当前开始计时
            {
                Serial.println("Partner is online");
                xTaskNotify(BeepTaskHandle, 3, eSetValueWithOverwrite);
                LastPackArrivedTS = millis();
            }
            if (LastPackSentTS == 0 || millis() - LastPackSentTS > 250) // 该发心跳包了
            {
                JsonObject& root = jsonBuffer.createObject();
                root["heart"] = 0x39;
                root.printTo(client);
                LastPackSentTS = millis();
            }
            if (client.available()) // 接受对方发来的消息，转发到CAN总线
            {
                JsonObject& root = jsonBuffer.parseObject(client);
                if (root.success())
                {
                    if (root.containsKey("heart") && root["heart"] == 0x39) LastPackArrivedTS = millis(); // 收到心跳包
                    else if(root.containsKey("id") && root.containsKey("dlc") && root.containsKey("data") && root.containsKey("crc")) // 收到数据包
                    {
                        #ifdef AUTO
                        Serial.print(F("A <-- M "));
                        #elif defined MANUAL
                        Serial.print(F("A --> M "));
                        #endif
                        CAN_Tx_Frame.id = root["id"];
                        Serial.printf("%08x ", CAN_Tx_Frame.id);
                        CAN_Tx_Frame.length = root["dlc"];
                        for (uint8_t i = 0; i < CAN_Tx_Frame.length; i++)
                        {
                            CAN_Tx_Frame.data.byte[i] = root["data"][i];
                            Serial.printf("%02x ", CAN_Tx_Frame.data.byte[i]);
                        }
                        uint16_t crc = crc16_be(0, (uint8_t *)&CAN_Tx_Frame.id, 4);
                        crc = crc16_be(crc, &CAN_Tx_Frame.length, 1);
                        crc = crc16_be(crc, (uint8_t *)CAN_Tx_Frame.data.byte, CAN_Tx_Frame.length);
                        if (crc == root["crc"]) // CRC校验通过
                        {
                            Serial.println();
                            CAN.sendFrame(CAN_Tx_Frame);
                            LastPackArrivedTS = millis();
                        }
                        else Serial.println("CRC mismatched");
                    }
                    else Serial.println("Unknown package");
                }
            }
            for (uint16_t i = 0; i < CAN.available(); i++) // CAN总线上收到消息，转发给对方
            {
                CAN.get_rx_buff(CAN_Rx_Frame);
                #ifdef AUTO
                Serial.print(F("A --> M "));
                #elif defined MANUAL
                Serial.print(F("A <-- M "));
                #endif
                JsonObject& root = jsonBuffer.createObject();
                root["id"] = CAN_Rx_Frame.id;
                Serial.printf("%08x ", CAN_Rx_Frame.id);
                root["dlc"] = CAN_Rx_Frame.length;
                JsonArray& dat = root.createNestedArray("data");
                for (uint8_t j = 0; j < CAN_Rx_Frame.length; j++)
                {
                    dat.add(CAN_Rx_Frame.data.byte[j]);
                    Serial.printf("%02x ", CAN_Rx_Frame.data.byte[i]);
                }
                Serial.println();
                uint16_t crc = crc16_be(0, (uint8_t *)&CAN_Rx_Frame.id, 4);
                crc = crc16_be(crc, &CAN_Rx_Frame.length, 1);
                crc = crc16_be(crc, (uint8_t *)CAN_Rx_Frame.data.byte, CAN_Rx_Frame.length);
                root["crc"] = crc;
                root.printTo(client);
                LastPackSentTS = millis();
            }
        }
        else // 客户端掉线，到达时间戳清零
        {
            if (LastPackArrivedTS)
            {
                Serial.println("Partner is offline");
                xTaskNotify(BeepTaskHandle, 2, eSetValueWithOverwrite);
                LastPackArrivedTS = 0;
            }
        }
    }
}

#ifdef MANUAL
void WiFiGuardTask(void * pvParameters)
{
    uint8_t Retry = 0;
    while (1)
    {
        delay(500);
        if (NetworkStatus != WiFi.isConnected())
        {
            NetworkStatus = WiFi.isConnected();
            if (NetworkStatus) // 网络刚连接上
            {
                Serial.println("WiFi connected");
                xTaskNotify(BeepTaskHandle, 2, eSetValueWithOverwrite);
                LastPackArrivedTS = 0;
            }
            else
            {
                Serial.println("WiFi disconnected");
                ESP.restart();
            }
        }
        if (!NetworkStatus) Retry++;
        else Retry = 0;
        if (Retry >= 20)
        {
            Serial.println("WiFi timeout");
            ESP.restart();
        }
    }
}
#endif

void BeepTask(void * pvParameters)
{
    struct
    {
        uint16_t On = 0;
        uint16_t Off = 0;
        uint8_t Repeat = 0;
        uint16_t Freeze = 0;
    }
    BeepCfg;
    uint8_t SM = 0;
    uint32_t TS = millis();
    pinMode(BUZZER_PIN, OUTPUT);
    while (1)
    {
        uint8_t TaskNotify = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        if (TaskNotify && SM != TaskNotify)
        {
            SM = TaskNotify;
            switch (SM)
            {
                case 1: // WiFi未连接
                BeepCfg.On = 250;
                BeepCfg.Off = 750;
                BeepCfg.Repeat = 1;
                BeepCfg.Freeze = 0;
                break;

                case 2: // WiFi已连接，心跳包超时
                BeepCfg.On = 250;
                BeepCfg.Off = 250;
                BeepCfg.Repeat = 3;
                BeepCfg.Freeze = 500;
                break;

                case 3: // WiFi已连接，心跳包正常
                BeepCfg.On = 250;
                BeepCfg.Off = 2250;
                BeepCfg.Repeat = 1;
                BeepCfg.Freeze = 0;
                break;

                case 4: // 请勿打扰
                default:
                BeepCfg.On = 0;
                BeepCfg.Off = 0;
                BeepCfg.Repeat = 0;
                BeepCfg.Freeze = 0;
                break;
            }
            TS = millis();
        }
        uint16_t SmallPeriod = BeepCfg.On + BeepCfg.Off;
        uint32_t BigPeriod = SmallPeriod * BeepCfg.Repeat + BeepCfg.Freeze;
        if (SmallPeriod && BigPeriod &&
            (millis() - TS) % BigPeriod < BigPeriod - BeepCfg.Freeze &&
            (millis() - TS) % BigPeriod % SmallPeriod < BeepCfg.On); //digitalWrite(BUZZER_PIN, HIGH);
        else digitalWrite(BUZZER_PIN, LOW);
    }
}
