#include <Arduino.h>
#include "WiFi.h"
#include "PubSubClient.h"
#include "Ticker.h"
#include "src/General-Function/General_Function.h"
#include "Motor.h"
#include <stdlib.h>
const char *mqtt_server = "39.101.179.153"; //IP地址
#define mqtt_pubid "esp32_Demo"     //客户端ID
#define mqtt_devid "sf001"  //设备ID
#define mqtt_username  ""  //EMQTT用户名称
#define mqtt_password  "" //EMQTT密码
String  ctrl_topic;
String  status_topic;
 
WiFiClient espClient;           //创建一个WIFI连接客户端
PubSubClient client(espClient); // 创建一个PubSub客户端
 
String Lock_Status = "close";
extern Motor mot;

void callback(char *topic, byte *payload, unsigned int length){
    char pay[length + 1];
    memcpy(pay, payload, length);
    pay[length] = '\0';
    String msg(pay); /** 接收到的消息转成字符串方便处理*/

    // Serial.println(str);
    // Serial.println(str.length());
    Serial.println("message rev:");
    Serial.println(topic);
    Serial.println(msg);

    if(!strcmp(topic, ctrl_topic.c_str())) {
        String str1 = stringSeparate(msg, "_", 2);
        if(!strcmp(str1.c_str(), "up")){
            Lock_Status = "up";
            String devid(mqtt_devid);
            String msg;
            msg = devid + "_" + Lock_Status;
            //client.publish(status_topic.c_str(), msg.c_str(), qos=0, retain=False); //发送数据到主题
            mot.Target += 1000;
        }else if(!strcmp(str1.c_str(), "down")){
            Lock_Status = "down";
            String devid(mqtt_devid);
            String msg;
            msg = devid + "_" + Lock_Status;
            //client.publish(status_topic.c_str(), msg.c_str(), qos=0, retain=False); //发送数据到主题
            mot.Target -= 1000;
        }
    }
}
 
void clientReconnect(){
    if(WiFi.isConnected()){
        if (!client.connected()) { //再重连客户端
            Serial.println("reconnect MQTT...");
            if (client.connect(mqtt_pubid, mqtt_username, mqtt_password)){
                Serial.println("MQTT connected");
                client.subscribe(ctrl_topic.c_str());
            }else{
                //Serial.println("failed");
                //Serial.println(client.state());
                vTaskDelay(5000);
            }
        }
    } 
}

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
WebServer server(80);
void handleRoot() {
    server.send(200, "text/plain", "Stepper motor web server \nCurrent angle:" + String(mot.ActualAngle));
}

#include "src/U8g2/src/U8g2lib.h"
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 10, /* data=*/ 9); 

void taskApp(void* parameter){
    String str1(mqtt_devid);
    String str2("_ctrl");
    String str3("_status");
    ctrl_topic = str1 + str2;
    status_topic = str1 + str3; /** 配置mqtt通信的话题名称 设备id+类型*/

    client.setServer(mqtt_server, 1883);    //设置客户端连接的服务器, 使用1883端口
    client.connect(mqtt_pubid, mqtt_username, mqtt_password);
    client.subscribe(ctrl_topic.c_str());
    client.setCallback(callback);                          //设置好客户端收到信息是的回调
    vTaskDelay(200);



    //web

    int cnt = 0;
    server.on("/", handleRoot);
    server.on("/inline", []() {
        server.send(200, "text/plain", "this works as well");
    });
    server.begin();

    u8g2.begin();

    while(1){
        /** MQTT */
        vTaskDelay(10);
        if(!client.connected()) //如果客户端没连接 重新连接
            clientReconnect();
        else client.loop(); //客户端循环检测
        
        if(cnt++ % 300 == 0){
              if (client.connected()){
                  String devid(mqtt_devid);
                  String msg;
                  msg = devid + "_" + Lock_Status;
                  client.publish(status_topic.c_str(), msg.c_str()); //发送数据到主题
              }
        }

        /** web */
        server.handleClient();
        server.send(200, "text/plain", "Stepper motor web server \nCurrent angle:" + String(mot.ActualAngle));

        /** OLED */
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_wqy14_t_gb2312);
        u8g2.setCursor(0, 15);
        if(WiFi.status() == WL_CONNECTED)
         u8g2.print("IP: " + WiFi.localIP().toString());
        else u8g2.print("DISCONNECTED");
        u8g2.setCursor(0, 30);
        u8g2.print("CURR: " + String(mot.ActualAngle));
        u8g2.setCursor(0, 45);
        u8g2.print("TARG: " + String(mot.Target));
        u8g2.setCursor(0, 60);
        u8g2.print("ERRO: " + String(mot.Error));
        u8g2.sendBuffer(); 
    }
}

//can接口的实现，暂时没有使用
#include "src/ESP32-CAN/src/ESP32CAN.h"
#include "src/ESP32-CAN/src/CAN_config.h"

CAN_device_t CAN_cfg;               // CAN Config
const int rx_queue_size = 10;       // Receive Queue size
void taskCan(void* parameter){
    CAN_cfg.speed = CAN_SPEED_1000KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_13;
    CAN_cfg.rx_pin_id = GPIO_NUM_15;
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
    // Init CAN Module
    ESP32Can.CANInit();

    while(1){
        /** can */
        vTaskDelay(10);
        CAN_frame_t rx_frame;

        // Receive next CAN frame from queue
        if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

            if (rx_frame.FIR.B.FF == CAN_frame_std) {
                printf("New standard frame");
            }
            else {
                printf("New extended frame");
            }

            if (rx_frame.FIR.B.RTR == CAN_RTR) {
                printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
            }
            else {
                printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
                for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
                    printf("0x%02X ", rx_frame.data.u8[i]);
                }
                printf("\n");
            }
        }
        // Send CAN Message
        // CAN_frame_t tx_frame;
        // tx_frame.FIR.B.FF = CAN_frame_std;
        // tx_frame.MsgID = 0x001;
        // tx_frame.FIR.B.DLC = 8;
        // tx_frame.data.u8[0] = 0x00;
        // tx_frame.data.u8[1] = 0x01;
        // tx_frame.data.u8[2] = 0x02;
        // tx_frame.data.u8[3] = 0x03;
        // tx_frame.data.u8[4] = 0x04;
        // tx_frame.data.u8[5] = 0x05;
        // tx_frame.data.u8[6] = 0x06;
        // tx_frame.data.u8[7] = 0x07;
        // ESP32Can.CANWriteFrame(&tx_frame);
        
    }
}  
