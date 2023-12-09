#include <Arduino.h>
#include "WiFi.h"
#include "Controller.h"
#include <SPI.h>

extern void taskApp(void* parameter);
extern void taskCan(void* parameter);

const char *ssid = "TP-LINK_DC0B"; //wifi
const char *password = "123qwe456asd";


void taskWiFi(void* parameter){
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    while(1){
        vTaskDelay(1000);
        if ((WiFi.status() != WL_CONNECTED)) {
            WiFi.disconnect();
            WiFi.reconnect();
            //Serial.printf("wifi status: WL_DISCONNECTED \n"); 
        }//else Serial.println(WiFi.localIP());         
    }
}

void setup() { 
    Serial.begin(115200); 

    delay(100);

    xTaskCreatePinnedToCore(taskWiFi,          /* Task function. */
            "taskWiFi",        /* String with name of task. */
            3000,            /* Stack size in bytes. */
            NULL,             /* Parameter passed as input of the task */
            2,                /* Priority of the task. */
            NULL, 0);         /* Task handle. */  
    xTaskCreatePinnedToCore(taskApp,          /* Task function. */
            "taskApp",        /* String with name of task. */
            10000,            /* Stack size in bytes. */
            NULL,             /* Parameter passed as input of the task */
            1,                /* Priority of the task. */
            NULL, 0);         /* Task handle. */ 
    xTaskCreatePinnedToCore(taskCan,          /* Task function. */
            "taskCan",        /* String with name of task. */
            3000,            /* Stack size in bytes. */
            NULL,             /* Parameter passed as input of the task */
            3,                /* Priority of the task. */
            NULL, 0);         /* Task handle. */  



    xTaskCreatePinnedToCore(taskRun,          /* Task function. */
            "taskRun",        /* String with name of task. */
            10000,            /* Stack size in bytes. */
            NULL,             /* Parameter passed as input of the task */
            1,                /* Priority of the task. */
            NULL, 1);         /* Task handle. */  
    xTaskCreatePinnedToCore(taskController,          /* Task function. */
            "taskController",        /* String with name of task. */
            10000,            /* Stack size in bytes. */
            NULL,             /* Parameter passed as input of the task */
            2,                /* Priority of the task. */
            NULL, 1);         /* Task handle. */  
              
} 
void loop() { // run over and over again 

} 
