#include <Arduino.h>
#include "WiFi.h"
#include "PubSubClient.h"
#include "Ticker.h"
#include "src/General-Function/General_Function.h"
#include "src/ESP32-PWM/ESP32_Pwm.h"
#include "Motor.h"

int t1, t2;
Motor mot(27, 14, 12, 25, 26, 33);
//Motor mot;
void taskRun(void* parameter){
    //vTaskDelay(2000);
    while(1){
        t1 = micros();
        switch(mot.Mode){
            case Motor::OPEN_LOOP:
                mot.UpdateEncoder();
                if(fabs(mot.Target - mot.Last_Target) < 0.00001f){
                    mot.hccount++;
                    if(mot.hccount >= 1000)
                        mot.hccount = 1000;
                } else mot.hccount = 0;
                if(mot.hccount >= 1000) mot.CurrentOutput(mot.Target, 0.4);
                else mot.CurrentOutput(mot.Target, 0.4);
                mot.Last_Target = mot.Target;
            break;
            case Motor::CLOSE_LOOP:
                /** 计算编码器角度（包含圈数） */
                mot.UpdateEncoder();

                mot.Error = mot.Target - mot.ActualAngle; 
                
                mot.iterm += mot.ki * mot.Error;                          
                if(mot.iterm > UMAXCL)
                    mot.iterm = UMAXCL;
                else if(mot.iterm < -UMAXCL) 
                mot.iterm = -UMAXCL;          

                mot.dterm = LPFA * mot.dterm - LPFB * mot.kd * (mot.Error - mot.Last_Error);      
                mot.Result = mot.kp * mot.Error + mot.iterm + mot.dterm; 	

                mot.Last_Error = mot.Error;

                float out ;
                if(mot.Result > 0)           
                    out = mot.ActualAngle + 1;
                else {
                    out = mot.ActualAngle - 1;
                    mot.Result = -mot.Result;
                }
                if(mot.Result > UMAXCL)   
                    mot.Result = UMAXCL;      
                mot.CurrentOutput(out, mot.Result);  
            break;
        }
        t2 = micros() - t1;
    }
}
char serial_buf[100];
void taskController(void* parameter){
    mot.kp = 0.1;
    mot.ki = 0.0001;
    mot.kd = 0;
    mot.Mode = Motor::CLOSE_LOOP;
    mot.OneStepAngle = 1.8;
    pinMode(32,INPUT);
    pinMode(34,INPUT);
    pinMode(35,INPUT);
    if (!EEPROM.begin(4096))
    {
        Serial.println("failed to initialise EEPROM"); 
        delay(1000000);
    } 
    vTaskDelay(2000);
    while(1){
        if(digitalRead(32) == 0){
            //注意： 如果校准过程中电机运行不正常，会校准失败，重新校准即可
            mot.CalibrateEncoder();
        }
        if(digitalRead(34) == 0){
            //mot.Target = 0.7 * mot.Target + 0.3*(mot.ActualAngle - 1.8);
            mot.Target = mot.ActualAngle - 100;
            //mot.ForwardOneStep();
        }
        if(digitalRead(35) == 0){
            mot.Target = mot.ActualAngle + 100;
            //mot.BackwardOneStep();
        }
        
        vTaskDelay(10);

        //这里是利用串口调参的，格式为类型+浮点数字符串+回车
        //例如： t0.1表示目标位置0.1°
        // p123.456表示参数p调整为123.456
        int len = Serial.available();
        if(len>100) len = 100;
        Serial.read(serial_buf, len);
        if(serial_buf[len-2] == 0x0d && serial_buf[len-1] == 0x0a){
            serial_buf[len-2] = '/0';
            char* p = &serial_buf[1];
            float result = atof(p);
            Serial.printf("%c : %f\n", serial_buf[0], result);
            switch(serial_buf[0]){
                case 't' : mot.Target = result; break;
                case 'p' : mot.kp = result; break;
                case 'i' : mot.ki = result; break;
                case 'd' : mot.kd = result; break;
            }
        }

    }
}


 
