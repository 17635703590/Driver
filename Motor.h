#ifndef MOTOR_H
#define MOTOR_H
#include "Arduino.h"
#include "src/ESP32-PWM/ESP32_Pwm.h"

#include <EEPROM.h>
#include <SPI.h>

#define HSPI_MISO   18
#define HSPI_MOSI   23
#define HSPI_SCLK   5
#define HSPI_SS     4


#define EEPROM_SIZE 4096
#define ENCODER_CPR_2 1024
#define ENCODER_CPR 2048

#ifndef REG2RAD
#define REG2RAD 0.017453292
#endif
//#define CURRENT2PWM 0.7575757576 //0.25 Ω
#define CURRENT2PWM 0.3030303030 //0.1 Ω

#define UMAXCL 0.4
#define LPFA  0.9630 
#define LPFB  0.0370

class Motor
{
    public: 
    Motor(){
        Motor(27, 14, 12, 25, 26, 33);
      //   int A_BIN1 = 12;
      //   int A_BIN2 = 13;
      //   int A_Pin_PWM = 15;
        
      //   int B_BIN1 = 2;
      //   int B_BIN2 = 4;
      //   int B_Pin_PWM = 14;

      //   pinMode(A_BIN1, OUTPUT);
      //   pinMode(A_BIN2, OUTPUT);
      //   pinMode(B_BIN1, OUTPUT);
      //   pinMode(B_BIN2, OUTPUT);
      //   A_PWM = new ESP32_Pwm(A_Pin_PWM, 0, 50000, 16);   
      //   B_PWM = new ESP32_Pwm(B_Pin_PWM, 1, 50000, 16);
      //   if (!EEPROM.begin(EEPROM_SIZE))
      //   {
      //       Serial.println("failed to initialise EEPROM"); 
      //       delay(1000000);
      //   }

      // hspi1 = new SPIClass(HSPI);
      // //hspi->begin();
      // hspi1->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS
      // pinMode(HSPI_SS, OUTPUT); //HSPI SS

    };
    Motor(int A_bin1, int A_bin2, int A_pin_pwm, int B_bin1, int B_bin2, int B_pin_pwm){
        int A_BIN1 = A_bin1;
        int A_BIN2 = A_bin2;
        int A_Pin_PWM = A_pin_pwm;

        int B_BIN1 = B_bin1;
        int B_BIN2 = B_bin2;
        int B_Pin_PWM = B_pin_pwm;

      //   pinMode(A_BIN1, OUTPUT);
      //   pinMode(A_BIN2, OUTPUT);
      //   pinMode(B_BIN1, OUTPUT);
      //   pinMode(B_BIN2, OUTPUT);

        A_PWM = new ESP32_Pwm(A_Pin_PWM, 0, 20000, 10);   
        B_PWM = new ESP32_Pwm(B_Pin_PWM, 1, 20000, 10);

        A_BIN1_PWM = new ESP32_Pwm(A_BIN1, 2, 20000, 10);   
        A_BIN2_PWM = new ESP32_Pwm(A_BIN2, 3, 20000, 10);
        B_BIN1_PWM = new ESP32_Pwm(B_BIN1, 4, 20000, 10);   
        B_BIN2_PWM = new ESP32_Pwm(B_BIN2, 5, 20000, 10);




      hspi1 = new SPIClass(HSPI);
      //hspi->begin();
      hspi1->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS
      pinMode(HSPI_SS, OUTPUT); //HSPI SS               
    };

    void CurrentOutput(float theta, float current) {	
		float v_coil_A;
		float v_coil_B;

		v_coil_A = current * CURRENT2PWM * sin(-50 * theta * REG2RAD);
		v_coil_B = current * CURRENT2PWM * cos(-50 * theta * REG2RAD);

		if(v_coil_A >= 0)  {
                  A_PWM->SetDutyCycle(v_coil_A);
                  A_BIN1_PWM->SetDutyCycle(1); 
                  A_BIN2_PWM->SetDutyCycle(0);
                  // digitalWrite(A_BIN1, HIGH);
                  // digitalWrite(A_BIN2, LOW); 
		}else {
                  A_PWM->SetDutyCycle(-v_coil_A); 
                  A_BIN1_PWM->SetDutyCycle(0); 
                  A_BIN2_PWM->SetDutyCycle(1);  
                  // digitalWrite(A_BIN1, LOW);
                  // digitalWrite(A_BIN2, HIGH);
		} 
		if(v_coil_B >= 0)  {
                  B_PWM->SetDutyCycle(v_coil_B); 
                  B_BIN1_PWM->SetDutyCycle(1); 
                  B_BIN2_PWM->SetDutyCycle(0);
                  // digitalWrite(B_BIN1, HIGH);
                  // digitalWrite(B_BIN2, LOW); 
		}else {
                  B_PWM->SetDutyCycle(-v_coil_B); 
                  B_BIN1_PWM->SetDutyCycle(0); 
                  B_BIN2_PWM->SetDutyCycle(1);  
                  // digitalWrite(B_BIN1, LOW);
                  // digitalWrite(B_BIN2, HIGH);
		} 
    } 

      int spiClk = 10000000; 
      SPIClass *hspi1 = NULL;

    int A_Pin_PWM = 15;
    int A_BIN1 = 12;
    int A_BIN2 = 13;
    ESP32_Pwm *A_PWM;

    int B_Pin_PWM = 14;
    int B_BIN1 = 2;
    int B_BIN2 = 4;
    ESP32_Pwm *B_PWM;

      ESP32_Pwm *A_BIN1_PWM;
      ESP32_Pwm *A_BIN2_PWM;
      ESP32_Pwm *B_BIN1_PWM;
      ESP32_Pwm *B_BIN2_PWM;
    

    enum {OPEN_LOOP, CLOSE_LOOP} Mode;
    float Target, Last_Target;
    int hccount;
    float OneStepAngle;
    float ActualAngle;
    uint16_t Angle, Last_Angle;

    float Error, Last_Error, Result, iterm, dterm, ki, kp, kd;

    void ForwardOneStep(){
      this->Target += this->OneStepAngle;
    }
    void BackwardOneStep(){
      this->Target -= this->OneStepAngle;
    }

    uint16_t ReadAngle(){
      word angle_register = 0x7fff | 0x8000;
      hspi1->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
      digitalWrite(HSPI_SS, LOW);
      word ret = hspi1->transfer16(angle_register);
      digitalWrite(HSPI_SS, HIGH);
      delayMicroseconds(50);
      digitalWrite(HSPI_SS, LOW);
      ret = hspi1->transfer16(angle_register);
      digitalWrite(HSPI_SS, HIGH);
      hspi1->endTransaction();

      ret = ret & 0x3fff;
      ret = ret >> 3;
      return ret;
    };
    uint16_t GetCalibrateEncoder(){
      uint16_t ang = ReadAngle();
      return EEPROM.readUShort(2*ang);
    }
    void UpdateEncoder(){
            this->Last_Angle = this->Angle;
            this->Angle = this->GetCalibrateEncoder();
            int d_angle = this->Angle - this->Last_Angle;

            if(d_angle > ENCODER_CPR_2){
                  this->ActualAngle += ((float)(d_angle - ENCODER_CPR)/ENCODER_CPR * 360.0); 
            }else if(d_angle < -ENCODER_CPR_2){
                  this->ActualAngle += ((float)(d_angle + ENCODER_CPR)/ENCODER_CPR * 360.0); 
            }else {
                  this->ActualAngle += ((float)d_angle/ENCODER_CPR * 360.0);
            }
    }
    //编码器校准方法参考了这个工程https://oshwhub.com/HyperCNC/yi-ti-hua-di-cheng-ben-gao-su-gao-jing-du-quan-bi-huan-bu-jin-dian-ji
    //注意： 如果校准过程中电机运行不正常，会校准失败，重新校准即可
      void CalibrateEncoder(void) {   
            int32_t encoderReading = 0;    
            int32_t currentencoderReading = 0;
            int32_t lastencoderReading = 0;        

            int32_t iStart = 0;     //encoder zero position index
            int32_t jStart = 0;
            int32_t stepNo = 0;

            int32_t fullStepReadings[200];
            int32_t ticks = 0;	
            uint32_t address = 0;

            uint16_t lookupAngle;

            Serial.println("Beginning calibration Encoder...\r\n");
            
            this->OneStepAngle = 1.8;
            this->Mode = Motor::OPEN_LOOP;
            this->Target = 0;
            vTaskDelay(2000);   
            for(uint16_t x=0; x<200; x++) {    
                  encoderReading = 0;             
                  lastencoderReading = ReadAngle();     
                  for(uint8_t reading = 0; reading<10; reading++) { 
                        vTaskDelay(5);
                        currentencoderReading = ReadAngle(); 
                        if(currentencoderReading - lastencoderReading < -ENCODER_CPR_2)
                              currentencoderReading += ENCODER_CPR;
                        else if(currentencoderReading - lastencoderReading > ENCODER_CPR_2)
                              currentencoderReading -= ENCODER_CPR;
                        encoderReading += currentencoderReading;
                        lastencoderReading = currentencoderReading;
                  }
                  encoderReading = encoderReading / 10;
                  if(encoderReading > ENCODER_CPR)
                        encoderReading -= ENCODER_CPR;
                  else if(encoderReading < 0)
                        encoderReading += ENCODER_CPR;
                  
                  fullStepReadings[x] = encoderReading;  

                  this->ForwardOneStep();
                  vTaskDelay(400); 
            }
            Serial.println("The ticks:\r\n");
            for(uint8_t i=0; i<200; i++) {
                  ticks = fullStepReadings[(i + 1) % 200] - fullStepReadings[i % 200];
                  if(ticks < -ENCODER_CPR_2) 
                        ticks	+= ENCODER_CPR;
                  else if(ticks > ENCODER_CPR_2)	
                        ticks -= ENCODER_CPR;	
                  for(int32_t j=0; j<ticks; j++) {
                        stepNo = (fullStepReadings[i] + j) % ENCODER_CPR;
                        if(stepNo == 0) {
                              iStart = i;
                              jStart = j; //找到编码器零点所在的位置，第i组的第j个数据
                        }
                  }
            }
            Serial.print("istart: ");Serial.println(iStart);
            Serial.print("jstart: ");Serial.println(jStart);
            Serial.println("The lookup data:\r\n");
            for(int32_t i = iStart; i<(iStart + 200 + 1); i++) {
                  ticks = fullStepReadings[(i+1) % 200] - fullStepReadings[i % 200];
                  if(ticks < -ENCODER_CPR_2) 
                        ticks += ENCODER_CPR; 
                  else if(ticks > ENCODER_CPR_2)	
                        ticks -= ENCODER_CPR;
                  Serial.printf("tick: %d \n", ticks);
                  if(i == iStart) { 
                        for(int32_t j=jStart; j<ticks; j++) {
                              lookupAngle = (ENCODER_CPR_2 * i + ENCODER_CPR_2 * j / ticks) % 204800 / 100;
                              EEPROM.writeUShort(address, (uint16_t)lookupAngle);
                              //Serial.printf("addr: %d : %d %d %d\n", address, lookupAngle,i,j);                             
                              address += 2;
                        }
                  }
                  else if(i == (iStart + 200)) { 
                        for(int32_t j=0; j<jStart; j++) {
                              lookupAngle=((ENCODER_CPR_2 * i + ENCODER_CPR_2 * j / ticks) % 204800) / 100;
                              EEPROM.writeUShort(address, (uint16_t)lookupAngle);
                              //Serial.printf("addr: %d : %d %d %d\n", address, lookupAngle,i,j);   
                              address += 2;
                        }
                  }
                  else{                        //this is the general case
                        for(int32_t j=0; j<ticks; j++) {
                              lookupAngle = ((ENCODER_CPR_2 * i + ENCODER_CPR_2 * j / ticks) % 204800) / 100;
                              EEPROM.writeUShort(address, (uint16_t)lookupAngle);
                              //Serial.printf("addr: %d : %d %d %d\n", address, lookupAngle,i,j);   
                              address += 2;
                        }
                  }
            }
            Serial.printf("addr: %d \n", address);
            EEPROM.commit();
      }
};

#endif