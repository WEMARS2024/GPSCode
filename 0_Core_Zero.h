/*
WE MARS
2023 07 24
  
\Core 0 code


*/


#ifndef CORE_ZERO_H
#define CORE_ZERO_H 1


#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

CAN_device_t CAN_cfg;               // CAN Config
CAN_frame_t rx_frame;
CAN_frame_t tx_frame;
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size

TaskHandle_t Core_Zero;

const int CR0_ciMainTimer =  10000;

unsigned char CR0_ucMainTimerCaseCore0;

uint32_t CR0_u32Now;  //for timing testing
uint32_t CR0_u32Last;


unsigned long CR0_ulPreviousMicrosCore0;
unsigned long CR0_ulCurrentMicrosCore0;

unsigned int CR0_uiTxFlags = 0;

void Core_ZeroCode( void * pvParameters );


void Core_ZEROInit()
{
   xTaskCreatePinnedToCore(
                    Core_ZeroCode,   /* Task function. */
                    "Core_Zero",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Core_Zero,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 
};


void Core_ZeroCode( void * pvParameters )
{
  Serial.print("Core - ");
  Serial.print(xPortGetCoreID());
  Serial.println("   running ");


  //Core 0 Setup
  //-------------------------------------------------------------------------------------------
  CAN_cfg.speed = CAN_SPEED_125KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_21;
  CAN_cfg.rx_pin_id = GPIO_NUM_22;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
   
   
  //loop function for core 0
  //-------------------------------------------------------------------------------------------
  for(;;)
  {
       unsigned long currentMillis = millis();

      // Receive next CAN frame from queue
      if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
       {

        
        switch(rx_frame.MsgID)
        {
          case 0:
          {
            CR0_uiTxFlags = 0;
            break;
          }
          case 100://requesting GPS data
          {
            CR0_uiTxFlags |= 0x01; //Send GPS data
            // if (rx_frame.FIR.B.RTR != CAN_RTR)
            // {
            //  for (int i = 0; i < rx_frame.FIR.B.DLC; i++) 
            //   {
            //     printf("0x%02X ", rx_frame.data.u8[i]);
            //   }
            //   printf("\n");
            // }
          break;
          }
          case 101:  //requesting IMU data 
          {
            CR0_uiTxFlags |= 0x02; //Send IMU data
            // if (rx_frame.FIR.B.RTR != CAN_RTR)
            // {
            //  for (int i = 0; i < rx_frame.FIR.B.DLC; i++) 
            //   {
            //     printf("0x%02X ", rx_frame.data.u8[i]);
            //   }
            //   printf("\n");
            // }
          break;
          }
        }
      }
      
      // Send CAN Message
      CR0_ulCurrentMicrosCore0 = micros();
      if ((CR0_ulCurrentMicrosCore0 - CR0_ulPreviousMicrosCore0) >= CR0_ciMainTimer)
      {
        
        
        
        ESP32Can.CANWriteFrame(&tx_frame);
       
      }
       
     
           // asm volatile("esync; rsr %0,ccount":"=a" (CR0_u32Last)); // @ 240mHz clock each tick is ~4nS  
            
          //  asm volatile("esync; rsr %0,ccount":"=a" (CR0_u32Now));    
           
          
        
      
  }
}

void LoadTxBuffer()
{
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = uiID;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0x09;
  tx_frame.data.u8[1] = 0x0A;
  tx_frame.data.u8[2] = 0x0B;
  tx_frame.data.u8[3] = 0x0C;
  tx_frame.data.u8[4] = 0x0D;
  tx_frame.data.u8[5] = 0x0E;
  tx_frame.data.u8[6] = 0x0F;
  tx_frame.data.u8[7] = 0x010;

}

#endif
