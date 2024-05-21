#include "main.h"
#include "cmsis_os.h"
#include "FREERTOS_tasks.h"
#include <stdbool.h>
#include "STO_tests_V2_nanopb.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart4;
extern FDCAN_TxHeaderTypeDef BCM_CANHS_R_04,BRAKE_CANHS_R_01,DTOOL_to_AIRBAG,CLUSTER_CANHS_RNr_01;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern FDCAN_TxHeaderTypeDef TxHeader;
extern volatile uint32_t time;
extern uint8_t UARTRXcmd[128];
extern uint8_t SPI_RXbuf[4];
extern uint8_t SPI_resp[4];
extern volatile double timeX,timeY;
extern uint32_t ctr2;
extern uint32_t ctr0;
extern uint32_t TTF;
/*флаг, управляющий работой эмулятора акселерометра*/
extern volatile bool SPI_STOP_FLAG;
/*объекты структуры, используемой для приема-передачи данных о тестах*/
TestData Output=TestData_init_zero;
TestData Input=TestData_init_zero;
TestData Debug=TestData_init_zero;
/*флаги, разрешающие/запрещающие отправку соответствующих CAN-сообщений*/
volatile bool SEND_PERIODIC_MESSAGES=false;
volatile bool SEND_DOORSTATE=false;
volatile bool SEND_CLUSTER=false;
volatile bool SEND_VEHICLE_STATE=true;
/*флаги, используемые для отслеживание сигнала CD до и после столкновения*/
extern volatile bool CRASH_DETECTED_BEFORE_COLLISION_TAKEN;
extern volatile bool CRASH_DETECTED_AFTER_COLLISION_TAKEN;\
/*массивы, отправляемые по UART*/
extern uint8_t Result[256];
extern uint8_t len[1];
/*данные для тестов самодиагностики БУ*/


void vApplicationIdleHook( void )
{
  HAL_GPIO_WritePin(GPIOE,SB_BP3_CTRL_Pin,fastened);
  HAL_UART_Receive(&huart4,UARTRXcmd,sizeof(UARTRXcmd),1000);
	/*Filling in the input structure using nanopb*/
  if(UARTRXcmd[0]==0x08)
  {
   pb_istream_t RXstream = pb_istream_from_buffer(UARTRXcmd, sizeof(UARTRXcmd));
   pb_decode(&RXstream, TestData_fields, &Input);
  }

  if(Input.method==Method_SET)//0
  {
	if(Input.testNumber == 0x11)
	{
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(Init_testHandle);
	}
	else if(Input.testNumber ==  0x12)
    {
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(CAN_periodHandle);
	}
	else if(Input.testNumber ==  0x13)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(Accelerometer_periodHandle);
	}
	else if(Input.testNumber ==  0x22)
    {
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(Accelerometer_runHandle);
	}
	else if(Input.testNumber == 0x31)
	{
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS1Handle);
	}
	else if(Input.testNumber ==  0x32)
    {
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS2Handle);
	}
	else if(Input.testNumber ==  0x33)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS3Handle);
	}
    else if(Input.testNumber ==  0x34)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS4aHandle);
	}
    else if(Input.testNumber ==  0x35)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS4bHandle);
	}
    else if(Input.testNumber ==  0x3C||Input.testNumber ==  0x3D||Input.testNumber ==  0x3E||Input.testNumber ==  0x3F||Input.testNumber ==  0x302)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS11_12_13_14_16Handle);
	}
    else if(Input.testNumber ==  0x301)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS15Handle);
	}
	else if(Input.testNumber ==  0x41)
    {
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(SBR1Handle);			
	}
	else if(Input.testNumber ==  0x42)
    {
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(SBR2Handle);			
	}
	else if(Input.testNumber ==  0x43||Input.testNumber ==  0x44)
    {
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(SBR3_4Handle);			
	}
	else if(Input.testNumber ==  0x45)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(SBR5Handle);			
	}
    else if(Input.testNumber ==  0x46)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(SBR6Handle);			
	}
    else if(Input.testNumber ==  0x47)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
     xTaskNotifyGive(SBR7Handle);					
    }
    else if(Input.testNumber ==  0x51)
    {		    
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(DIAG1Handle);					
	}
    else if(Input.testNumber ==  0x52||Input.testNumber ==  0x53)
    {		    
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(DIAG2_3Handle);					
	}
    else if(Input.testNumber ==  0x61)
    {		    
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(EDRHandle);					
	}    
	 else __NOP();
  }
 else __NOP();
}
		

void Init_test_run(void *argument)
{
/*Certain implementation is given in HAL_FDCAN_RxFifo0Callback*/
  for(;;)
  {
   ulTaskNotifyTake( pdTRUE,portMAX_DELAY );
   FDCAN_ENABLE_INTERRUPTS();//ENABLE FDCAN INTERRUPT LINE0
   time=0;		
  }
 
}

void CAN_2_RUN(void *argument)
{
/*Certain implementation is given in HAL_FDCAN_RxFifo0Callback*/
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   FDCAN_ENABLE_INTERRUPTS();
   for(uint16_t j=0;j<0xFFF;j++){__NOP();}
  }
}
void Accelerometer_period_RUN(void *argument)
{
	/*Runs the accelerometer emualator, set of accelerations is hardcoded*/
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   FDCAN_ENABLE_INTERRUPTS();
   while(timeX<0)
   {		
	HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);
    #ifdef DEBUG_MODE   
    HAL_Delay(25);
    #endif    
   }
   HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
   time = 0;
   while(timeX<290&&SPI_STOP_FLAG==false)
   {
	HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
  } 
}

void Accelerometer1_RUN(void *argument)
{
  /* USER CODE BEGIN Accelerometer1_RUN */
/*Runs the accelerometer emualator, set of accelerations is depended on byte received via UART,frame forming is given in HAL_SPI_TxRxCpltCallback*/
  size_t message_length;
  /* Infinite loop */
  for(;;)
  {
   /*if(Input.AIRBAG_OFF==true)
   {
	Нажимаем кнопку отключения ПБ  
   }
   else
   {
	  __NOP();
   }*/
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   SPI_STOP_FLAG=false;
   Output.measuredValue_count++;
   FDCAN_ENABLE_INTERRUPTS();
   while(timeX<0)
   {		
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
   time = 0;
   while(timeX<290&&SPI_STOP_FLAG==false)
   {
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   Output.has_accDataNumber=Input.has_accDataNumber;
   Output.accDataNumber=Input.accDataNumber;
   pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
   pb_encode(&streamwrt, TestData_fields, &Output);
   message_length=streamwrt.bytes_written;
   len[0]=(uint8_t)message_length;
   HAL_UART_Transmit(&huart4,len,1,1000);
   HAL_UART_Transmit(&huart4,(uint8_t*)Result,message_length,1000);
   CRASH_DETECTED_BEFORE_COLLISION_TAKEN=false;
   CRASH_DETECTED_AFTER_COLLISION_TAKEN=false;
   Accelerometer_reset();
   CLEAR_OUTPUT();

  }
  /* USER CODE END Accelerometer1_RUN */
}
void Send_periodic_start(void *argument)
{
/*Задача разблокируется при запуске любого из тестов SBR/самодиагностики,отправляемые сообщения зависят от полученной команды*/
	uint8_t BCM_CANHS_R_04_data_ER[8]={0x70,0,0,0,0,0,0,0};	//engine running
	uint8_t BCM_CANHS_R_04_data_S[8]={0x00,0,0,0,0,0,0,0};	//sleeping
	uint8_t Vehicle_Speed_15kmh[8]={0x05,0xDC,0,0,0,0,0,0};
	uint8_t Vehicle_Speed_40kmh[8]={0x0F,0xA0,0,0,0,0,0,0};
	uint8_t *Speed,*VehicleState;
	uint8_t Cluster[8]={0,0x01,0,0,0,0,0,0};
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
/*------------------------Выбор позиции двери--------------------------*/
   if(SEND_DOORSTATE==true)
   {   
	 BCM_CANHS_R_04_data_ER[6]|=DRIVER_DOOR_OPEN;
   }
/*----------Выбор отправляемой скорости--------------*/   
   if(Input.has_vehicle_speed==true)
   {
     if(Input.vehicle_speed==_40KMH)
	 {
	  Speed=Vehicle_Speed_40kmh;
	 }	 
	 else if(Input.vehicle_speed==_15KMH)
	 {
	  Speed=Vehicle_Speed_15kmh;
	 }	
   }
/*---------------Выбор VehicleStateExtended---------------------------*/
   if(Input.VehicleStateExtended==Sleeping)
   {
    VehicleState=BCM_CANHS_R_04_data_S;
   }
   else if(Input.VehicleStateExtended==EngineRunning)
   {
	VehicleState=BCM_CANHS_R_04_data_ER;
   }
/*----------------------Отправка сообщений---------------------------*/
   while(SEND_PERIODIC_MESSAGES==true)
   {
     if(SEND_VEHICLE_STATE==true)
     {         
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BCM_CANHS_R_04,VehicleState); //0x350
     }
     if(Input.has_vehicle_speed==true)
     {
	  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BRAKE_CANHS_R_01,Speed); //0x5D7
     }
     if(SEND_CLUSTER==true)
     {
	  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&CLUSTER_CANHS_RNr_01,Cluster); //0x4F8
     }
     osDelay(100);
   }
  }
}
void SBR1_RUN(void *argument)
{
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_PERIODIC_MESSAGES=true;
/*------------------------------Select seatbelt--------------------------*/
   if(Input.Seatbelt_position==Driver)
   {
	SB_PORT=SB_DR_CTRL_GPIO_Port;
	SB_PIN=SB_DR_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Front_passenger)
   {
	SB_PORT=SB_FP_CTRL_GPIO_Port;
	SB_PIN=SB_FP_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerRight)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
	SB_PIN=SB_BP1_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerCenter)
   {
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerLeft)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
   }
/*----------------------------Test implementation----------------------------------*/	
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
   HAL_Delay(1000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
   HAL_Delay(1000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*--------------------------------Forming output--------------------------------------*/
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void SBR2_RUN(void *argument)
{
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_PERIODIC_MESSAGES=true;
/*------------------------------Select seatbelt--------------------------*/
   if(Input.Seatbelt_position==Driver)
   {
	SB_PORT=SB_DR_CTRL_GPIO_Port;
	SB_PIN=SB_DR_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Front_passenger)
   {
	SB_PORT=SB_FP_CTRL_GPIO_Port;
	SB_PIN=SB_FP_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerRight)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
    SB_PIN=SB_BP1_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerCenter)
   {
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerLeft)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
   }
/*------------------------Test implementation----------------------------------*/
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
   osDelay(1000);//?????			
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*--------------------------------Forming output--------------------------------------*/
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;
   Output.method=Method_GET;	
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void SBR3_4_RUN(void *argument)
{
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_PERIODIC_MESSAGES=true;
/*------------------------------Select seatbelt--------------------------*/	  
   if(Input.Seatbelt_position==Driver)
   {
	SB_PORT=SB_DR_CTRL_GPIO_Port;
	SB_PIN=SB_DR_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Front_passenger)
   {
	SB_PORT=SB_FP_CTRL_GPIO_Port;
	SB_PIN=SB_FP_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerRight)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
	SB_PIN=SB_BP1_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerCenter)
   {
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerLeft)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
   }
/*------------------------Test implementation----------------------------------*/   
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
   osDelay(1000);			
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*---------------------------Forming output-------------------------------------*/
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void SBR5_RUN(void *argument)
{	
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN,delay;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_PERIODIC_MESSAGES=true;
/*------------------------------Select seatbelt and timeout--------------------------*/	  
   if(Input.Seatbelt_position==Driver)
   {
	SB_PORT=SB_DR_CTRL_GPIO_Port;
	SB_PIN=SB_DR_CTRL_Pin;
	delay=33000;
   }
   else if(Input.Seatbelt_position==Front_passenger)
   {
	SB_PORT=SB_FP_CTRL_GPIO_Port;
	SB_PIN=SB_FP_CTRL_Pin;
	delay=33000;
   }
   else if(Input.Seatbelt_position==Rear_passengerRight) 
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
	SB_PIN=SB_BP1_CTRL_Pin;
	delay=63000;
   }
   else if(Input.Seatbelt_position==Rear_passengerCenter)
   {
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
	delay=63000;
   }
   else if(Input.Seatbelt_position==Rear_passengerLeft)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
	delay=63000;
   }
/*------------------------Test implementation----------------------------------*/   
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);	
   osDelay(delay);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*---------------------------Forming output-------------------------------------*/   
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;   
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void SBR6_RUN(void *argument)
{
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN,delay;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_PERIODIC_MESSAGES=true;
   Input.testNumber=0;
/*------------------------------Select seatbelt and timeout--------------------------*/	  
   if(Input.Seatbelt_position==Driver)
   {
	SB_PORT=SB_DR_CTRL_GPIO_Port;
	SB_PIN=SB_DR_CTRL_Pin;
	delay=33000;
   }
   else if(Input.Seatbelt_position==Front_passenger)
   {
	SB_PORT=SB_FP_CTRL_GPIO_Port;
	SB_PIN=SB_FP_CTRL_Pin;
	delay=33000;
   }
   else if(Input.Seatbelt_position==Rear_passengerRight)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
	SB_PIN=SB_BP1_CTRL_Pin;
	delay=63000;
   }
   else if(Input.Seatbelt_position==Rear_passengerCenter)
   {
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
	delay=63000;
   }
   else if(Input.Seatbelt_position==Rear_passengerLeft)
   {
    SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
	delay=63000;
   }
/*------------------------Test implementation----------------------------------*/   
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
   osDelay(delay);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);	
   osDelay(1000);
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*---------------------------Forming output-------------------------------------*/   				
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void SBR7_RUN(void *argument)
{
	uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
	GPIO_TypeDef* SB_PORT;
	uint16_t SB_PIN;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_PERIODIC_MESSAGES=true;
/*------------------------------Select seatbelt--------------------------*/	  
   if(Input.Seatbelt_position==Driver)
   {
	SB_PORT=SB_DR_CTRL_GPIO_Port;
	SB_PIN=SB_DR_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Front_passenger)
   {
	SB_PORT=SB_FP_CTRL_GPIO_Port;
    SB_PIN=SB_FP_CTRL_Pin;
   }
   else if(Input.Seatbelt_position== Rear_passengerRight)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
	SB_PIN=SB_BP1_CTRL_Pin;
   }
   else if(Input.Seatbelt_position== Rear_passengerCenter)
   {
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
   }
   else if(Input.Seatbelt_position== Rear_passengerLeft)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
    SB_PIN=SB_BP3_CTRL_Pin;
   }
/*------------------------Test implementation----------------------------------*/   
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
   SEND_DOORSTATE=true;
   osDelay(1000);	
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
/*---------------------------Forming output-------------------------------------*/
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));;			
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   SEND_DOORSTATE=false;
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void UDS1_RUN(void *argument)
{
  uint8_t ECU_reset[3]={0x02,0x11,0x01};
  uint8_t UDS_response[8]={0,};
  uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0; 	  
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,ECU_reset);
   store_CANframeTX(0,ECU_reset,sizeof(ECU_reset),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(1,UDS_response, RxHeader.DataLength>>16);
/*------------------------Forming output-------------------------------------*/		
   Send_Result();
   memset(UDS_response,0x00,sizeof(UDS_response));   
  }
}	
void UDS2_RUN(void *argument)
{
  uint8_t UDS_request[6]={0x05,0x2F,0x38,0x01,0x03,0x00};
  uint8_t UDS_response[8]={0,};
  uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_6;  
//---------------------------DIAG LED ON-------------------------------------//
   if(Input.LED==UDS_LED_DIAG_LED_ON)
   {     
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
    store_CANframeTX(0,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);    
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
       __NOP();
	}
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(1,UDS_response, RxHeader.DataLength>>16);	
   }
//-------------------------DIAG LED OFF-----------------------------------// 
   if(Input.LED==UDS_LED_DIAG_LED_OFF)
   {
    UDS_request[5]=0x01; 
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
    store_CANframeTX(0,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
	 __NOP();
	}
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(1,UDS_response, RxHeader.DataLength>>16);	
   }
//--------------------------SB LED ON-----------------------------------//
   else if(Input.LED==UDS_LED_SB_LED_ON)
   {
    UDS_request[5]=0x00;
    UDS_request[3]=0x02;		 
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
    store_CANframeTX(0,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
	 __NOP();
	}
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(1,UDS_response, RxHeader.DataLength>>16);	
   }
//----------------------------SB LED OFF--------------------------//
   else if(Input.LED==UDS_LED_SB_LED_OFF)
   {
    UDS_request[5]=0x01; 
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
    store_CANframeTX(0,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
	 __NOP();
	}
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(1,UDS_response, RxHeader.DataLength>>16);	
   }	   
//------------------------Forming output-------------------------------------//		 
	Send_Result();
    memset(UDS_response,0x00,sizeof(UDS_response));   
  }
}
void UDS3_RUN(void *argument)
{
 uint8_t UDS_request[3]={0x02,0x3E,0x00};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);	
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   store_CANframeTX(0,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(1,UDS_response, sizeof(UDS_response));	   
/*------------------------Forming output-------------------------------------*/		
   Send_Result();		 
  }
}
void UDS4a_RUN(void *argument)
{
 uint8_t UDS_request[4]={0x03,0x28,0x01,0x01};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1;
 uint32_t Put_indexF0_1,Put_indexF0_2 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   store_CANframeTX(0,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(1,UDS_response,RxHeader.DataLength>>16);
   Put_indexF0_1=FDCAN_Get_FIFO_Put_index(FIFO0);
//--------------------------Wait 10s and check new messages----------------------------------//	  
   osDelay(10000);
   Put_indexF0_2=FDCAN_Get_FIFO_Put_index(FIFO0);
   Output.measuredValue[0]=Put_indexF0_2-Put_indexF0_1;
   Output.measuredValue_count++;	
//---------------------------Send second frame-----------------------//		
   UDS_request[2]=0;
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   store_CANframeTX(2,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
    __NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(3,UDS_response,RxHeader.DataLength>>16);   
/*------------------------Forming output-------------------------------------*/
   Put_indexF0_1=FDCAN_Get_FIFO_Put_index(FIFO0);
   osDelay(1000);
   Put_indexF0_2=FDCAN_Get_FIFO_Put_index(FIFO0);
   Output.measuredValue[1]=Put_indexF0_2-Put_indexF0_1;
   Output.measuredValue_count++;		
   Send_Result();	 
  }
}

void UDS4b_RUN(void *argument)
{
 uint8_t UDS_request[4]={0x03,0x28,0x03,0x03};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
 uint32_t Put_indexF0_1,Put_indexF0_2 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   store_CANframeTX(0,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(1,UDS_response, RxHeader.DataLength>>16);	   
/*------------------------Forming output-------------------------------------*/	
   Put_indexF0_1=FDCAN_Get_FIFO_Put_index(FIFO0);
   osDelay(2000);
   Put_indexF0_2=FDCAN_Get_FIFO_Put_index(FIFO0);
   Output.measuredValue[0]=Put_indexF0_2-Put_indexF0_1;
   Output.measuredValue_count++;		   
   Send_Result();		 
  }
}
void UDS5_RUN(void *argument)
{
 uint8_t UDS_request[5]={0x04,0x14,0xff,0xff,0xff};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
/*читаем активные ошикби(0х09) и выводим на экран*/
   UDS_READ_ERRORS(0x09);
   Send_Result();
   /*Erase_crash();----cтираем запись об аварии*/
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_5;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);            
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response); 
   UDS_READ_ERRORS(0x09);
   Send_Result();   
  }
  
}
void UDS6_RUN(void *argument)
{
 uint8_t UDS_request[3]={0x02,0x19,0x0A};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1=0,Put_index2 =0;
 uint8_t NEW_MESSAGES_COUNT=0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   store_CANframeTX(0,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response); 
   store_CANframeRX(1,UDS_response, sizeof(UDS_response));
   UDS_request[0]=0x30; UDS_request[1]=0x00; UDS_request[2]=0x00;
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   store_CANframeTX(2,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   osDelay(UDS_DELAY);
   Put_index2=FDCAN_Get_FIFO_Put_index(FIFO1);
   if(Put_index2>Put_index1)
    {
      NEW_MESSAGES_COUNT=Put_index2-Put_index1;
    }
    else
    {
      NEW_MESSAGES_COUNT=0x40-Put_index1+Put_index2;
    }
   for(uint8_t i=0;i<NEW_MESSAGES_COUNT;i++)
    {
      HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   	  store_CANframeRX(i+3,UDS_response,sizeof(UDS_response));
    }
    Send_Result();
  }
}
void UDS7_RUN(void *argument)
{
 uint8_t Erase_DTC[5]={0x04,0x14,0xFF,0xFF,0xFF};
 uint8_t DIAG_OFF[3]={0x02,0x85,0x02};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
  /* ПРОВЕРЬТЕ СОПРОТИВЛЕНИЕ*/
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
/*-------------------------очистка DTC--------------------------------------*/   
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1); 
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_5;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Erase_DTC);
   //store_CANframeTX(0,Erase_DTC,sizeof(Erase_DTC),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   //store_CANframeRX(1,UDS_response,sizeof(RxHeader.DataLength));
/*-------------------------отключение самодиагностики и чтение ошибок--------------------------------------*/ 
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,DIAG_OFF);
   //store_CANframeTX(2,Erase_DTC,sizeof(Erase_DTC),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   UDS_READ_ERRORS(0x09);
   Send_Result();
   /*вывести dtc и Warning с указанием поменять сопротивление*/
   /*ждем сообщения что мы изменили сопротивление, считываем ошибки, отправляем*/
   while(UARTRXcmd[0]==0){__NOP();}
   UARTRXcmd[0]=0;
   UDS_READ_ERRORS(0x09);
   Send_Result();
   /*-------------------------Включение самодиагностики,ожидание 10с. и чтение ошибок--------------------------------------*/ 
   DIAG_OFF[2]-=1;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,DIAG_OFF);
   //store_CANframeTX(2,Erase_DTC,sizeof(Erase_DTC),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   osDelay(10000);
   UDS_READ_ERRORS(0x09);
   Send_Result();
   		 
  }
}
void UDS8_RUN(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   EnterSecurityAccess();
   Send_Result();
  }
}
void UDS9_RUN(void *argument)
{
 uint8_t Read_DID[4]={0x03,0x22,0xd1,0x00};
 uint8_t Working[5]={0x04,0x2E,0xD1,0x00,0xA5};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
/*-------------------Входим в securityaccess---------------*/      
   EnterSecurityAccess();
/*----------------читаем DID-------------------------*/
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Read_DID);
   store_CANframeTX(0,Read_DID,sizeof(Read_DID),DTOOL_to_AIRBAG.Identifier);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(1,UDS_response,RxHeader.DataLength);   
/*------------------тест на срабатывание------------------------*/
   while(timeX<0)
   {		
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
   time = 0;
   while(timeX<290&&SPI_STOP_FLAG==false)
   {
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   Accelerometer_reset();
   Output.measuredValue[0]=SPI_STOP_FLAG;
   Output.measuredValue_count++;
/*-------------------изменение режима на working-----------------------*/
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_5;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Working);
   store_CANframeTX(2,Read_DID,sizeof(Read_DID),DTOOL_to_AIRBAG.Identifier);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(3,UDS_response,RxHeader.DataLength);
/*------------------повторный тест на срабатывание---------------------*/
   while(timeX<0)
   {		
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
   time = 0;
   while(timeX<290&&SPI_STOP_FLAG==false)
   {
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   Accelerometer_reset();
   Output.measuredValue[0]=SPI_STOP_FLAG;
   Output.measuredValue_count++;
   Send_Result();
  }
}
void UDS10_RUN(void *argument)
{
  uint8_t ECU_reset[3]={0x02,0x11,0x01};
 uint8_t Read_DID[4]={0x03,0x22,0xE1,0x80};
 uint8_t UDS_response[8]={0,};
 uint8_t SB_FP_OFF[5]={0x04,0x2E,0xE1,0x80,0xFE};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   EnterSecurityAccess();
/*-----------------------Read DID--------------------------------------*/      
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Read_DID);
   store_CANframeTX(6,Read_DID,sizeof(Read_DID),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(7,UDS_response,RxHeader.DataLength);
/*---------------------Отключение передней подушки---------------------*/
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_5;   
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Read_DID);
   store_CANframeTX(8,Read_DID,sizeof(Read_DID),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(9,UDS_response,RxHeader.DataLength);
/*----------------------Перезапуск блока-------------------------------------*/ 
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,ECU_reset);
   store_CANframeTX(10,ECU_reset,sizeof(ECU_reset),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(11,UDS_response, RxHeader.DataLength);
/*-----------------Тест на срабатывание--------------------------------------*/
   while(timeX<0)
   {		
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
   time = 0;
   while(timeX<290&&SPI_STOP_FLAG==false)
   {
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   Output.measuredValue[0]=SPI_STOP_FLAG;
   Output.measuredValue_count++;
/*-----------------------Read DID--------------------------------------*/  
   Read_DID[3]+=2;   
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Read_DID);
   store_CANframeTX(12,Read_DID,sizeof(Read_DID),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(13,UDS_response,RxHeader.DataLength);
/*-----------------------отключение РБ--------------------------------------*/  
   Read_DID[3]+=2;   
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,SB_FP_OFF);
   store_CANframeTX(13,Read_DID,sizeof(Read_DID),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(14,UDS_response,RxHeader.DataLength);
/*----------------------Перезапуск блока-------------------------------------*/ 
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,ECU_reset);
   store_CANframeTX(10,ECU_reset,sizeof(ECU_reset),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(11,UDS_response, RxHeader.DataLength);
/*------------------отправка-----------------------------------------*/
   Send_Result();   
  }
}

void UDS11_12_13_14_16RUN(void *argument)
{
 uint8_t UDS_request[4]={0x03,0x22,0x00,0x00};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;  
   if(Input.testNumber==0x3C)
   {
       UDS_request[2]=0x59;
       UDS_request[3]=0x10;
   }
   else if(Input.testNumber==0x3D)
   {
       UDS_request[2]=0x59;
       UDS_request[3]=0x18;
   }
   else if(Input.testNumber==0x3E)//SPPED
   {
       UDS_request[2]=0xC9;
       UDS_request[3]=0x21;
   }
   else if(Input.testNumber==0x3F)//VOLTAGE
   {
       UDS_request[2]=0xC9;
       UDS_request[3]=0x53;
   }
   else if(Input.testNumber==0x302)
   {
       UDS_request[2]=0xC9;
       UDS_request[3]=0x57;
   }
   Input.testNumber=0;
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength = FDCAN_DLC_BYTES_4;	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   store_CANframeTX(0,UDS_request, sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
/*---------------------Ожидание ответа--------------------------------------*/
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(1,UDS_response, RxHeader.DataLength>>16);
/*------------------------Forming output-------------------------------------*/   
   Send_Result();		 
  }
}

void UDS15_RUN(void *argument)
{
 uint8_t UDS_request[4]={0x03,0x22,0xC9,0x14};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   Put_index1=((FDCAN1->RXF1S)&0x00FF0000)>>16;
/*----------------------читаем lifetimer и получаем ответ--------------------------*/	
   DTOOL_to_AIRBAG.DataLength = FDCAN_DLC_BYTES_4;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   store_CANframeTX(0,UDS_request, sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   }
   store_CANframeRX(1,UDS_response, RxHeader.DataLength>>16);
 /*------------------Ожиднаие 5с.-----------------------*/
   osDelay(5000);
/*----------------------повторно читаем lifetimer и получаем ответ--------------------------*/	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   store_CANframeTX(2,UDS_request, sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   }
   store_CANframeRX(3,UDS_response, RxHeader.DataLength>>16);
/*------------------------Forming output-------------------------------------*/   
   Send_Result();
  }
}

void DIAG1_RUN(void *argument)
{	
  size_t message_length;
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    SEND_PERIODIC_MESSAGES=true;
    SEND_CLUSTER=true;
    xTaskNotifyGive(Send_periodicHandle);
    Output.testNumber=Input.testNumber;
    Input.testNumber=0;
    osDelay(4000);
    /*добавить чтение нормального сопротивления*/
    UDS_READ_ERRORS(0x09);
/*------------------------Forming output-------------------------------------*/
    Output.method=Method_GET;
    pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    len[0]=(uint8_t)message_length;
    HAL_UART_Transmit(&huart4,len,1,1000);
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
    CLEAR_OUTPUT();
    SEND_PERIODIC_MESSAGES=false;
    SEND_CLUSTER=false;      
  }
}
void DIAG2_3_RUN(void *argument)
{	
  size_t message_length;
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    SEND_PERIODIC_MESSAGES=true;
    SEND_CLUSTER=true;
    xTaskNotifyGive(Send_periodicHandle);
    Output.testNumber=Input.testNumber;
    Input.testNumber=0;
    osDelay(10000);
    UDS_READ_ERRORS(0x09);   
    if(Input.testNumber==0x52)//высокое напряжение
    {
/*-------------------Остановка отправки сообщений 0х5D7-------------------------*/
     Input.has_vehicle_speed=false;
     osDelay(1000);
/*-----------------------------------------------------------------------------*/
/*------------------------Считывание ошибок 0х09--------------------------------*/
     UDS_READ_ERRORS(0x09);
    }    
/*----------------------------------------------------------------------------------*/    
    else if(Input.testNumber==0x53)
    {
     xTaskNotifyGive(DIAG1Handle);//проводим тест на изменение состояний выходов пиропатронов
    }    
/*------------------------Forming output-------------------------------------*/
    Output.method=Method_GET;
    pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    len[0]=(uint8_t)message_length;
    HAL_UART_Transmit(&huart4,len,1,1000);
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
    CLEAR_OUTPUT();
    SEND_PERIODIC_MESSAGES=false;
    SEND_CLUSTER=false;     
  }  
}

void DIAG4_RUN(void *argument)
{	
  uint8_t VIN_RESET_1[8]={0x10,0x11,0x2E,0xF1,0x90,0x00,0x00,0x00};
  uint8_t VIN_RESET_2[8]={0x21,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint8_t DIAG_RESPONSE[8]={0,};
  uint8_t RESTART_REQUEST[3]={0x02,0x11,0x01};
  uint32_t Put_index1;
  size_t message_length;
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    SEND_PERIODIC_MESSAGES=true;
    SEND_CLUSTER=true;
    xTaskNotifyGive(Send_periodicHandle);
/*-----------------------------Запись VIN=0----------------------------------------*/      
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,VIN_RESET_1);
    Put_index1=((FDCAN1->RXF1S)&0x00FF0000)>>16;
	while((((FDCAN1->RXF1S)&0x00FF0000)>>16)<Put_index1)
	{
	 HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, DIAG_RESPONSE);
	}
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,VIN_RESET_2);
    VIN_RESET_2[0]+=0x01;
    osDelay(10);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,VIN_RESET_2);
    osDelay(10);
/*-------------------------Перезапуск БУ--------------------------------*/
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,RESTART_REQUEST);
    osDelay(1000);
/*---------------------------Считывание ошибок 0х09---------------------------------------*/   
    UDS_READ_ERRORS(0x09); 
/*--------------------------------Запись ненулевого VIN-----------------------------------*/ 
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,VIN_RESET_1);
    Put_index1=((FDCAN1->RXF1S)&0x00FF0000)>>16;
	while((((FDCAN1->RXF1S)&0x00FF0000)>>16)<Put_index1)
	{
	 HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, DIAG_RESPONSE);
	}
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,VIN_RESET_2);
    VIN_RESET_2[0]+=0x01;
    VIN_RESET_2[1]=0xAA;
    osDelay(10);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,VIN_RESET_2);
    osDelay(1000);
/*------------------------------------------Cчитывание ошибок 0х08--------------------------------------------------------*/
    UDS_READ_ERRORS(0x08);
///-----------------------------------------------------------------------------------------------------------------------------///       
/*------------------------Forming output-------------------------------------*/
    Output.method=Method_GET;
    pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    len[0]=(uint8_t)message_length;
    HAL_UART_Transmit(&huart4,len,1,1000);
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
    CLEAR_OUTPUT();
    SEND_PERIODIC_MESSAGES=false;
    SEND_CLUSTER=false;     
	
  }
}
void DIAG5_RUN(void *argument)
{	

  size_t message_length;
  for(;;)
  {
/*------------------------------Тест на срабатывание----------------------------------*/
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   SEND_PERIODIC_MESSAGES=true;
   SEND_CLUSTER=true;
   xTaskNotifyGive(Send_periodicHandle);      
   while(timeX<0)
   {		
	HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
   time = 0;
   while(timeX<290&&SPI_STOP_FLAG==false)
   {
	HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif      
   }
/*---------------------------------Считывание ошибок 0х09---------------------------------------------------*/
    UDS_READ_ERRORS(0x09);
///---------------------------------------Forming output----------------------------------------------///
    Output.method=Method_GET;
    pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    len[0]=(uint8_t)message_length;
    HAL_UART_Transmit(&huart4,len,1,1000);
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
    CLEAR_OUTPUT();
    SEND_PERIODIC_MESSAGES=false;
    SEND_CLUSTER=false;   
  }  
}
void DIAG6_RUN(void *argument)
{	
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    SEND_PERIODIC_MESSAGES=true;
    SEND_CLUSTER=true;
    xTaskNotifyGive(Send_periodicHandle); 
/*нужно отредактировать proto файл и добавить выбор сообщения, 
которое мы перестаём отправлять*/      
  }
}
void DIAG7_8_RUN(void *argument)
{	
  size_t message_length;
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    SEND_PERIODIC_MESSAGES=true;
    SEND_CLUSTER=true;
    xTaskNotifyGive(Send_periodicHandle);
    osDelay(4000);
    UDS_READ_ERRORS(0x09);
///---------------------------------------Forming output----------------------------------------------///
    Output.method=Method_GET;
    pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    len[0]=(uint8_t)message_length;
    HAL_UART_Transmit(&huart4,len,1,1000);
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
    CLEAR_OUTPUT();
    SEND_PERIODIC_MESSAGES=false;
    SEND_CLUSTER=false;         
  }
}
void DIAG9_RUN(void *argument)
{
  size_t message_length;  
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    SEND_PERIODIC_MESSAGES=true;
    SEND_CLUSTER=true;
    xTaskNotifyGive(Send_periodicHandle);
    osDelay(4000);
    UDS_READ_ERRORS(0x08);
///---------------------------------------Forming output----------------------------------------------///
    Output.method=Method_GET;
    pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    len[0]=(uint8_t)message_length;
    HAL_UART_Transmit(&huart4,len,1,1000);
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
    CLEAR_OUTPUT();
    SEND_PERIODIC_MESSAGES=false;
    SEND_CLUSTER=false;         
  }
}
void EDR_Transmitter(void *argument)
{
	uint8_t EDR_request[4]={0x03,0x22,0xFA,0x11};
	uint8_t EDR_consequtive[4]={0x30,0x00,0x00,0x00};
	uint8_t CANRxdata[8]={0,};
	uint8_t EDR_buffer[256]={0,};
	int8_t RXcounter=-1;
  /* Infinite loop */
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);		
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,EDR_request);
	HAL_Delay(5000);
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, CANRxdata);
	for(uint8_t i=0;i<3;i++){EDR_buffer[i]=CANRxdata[i+5];}
	while(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1,FDCAN_RX_FIFO1)>0)
	 {
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,EDR_consequtive);
		HAL_Delay(5000);
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, CANRxdata);
		RXcounter++;
		if((RxHeader.DataLength)>>16!=8) break;
    for(uint8_t i=1;i<=7;i++){EDR_buffer[7*RXcounter+2+i]=CANRxdata[i];}	       			
	 }
	for(uint8_t i=0;i<5;i++)
	  {
		EDR_buffer[7*(RXcounter+1)+3+i]=CANRxdata[i+1];
	  }
	HAL_UART_Transmit(&huart4,EDR_buffer,7*(RXcounter+1)+8,1000);
  }
}
