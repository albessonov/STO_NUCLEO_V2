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
extern volatile bool SPI_STOP_FLAG;
TestData Output=TestData_init_zero;
TestData Input=TestData_init_zero;
TestData Debug=TestData_init_zero;
/*флаги, разрешающие/запрещающие отправку соответствующих CAN-сообщений*/
volatile bool SEND_PERIODIC_MESSAGES=false;
volatile bool SEND_DOORSTATE=false;
volatile bool SEND_CLUSTER=false;
/*-----------------------------------------------------------------*/
extern volatile bool CRASH_DETECTED_BEFORE_COLLISION_TAKEN;
extern volatile bool CRASH_DETECTED_AFTER_COLLISION_TAKEN;
extern uint8_t Result[256];
extern uint8_t len[1];



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
    else if(Input.testNumber ==  0x61)
    {		    
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(EDRHandle);					
	}
    else if(Input.testNumber ==  0x52||Input.testNumber ==  0x53)
    {		    
	 memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(DIAG2Handle);					
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
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   FDCAN_ENABLE_INTERRUPTS();
   for(uint16_t j=0;j<0xFFF;j++){__NOP();}
  }
}
void Accelerometer_period_RUN(void *argument)
{
  /* USER CODE BEGIN Accelerometer1_RUN */
	/*Runs the accelerometer emualator, set of accelerations is hardcoded*/
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   FDCAN_ENABLE_INTERRUPTS();
   while(timeX<0)
   {		
	HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    HAL_Delay(25);//Must be removed
   }
   HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
   time = 0;
   while(timeX<290&&SPI_STOP_FLAG==false)
   {
	HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    HAL_Delay(25);//Must be removed
   }
  }
  /* USER CODE END Accelerometer_period_RUN */
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
    HAL_Delay(25);//Must be removed
   }
   HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
   time = 0;
   while(timeX<290&&SPI_STOP_FLAG==false)
   {
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    HAL_Delay(25);//Must be removed
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
	/*Задача разблокируется при запуске любого из тестов SBR, отправляемые сообщениязависят от полученной команды*/
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
   if(Input.has_Door_position==true)
   {
     if(Input.Door_position==Driver&&SEND_DOORSTATE==true)
     {
	  BCM_CANHS_R_04_data_ER[6]|=DRIVER_DOOR_OPEN;
     }
     else if(Input.Door_position==Front_passenger)
     {
	  BCM_CANHS_R_04_data_ER[6]|=FRONT_PASSENGER_DOOR_OPEN;
     }
     else if(Input.Door_position==Rear_passengerRight)
     {
	  BCM_CANHS_R_04_data_ER[7]|=REAR_PASSENGER_RIGHT_DOOR_OPEN;
     }
     else if(Input.Door_position==Rear_passengerLeft)
     {
	  BCM_CANHS_R_04_data_ER[7]|=REAR_PASSENGER_LEFT_DOOR_OPEN;
     }
   }			
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
   if(Input.VehicleStateExtended==Sleeping)
   {
    VehicleState=BCM_CANHS_R_04_data_S;
   }
   else if(Input.VehicleStateExtended==EngineRunning)
   {
	VehicleState=BCM_CANHS_R_04_data_ER;
   }
   while(SEND_PERIODIC_MESSAGES==true)
   {
     HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BCM_CANHS_R_04,VehicleState); //0x350
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
  CanFrame ReceivedFrame;
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN;
  size_t message_length;
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
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
   Output.frame[0]=ReceivedFrame;
   Output.frame_count++;
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
   HAL_Delay(1000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*--------------------------------Forming output--------------------------------------*/	
   Output.frame[1]=ReceivedFrame;
   Output.frame_count++;
   Output.method=Method_GET;
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;		
   pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
   pb_encode(&streamwrt, TestData_fields, &Output);
   message_length=streamwrt.bytes_written;
   len[0]=(uint8_t)message_length;
   HAL_UART_Transmit(&huart4,len,1,1000);
   HAL_UART_Transmit(&huart4,Result,message_length,1000);
   Input.testNumber=0;		
   SEND_PERIODIC_MESSAGES=false;
   CLEAR_OUTPUT();
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void SBR2_RUN(void *argument)
{
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  CanFrame ReceivedFrame;
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN;
  size_t message_length;
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
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*--------------------------------Forming output--------------------------------------*/		
   Output.frame[0]=ReceivedFrame;
   Output.frame_count++;
   Output.method=Method_GET;
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;	
   pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
   pb_encode(&streamwrt, TestData_fields, &Output);
   message_length=streamwrt.bytes_written;
   len[0]=(uint8_t)message_length;
   HAL_UART_Transmit(&huart4,len,1,1000);
   HAL_UART_Transmit(&huart4,Result,message_length,1000);	
   SEND_PERIODIC_MESSAGES=false;
   CLEAR_OUTPUT();
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void SBR3_4_RUN(void *argument)
{
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  CanFrame ReceivedFrame;
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN;
  size_t message_length;
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
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*---------------------------Forming output-------------------------------------*/
   Output.frame[0]=ReceivedFrame;
   Output.frame_count++;
   Output.method=Method_GET;
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;	
   pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
   pb_encode(&streamwrt, TestData_fields, &Output);
   message_length=streamwrt.bytes_written;
   len[0]=(uint8_t)message_length;
   HAL_UART_Transmit(&huart4,len,1,1000);
   HAL_UART_Transmit(&huart4,Result,message_length,1000);		
   SEND_PERIODIC_MESSAGES=false;
   CLEAR_OUTPUT();
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void SBR5_RUN(void *argument)
{	
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  CanFrame ReceivedFrame;
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN,delay;
  size_t message_length;
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
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*---------------------------Forming output-------------------------------------*/
   Output.frame[0]=ReceivedFrame;
   Output.frame_count++;			
   Output.method=Method_GET;
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;	
   pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
   pb_encode(&streamwrt, TestData_fields, &Output);
   message_length=streamwrt.bytes_written;
   len[0]=(uint8_t)message_length;
   HAL_UART_Transmit(&huart4,len,1,1000);
   HAL_UART_Transmit(&huart4,Result,message_length,1000);		
   CLEAR_OUTPUT();
   xTaskNotifyStateClear(Send_periodicHandle);

  }
}
void SBR6_RUN(void *argument)
{
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  CanFrame ReceivedFrame;
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN,delay;
  size_t message_length;
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
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
   Output.frame[0]=ReceivedFrame;
   Output.frame_count++;		
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);	
   osDelay(1000);
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*---------------------------Forming output-------------------------------------*/   
   Output.frame[1]=ReceivedFrame;
   Output.frame_count++;				
   Output.method=Method_GET;
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;	
   pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
   pb_encode(&streamwrt, TestData_fields, &Output);
   message_length=streamwrt.bytes_written;
   len[0]=(uint8_t)message_length;
   HAL_UART_Transmit(&huart4,len,1,1000);
   HAL_UART_Transmit(&huart4,Result,message_length,1000);
   SEND_PERIODIC_MESSAGES=false;
   CLEAR_OUTPUT();
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void SBR7_RUN(void *argument)
{
	uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
	CanFrame ReceivedFrame;
	GPIO_TypeDef* SB_PORT;
	uint16_t SB_PIN;
	size_t message_length;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_PERIODIC_MESSAGES=true;
/*------------------------------Select door--------------------------*/	  
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
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));	
   Output.frame[0]=ReceivedFrame;
   Output.frame_count++;			
   Output.method=Method_GET;
   Output.has_Seatbelt_position=Input.has_Door_position;
   Output.Seatbelt_position=Input.Door_position;	
   pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
   pb_encode(&streamwrt, TestData_fields, &Output);
   message_length=streamwrt.bytes_written;
   len[0]=(uint8_t)message_length;
   HAL_UART_Transmit(&huart4,len,1,1000);
   HAL_UART_Transmit(&huart4,Result,message_length,1000);
   SEND_PERIODIC_MESSAGES=false;
   CLEAR_OUTPUT();
   xTaskNotifyStateClear(Send_periodicHandle);
  }
}
void UDS1_RUN(void *argument)
{
  uint8_t UDS_request[3]={0x02,0x11,0x01};
  uint8_t UDS_response[8]={0,};
  CanFrame ReceivedFrame;
  size_t message_length;
  uint32_t tmp =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0; 	  
   tmp=FDCAN1->RXF1S;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   ReceivedFrame.id=DTOOL_to_AIRBAG.Identifier;				
   ReceivedFrame.length=(DTOOL_to_AIRBAG.DataLength)>>16;
   ReceivedFrame.data.size=3;
   memcpy(ReceivedFrame.data.bytes,UDS_request,sizeof(UDS_request));
   Output.frame[0]=ReceivedFrame;
   Output.frame_count++;
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   }
/*------------------------Forming output-------------------------------------*/
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,UDS_response,sizeof(UDS_response));  
   Output.frame[1]=ReceivedFrame;
   Output.frame_count++;			
   Output.method=Method_GET;
   pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
   pb_encode(&streamwrt, TestData_fields, &Output);
   message_length=streamwrt.bytes_written;
   len[0]=(uint8_t)message_length;
   HAL_UART_Transmit(&huart4,len,1,1000);
   HAL_UART_Transmit(&huart4,Result,message_length,1000);
   CLEAR_OUTPUT();
   memset(UDS_response,0x00,sizeof(UDS_response));   
  }
}	
void UDS2_RUN(void *argument)
{
  uint8_t UDS_request[6]={0x05,0x2F,0x38,0x01,0x03,0x00};
  uint8_t UDS_response[8]={0,};
  CanFrame ReceivedFrame;
  size_t message_length;
  uint32_t tmp =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   tmp=FDCAN1->RXF1S;
//---------------------------DIAG LED ON-------------------------------------//
   if(Input.LED==UDS_LED_DIAG_LED_ON)
   {	  
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
	ReceivedFrame.id=DTOOL_to_AIRBAG.Identifier;				
    ReceivedFrame.length=(DTOOL_to_AIRBAG.DataLength)>>16;
    ReceivedFrame.data.size=6;
    memcpy(ReceivedFrame.data.bytes,UDS_request,sizeof(UDS_request));
    Output.frame[0]=ReceivedFrame;
    Output.frame_count++;   
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
	 HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
	}
    ReceivedFrame.timestamp=RxHeader.RxTimestamp;
    ReceivedFrame.id=RxHeader.Identifier;				
	ReceivedFrame.length=(RxHeader.DataLength)>>16;
	ReceivedFrame.data.size=ReceivedFrame.length;
	memcpy(ReceivedFrame.data.bytes,UDS_response,sizeof(UDS_response));	
	Output.frame[1]=ReceivedFrame;
	Output.frame_count++;
   }
//-------------------------DIAG LED OFF-----------------------------------// 
   if(Input.LED==UDS_LED_DIAG_LED_OFF)
   {
    UDS_request[5]=0x01; 
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
	ReceivedFrame.id=DTOOL_to_AIRBAG.Identifier;				
    ReceivedFrame.length=(DTOOL_to_AIRBAG.DataLength)>>16;
    ReceivedFrame.data.size=6;
    memcpy(ReceivedFrame.data.bytes,UDS_request,sizeof(UDS_request));
    Output.frame[0]=ReceivedFrame;
    Output.frame_count++;
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
	 HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
	}
    ReceivedFrame.timestamp=RxHeader.RxTimestamp;
    ReceivedFrame.id=RxHeader.Identifier;				
	ReceivedFrame.length=(RxHeader.DataLength)>>16;
	ReceivedFrame.data.size=ReceivedFrame.length;
	memcpy(ReceivedFrame.data.bytes,UDS_response,sizeof(UDS_response));	
	Output.frame[1]=ReceivedFrame;
	Output.frame_count++;
   }
//--------------------------SB LED ON-----------------------------------//
   else if(Input.LED==UDS_LED_SB_LED_ON)
   {
    UDS_request[5]=0x00;
    UDS_request[3]=0x02;		 
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);\
	ReceivedFrame.id=DTOOL_to_AIRBAG.Identifier;				
    ReceivedFrame.length=(DTOOL_to_AIRBAG.DataLength)>>16;
    ReceivedFrame.data.size=6;
    memcpy(ReceivedFrame.data.bytes,UDS_request,sizeof(UDS_request));
    Output.frame[0]=ReceivedFrame;
    Output.frame_count++;
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
	 HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
	}
    ReceivedFrame.timestamp=RxHeader.RxTimestamp;
    ReceivedFrame.id=RxHeader.Identifier;				
	ReceivedFrame.length=(RxHeader.DataLength)>>16;
	ReceivedFrame.data.size=ReceivedFrame.length;
	memcpy(ReceivedFrame.data.bytes,UDS_response,sizeof(UDS_response));	
	Output.frame[1]=ReceivedFrame;
	Output.frame_count++;
   }
//----------------------------SB LED OFF--------------------------//
   else if(Input.LED==UDS_LED_SB_LED_OFF)
   {
    UDS_request[5]=0x01; 
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
	ReceivedFrame.id=DTOOL_to_AIRBAG.Identifier;				
    ReceivedFrame.length=(DTOOL_to_AIRBAG.DataLength)>>16;
    ReceivedFrame.data.size=6;
    memcpy(ReceivedFrame.data.bytes,UDS_request,sizeof(UDS_request));
    Output.frame[0]=ReceivedFrame;
    Output.frame_count++;
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
	 HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
	}
    ReceivedFrame.timestamp=RxHeader.RxTimestamp;
    ReceivedFrame.id=RxHeader.Identifier;				
	ReceivedFrame.length=(RxHeader.DataLength)>>16;
	ReceivedFrame.data.size=ReceivedFrame.length;
	memcpy(ReceivedFrame.data.bytes,UDS_response,sizeof(UDS_response));	
	Output.frame[1]=ReceivedFrame;
	Output.frame_count++;
   }	   
//------------------------Forming output-------------------------------------//		 
	Output.method=Method_GET;
	pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
	len[0]=(uint8_t)message_length;
	HAL_UART_Transmit(&huart4,len,1,1000);
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
	SEND_PERIODIC_MESSAGES=false;
	CLEAR_OUTPUT();
    memset(UDS_response,0x00,sizeof(UDS_response));	 
  }
}
void UDS3_RUN(void *argument)
{
 uint8_t UDS_request[4]={0x03,0x28,0x01,0x01};
 uint8_t UDS_response[8]={0,};
 CanFrame ReceivedFrame;
 size_t message_length;
 uint32_t tmp =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   tmp=FDCAN1->RXF1S;	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   ReceivedFrame.id=DTOOL_to_AIRBAG.Identifier;				
   ReceivedFrame.length=(DTOOL_to_AIRBAG.DataLength)>>16;
   ReceivedFrame.data.size=4;
   memcpy(ReceivedFrame.data.bytes,UDS_request,sizeof(UDS_request));
   Output.frame[0]=ReceivedFrame;
   Output.frame_count++;
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   }
   tmp=FDCAN1->RXF1S;
//--------------------------Wait 5s----------------------------------//	  
   osDelay(4000);
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,UDS_response,sizeof(UDS_response));
   memset(UDS_response,0x00,sizeof(UDS_response));	  
   Output.frame[1]=ReceivedFrame;
   Output.frame_count++;
   Output.measuredValue[0]=FDCAN1->RXF0S;
   Output.measuredValue_count++;
   osDelay(1000);
   Output.measuredValue[1]=FDCAN1->RXF0S;
   Output.measuredValue_count++;  
//---------------------------Send second frame-----------------------//		
   UDS_request[2]=0;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   ReceivedFrame.id=DTOOL_to_AIRBAG.Identifier;				
   ReceivedFrame.length=(DTOOL_to_AIRBAG.DataLength)>>16;
   ReceivedFrame.data.size=4;
   memcpy(ReceivedFrame.data.bytes,UDS_request,sizeof(UDS_request));
   Output.frame[2]=ReceivedFrame;
   Output.frame_count++;
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   }		
/*------------------------Forming output-------------------------------------*/
   ReceivedFrame.timestamp=RxHeader.RxTimestamp;
   ReceivedFrame.id=RxHeader.Identifier;				
   ReceivedFrame.length=(RxHeader.DataLength)>>16;
   ReceivedFrame.data.size=ReceivedFrame.length;
   memcpy(ReceivedFrame.data.bytes,UDS_response,sizeof(UDS_response));	
   Output.frame[3]=ReceivedFrame;
   Output.measuredValue[2]=FDCAN1->RXF0S;
   Output.measuredValue_count++;
   osDelay(1000);
   Output.measuredValue[3]=FDCAN1->RXF0S;
   Output.measuredValue_count++;
   Output.frame_count++;			
   Output.method=Method_GET;
   pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
   pb_encode(&streamwrt, TestData_fields, &Output);
   message_length=streamwrt.bytes_written;
   len[0]=(uint8_t)message_length;
   HAL_UART_Transmit(&huart4,len,1,1000);
   HAL_UART_Transmit(&huart4,Result,message_length,1000);
   CLEAR_OUTPUT();		 
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
void DIAG2_RUN(void *argument)
{	
  uint8_t VOLTAGE_REQUEST_H[8]={0x05,0x19,0x05,0x90,0x98,0x17};
  uint8_t VOLTAGE_REQUEST_L[8]={0x05,0x19,0x05,0x90,0x98,0x17};
  uint8_t* SB_STATE_REQUEST;
  uint8_t DIAG_RESPONSE[8]={0,};
  CanFrame ReceivedFrame;
  size_t message_length;
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    SEND_PERIODIC_MESSAGES=true;
    SEND_CLUSTER=true;
    xTaskNotifyGive(Send_periodicHandle);
    if(Input.testNumber==0x0E)
			{
        SB_STATE_REQUEST=VOLTAGE_REQUEST_H;
      }
    else
		  {
       SB_STATE_REQUEST=VOLTAGE_REQUEST_L;
      }				
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,SB_STATE_REQUEST);
		while(DIAG_RESPONSE[0]==0)
			{
		    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, DIAG_RESPONSE);
			}
//--------------------------------ReceivedFrame --------------------------------------------------------------------//
	ReceivedFrame.timestamp=RxHeader.RxTimestamp;
    ReceivedFrame.id=RxHeader.Identifier;				
	ReceivedFrame.length=(RxHeader.DataLength)>>16;
	ReceivedFrame.data.size=ReceivedFrame.length;			
	memcpy(ReceivedFrame.data.bytes,DIAG_RESPONSE,sizeof(DIAG_RESPONSE));				
//------------------------------ Forming Output ---------------------------------------------------------------------//
	Output.frame[0]=ReceivedFrame;
	Output.frame_count++;			
	Output.method=Method_GET;
	Output.testNumber=Input.testNumber;
	Output.has_Seatbelt_position=Input.has_Door_position;
	Output.Seatbelt_position=Input.Door_position;	
	pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
	len[0]=(uint8_t)message_length;
	HAL_UART_Transmit(&huart4,len,1,1000);
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
	SEND_PERIODIC_MESSAGES=false;
	CLEAR_OUTPUT();
	xTaskNotifyStateClear(Send_periodicHandle);
	
  }
}
void DIAG3_RUN(void *argument)
{	
  uint8_t VIN_RESET_1[8]={0x10,0x11,0x2E,0xF1,0x90,0x00,0x00,0x00};
  uint8_t VIN_RESET_2[8]={0x21,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint8_t VIN_RESET_3[8]={0x22,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint8_t VIN_REQUEST[8]={0x05,0x19,0x05,0x90,0x98,0x17};
  uint8_t DIAG_RESPONSE[8]={0,};
  CanFrame ReceivedFrame;
  size_t message_length;
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    SEND_PERIODIC_MESSAGES=true;
    SEND_CLUSTER=true;
    xTaskNotifyGive(Send_periodicHandle);				
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,VIN_RESET_1);
	while(RxHeader.Identifier!=0x772)
		{
		  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, DIAG_RESPONSE);
		}
	ReceivedFrame.timestamp=RxHeader.RxTimestamp;
    ReceivedFrame.id=RxHeader.Identifier;				
	ReceivedFrame.length=(RxHeader.DataLength)>>16;
	ReceivedFrame.data.size=ReceivedFrame.length;
	memcpy(ReceivedFrame.data.bytes,DIAG_RESPONSE,sizeof(DIAG_RESPONSE));	
	Output.frame[0]=ReceivedFrame;
	Output.frame_count++;			
	Output.method=Method_GET;
	Output.testNumber=Input.testNumber;
	Output.has_Seatbelt_position=Input.has_Door_position;
	Output.Seatbelt_position=Input.Door_position;	
	pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
	len[0]=(uint8_t)message_length;
	HAL_UART_Transmit(&huart4,len,1,1000);
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
		/*Used for debug purposes*/
 /* pb_istream_t streamrd = pb_istream_from_buffer(Result, sizeof(Result));
    pb_decode(&streamrd, TestData_fields, &Debug);*/
	SEND_PERIODIC_MESSAGES=false;
	CLEAR_OUTPUT();
	xTaskNotifyStateClear(Send_periodicHandle);
	
  }
}
