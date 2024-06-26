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
extern UART_HandleTypeDef huart7;//,huart4;
extern FDCAN_TxHeaderTypeDef BCM_CANHS_R_04,BRAKE_CANHS_R_01,DTOOL_to_AIRBAG,CLUSTER_CANHS_RNr_01,TORQUE_AT_CANHS_RNr_02,SWA_RNR_1,TIME_R_1,TORQUE_ECM,MIL_ECM,BRAKE_RNR4,BRAKE_RNR1;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern FDCAN_TxHeaderTypeDef TxHeader;
extern volatile uint32_t time;
extern uint8_t UARTRXcmd[128];
extern uint8_t SPI_RXbuf[4];
extern uint8_t SPI_resp[4];//={0x87,0xF3,0x40,0x04};
extern volatile double timeX,timeY;
extern uint32_t ctr2;
extern uint32_t ctr0;
extern uint32_t TTF_DAB,TTF_PAB,TTF_DPT,TTF_PPT,TTF_DSAB,TTF_PSAB,TTF_DCAB;
extern uint32_t FRONT_100_50_X_transformed[801];
extern uint32_t FRONT_100_50_Y_transformed[801];
extern uint8_t Request0x2cmd[4];
extern uint8_t Request0x0cmd[4];
/*флаг, управляющий работой эмулятора акселерометра*/
extern volatile bool CRASH_OCCURED_FLAG;
/*объекты структуры, используемой для приема-передачи данных о тестах*/
TestData Output=TestData_init_zero;
TestData Input=TestData_init_zero;
TestData Debug=TestData_init_zero;
/*флаги, разрешающие/запрещающие отправку соответствующих CAN-сообщений*/
volatile bool SEND_PERIODIC_MESSAGES=false;
volatile bool SEND_DOORSTATE=false;
volatile bool SEND_CLUSTER=false;
volatile bool SEND_VEHICLE_STATE=true;
volatile bool SEND_TESTER_PRESENT=false;
volatile bool SEND_GEAR_REVERSE=false;
volatile bool SEND_GEAR_LEVER=false;
/*флаги, используемые для отслеживание сигнала CD до и после столкновения*/
extern volatile bool CRASH_DETECTED_BEFORE_COLLISION_TAKEN;
extern volatile bool CRASH_DETECTED_AFTER_COLLISION_TAKEN;
/*массивы, отправляемые по UART*/
extern uint8_t Result[512];

/*данные для тестов самодиагностики БУ*/


void vApplicationIdleHook( void )
{
 // HAL_GPIO_WritePin(GPIOE,SB_BP3_CTRL_Pin,fastened);
  HAL_UART_Receive(&huart7,UARTRXcmd,sizeof(UARTRXcmd),1000);
	/*Filling in the input structure using nanopb*/
  if(UARTRXcmd[0]==0x08)
  {
   pb_istream_t RXstream = pb_istream_from_buffer(UARTRXcmd, sizeof(UARTRXcmd));
   pb_decode(&RXstream, TestData_fields, &Input);
  }
  if(UARTRXcmd[0]==0xBB)
  {
    HAL_UART_Transmit(&huart7,UARTRXcmd,1,1000);
    memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
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
	else if(Input.testNumber ==  0x14)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(ValidABHandle);
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
    else if(Input.testNumber ==  0x36)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS5Handle);
	}
	else if(Input.testNumber ==  0x37)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS6Handle);
	}
	else if(Input.testNumber ==  0x38)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS7Handle);
	}
    else if(Input.testNumber ==  0x39)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS8Handle);
	}
    else if(Input.testNumber ==  0x3A)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS9Handle);
	}
	else if(Input.testNumber ==  0x3B)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS9Handle);
	}
    else if(Input.testNumber ==  0x3C)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS10Handle);
	}
    else if(Input.testNumber ==  0x3D||Input.testNumber ==  0x3E||Input.testNumber ==  0x3F||Input.testNumber ==  0x301||Input.testNumber ==  0x303)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	 xTaskNotifyGive(UDS11_12_13_14_16Handle);
	}
    else if(Input.testNumber ==  0x302)
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
	else if(Input.testNumber ==  0x48)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
     xTaskNotifyGive(SBR8Handle);					
    }
	else if(Input.testNumber ==  0x49)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
     xTaskNotifyGive(SBR9Handle);					
    }
	else if(Input.testNumber ==  0x4A)
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
     xTaskNotifyGive(SBR10Handle);					
    }
    else if(Input.testNumber ==  0x51||Input.testNumber ==  0x52)////считываем DTC 0х09
    { 
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));		
     xTaskNotifyGive(READDTCHandle);
	}
    else if(Input.testNumber ==  0x53)////чистим DTC 
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
	/* SEND_PERIODIC_MESSAGES=true;
	 xTaskNotifyGive(Send_periodicHandle);*/		
	 Output.testNumber=Input.testNumber;
     Input.testNumber=0;        
	 Send_Request(CLEAR_DIAGNOSTIC_INFORMATION,6);
	 EnterSecurityAccess(); 
     Send_Request(ERASE_CRASH,8);
     Send_Result();				
	}
    else if(Input.testNumber ==  0x54)////записываем VIN!=0
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));        
	 Output.testNumber=Input.testNumber;
     Input.testNumber=0;        
	 Write_VIN(1);
     Send_Result();						
	}
    else if(Input.testNumber ==  0x55)///записываем VIN=0 
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));        
	 Output.testNumber=Input.testNumber;
     Input.testNumber=0;        
	 Write_VIN(0);
     Send_Result();						
	}
    else if(Input.testNumber ==  0x56)////Перезагрузка блока 
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));        
	 Output.testNumber=Input.testNumber;
     Input.testNumber=0;        
	 Send_Request(ECU_RESET,0);
     Send_Result();						
	}
    else if(Input.testNumber ==  0x57)////Входим в ExtendedDiagnosticSession
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));        
	 Output.testNumber=Input.testNumber;
     Input.testNumber=0;        
	 Send_Request(ENTER_EXTENDED_DIAGNOSTIC,0);
     Send_Result();						
	}
    else if(Input.testNumber ==  0x58)////Тест на срабатывание
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));        
	 Output.testNumber=Input.testNumber;
     Input.testNumber=0;        
	 while(timeX<0)
     {		
	  HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);
      #ifdef DEBUG_MODE   
      HAL_Delay(25);
      #endif    
     }
     HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
     time = 0;
     while(timeX<290&&CRASH_OCCURED_FLAG==false)
     {
	  HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);
      #ifdef DEBUG_MODE
      HAL_Delay(25);
      #endif
     }
     Output.measuredValue[0]=CRASH_OCCURED_FLAG;
     Output.measuredValue_count++;
     Send_Result();
     Accelerometer_reset();     
	}
    else if(Input.testNumber ==  0x59)////Отправка периодических сообщений
    {
	 SEND_PERIODIC_MESSAGES=false;
     xTaskNotifyStateClear(Send_periodicHandle);  		
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));        
	 //Output.testNumber=Input.testNumber;
     Input.testNumber=0;
     SEND_PERIODIC_MESSAGES=true;   
     Input.has_vehicle_speed=Input.DIAG_SEND_0x5D7;
     SEND_CLUSTER=Input.DIAG_SEND_0x4F8;
     SEND_VEHICLE_STATE=Input.DIAG_SEND_0x350;        
     xTaskNotifyGive(Send_periodicHandle) ;  						
	}
    else if(Input.testNumber ==  0x5A)////Запись ACU configuration
    {
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd)); 
     EnterSecurityAccess();
     Send_Request(ENTER_EXTENDED_DIAGNOSTIC,0);		
     CheckACUConfiguration();
	 for(uint16_t i=0;i<0x7ff;i++){__NOP;}
     Send_Request(ECU_RESET,0);
     Send_Result();	 
	}
    else if(Input.testNumber ==  0x5B)////Проверка CrashDetectionOutOfOrder
    {
	 uint8_t CANRxData[4]={0,0,0,0};
	 bool CrashDetectionOutOfOrder;
     memset(UARTRXcmd,0x00,sizeof(UARTRXcmd));
     while(RxHeader.Identifier!=0x653){ 
     HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, CANRxData);}
	 CrashDetectionOutOfOrder=((CANRxData[0])&0b01000000)>>6;
	 Output.measuredValue[0]=CrashDetectionOutOfOrder;
	 Output.measuredValue_count++;
     Send_Result();	 
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
   Output.testNumber=Input.testNumber;
   //Input.testNumber=0;
   time=0;		
  }
 
}
void ValidAB_RUN(void *argument)
{
  uint8_t CANRxData[8]={0,};
/*Certain implementation is given in HAL_FDCAN_RxFifo0Callback*/
  for(;;)
  {
   ulTaskNotifyTake( pdTRUE,portMAX_DELAY );
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   Send_Request(ECU_RESET,0);
   osDelay(8000);	  
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, CANRxData);
   Output.measuredValue[0]=(CANRxData[1]&0b00010000)>>4;
   Output.measuredValue_count++;	  
   store_CANframeRX(2,CANRxData,RxHeader.DataLength);
   osDelay(5000);	  
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, CANRxData);
   Output.measuredValue[1]=(CANRxData[1]&0b00010000)>>4;
   Output.measuredValue_count++;	  
   store_CANframeRX(3,CANRxData,RxHeader.DataLength);
   Send_Result();	  
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
   CheckACUConfiguration();
   Send_Request(ERASE_CRASH,0);
   ChangeOperatingState(WORKING_MODE,0);
   Output.testNumber=Input.testNumber;
  // Input.testNumber=0;
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
   while(timeX<290&&CRASH_OCCURED_FLAG==false)
   {
	HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
    Accelerometer_reset();
  } 
}

void Accelerometer1_RUN(void *argument)
{
  /* USER CODE BEGIN Accelerometer1_RUN */
	uint8_t CANRxData[4]={0,0,0,0};
	uint8_t CANRxData2[4]={0,0,0,0};
/*Runs the accelerometer emualator, set of accelerations is depended on byte received via UART,frame forming is given in HAL_SPI_TxRxCpltCallback*/
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   TTF_DAB=0;
   TTF_PAB=0;
   TTF_DPT=0;
   TTF_PPT=0;
   TTF_DSAB=0;
   TTF_PSAB=0;
   TTF_DCAB=0;
   if(Input.AIRBAG_OFF==true)
   {
	HAL_GPIO_WritePin(SQUIB_SW_CTRL_GPIO_Port,SQUIB_SW_CTRL_Pin,GPIO_PIN_RESET);//Нажатие кнопки отключения ПБ переднего пассажира       
   }
   CRASH_OCCURED_FLAG=false;
   Output.measuredValue_count=7;
   Send_Request(ECU_RESET,0);
   osDelay(10000);
   CheckACUConfiguration();
   Send_Request(ERASE_CRASH,0);
   osDelay(1);
   ChangeOperatingState(WORKING_MODE,0);
   while (RxHeader.Identifier!=0x653)
   {
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, CANRxData);
   }	     
   store_CANframeRX(0,CANRxData,RxHeader.DataLength);
  while(timeX<0)
   {		
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   time = 0;
  
   while(timeX<290&&CRASH_OCCURED_FLAG==false)
   {
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   HAL_Delay(2000);
   while (1)
   {
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, CANRxData2);
    if(RxHeader.Identifier==0x653)
	{	
		break;
	}
   }
   //HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, CANRxData2);   
   store_CANframeRX(1,CANRxData2,RxHeader.DataLength);
   Output.has_accDataNumber=Input.has_accDataNumber;
   Output.accDataNumber=Input.accDataNumber;
   Send_Result();
   CRASH_DETECTED_BEFORE_COLLISION_TAKEN=false;
   CRASH_DETECTED_AFTER_COLLISION_TAKEN=false;
   Accelerometer_reset();
   FDCAN_DISABLE_INTERRUPTS();
   //Send_Request(ECU_RESET,0);
   //CLEAR_OUTPUT();
  }
  /* USER CODE END Accelerometer1_RUN */
}
void Send_GearLever(void *argument)
{
  uint8_t GEAR_LEVER[8]={0x0,0x3,0,0,0,0,0,0};
 /* uint8_t cmd1[8]={0x10,0x0a,0x31,0x01,0xff,0,0x01,0x80};
  uint8_t cmd2[5]={0x21,0x00,0x2f,0xff,0xff};
  uint8_t RX[8];
  uint8_t RX2[8]={0,0,0,0,0,0,0,0};
  uint32_t Put_index1;*/
  for(;;)
  {
	  /*DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_8;
	  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,cmd1);
	  //osDelay(2000);
	  Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
	  while(_NO_RX_FIFO1_NEW_MESSAGE)
	  {
		  __NOP();
	  }
	  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RX);
	  DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_5;
	  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,cmd2);
	  //osDelay(2000);
	  Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
	  while(_NO_RX_FIFO1_NEW_MESSAGE)
	  {
		  __NOP();
	  }
	  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RX2);
	  __NOP();*/
	 ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
	  
	  while(SEND_GEAR_LEVER==true)
	 {
	   if(SEND_GEAR_REVERSE==true)
      {
		 GEAR_LEVER[1]=0x01; 
	  }
	  else
      {
		 GEAR_LEVER[1]=0x03; 
	  } 
	  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TORQUE_AT_CANHS_RNr_02,GEAR_LEVER);
	  osDelay(10);
	 }
	  
      
  }
}
void Send_periodic_start(void *argument)
{
/*Задача разблокируется при запуске любого из тестов SBR/самодиагностики,отправляемые сообщения зависят от полученной команды*/
	//uint8_t BCM_CANHS_R_04_data_ER[8]={0x70,0,0,0,0,0,0,0xE0};	//engine running
	uint8_t BCM_CANHS_R_04_data[8]={0x00,0,0,0,0,0,0,0};	//sleeping
	uint8_t Vehicle_Speed[8]={0x0,0x0,0,0,0,0,0,0};
	//uint8_t Vehicle_Speed_40kmh[8]={0xA0,0x0F,0,0,0,0,0,0};
	//uint8_t *Speed;
	uint8_t Cluster[8]={0,0x01,0,0,0,0,0,0};
    uint8_t Tester_Present[3]={0x02,0x3E,0x00};
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
/*------------------------Выбор позиции двери--------------------------*/
   if(SEND_DOORSTATE==true)
   {   
	 BCM_CANHS_R_04_data[1]=DRIVER_DOOR_OPEN;
   }
/*----------Выбор отправляемой скорости--------------*/   
   if(Input.has_vehicle_speed==true)
   {
   if(Input.vehicle_speed==_40KMH)
	 {
	  Vehicle_Speed[6]=0x0F;
	  Vehicle_Speed[7]=0xA0;
	 }	 
	 else if(Input.vehicle_speed==_15KMH)
	 {
	  Vehicle_Speed[6]=0x05;
	  Vehicle_Speed[7]=0xDC;
	 }	
   }
/*---------------Выбор VehicleStateExtended---------------------------*/
   if(Input.VehicleStateExtended==Sleeping)
   {
    BCM_CANHS_R_04_data[7]=0;//sleeping
   }
   else if(Input.VehicleStateExtended==EngineRunning)
   {
	BCM_CANHS_R_04_data[7]=0x70;//eng running
	//BCM_CANHS_R_04_data[0]=0x70;//eng running
   }
  /*------------------GenericApplicativeDiagEnable в диагностических тестах*/
   if(Input.has_GenDiagEnable==true)
   {
    BCM_CANHS_R_04_data[5]=(Input.GenDiagEnable<<4);
   }
/*-----------------Для теста UDS CommunicationControl 0x28----------*/
   if(SEND_TESTER_PRESENT==true)
   {
      DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Tester_Present);
      osDelay(2000);
   }
/*----------------------Отправка сообщений---------------------------*/
   while(SEND_PERIODIC_MESSAGES==true)
   {
     if(SEND_VEHICLE_STATE==true)
     {         
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BCM_CANHS_R_04,BCM_CANHS_R_04_data); //0x350
     }
     if(Input.has_vehicle_speed==true)
     {
	  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BRAKE_CANHS_R_01,Vehicle_Speed); //0x5D7
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
   CheckACUConfiguration();
   SEND_PERIODIC_MESSAGES=true;
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_GEAR_LEVER=true;
   xTaskNotifyGive(SendGearLeverHandle);
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
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerCenter)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerLeft)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
	SB_PIN=SB_BP1_CTRL_Pin;
   }
/*----------------------------Test implementation----------------------------------*/	
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
   osDelay(7000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
   osDelay(7000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(1,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*--------------------------------Forming output--------------------------------------*/
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   xTaskNotifyStateClear(Send_periodicHandle);
   SEND_GEAR_LEVER=false;
   xTaskNotifyStateClear(SendGearLeverHandle);
   //HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
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
   CheckACUConfiguration();
   SEND_PERIODIC_MESSAGES=true;
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_GEAR_LEVER=true;
   xTaskNotifyGive(SendGearLeverHandle);
   Send_Request(ECU_RESET,0);
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
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
    SB_PIN=SB_BP1_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerCenter)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerLeft)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
   }
/*------------------------Test implementation----------------------------------*/
   osDelay(5000);
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
   osDelay(8000);//?????			
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*--------------------------------Forming output--------------------------------------*/
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;
   Output.method=Method_GET;	
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   xTaskNotifyStateClear(Send_periodicHandle);
   SEND_GEAR_LEVER=false;
   xTaskNotifyStateClear(SendGearLeverHandle);
   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
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
   CheckACUConfiguration();
   SEND_PERIODIC_MESSAGES=true;	  
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_GEAR_LEVER=true;
   xTaskNotifyGive(SendGearLeverHandle);
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
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerCenter)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
   }
   else if(Input.Seatbelt_position==Rear_passengerLeft)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
	SB_PIN=SB_BP1_CTRL_Pin;
   }
/*------------------------Test implementation----------------------------------*/   
   if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	  }
   osDelay(8000);			
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*---------------------------Forming output-------------------------------------*/
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   xTaskNotifyStateClear(Send_periodicHandle);
   SEND_GEAR_LEVER=false;
   xTaskNotifyStateClear(SendGearLeverHandle);
   if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	  }
  }
}
void SBR5_RUN(void *argument)
{	
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN;
  uint32_t delay=40000;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   CheckACUConfiguration();
   SEND_PERIODIC_MESSAGES=true;	
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_GEAR_LEVER=true;
   xTaskNotifyGive(SendGearLeverHandle);
/*------------------------------Select seatbelt and timeout--------------------------*/	  
   if(Input.Seatbelt_position==Driver)
   {
	SB_PORT=SB_DR_CTRL_GPIO_Port;
	SB_PIN=SB_DR_CTRL_Pin;
	delay=40000;
   }
   else if(Input.Seatbelt_position==Front_passenger)
   {
	SB_PORT=SB_FP_CTRL_GPIO_Port;
	SB_PIN=SB_FP_CTRL_Pin;
	delay=40000;
   }
   else if(Input.Seatbelt_position==Rear_passengerRight) 
   {
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
	delay=80000;
   }
   else if(Input.Seatbelt_position==Rear_passengerCenter)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
	delay=80000;
   }
   else if(Input.Seatbelt_position==Rear_passengerLeft)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
	SB_PIN=SB_BP1_CTRL_Pin;
	delay=80000;
   }
/*------------------------Test implementation----------------------------------*/
   if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	  }
   osDelay(5000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));   	
   osDelay(delay-5000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(1,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*---------------------------Forming output-------------------------------------*/   
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;   
   xTaskNotifyStateClear(Send_periodicHandle);
   SEND_GEAR_LEVER=false;
   xTaskNotifyStateClear(SendGearLeverHandle);
    if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	  }	  
  }
}
void SBR6_RUN(void *argument)
{
  uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  GPIO_TypeDef* SB_PORT;
  uint16_t SB_PIN;
  uint32_t delay=40000;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   CheckACUConfiguration();
   SEND_PERIODIC_MESSAGES=true;	
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_GEAR_LEVER=true;
   xTaskNotifyGive(SendGearLeverHandle);
/*------------------------------Select seatbelt and timeout--------------------------*/	  
   if(Input.Seatbelt_position==Driver)
   {
	SB_PORT=SB_DR_CTRL_GPIO_Port;
	SB_PIN=SB_DR_CTRL_Pin;
	delay=40000;
   }
   else if(Input.Seatbelt_position==Front_passenger)
   {
	SB_PORT=SB_FP_CTRL_GPIO_Port;
	SB_PIN=SB_FP_CTRL_Pin;
	delay=40000;
   }
   else if(Input.Seatbelt_position==Rear_passengerRight)
   {
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
	delay=80000;
   }
   else if(Input.Seatbelt_position==Rear_passengerCenter)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
	delay=80000;
   }
   else if(Input.Seatbelt_position==Rear_passengerLeft)
   {
    SB_PORT=SB_BP1_CTRL_GPIO_Port;
	SB_PIN=SB_BP1_CTRL_Pin;
	delay=80000;
   }
/*------------------------Test implementation----------------------------------*/   
   if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	  }
   osDelay(delay);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
   if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	  }	  	
   osDelay(5000);
   if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	  }
   osDelay(8000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(1,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*---------------------------Forming output-------------------------------------*/   				
   Output.has_Seatbelt_position=Input.has_Seatbelt_position;
   Output.Seatbelt_position=Input.Seatbelt_position;
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   xTaskNotifyStateClear(Send_periodicHandle);
   SEND_GEAR_LEVER=false;
   xTaskNotifyStateClear(SendGearLeverHandle);
   if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	  }	  
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
   CheckACUConfiguration();
   SEND_PERIODIC_MESSAGES=true;	
   SEND_DOORSTATE=true;
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_GEAR_LEVER=true;
   xTaskNotifyGive(SendGearLeverHandle);
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
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
   }
   else if(Input.Seatbelt_position== Rear_passengerCenter)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
   }
   else if(Input.Seatbelt_position== Rear_passengerLeft)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
    SB_PIN=SB_BP1_CTRL_Pin;
   }
/*------------------------Test implementation----------------------------------*/   
   if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	  }
   osDelay(8000);	
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
/*---------------------------Forming output-------------------------------------*/
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));;			
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   SEND_DOORSTATE=false;
   xTaskNotifyStateClear(Send_periodicHandle);
   SEND_GEAR_LEVER=false;
   xTaskNotifyStateClear(SendGearLeverHandle);
	if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	  }
  }
}
void SBR8_RUN(void *argument)
{
	uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
	GPIO_TypeDef* SB_PORT;
	uint16_t SB_PIN;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   CheckACUConfiguration();
   SEND_PERIODIC_MESSAGES=true;
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_GEAR_LEVER=true;
   xTaskNotifyGive(SendGearLeverHandle);
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
	SB_PORT=SB_BP2_CTRL_GPIO_Port;
	SB_PIN=SB_BP2_CTRL_Pin;
   }
   else if(Input.Seatbelt_position== Rear_passengerCenter)
   {
	SB_PORT=SB_BP3_CTRL_GPIO_Port;
	SB_PIN=SB_BP3_CTRL_Pin;
   }
   else if(Input.Seatbelt_position== Rear_passengerLeft)
   {
	SB_PORT=SB_BP1_CTRL_GPIO_Port;
    SB_PIN=SB_BP1_CTRL_Pin;
   }
   osDelay(200);
/*------------------------Test implementation----------------------------------*/   
   if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	  }
   osDelay(8000);	
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
   SEND_GEAR_REVERSE=true;
   osDelay(8000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(1,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
/*---------------------------Forming output-------------------------------------*/		
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   SEND_GEAR_REVERSE=false;
   SEND_GEAR_LEVER=false;
   xTaskNotifyStateClear(Send_periodicHandle);
   xTaskNotifyStateClear(SendGearLeverHandle);
	  if(SB_PIN!=SB_FP_CTRL_Pin)
	   {   
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
	   }
   else
	  {
	   HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
	  }
  }
}
void SBR9_RUN(void *argument)
{
	uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   CheckACUConfiguration();
   SEND_PERIODIC_MESSAGES=true;	
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_GEAR_LEVER=true;
   xTaskNotifyGive(SendGearLeverHandle);
/*------------------------Test implementation----------------------------------*/
   HAL_GPIO_WritePin(SB_FP_CTRL_GPIO_Port,SB_FP_CTRL_Pin,fastened);	  
   HAL_GPIO_WritePin(SBRS_CTRL_GPIO_Port,SBRS_CTRL_Pin,GPIO_PIN_RESET);	  
   osDelay(9000);	
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));

   HAL_GPIO_WritePin(SBRS_CTRL_GPIO_Port,SBRS_CTRL_Pin,GPIO_PIN_SET);	  
   osDelay(9000);	
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(1,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));	  
/*---------------------------Forming output-------------------------------------*/			
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   xTaskNotifyStateClear(Send_periodicHandle);
   SEND_GEAR_LEVER=false;
   xTaskNotifyStateClear(SendGearLeverHandle);
  }
}
void SBR10_RUN(void *argument)
{
	uint8_t AIRBAG_CANHS_R_01_data[4]={0,};
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   CheckACUConfiguration();
   SEND_PERIODIC_MESSAGES=true;	
   xTaskNotifyGive(Send_periodicHandle);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_GEAR_LEVER=true;
   xTaskNotifyGive(SendGearLeverHandle);
/*------------------------Test implementation----------------------------------*/   
   HAL_GPIO_WritePin(SQUIB_SW_CTRL_GPIO_Port,SQUIB_SW_CTRL_Pin,GPIO_PIN_SET);//отключение ПБ переднего пассажира 
   osDelay(6000);	
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(0,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
   
   HAL_GPIO_WritePin(SQUIB_SW_CTRL_GPIO_Port,SQUIB_SW_CTRL_Pin,GPIO_PIN_RESET);//Вкл ПБ переднего пассажира 
   osDelay(6000);	
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
   store_CANframeRX(1,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));	  
/*---------------------------Forming output-------------------------------------*/			
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;
   xTaskNotifyStateClear(Send_periodicHandle);
   SEND_GEAR_LEVER=false;
   xTaskNotifyStateClear(SendGearLeverHandle);
  }
}
void UDS1_RUN(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0; 	  
   Send_Request(ECU_RESET,0);
/*------------------------Forming output-------------------------------------*/		
   Send_Result(); 
  }
}	
void UDS2_RUN(void *argument)
{
  uint8_t DIAG_LED_ON[6]={0x05,0x2F,0x38,0x01,0x03,0x00};
  uint8_t DIAG_LED_OFF[6]={0x05,0x2F,0x38,0x01,0x03,0x01};
  uint8_t SB_LED_ON[6]={0x05,0x2F,0x38,0x02,0x03,0x00};
  uint8_t SB_LED_OFF[6]={0x05,0x2F,0x38,0x02,0x03,0x01};
  uint8_t UDS_response[8]={0,};
  uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   Output.has_LED=Input.has_LED;
   Output.LED=Input.LED;
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_6;  
//-----------Вход в диагностическую сессию и Security доступ------------------//
   Send_Request(ENTER_EXTENDED_DIAGNOSTIC,6);
   EnterSecurityAccess();  
//---------------------------DIAG LED ON-------------------------------------//
   if(Input.LED==UDS_LED_DIAG_LED_ON)
   { 
	
    DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_6;	   
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,DIAG_LED_ON);
    store_CANframeTX(8,DIAG_LED_ON,sizeof(DIAG_LED_ON),DTOOL_to_AIRBAG.Identifier);
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);    
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
       __NOP();
	}
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(9,UDS_response, RxHeader.DataLength);	
   }
//-------------------------DIAG LED OFF-----------------------------------// 
   if(Input.LED==UDS_LED_DIAG_LED_OFF)
   {

    DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_6;	   
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,DIAG_LED_OFF);
    store_CANframeTX(8,DIAG_LED_OFF,sizeof(DIAG_LED_OFF),DTOOL_to_AIRBAG.Identifier);
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
	 __NOP();
	}
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(9,UDS_response, RxHeader.DataLength);	
   }
   
//--------------------------SB LED ON-----------------------------------//
   else if(Input.LED==UDS_LED_SB_LED_ON)
   {		 
	DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_6;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,SB_LED_ON);
    store_CANframeTX(8,SB_LED_ON,sizeof(SB_LED_ON),DTOOL_to_AIRBAG.Identifier);
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
	 __NOP();
	}
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(9,UDS_response, RxHeader.DataLength);	
   }
//----------------------------SB LED OFF--------------------------//
   else if(Input.LED==UDS_LED_SB_LED_OFF)
   {
    DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_6;	   
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,SB_LED_OFF);
    store_CANframeTX(8,SB_LED_OFF,sizeof(SB_LED_OFF),DTOOL_to_AIRBAG.Identifier);
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
    while(_NO_RX_FIFO1_NEW_MESSAGE)
	{
	 __NOP();
	}
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(9,UDS_response, RxHeader.DataLength);	
   }	   
   else if(Input.LED==UDS_LED_RETURN_CTRL_TO_ECU)
   {
    Send_Request(RETURN_CONTROL_TO_ECU,8);
   }	   
//------------------------Forming output-------------------------------------//		 
	Send_Result();
    memset(UDS_response,0x00,sizeof(UDS_response));   
  }//enter security access
}
void UDS3_RUN(void *argument)
{
 uint32_t Put_index1 =0;
 uint8_t Tester_present[3]={0x02,0x3E,0x00};
 uint8_t UDS_response[3]={0,0,0};
  /* Infinite loop */
  for(;;)
  {      
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Tester_present);
   store_CANframeTX(0,Tester_present,sizeof(Tester_present),DTOOL_to_AIRBAG.Identifier);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(1,UDS_response,RxHeader.DataLength);   
/*----------------------Отправка результата----------------------------*/
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
   SEND_TESTER_PRESENT=true;
   Send_Request(ENTER_EXTENDED_DIAGNOSTIC,10);
   xTaskNotifyGive(Send_periodicHandle);
   FDCAN_ENABLE_INTERRUPTS();	  
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   store_CANframeTX(0,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(1,UDS_response,RxHeader.DataLength);
   osDelay(1000);
   Put_indexF0_1=FDCAN_Get_FIFO_Put_index(FIFO0);
//--------------------------Wait 10s and check new messages----------------------------------//	  
   osDelay(1000);
   Put_indexF0_2=FDCAN_Get_FIFO_Put_index(FIFO0);
   Output.measuredValue[0]=Put_indexF0_2-Put_indexF0_1;
   Output.measuredValue_count++;	
//---------------------------Send second frame-----------------------//
   osDelay(3000);
   Send_Request(ENTER_EXTENDED_DIAGNOSTIC,6);   
   UDS_request[2]=0;
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   store_CANframeTX(2,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
    __NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(3,UDS_response,RxHeader.DataLength);   
/*------------------------Forming output-------------------------------------*/
   Put_indexF0_1=FDCAN_Get_FIFO_Put_index(FIFO0);
   osDelay(1000);
   Put_indexF0_2=FDCAN_Get_FIFO_Put_index(FIFO0);
   Output.measuredValue[1]=Put_indexF0_2-Put_indexF0_1;
   Output.measuredValue_count++;		
   Send_Result();
   xTaskNotifyStateClear(Send_periodicHandle);
   SEND_TESTER_PRESENT=false;
   FDCAN_DISABLE_INTERRUPTS();
   UDS_request[2]=1;   
  }
}

void UDS4b_RUN(void *argument)
{
 uint8_t UDS_request[4]={0x03,0x28,0x03,0x03};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);	
   FDCAN_ENABLE_INTERRUPTS();	  
   Send_Request(ENTER_EXTENDED_DIAGNOSTIC,6);
   EnterSecurityAccess();
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   SEND_TESTER_PRESENT=true;
   xTaskNotifyGive(Send_periodicHandle);
   Send_Request(ENTER_EXTENDED_DIAGNOSTIC,10);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   store_CANframeTX(8,UDS_request,sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(9,UDS_response, RxHeader.DataLength);
   osDelay(1000);   
/*------------------------Forming output-------------------------------------*/	
  // osDelay(1000);
   HAL_FDCAN_MspDeInit(&hfdcan1);
   __NOP();
   __NOP();
   HAL_FDCAN_MspInit(&hfdcan1);
   osDelay(1000);
   Output.measuredValue[0]=(FDCAN1->PSR)&0x00000001;
   Output.measuredValue_count++;		   
   Send_Result();
   xTaskNotifyStateClear(Send_periodicHandle);
   SEND_TESTER_PRESENT=false;  
   FDCAN_DISABLE_INTERRUPTS();   
  }
}
void UDS5_RUN(void *argument)
{
 uint8_t Erase_Crash[5]={0x04,0x2E,0xB0,0x12,0x00};
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
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Erase_Crash); 
   store_CANframeTX(2,Erase_Crash,sizeof(ERASE_CRASH),DTOOL_to_AIRBAG.Identifier);      
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   store_CANframeRX(3,UDS_response, RxHeader.DataLength);	
   Send_Request(CLEAR_DIAGNOSTIC_INFORMATION,0);
   Send_Result(); 
   UDS_READ_ERRORS(0x09);
   Send_Result();   
  }//Выводить фреймы 2 3 0 1
  
}
void UDS6_RUN(void *argument)
{
uint8_t READ_SUPPORTED_DTC[3]={0x02,0x19,0x0A};
uint8_t print_DTC[3]={0x30,0x00,0x00};
uint8_t UDS_response[8];
uint32_t Put_index1,Put_index2;
uint8_t NEW_MESSAGES_COUNT; 
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    Output.testNumber=Input.testNumber;
    Input.testNumber=0;
	DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,READ_SUPPORTED_DTC);
	HAL_Delay(1000);
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);  
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,print_DTC);
   /*---------------------Получение первого DTC-----------------------*/ 
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(0,UDS_response,RxHeader.DataLength);
	HAL_Delay(1000);
/*-------------------Запрос на считывание остальных DTC-----------------------------*/
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
    DTOOL_to_AIRBAG.DataLength = FDCAN_DLC_BYTES_3;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,print_DTC);
    HAL_Delay(6000);/*таймаут для приёма сообщений*/ 
	Put_index2=FDCAN_Get_FIFO_Put_index(FIFO1);
	NEW_MESSAGES_COUNT=Put_index2-Put_index1;
	//if(Put_index2>Put_index1)
    //{
	  NEW_MESSAGES_COUNT=Put_index2-Put_index1;
	//}
	//else
	//{
	  //NEW_MESSAGES_COUNT=0x40-Put_index2+Put_index1;	
	//}
/*------------------Подсчет числа сообщений-----------------------------*/
    for(uint8_t i=0; i<NEW_MESSAGES_COUNT;i++)
    {
	  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   	  store_CANframeRX(i+1,UDS_response,RxHeader.DataLength);
	}
	Send_Result();
	NVIC_SystemReset();
  }
}
void UDS7_RUN(void *argument)
{
 uint8_t DIAG_OFF[3]={0x02,0x85,0x02};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
/*-------------------------очистка DTC--------------------------------------*/ 
   Send_Request(ENTER_EXTENDED_DIAGNOSTIC,0);
   EnterSecurityAccess();	  
   Send_Request(CLEAR_DIAGNOSTIC_INFORMATION,0);
/*-------------------------отключение самодиагностики и чтение ошибок--------------------------------------*/ 
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DIAG_OFF[2]=2;	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,DIAG_OFF);
   store_CANframeTX(2,DIAG_OFF,sizeof(DIAG_OFF),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(3,UDS_response,sizeof(UDS_response));
   Send_Result();
   UDS_READ_ERRORS(0x09);
   Send_Result();
   /*вывести dtc и Warning с указанием поменять сопротивление*/
   /*ждем сообщения что мы изменили сопротивление, считываем ошибки, отправляем*/
   
   /*-------------------------Включение самодиагностики,ожидание 10с. и чтение ошибок--------------------------------------*/ 
   Send_Request(ENTER_EXTENDED_DIAGNOSTIC,0);
   EnterSecurityAccess();
   CLEAR_OUTPUT();
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DIAG_OFF[2]=1;
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_3;   
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,DIAG_OFF);
   store_CANframeTX(0,DIAG_OFF,sizeof(DIAG_OFF),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(1,UDS_response,sizeof(UDS_response));
   Send_Result();
   HAL_Delay(15000);
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
   //Send_Request(ENTER_EXTENDED_DIAGNOSTIC,14);
   EnterSecurityAccess();
   Send_Result();
  }
}
void UDS9_RUN(void *argument)
{
 uint8_t Read_DID[4]={0x03,0x22,0xd1,0x00};
 uint8_t Mode[5]={0x04,0x2E,0xD1,0x00,0xA5};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   if(Input.testNumber==0x3B)
   {
	   Mode[4]=0x5a;
   }
   else
   {
	  Mode[4]=0xA5; 
   }
   Input.testNumber=0;
/*-------------------Входим в securityaccess---------------*/
   EnterSecurityAccess();
/*-------------------изменение режима на working-----------------------*/
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_5;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Mode);
   store_CANframeTX(6,Mode,sizeof(Mode),DTOOL_to_AIRBAG.Identifier);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(7,UDS_response,RxHeader.DataLength);
   HAL_Delay(150);
/*------------------------Перезагрузка блока----------------------------*/
   Send_Request(ECU_RESET,8);
 /*------------------------------Повторное чтение DID-------------------------*/
   HAL_Delay(6000);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Read_DID);
   store_CANframeTX(10,Read_DID,sizeof(Read_DID),DTOOL_to_AIRBAG.Identifier);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(11,UDS_response,RxHeader.DataLength);
/*---------------------Отправка результата----------------------------*/
   Send_Result();
  }
}
void UDS10_RUN(void *argument)
{
 uint8_t Read_DID[4]={0x03,0x22,0xE1,0x80};
 uint8_t UDS_response[8]={0,};
 uint8_t airbag_OFF[5]={0x04,0x2E,0xE1,0x80,0xFE};
 uint8_t Driver_SB_OFF[5]={0x04,0x2E,0xE1,0x82,0xFE};
 uint32_t Put_index1 =0;
 uint8_t DriverSafetyBeltState=0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   CheckACUConfiguration();	  
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
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,airbag_OFF);
   store_CANframeTX(8,airbag_OFF,sizeof(airbag_OFF),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(9,UDS_response,RxHeader.DataLength);
/*----------------------Перезапуск блока-------------------------------------*/ 
   Send_Request(ECU_RESET,10);
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
   while(timeX<290&&CRASH_OCCURED_FLAG==false)
   {
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   Output.measuredValue[0]=CRASH_OCCURED_FLAG;
   Output.measuredValue_count++;
/*-----------------------Read DID 0X80--------------------------------------*/    
   HAL_Delay(5000);
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
/*-----------------------Read DID 0X82--------------------------------------*/
      
   Read_DID[3]+=2;   
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Read_DID);
   store_CANframeTX(14,Read_DID,sizeof(Read_DID),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(15,UDS_response,RxHeader.DataLength);
/*-----------------------отключение РБ--------------------------------------*/ 
   EnterSecurityAccess();      
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_5;	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Driver_SB_OFF);
   store_CANframeTX(16,Driver_SB_OFF,sizeof(Driver_SB_OFF),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(17,UDS_response,RxHeader.DataLength);
/*----------------------Проверка сигнала DriverDafetyBeltState до перезагрузки-------------------------------------*/
   HAL_Delay(1000);//на верочку
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, UDS_response);
   DriverSafetyBeltState=((UDS_response[0])>>2)&0xFF;
   Output.measuredValue[1]=DriverSafetyBeltState;
   Output.measuredValue_count++;
   Accelerometer_reset();
/*----------------------Перезапуск блока-------------------------------------*/ 
   Send_Request(ECU_RESET,18);
/*-----------------------Read DID 0X82 после перезапуска--------------------------------------*/
   HAL_Delay(5000);      
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Read_DID);
   store_CANframeTX(20,Read_DID,sizeof(Read_DID),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(21,UDS_response,RxHeader.DataLength);
/*----------------------Проверка сигнала DriverDafetyBeltState-------------------------------------*/
   HAL_Delay(1000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, UDS_response);
   DriverSafetyBeltState=((UDS_response[0])>>2)&0xFF;
   Output.measuredValue[2]=DriverSafetyBeltState;
   Output.measuredValue_count+=2;
   Accelerometer_reset();
/*------------------отправка-----------------------------------------*/
   Output.frame_count-=6;
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
   if(Input.testNumber==0x3D)
   {
       UDS_request[2]=0x59;
       UDS_request[3]=0x10;
   }
   else if(Input.testNumber==0x3E)
   {
       UDS_request[2]=0x59;
       UDS_request[3]=0x18;
   }
   else if(Input.testNumber==0x3F)//SPPED
   {
       SEND_PERIODIC_MESSAGES=true;
	   Input.has_vehicle_speed=true;
	   Input.vehicle_speed=_40KMH;
	   xTaskNotifyGive(Send_periodicHandle);
	   osDelay(500);
       UDS_request[2]=0xC9;
       UDS_request[3]=0x21;
	   //HAL_Delay(500);
   }
   else if(Input.testNumber==0x301)//VOLTAGE
   {
       UDS_request[2]=0xC9;
       UDS_request[3]=0x53;
   }
   else if(Input.testNumber==0x303)
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
   store_CANframeRX(1,UDS_response, RxHeader.DataLength);
/*------------------------Forming output-------------------------------------*/   
   Send_Result();
   SEND_PERIODIC_MESSAGES=false;   
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
   Put_index1= FDCAN_Get_FIFO_Put_index(FIFO1);
/*----------------------читаем lifetimer и получаем ответ--------------------------*/	
   DTOOL_to_AIRBAG.DataLength = FDCAN_DLC_BYTES_4;      
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   store_CANframeTX(0,UDS_request, sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(1,UDS_response, RxHeader.DataLength);
 /*------------------Ожиднаие 5с.-----------------------*/
   HAL_Delay(5000);
/*----------------------повторно читаем lifetimer и получаем ответ--------------------------*/	  
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   store_CANframeTX(2,UDS_request, sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(3,UDS_response, RxHeader.DataLength);
/*------------------------Forming output-------------------------------------*/   
   Send_Result();
  }
}
void UDS17_RUN(void *argument)
{
 uint8_t Request_Seed[4]={0x03,0x27,0x01,0x00};
 uint8_t Wrong_key[7]={0x06,0x67,0x02,0x11,0x11,0x11,0x11};
 uint8_t UDS_response[8]={0,};
 uint32_t Put_index1 =0;
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Output.testNumber=Input.testNumber;
   Input.testNumber=0;
   Send_Request(ENTER_EXTENDED_DIAGNOSTIC,0);
   Put_index1= FDCAN_Get_FIFO_Put_index(FIFO1);
/*---------------------------трижды запрашиваем ключ и отвечаем неправильно---------------------------*/
   for(uint8_t i=0;i<4;i++)
   {	  
	DTOOL_to_AIRBAG.DataLength = FDCAN_DLC_BYTES_4;      
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Request_Seed);
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
    store_CANframeTX(2,Request_Seed, sizeof(Request_Seed),DTOOL_to_AIRBAG.Identifier);
    while(_NO_RX_FIFO1_NEW_MESSAGE)
    {
	 __NOP();
    }
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(3,UDS_response, RxHeader.DataLength);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Wrong_key);
    Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
    store_CANframeTX(4,Wrong_key, sizeof(Wrong_key),DTOOL_to_AIRBAG.Identifier);
    while(_NO_RX_FIFO1_NEW_MESSAGE)
    {
	 __NOP();
    }
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
    store_CANframeRX(5,UDS_response, RxHeader.DataLength);
   }
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Request_Seed);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   store_CANframeTX(2,Request_Seed, sizeof(Request_Seed),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(3,UDS_response, RxHeader.DataLength);
   osDelay(10000);
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,Request_Seed);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   store_CANframeTX(2,Request_Seed, sizeof(Request_Seed),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(3,UDS_response, RxHeader.DataLength);
/*-----------------------------------------------*/	  
  /* HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,UDS_request);
   Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
   store_CANframeTX(2,UDS_request, sizeof(UDS_request),DTOOL_to_AIRBAG.Identifier);
   while(_NO_RX_FIFO1_NEW_MESSAGE)
   {
	__NOP();
   }
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, UDS_response);
   store_CANframeRX(3,UDS_response, RxHeader.DataLength);*/
/*------------------------Forming output-------------------------------------*/   
   Send_Result();
  }
}


void EDR_Transmitter(void *argument)
{
	uint8_t EDR_request[4]={0x03,0x22,0xFA,0x16};
	uint8_t EDR_consequtive[4]={0x30,0x00,0x00,0x00};
	uint8_t CANRxdata[8]={0,};
	uint8_t EDR_buffer[900]={0,};
	uint8_t num[2]={0,0};
	int8_t RXcounter=-1;
	uint32_t Put_index1=0;
	uint8_t GET_NUMBER_REQUEST[4]={0x03,0x22,0xE3,0x14};
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
   Input.testNumber=0;
   CheckACUConfiguration();
   Input.has_vehicle_speed=true;
   Input.vehicle_speed=_40KMH;
   SEND_PERIODIC_MESSAGES=true;
   SEND_VEHICLE_STATE=true;
   xTaskNotifyGive(Send_periodicHandle);
   xTaskNotifyGive(EDR10msHandle);
   xTaskNotifyGive(EDR20msHandle);
   xTaskNotifyGive(EDR3000msHandle);
   osDelay(7000);
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,GET_NUMBER_REQUEST);
   HAL_Delay(1000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, CANRxdata);	
   osDelay(10);
   num[0]=CANRxdata[4];  
   while(timeX<0)
   {		
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
   time = 0;
   while(timeX<290&&CRASH_OCCURED_FLAG==false)
   {
    HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    #ifdef DEBUG_MODE
    HAL_Delay(25);
    #endif
   }
   HAL_Delay(5500);
   /*-----------------------получаем номер edr-----------------------------*/
   DTOOL_to_AIRBAG.DataLength=FDCAN_DLC_BYTES_4;
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,GET_NUMBER_REQUEST);
   HAL_Delay(1000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, CANRxdata);
   num[1]=CANRxdata[4];
   switch(CANRxdata[4])
   {
	   case 0:
		   EDR_request[3]=0x16;
	       break;
	   case 1:
		   EDR_request[3]=0x17;
		   break;
	   case 2:
		   EDR_request[3]=0x18;
	       break;
   }
   /*-----------------------------------------------------------------------------*/
   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,EDR_request);
   HAL_Delay(5000);
   HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, CANRxdata);
   for(uint8_t i=0;i<3;i++){EDR_buffer[i]=CANRxdata[i+5];}
   //HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,EDR_consequtive);
	 while(1)
	 {
	  if((RxHeader.DataLength)!=8) break;
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&DTOOL_to_AIRBAG,EDR_consequtive);
	  if(Put_index1<=0x40)
	  {
	  Put_index1=FDCAN_Get_FIFO_Put_index(FIFO1);
	  }
      else Put_index1+=1;	  
	  while(_NO_RX_FIFO1_NEW_MESSAGE)
      {
	   __NOP;
	  }
	  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, CANRxdata);
	  RXcounter++;
	 // if((RxHeader.DataLength)!=8) break;
      for(uint8_t i=1;i<=7;i++){EDR_buffer[7*RXcounter+2+i]=CANRxdata[i];}	       			
	 }
	for(uint8_t i=0;i<5;i++)
	  {
		EDR_buffer[7*(RXcounter+1)+3+i]=CANRxdata[i+1];
	  }
	HAL_UART_Transmit(&huart7,EDR_buffer,7*(RXcounter+1)+8,1000);
	osDelay(1000);
	HAL_UART_Transmit(&huart7,num,2,1000);
	SEND_PERIODIC_MESSAGES=false;
    xTaskNotifyStateClear(Send_periodicHandle);
    xTaskNotifyStateClear(EDR10msHandle);
    xTaskNotifyStateClear(EDR20msHandle);
    xTaskNotifyStateClear(EDR3000msHandle);	  
  }
}
void EDR10ms_RUN(void *argument)
{
uint8_t swa_rnr1[8]={0,0,0,0x01,0x09,0,0x1,0x09};
uint8_t torque_at[8]={0,0x03,0,0,0,0,0,0};
uint8_t torque_ecm[7]={0,0x03,0,0,0,0x3,0x19};
uint8_t brake_rnr4[8]={0,0,0,0x05,0xD4,0,0,0};
	for(;;)
    {
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&SWA_RNR_1,swa_rnr1);
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TORQUE_AT_CANHS_RNr_02,torque_at);
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TORQUE_ECM,torque_ecm);
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BRAKE_RNR4,brake_rnr4);
		osDelay(10);
	}
}
void EDR20ms_RUN(void *argument)
{
uint8_t brake_rnr1[8]={0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BRAKE_RNR1,brake_rnr1);
		osDelay(20);
	}
}
void EDR3000ms_RUN(void *argument)
{
	uint8_t time_r_1[6]={0x12,0x04,0x78,0x18,0x0A,0x0C};
	uint8_t mil_ecm[8]={0,0,0,0,0,0,0,0x20};
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TIME_R_1,time_r_1);
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&MIL_ECM,mil_ecm);
		osDelay(3000);
	}
}
void READDTC_RUN(void *argument)
{
	uint8_t status_byte;
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		if(Input.testNumber==0x51)
		{
			status_byte=0x09;
		}
		else
		{
			status_byte=0x08;
		}
		UDS_READ_ERRORS(status_byte);
		Send_Result();
	}
}
