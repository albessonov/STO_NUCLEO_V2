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
extern FDCAN_TxHeaderTypeDef BCM_CANHS_R_04,BRAKE_CANHS_R_01;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern volatile uint32_t time;
extern uint8_t UARTRXcmd[15];
extern uint8_t SPI_RXbuf[4];
extern uint8_t SPI_resp[4];
extern volatile double timeX,timeY;
extern uint32_t ctr2;
extern uint32_t ctr0;
extern volatile bool SPI_STOP_FLAG;
TestData Output, Input,Debug;
bool SEND_DOORSTATE=false;

extern uint8_t Result[256];


extern osThreadId_t Init_testHandle;
extern osThreadId_t CAN_periodHandle;
extern osThreadId_t Accelerometer_runHandle;
extern osThreadId_t Send_periodicHandle;
extern osThreadId_t SBR1Handle;
extern osThreadId_t SBR2Handle;
extern osThreadId_t SBR3_4Handle;
extern osThreadId_t SBR5Handle;
extern osThreadId_t SBR6Handle;
extern osThreadId_t SBR7Handle;

static void Accelerometer_reset(void)
{
   	SPI_STOP_FLAG=false;
	  UARTRXcmd[0]=0;
		ctr0=0;
		timeX=-100.5;
		ctr2=0;
		timeY=-100.5;
}

void vApplicationIdleHook( void )
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);//green
	HAL_UART_Receive(&huart4,UARTRXcmd,sizeof(UARTRXcmd),3000);
  Input.method=UARTRXcmd[0];
	Input.testNumber=UARTRXcmd[1];
	Input.has_accDataNumber=UARTRXcmd[2];
	Input.accDataNumber=UARTRXcmd[3];
	Input.has_Seatbelt_position=UARTRXcmd[4];
	Input.Seatbelt_position=UARTRXcmd[5];	
	Input.has_vehicle_speed=UARTRXcmd[7];
	Input.vehicle_speed=UARTRXcmd[8];
	Input.has_VehicleStateExtended=UARTRXcmd[9];
	Input.VehicleStateExtended=UARTRXcmd[10];
	if(Input.method==0x01)//SET
	  {
	  if(Input.testNumber == 0x01)
	    {
		    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		    xTaskNotifyGive(Init_testHandle);
	    }
	  else if(Input.testNumber ==  0x02)
      {
		    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		    xTaskNotifyGive(CAN_periodHandle);
	    }
		else if(Input.testNumber ==  0x03)
      {
		    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		    xTaskNotifyGive(Accelerometer_runHandle);
	    }
		else if(Input.testNumber ==  0x04)
      {
		    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		    xTaskNotifyGive(SBR1Handle);			
	    }
		else if(Input.testNumber ==  0x05)
      {
		    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		    xTaskNotifyGive(SBR2Handle);			
	    }
		else if(Input.testNumber ==  0x06)
      {
		    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		    xTaskNotifyGive(SBR3_4Handle);			
	    }
		else if(Input.testNumber ==  0x07)
      {
		    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		    xTaskNotifyGive(SBR5Handle);			
	    }
     else if(Input.testNumber ==  0x08)
      {
		    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		    xTaskNotifyGive(SBR6Handle);			
	    }
     else if(Input.testNumber ==  0x09)
      {
		    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		    xTaskNotifyGive(SBR7Handle);			
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
		ulTaskNotifyTake( pdFALSE,portMAX_DELAY );
		FDCAN_ENABLE_INTERRUPTS();//ENABLE FDCAN INTERRUPT LINE0
		time=0;
    xTaskNotifyStateClear(Init_testHandle);		
  }
 
}

void CAN_2_RUN(void *argument)
{
	/*Certain implementation is given in HAL_FDCAN_RxFifo0Callback*/
  /* Infinite loop */
  for(;;)
  {
		ulTaskNotifyTake(0,portMAX_DELAY);
		FDCAN_ENABLE_INTERRUPTS();
    for(uint16_t j=0;j<0xFFFF;j++){__NOP();}
		xTaskNotifyStateClear(CAN_periodHandle);
  }
}


void Accelerometer1_RUN(void *argument)
{
  /* USER CODE BEGIN Accelerometer1_RUN */
	/*Runs the accelerometer emualator, set of accelerations is depended on byte received via UART
	Frame forming is given in HAL_SPI_TxRxCpltCallback
	Period of AirbagCrashOrder signal is being measured in HAL_FDCAN_RxFifo0Callback*/
	
  /* Infinite loop */
  for(;;)
  {
		ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		FDCAN_ENABLE_INTERRUPTS();
    while(timeX<0)
		{		
		HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
    HAL_Delay(25);//Must be removed
		}
		HAL_GPIO_WritePin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin,GPIO_PIN_SET);
		time = 0;
		while(timeX<300&&SPI_STOP_FLAG==false)
		{
			HAL_SPI_TransmitReceive_IT(&hspi3,SPI_resp,SPI_RXbuf,4);//1000);	
      HAL_Delay(25);//Must be removed
		}
		Accelerometer_reset();
		xTaskNotifyStateClear(Accelerometer_runHandle);
  }
  /* USER CODE END Accelerometer1_RUN */
}
void Send_periodic_start(void *argument)
{
	/*Задача разблокируется при запуске любого из тестов SBR, отправляемые сообщения
	зависят от полученной команды*/
	uint8_t BCM_CANHS_R_04_data_ER[8]={0x70,0,0,0,0,0,0,0};	//engine running
	uint8_t BCM_CANHS_R_04_data_S[8]={0x00,0,0,0,0,0,0,0};	//sleeping
	uint8_t Vehicle_Speed_15kmh[8]={0x05,0xDC,0,0,0,0,0,0};
	uint8_t Vehicle_Speed_40kmh[8]={0x0F,0xA0,0,0,0,0,0,0};
	uint8_t *Speed,*VehicleState;
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
			  else if(Input.Door_position==Rear_passenger1)
			  {
			    BCM_CANHS_R_04_data_ER[7]|=REAR_PASSENGER1_DOOR_OPEN;
			  }
			  else if(Input.Door_position==Rear_passenger2)
			  {
			    BCM_CANHS_R_04_data_ER[7]|=REAR_PASSENGER2_DOOR_OPEN;
			  }
			}			
		if(Input.has_vehicle_speed==true)
			{
				if(Input.vehicle_speed==_40KMH)
					{
				    Speed=Vehicle_Speed_40kmh;
					}	
		 
				if(Input.vehicle_speed==_15KMH)
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
		while(1)
		{
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BCM_CANHS_R_04,VehicleState);
		if(Input.has_vehicle_speed==true)
			{
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BRAKE_CANHS_R_01,Speed);
		  }
		HAL_Delay(100);
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
		ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		xTaskNotifyGive(Send_periodicHandle);
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
			else if(Input.Seatbelt_position==Rear_passenger1)
			{
			  SB_PORT=SB_BP1_CTRL_GPIO_Port;
			  SB_PIN=SB_BP1_CTRL_Pin;
			}
			else if(Input.Seatbelt_position==Rear_passenger2)
			{
			  SB_PORT=SB_BP2_CTRL_GPIO_Port;
			  SB_PIN=SB_BP2_CTRL_Pin;
			}
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
		Output.frame[1]=ReceivedFrame;
		Output.frame_count++;
		Output.method=Method_GET;
		Output.testNumber=Input.testNumber;
		Output.has_Seatbelt_position=Input.has_Seatbelt_position;
		Output.Seatbelt_position=Input.Seatbelt_position;
		
		pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
		FDCAN_DISABLE_INTERRUPTS();
    UARTRXcmd[0]=0;
    UARTRXcmd[1]=0;		
    	/*Used for debug purposes*/
 /* pb_istream_t streamrd = pb_istream_from_buffer(Result, sizeof(Result));
    pb_decode(&streamrd, TestData_fields, &Debug);*/
		xTaskNotifyStateClear(Send_periodicHandle);
		xTaskNotifyStateClear(SBR1Handle);
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
		ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		xTaskNotifyGive(Send_periodicHandle);
			
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
			else if(Input.Seatbelt_position==Rear_passenger1)
			{
			  SB_PORT=SB_BP1_CTRL_GPIO_Port;
			  SB_PIN=SB_BP1_CTRL_Pin;
			}
			else if(Input.Seatbelt_position==Rear_passenger2)
			{
			  SB_PORT=SB_BP2_CTRL_GPIO_Port;
			  SB_PIN=SB_BP2_CTRL_Pin;
			}
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);		
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
		ReceivedFrame.timestamp=RxHeader.RxTimestamp;
    ReceivedFrame.id=RxHeader.Identifier;				
		ReceivedFrame.length=(RxHeader.DataLength)>>16;
		ReceivedFrame.data.size=ReceivedFrame.length;
		memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
		Output.frame[0]=ReceivedFrame;
		Output.frame_count++;
		Output.method=Method_GET;
		Output.testNumber=Input.testNumber;
		Output.has_Seatbelt_position=Input.has_Seatbelt_position;
		Output.Seatbelt_position=Input.Seatbelt_position;	
		pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
		FDCAN_DISABLE_INTERRUPTS();
    UARTRXcmd[0]=0;
    UARTRXcmd[1]=0;		
		/*Used for debug purposes*/
 /* pb_istream_t streamrd = pb_istream_from_buffer(Result, sizeof(Result));
    pb_decode(&streamrd, TestData_fields, &Debug);*/
		xTaskNotifyStateClear(Send_periodicHandle);
		xTaskNotifyStateClear(SBR2Handle);
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
		ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		xTaskNotifyGive(Send_periodicHandle);
			
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
			else if(Input.Seatbelt_position==Rear_passenger1)
			{
			  SB_PORT=SB_BP1_CTRL_GPIO_Port;
			  SB_PIN=SB_BP1_CTRL_Pin;
			}
			else if(Input.Seatbelt_position==Rear_passenger2)
			{
			  SB_PORT=SB_BP2_CTRL_GPIO_Port;
			  SB_PIN=SB_BP2_CTRL_Pin;
			}
    HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
    osDelay(1000);			
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
		ReceivedFrame.timestamp=RxHeader.RxTimestamp;
    ReceivedFrame.id=RxHeader.Identifier;				
		ReceivedFrame.length=(RxHeader.DataLength)>>16;
		ReceivedFrame.data.size=ReceivedFrame.length;
		memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
		Output.frame[0]=ReceivedFrame;
		Output.frame_count++;
		Output.method=Method_GET;
		Output.testNumber=Input.testNumber;
		Output.has_Seatbelt_position=Input.has_Seatbelt_position;
		Output.Seatbelt_position=Input.Seatbelt_position;	
		pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
		FDCAN_DISABLE_INTERRUPTS();
    UARTRXcmd[0]=0;
    UARTRXcmd[1]=0;		
		/*Used for debug purposes*/
 /* pb_istream_t streamrd = pb_istream_from_buffer(Result, sizeof(Result));
    pb_decode(&streamrd, TestData_fields, &Debug);*/
		xTaskNotifyStateClear(Send_periodicHandle);
		xTaskNotifyStateClear(SBR3_4Handle);
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
		ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		xTaskNotifyGive(Send_periodicHandle);
			
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
		else if(Input.Seatbelt_position==Rear_passenger1)
			{
			  SB_PORT=SB_BP1_CTRL_GPIO_Port;
			  SB_PIN=SB_BP1_CTRL_Pin;
				delay=63000;
			}
		else if(Input.Seatbelt_position==Rear_passenger2)
			{
			  SB_PORT=SB_BP2_CTRL_GPIO_Port;
			  SB_PIN=SB_BP2_CTRL_Pin;
				delay=63000;
			}
	
	  HAL_GPIO_WritePin(SB_PORT,SB_PIN,fastened);
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
			
		Output.method=Method_GET;
		Output.testNumber=Input.testNumber;
		Output.has_Seatbelt_position=Input.has_Seatbelt_position;
		Output.Seatbelt_position=Input.Seatbelt_position;	
		pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
		FDCAN_DISABLE_INTERRUPTS();
    UARTRXcmd[0]=0;
    UARTRXcmd[1]=0;		
		/*Used for debug purposes*/
 /* pb_istream_t streamrd = pb_istream_from_buffer(Result, sizeof(Result));
    pb_decode(&streamrd, TestData_fields, &Debug);*/
		xTaskNotifyStateClear(Send_periodicHandle);
		xTaskNotifyStateClear(SBR5Handle);
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
		ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		xTaskNotifyGive(Send_periodicHandle);			
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
		else if(Input.Seatbelt_position==Rear_passenger1)
			{
			  SB_PORT=SB_BP1_CTRL_GPIO_Port;
			  SB_PIN=SB_BP1_CTRL_Pin;
				delay=63000;
			}
		else if(Input.Seatbelt_position==Rear_passenger2)
			{
			  SB_PORT=SB_BP2_CTRL_GPIO_Port;
			  SB_PIN=SB_BP2_CTRL_Pin;
				delay=63000;
			}
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
		Output.frame[1]=ReceivedFrame;
		Output.frame_count++;	
			
		Output.method=Method_GET;
		Output.testNumber=Input.testNumber;
		Output.has_Seatbelt_position=Input.has_Seatbelt_position;
		Output.Seatbelt_position=Input.Seatbelt_position;	
		pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
		FDCAN_DISABLE_INTERRUPTS();
    UARTRXcmd[0]=0;
    UARTRXcmd[1]=0;		
		/*Used for debug purposes*/
 /* pb_istream_t streamrd = pb_istream_from_buffer(Result, sizeof(Result));
    pb_decode(&streamrd, TestData_fields, &Debug);*/
		xTaskNotifyStateClear(Send_periodicHandle);
		xTaskNotifyStateClear(SBR6Handle);
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
		ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		xTaskNotifyGive(Send_periodicHandle);			
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
		else if(Input.Seatbelt_position==Rear_passenger1)
			{
			  SB_PORT=SB_BP1_CTRL_GPIO_Port;
			  SB_PIN=SB_BP1_CTRL_Pin;
			}
		else if(Input.Seatbelt_position==Rear_passenger2)
			{
			  SB_PORT=SB_BP2_CTRL_GPIO_Port;
			  SB_PIN=SB_BP2_CTRL_Pin;
			}
	  HAL_GPIO_WritePin(SB_PORT,SB_PIN,unfastened);
		SEND_DOORSTATE=true;
		osDelay(1000);	
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
		ReceivedFrame.timestamp=RxHeader.RxTimestamp;
    ReceivedFrame.id=RxHeader.Identifier;				
		ReceivedFrame.length=(RxHeader.DataLength)>>16;
		ReceivedFrame.data.size=ReceivedFrame.length;
		memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));	
		Output.frame[0]=ReceivedFrame;
		Output.frame_count++;			
		Output.method=Method_GET;
		Output.testNumber=Input.testNumber;
		Output.has_Seatbelt_position=Input.has_Door_position;
		Output.Seatbelt_position=Input.Door_position;	
		pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
		FDCAN_DISABLE_INTERRUPTS();
    UARTRXcmd[0]=0;
    UARTRXcmd[1]=0;		
		/*Used for debug purposes*/
 /* pb_istream_t streamrd = pb_istream_from_buffer(Result, sizeof(Result));
    pb_decode(&streamrd, TestData_fields, &Debug);*/
		xTaskNotifyStateClear(Send_periodicHandle);
		xTaskNotifyStateClear(SBR1Handle);
  }
}

