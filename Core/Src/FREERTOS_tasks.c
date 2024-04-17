
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
extern uint8_t UARTRXcmd[10];
extern uint8_t SPI_RXbuf[4];
extern uint8_t SPI_resp[4];
extern volatile double timeX,timeY;
extern uint32_t ctr2;
extern uint32_t ctr0;
extern volatile bool SPI_STOP_FLAG;
TestData Output, Input,Debug;

extern uint8_t Result[256];


extern osThreadId_t Init_testHandle;
extern osThreadId_t CAN_periodHandle;
extern osThreadId_t Accelerometer_runHandle;
extern osThreadId_t Send_periodicHandle;
extern osThreadId_t SBR1Handle;

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
	Input.has_seatbelt_number=UARTRXcmd[4];
	Input.seatbelt_number=UARTRXcmd[5];
	Input.has_vehicle_speed=UARTRXcmd[6];
	Input.vehicle_speed=UARTRXcmd[7];
	Input.has_VehicleStateExtended=UARTRXcmd[8];
	Input.VehicleStateExtended=UARTRXcmd[9];
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
	/*Certain implementation is given in HAL_FDCAN_RxFifo0Callback*/
	uint8_t BCM_CANHS_R_04_data_ER[8]={0x70,0,0,0,0,0,0,0};	//engine running
	uint8_t BCM_CANHS_R_04_data_S[8]={0x00,0,0,0,0,0,0,0};	//sleeping
	uint8_t Vehicle_Speed_15kmh[8]={0x05,0xDC,0,0,0,0,0,0};
	uint8_t Vehicle_Speed_40kmh[8]={0x0F,0xA0,0,0,0,0,0,0};
	uint8_t *Speed,*VehicleState;
  /* Infinite loop */
  for(;;)
  {
		ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
		if(Input.has_vehicle_speed==1&&Input.vehicle_speed==1)
			{
				Speed=Vehicle_Speed_40kmh;
		  }
		else if(Input.has_vehicle_speed==1&&Input.vehicle_speed==0)
			{
				Speed=Vehicle_Speed_15kmh;
		  }
    if(Input.VehicleStateExtended==0)
		  {
				VehicleState=BCM_CANHS_R_04_data_S;
			}
		else if(Input.VehicleStateExtended==0)
		  {
				VehicleState=BCM_CANHS_R_04_data_ER;
			}
		while(1)
		{
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BCM_CANHS_R_04,VehicleState);
		if(Input.has_vehicle_speed==1)
			{
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&BRAKE_CANHS_R_01,Speed);
		  }
		HAL_Delay(100);
		}
  }
}
void SBR1_RUN(void *argument)
{
	/*Certain implementation is given in HAL_FDCAN_RxFifo0Callback*/
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
			if(Input.seatbelt_number==1)
			{
			  SB_PORT=SB_DR_CTRL_GPIO_Port;
			  SB_PIN=SB_DR_CTRL_Pin;
			}
			else if(Input.seatbelt_number==2)
			{
			  SB_PORT=SB_FP_CTRL_GPIO_Port;
			  SB_PIN=SB_FP_CTRL_Pin;
			}
			else if(Input.seatbelt_number==3)
			{
			  SB_PORT=SB_BP1_CTRL_GPIO_Port;
			  SB_PIN=SB_BP1_CTRL_Pin;
			}
			else if(Input.seatbelt_number==4)
			{
			  SB_PORT=SB_BP2_CTRL_GPIO_Port;
			  SB_PIN=SB_BP2_CTRL_Pin;
			}
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, AIRBAG_CANHS_R_01_data);
		ReceivedFrame.timestamp=RxHeader.RxTimestamp;
    ReceivedFrame.id=RxHeader.Identifier;				
		ReceivedFrame.length=(RxHeader.DataLength)>>16;
		ReceivedFrame.data.size=ReceivedFrame.length;
		memcpy(ReceivedFrame.data.bytes,AIRBAG_CANHS_R_01_data,sizeof(AIRBAG_CANHS_R_01_data));
		Output.frame[0]=ReceivedFrame;
		Output.frame_count++;
		
		
		HAL_GPIO_WritePin(SB_PORT,SB_PIN,GPIO_PIN_RESET);
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
		Output.has_seatbelt_number=Input.has_seatbelt_number;
		Output.seatbelt_number=Input.seatbelt_number;
		
		pb_ostream_t streamwrt = pb_ostream_from_buffer(Result, sizeof(Result));
    pb_encode(&streamwrt, TestData_fields, &Output);
    message_length=streamwrt.bytes_written;
    HAL_UART_Transmit(&huart4,Result,message_length,1000);
		FDCAN_DISABLE_INTERRUPTS();
    UARTRXcmd[0]=0;
    UARTRXcmd[1]=0;
		
    pb_istream_t streamrd = pb_istream_from_buffer(Result, sizeof(Result));
    pb_decode(&streamrd, TestData_fields, &Debug);
  }
}

