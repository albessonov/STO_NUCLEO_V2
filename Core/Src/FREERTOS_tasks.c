
#include "main.h"
#include "cmsis_os.h"
#include "FREERTOS_tasks.h"
#include <stdbool.h>
#include "STO_tests_V2_nanopb.pb.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart4;
extern FDCAN_TxHeaderTypeDef BCM_CANHS_R_04;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern volatile uint32_t time;
extern uint8_t UARTRXcmd[4];
extern uint8_t testbuf[4];
extern uint32_t result_1;//must be moved to first task
extern uint32_t timelist[50];
extern uint8_t SPI_RXbuf[4];
extern uint8_t SPI_resp[4];
extern  float XGE_X[801];
extern  float XGE_X[801];
extern  float XGE_X[801];
extern  float XGE_X[801];
extern const uint8_t Request0x0cmd[];
extern const uint8_t Request0x2cmd[];
extern volatile double timeX,timeY;
extern uint8_t RCOMMAND0x00;
extern uint8_t RCOMMAND0x02;
extern uint32_t ctr2;
extern uint32_t ctr0;
extern volatile bool SPI_STOP_FLAG;
TestData Output, Input,Debug;

extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;


extern osThreadId_t Init_testHandle;
extern osThreadId_t CAN_periodHandle;
extern osThreadId_t Accelerometer_runHandle;


static void Accelerometer_reset(void)
{
   	SPI_STOP_FLAG=false;
	  UARTRXcmd[0]=0;
		ctr0=0;
		timeX=-100.5;
		ctr2=0;
		timeY=-100.5;
}
static void FDCAN_ENABLE_INTERRUPTS(void)
{
	FDCAN1->ILE |= FDCAN_ILE_EINT0; //enable fdcan line 0 interrupt
}
void vApplicationIdleHook( void )
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);//green
	HAL_UART_Receive(&huart4,UARTRXcmd,sizeof(UARTRXcmd),3000);
  Input.method=UARTRXcmd[0];
	Input.testNumber=UARTRXcmd[1];
	Input.has_accDataNumber=UARTRXcmd[2];
	Input.accDataNumber=UARTRXcmd[3];
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
		    xTaskNotifyGive(Accelerometer_runHandle);
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

