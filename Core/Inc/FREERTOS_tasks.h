/*Private defines*/
#define fastened GPIO_PIN_SET
#define unfastened GPIO_PIN_RESET
#define _40KMH 1
#define _15KMH 0
#define DRIVER_DOOR_OPEN 0b00001000
#define FRONT_PASSENGER_DOOR_OPEN 0b00100000
#define REAR_PASSENGER_RIGHT_DOOR_OPEN 0b00100000
#define REAR_PASSENGER_LEFT_DOOR_OPEN 0b10000000
#define Sleeping 0
#define EngineRunning 1
#define _NO_RX_FIFO1_NEW_MESSAGE ((FDCAN1->RXF1S)&0x00FF0000)>>16==Put_index1

/*Masks to generate key to enter security access*/
#define Mask02 0xE94A2291 // ECUProgrammingSession
#define Mask03 0xD2944523 // ExtendedDiagnosticSession 
#define FIFO0 0
#define FIFO1 1
/*Task handling functions*/
void vApplicationIdleHook( void );
void Init_test_run(void *argument);
void CAN_2_RUN(void *argument);
void Accelerometer1_RUN(void *argument);
void Accelerometer_period_RUN(void *argument);
void Send_periodic_start(void *argument);

void SBR1_RUN(void *argument);
void SBR2_RUN(void *argument);
void SBR3_4_RUN(void *argument);
void SBR5_RUN(void *argument);
void SBR6_RUN(void *argument);
void SBR7_RUN(void *argument);

void UDS1_RUN(void *argument);
void UDS2_RUN(void *argument);
void UDS3_RUN(void *argument);
void UDS4a_RUN(void *argument);
void UDS4b_RUN(void *argument);
void UDS5_RUN(void *argument);
void UDS6_RUN(void *argument);

void EDR_Transmitter(void *argument);
void DIAG1_RUN(void *argument);
void DIAG2_3_RUN(void *argument);
void DIAG4_RUN(void *argument);
void DIAG5_RUN(void *argument);
void DIAG6_RUN(void *argument);
void DIAG7_8_RUN(void *argument);
void DIAG9_RUN(void *argument);
//void DIAG10_RUN(void *argument);


/*Exported task prototypes*/
extern osThreadId_t Init_testHandle;
extern osThreadId_t CAN_periodHandle;
extern osThreadId_t Accelerometer_periodHandle;
extern osThreadId_t Accelerometer_runHandle;
extern osThreadId_t Send_periodicHandle;

extern osThreadId_t UDS1Handle;
extern osThreadId_t UDS2Handle;
extern osThreadId_t UDS3Handle;
extern osThreadId_t UDS4aHandle;
extern osThreadId_t UDS4bHandle;
extern osThreadId_t UDS5Handle;
extern osThreadId_t UDS6Handle;

extern osThreadId_t SBR1Handle;
extern osThreadId_t SBR2Handle;
extern osThreadId_t SBR3_4Handle;
extern osThreadId_t SBR5Handle;
extern osThreadId_t SBR6Handle;
extern osThreadId_t SBR7Handle;

extern osThreadId_t EDRHandle;


extern osThreadId_t DIAG1Handle;
extern osThreadId_t DIAG2_3Handle;
extern osThreadId_t DIAG4Handle;
extern osThreadId_t DIAG5Handle;
extern osThreadId_t DIAG6Handle;
extern osThreadId_t DIAG7_8Handle;
extern osThreadId_t DIAG8Handle;
extern osThreadId_t DIAG9Handle;
//extern osThreadId_t DIAG10Handle;


