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
void EDR_Transmitter(void *argument);





