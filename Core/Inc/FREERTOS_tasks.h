void vApplicationIdleHook( void );
void Init_test_run(void *argument);
void CAN_2_RUN(void *argument);
void Accelerometer1_RUN(void *argument);
void Send_periodic_start(void *argument);
void SBR1_RUN(void *argument);

static void Accelerometer_reset(void);
void FDCAN_ENABLE_INTERRUPTS(void);
