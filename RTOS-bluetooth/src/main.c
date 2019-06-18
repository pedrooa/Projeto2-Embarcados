#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define AFEC_CHANNEL_RES_PIN 0 //PD30
#define AFEC_CHANNEL_RES_PIN1 8 //PA19
#define AFEC_CHANNEL_CX_PIN 1 //PC13
#define AFEC_CHANNEL_CY_PIN 6 //PC31

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)


//BOTOES

//Configs button A
#define BUTA_PIO           PIOD
#define BUTA_PIO_ID        ID_PIOD
#define BUTA_PIO_IDX       11u
#define BUTA_PIO_IDX_MASK  (1u << BUTA_PIO_IDX)

//Configs button B
#define BUTB_PIO           PIOA
#define BUTB_PIO_ID        ID_PIOA
#define BUTB_PIO_IDX       6u
#define BUTB_PIO_IDX_MASK  (1u << BUTB_PIO_IDX)

//Configs button Z
#define BUTZ_PIO           PIOA
#define BUTZ_PIO_ID        ID_PIOA
#define BUTZ_PIO_IDX       24u
#define BUTZ_PIO_IDX_MASK  (1u << BUTZ_PIO_IDX)


//Configs button Start
#define BUTSTART_PIO           PIOA
#define BUTSTART_PIO_ID        ID_PIOA
#define BUTSTART_PIO_IDX       2u
#define BUTSTART_PIO_IDX_MASK  (1u << BUTSTART_PIO_IDX)


// usart (bluetooth)
#define USART_COM_ID ID_USART0
#define USART_COM    USART0


/** RTOS  */
#define TASK_PROCESS_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_PROCESS_STACK_PRIORITY        (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

//Semaforos
SemaphoreHandle_t xSemaphoreStart;
SemaphoreHandle_t xSemaphoreA;
SemaphoreHandle_t xSemaphoreB;
SemaphoreHandle_t xSemaphoreZ;
SemaphoreHandle_t xSemaphorePower;

/** prototypes */
void but_callback(void);
static void ECHO_init(void);
static void USART1_init(void);
uint32_t usart_puts(uint8_t *pstring);

QueueHandle_t xQueue1;
QueueHandle_t xQueueBUTA;
QueueHandle_t xQueueBUTB;
QueueHandle_t xQueueBUTZ;
QueueHandle_t xQueueBUTSTART;
QueueHandle_t xQueueAnalogX;
QueueHandle_t xQueueAnalogY;
QueueHandle_t xQueueJX;
QueueHandle_t xQueueJY;
QueueHandle_t xQueueAnalogCX;
QueueHandle_t xQueueAnalogCY;
volatile uint32_t g_tcCv = 0;

/************************************************************************/
/* ADC                                                                  */
/************************************************************************/
/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;

/** The conversion data value */
volatile uint32_t g_res_value = 0;
volatile uint32_t g_res_value1 = 0;
volatile uint32_t g_res_value2 = 0;
volatile uint32_t g_res_value3 = 0;


/************************************************************************/
/* callbacks                                                            */
/************************************************************************/
/**
 * \brief AFEC interrupt callback function.
 */
static void AFEC_Res_callback(void)
{
	g_res_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_RES_PIN);
	afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN1);
	afec_start_software_conversion(AFEC0);
	xQueueSendFromISR( xQueueAnalogX, &g_res_value, 0);
	
}

static void AFEC_Res_callback1(void)
{
	
	g_res_value1 = afec_channel_get_value(AFEC0, AFEC_CHANNEL_RES_PIN1);
	
	afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
	afec_start_software_conversion(AFEC0);
	xQueueSendFromISR( xQueueAnalogY, &g_res_value1, 0);
		
	
}
static void AFEC_Res_callback2(void)
{
	g_res_value2 = afec_channel_get_value(AFEC1, AFEC_CHANNEL_CX_PIN);
		afec_channel_enable(AFEC1, AFEC_CHANNEL_CY_PIN);
		afec_start_software_conversion(AFEC1);
	xQueueSendFromISR( xQueueAnalogCX, &g_res_value2, 0);
}

static void AFEC_Res_callback3(void)
{
	
	g_res_value3 = afec_channel_get_value(AFEC1, AFEC_CHANNEL_CY_PIN);
			afec_channel_enable(AFEC1, AFEC_CHANNEL_CX_PIN);
					afec_start_software_conversion(AFEC1);


	xQueueSendFromISR( xQueueAnalogCY, &g_res_value3, 0);
}
void butA_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreA, &xHigherPriorityTaskWoken);
}

void butB_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreB, &xHigherPriorityTaskWoken);
}

void butZ_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreZ, &xHigherPriorityTaskWoken);
}

void butStart_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreStart, &xHigherPriorityTaskWoken);
}


/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/



char set_analog_result_cx(uint32_t input) {
	if (input > 3600) {
		return '5';//d
		} else if (input < 1100) {
		return '6';//e
		} else {
		return '0';
	}
}

char set_analog_result_cy(uint32_t input) {
	if (input > 3600) {
		return '7';//b
		} else if (input < 1100) {
		return '8';//c
		} else {
		return '0';
	}
}


//Configura envio de comandos pelo bluetooth



void send_command(char buttonStart , char buttonA, char buttonB, char buttonZ,char analog_cx,char analog_cy, char analog_x[], char analog_y[], char eof ){
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, buttonStart);
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, buttonA);
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, buttonB);
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, buttonZ);
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, analog_cx);
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, analog_cy);
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, analog_x[0]);
	int x = 1;
	while(analog_x[x]!= ';'){
		while(!usart_is_tx_ready(USART_COM));
		usart_write(USART_COM, analog_x[x]);
		x+=1;
	}
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, analog_x[x]);
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, analog_y[0]);
	int y = 1;
	while(analog_y[y]!= ';'){
		while(!usart_is_tx_ready(USART_COM));
		usart_write(USART_COM, analog_y[y]);
		y+=1;
	}
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, analog_y[y]);

	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, eof);
}


/**
 * \brief Configure the console UART.
 */

static void configure_console(void){
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

uint32_t usart_puts(uint8_t *pstring){
	uint32_t i ;

	while(*(pstring + i))
		if(uart_is_tx_empty(USART_COM))
			usart_serial_putchar(USART_COM, *(pstring+i++));
}

void io_init(void){
	
		//desativa watchdog timer
	WDT->WDT_MR = WDT_MR_WDDIS;

// Inicializa clock do periférico PIO responsavel pelo botao A
	pmc_enable_periph_clk(BUTA_PIO_ID);

  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
	pio_configure(BUTA_PIO, PIO_INPUT, BUTA_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	//pio_set_debounce_filter(BUTA_PIO,BUTA_PIO_IDX_MASK,200);

  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()
  pio_handler_set(BUTA_PIO,
                  BUTA_PIO_ID,
                  BUTA_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butA_callback);

  // Ativa interrupção
  pio_enable_interrupt(BUTA_PIO, BUTA_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUTA_PIO_ID);
  NVIC_SetPriority(BUTA_PIO_ID, 4); // Prioridade 4s


  //**************************************************************

  // Inicializa clock do periférico PIO responsavel pelo botao B
	pmc_enable_periph_clk(BUTB_PIO_ID);

  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
	pio_configure(BUTB_PIO, PIO_INPUT, BUTB_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()
  pio_handler_set(BUTB_PIO,
                  BUTB_PIO_ID,
                  BUTB_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butB_callback);

  // Ativa interrupção
  pio_enable_interrupt(BUTB_PIO, BUTB_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUTB_PIO_ID);
  NVIC_SetPriority(BUTB_PIO_ID, 4); // Prioridade 4

  //**************************************************************

  // Inicializa clock do periférico PIO responsavel pelo botao Select
	pmc_enable_periph_clk(BUTZ_PIO_ID);

  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up

	pio_configure(BUTZ_PIO, PIO_INPUT, BUTZ_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);


  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()

  pio_handler_set(BUTZ_PIO,
                  BUTZ_PIO_ID,
                  BUTZ_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butZ_callback);

  // Ativa interrupção
  pio_enable_interrupt(BUTZ_PIO, BUTZ_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUTZ_PIO_ID);
  NVIC_SetPriority(BUTZ_PIO_ID, 4); // Prioridade 4


  // **************************************************************

  // Inicializa clock do periférico PIO responsavel pelo botao Start
	pmc_enable_periph_clk(BUTSTART_PIO_ID);

  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
	pio_configure(BUTSTART_PIO, PIO_INPUT, BUTSTART_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()
  pio_handler_set(BUTSTART_PIO,
                  BUTSTART_PIO_ID,
                  BUTSTART_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butStart_callback);

  // Ativa interrupção
  pio_enable_interrupt(BUTSTART_PIO, BUTSTART_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUTSTART_PIO_ID);
  NVIC_SetPriority(BUTSTART_PIO_ID, 4); // Prioridade 4
	
}

static void config_ADC_TEMP_RES(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);
	afec_enable(AFEC1);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);
		afec_init(AFEC1, &afec_cfg);
	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);
		afec_set_trigger(AFEC1, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_0, AFEC_Res_callback, 5);
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_8, AFEC_Res_callback1, 5);
	afec_set_callback(AFEC1, AFEC_INTERRUPT_EOC_1, AFEC_Res_callback2, 5);
	afec_set_callback(AFEC1, AFEC_INTERRUPT_EOC_6, AFEC_Res_callback3, 5);
	

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_RES_PIN, &afec_ch_cfg);
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_RES_PIN1, &afec_ch_cfg);
		afec_ch_set_config(AFEC1, AFEC_CHANNEL_CX_PIN, &afec_ch_cfg);
		afec_ch_set_config(AFEC1, AFEC_CHANNEL_CY_PIN, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_RES_PIN1, 0x200);
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_RES_PIN, 0x200);
	afec_channel_set_analog_offset(AFEC1, AFEC_CHANNEL_CX_PIN, 0x200);
	afec_channel_set_analog_offset(AFEC1, AFEC_CHANNEL_CY_PIN, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);
		afec_temp_sensor_set_config(AFEC1, &afec_temp_sensor_cfg);

}

void usart_put_string(Usart *usart, char str[]) {
  usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
  uint32_t rx;
  uint32_t counter = 0;
  
  uint32_t start = xTaskGetTickCount();
  while( (xTaskGetTickCount() - start < timeout_ms) && (counter < bufferlen - 1)) {
    if(usart_read(usart, &rx) == 0) {
      buffer[counter++] = rx;
    }
  }
  buffer[counter] = 0x00;
  return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
  usart_put_string(usart, buffer_tx);
  usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void hc05_config_server(void) {
  sysclk_enable_peripheral_clock(USART_COM_ID);
  usart_serial_options_t config;
  config.baudrate = 9600;
  config.charlength = US_MR_CHRL_8_BIT;
  config.paritytype = US_MR_PAR_NO;
  config.stopbits = false;
  usart_serial_init(USART_COM, &config);
  usart_enable_tx(USART_COM);
  usart_enable_rx(USART_COM);
  
  // RX - PB0  TX - PB1
  pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
  pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
  char buffer_rx[128];
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100); printf(buffer_rx);
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100); printf(buffer_rx);
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100);printf(buffer_rx);
  usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEPedro", 1000);printf(buffer_rx);
  usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);printf(buffer_rx);
  usart_send_command(USART0, buffer_rx, 1000, "AT+PIN5555", 1000);printf(buffer_rx);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_bluetooth(void){
	
		
	xQueueBUTA = xQueueCreate( 10, sizeof( char ) );
	xQueueBUTB = xQueueCreate( 10, sizeof( char ) );
	xQueueBUTZ = xQueueCreate( 10, sizeof( char ) );
	xQueueBUTSTART = xQueueCreate( 10, sizeof( char ) );
	xQueueAnalogX = xQueueCreate( 10, sizeof( int32_t ) );
	xQueueAnalogY = xQueueCreate( 10, sizeof( int32_t ) );
	xQueueAnalogCX = xQueueCreate( 10, sizeof( int32_t ) );
	xQueueAnalogCY = xQueueCreate( 10, sizeof( int32_t ) );

	char eof = 'X';
	char buffer[1024];
  
	printf("Bluetooth initializing \n");
	hc05_config_server();
	hc05_server_init();
	printf("Config done \n");
	config_ADC_TEMP_RES();
	io_init();


	
	afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
	afec_start_software_conversion(AFEC0);
	afec_channel_enable(AFEC1, AFEC_CHANNEL_CX_PIN);
	afec_start_software_conversion(AFEC1);
  
	while(1){
		char buttonA = '0';
		char buttonB = '0';
		char buttonZ = '0';
		char buttonStart = '0';
		char analogcx = '0';
		char analogcy = '0';
		uint32_t Jx = 2150;
		uint32_t Jy = 2150;
		uint32_t Cx = 2150;
		uint32_t Cy = 2150;
		char analog_x[32];
		char analog_y[32];
	
		if (xQueueReceive( xQueueBUTA, &(buttonA), ( TickType_t ) 0 / portTICK_PERIOD_MS) || 
			xQueueReceive( xQueueBUTB, &(buttonB), ( TickType_t ) 0 / portTICK_PERIOD_MS) || 
			xQueueReceive( xQueueBUTZ, &(buttonZ), ( TickType_t ) 0 / portTICK_PERIOD_MS) || 
			xQueueReceive( xQueueBUTSTART, &(buttonStart), ( TickType_t ) 0 / portTICK_PERIOD_MS) || 
			(xQueueReceive( xQueueAnalogY, &(Jy), ( TickType_t ) 5 / portTICK_PERIOD_MS) &&
			xQueueReceive( xQueueAnalogX, &(Jx), ( TickType_t ) 5 / portTICK_PERIOD_MS) &&
			xQueueReceive( xQueueAnalogCY, &(Cy), ( TickType_t ) 5 / portTICK_PERIOD_MS) &&
			xQueueReceive( xQueueAnalogCX, &(Cx), ( TickType_t ) 5 / portTICK_PERIOD_MS))
			)
			{ 
			sprintf(analog_x,";%d;",Jx);
			sprintf(analog_y,";%d;",Jy);
			analogcx = set_analog_result_cx(Cx);
			analogcy = set_analog_result_cy(Cy);
		
			printf("%d\n",Cy);
			printf("%d\n",Cx);
			send_command(buttonStart,buttonA,buttonB,buttonZ,analogcx,analogcy,analog_x,analog_y,eof);
		}
		//vTaskDelay( 10 );
	}
}
void task_buttons(void *pvParameters)
{
	xSemaphoreA = xSemaphoreCreateBinary();
	xSemaphoreB = xSemaphoreCreateBinary();
	xSemaphoreZ = xSemaphoreCreateBinary();
	xSemaphoreStart = xSemaphoreCreateBinary();
	
	
	char buttonA = '1';
	char buttonB = '1'; 
	char buttonStart = '1';
	char buttonZ = '1';

	
	

	if (xSemaphoreA == NULL) {
		printf("falha em criar o semaforo A\n");
	}
	if (xSemaphoreB == NULL) {
		printf("falha em criar o semaforo B\n");
	}
	if (xSemaphoreZ == NULL) {
		printf("falha em criar o semaforo B\n");
	}
	if (xSemaphoreStart == NULL) {
		printf("falha em criar o semaforo B\n");
	}

	while (true) {
		if( xSemaphoreTake(xSemaphoreA, ( TickType_t ) 0) == pdTRUE){

			xQueueSend(xQueueBUTA, &buttonA,1);
		}
		if( xSemaphoreTake(xSemaphoreB, ( TickType_t ) 0) == pdTRUE ){
			xQueueSend(xQueueBUTB, &buttonB,1);
		}
		if( xSemaphoreTake(xSemaphoreZ, ( TickType_t ) 0) == pdTRUE ){
			xQueueSend(xQueueBUTZ, &buttonZ,1);
		}
		if( xSemaphoreTake(xSemaphoreStart, ( TickType_t ) 0) == pdTRUE ){
			xQueueSend(xQueueBUTSTART, &buttonStart,1);
		}
	//vTaskDelay(10);
		
	}
}

void task_afec(void){
	xQueueAnalogX = xQueueCreate( 10, sizeof( int32_t ) );
	xQueueAnalogY = xQueueCreate( 10, sizeof( int32_t ) );
	config_ADC_TEMP_RES();
	afec_start_software_conversion(AFEC0);
	uint32_t Jx;
	uint32_t Jy;
	
	afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
	afec_start_software_conversion(AFEC0);
	
	while(1){
		if (xQueueReceive( xQueueAnalogX, &(Jx), ( TickType_t )  1 / portTICK_PERIOD_MS)) {
			xQueueSend( xQueueJX, &Jx, 1);

		}
		if (xQueueReceive( xQueueAnalogY, &(Jy), ( TickType_t )  1 / portTICK_PERIOD_MS)) {
			xQueueSend( xQueueJY, &Jy, 1);

		}
		
		/* Selecina canal e inicializa convers?o */
		
		vTaskDelay(10);
		
		
	}

	
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void){
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_PROCESS_STACK_SIZE, NULL,	TASK_PROCESS_STACK_PRIORITY, NULL);
	
	
	/* Create task to make buttons work */
	xTaskCreate(task_buttons, "BUTS", TASK_PROCESS_STACK_SIZE, NULL,	TASK_PROCESS_STACK_PRIORITY, NULL);
  
  	/* Create task to make buttons work */
  	//xTaskCreate(task_afec, "AFEC", TASK_PROCESS_STACK_SIZE, NULL,	TASK_PROCESS_STACK_PRIORITY, NULL);
  	
  
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
