#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/


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

//Configs button Select
#define BUTSELECT_PIO           PIOA
#define BUTSELECT_PIO_ID        ID_PIOA
#define BUTSELECT_PIO_IDX       24u
#define BUTSELECT_PIO_IDX_MASK  (1u << BUTSELECT_PIO_IDX)

//Configs button Start
#define BUTSTART_PIO           PIOA
#define BUTSTART_PIO_ID        ID_PIOA
#define BUTSTART_PIO_IDX       2u
#define BUTSTART_PIO_IDX_MASK  (1u << BUTSTART_PIO_IDX)

//LEDS

//Configs LED A
#define LEDA_PIO  PIOB
#define LEDA_PIO_ID ID_PIOB
#define LEDA_PIO_IDX 2u
#define LEDA_PIO_IDX_MASK (1u << LEDA_PIO_IDX)


/*
// TRIGGER
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      30
#define LED_IDX_MASK (1 << LED_IDX)*/

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
volatile uint32_t g_tcCv = 0;




/************************************************************************/
/* callbacks                                                            */
/************************************************************************/

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


void pisca_led(uint LED_PIO, uint LED_IDX_MASK){
	pio_set(LED_PIO, LED_IDX_MASK);
	delay_ms(50);
	pio_clear(LED_PIO, LED_IDX_MASK);
}

//Configura envio de comandos pelo bluetooth

/*
void send_command(char buttonStart, char buttonA, char buttonB, char eof ){
			while(!usart_is_tx_ready(USART_COM));
			usart_write(USART_COM, buttonStart);
			while(!usart_is_tx_ready(USART_COM));
			usart_write(USART_COM, buttonA);
			while(!usart_is_tx_ready(USART_COM));
			usart_write(USART_COM, buttonB);
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
}*/

void send_command(char buttonStart , char buttonA, char buttonB, char eof ){
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, buttonStart);
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, buttonA);
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, buttonB);
	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, '0');

	while(!usart_is_tx_ready(USART_COM));
	usart_write(USART_COM, '0');

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

// Inicializa clock do perif�rico PIO responsavel pelo botao A
	pmc_enable_periph_clk(BUTA_PIO_ID);

  // Configura PIO para lidar com o pino do bot�o como entrada
  // com pull-up
	pio_configure(BUTA_PIO, PIO_INPUT, BUTA_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	//pio_set_debounce_filter(BUTA_PIO,BUTA_PIO_IDX_MASK,200);

  // Configura interrup��o no pino referente ao botao e associa
  // fun��o de callback caso uma interrup��o for gerada
  // a fun��o de callback � a: but_callback()
  pio_handler_set(BUTA_PIO,
                  BUTA_PIO_ID,
                  BUTA_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butA_callback);

  // Ativa interrup��o
  pio_enable_interrupt(BUTA_PIO, BUTA_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais pr�ximo de 0 maior)
  NVIC_EnableIRQ(BUTA_PIO_ID);
  NVIC_SetPriority(BUTA_PIO_ID, 4); // Prioridade 4s
  
  
    pmc_enable_periph_clk(LEDA_PIO_ID);
    pio_configure(LEDA_PIO, PIO_OUTPUT_0, LEDA_PIO_IDX_MASK, PIO_DEFAULT);

  //**************************************************************
/*

  // Inicializa clock do perif�rico PIO responsavel pelo botao B
	pmc_enable_periph_clk(BUTB_PIO_ID);

  // Configura PIO para lidar com o pino do bot�o como entrada
  // com pull-up
	pio_configure(BUTB_PIO, PIO_INPUT, BUTB_PIO_IDX_MASK, PIO_PULLUP);

  // Configura interrup��o no pino referente ao botao e associa
  // fun��o de callback caso uma interrup��o for gerada
  // a fun��o de callback � a: but_callback()
  pio_handler_set(BUTB_PIO,
                  BUTB_PIO_ID,
                  BUTB_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butB_callback);

  // Ativa interrup��o
  pio_enable_interrupt(BUTB_PIO, BUTB_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais pr�ximo de 0 maior)
  NVIC_EnableIRQ(BUTB_PIO_ID);
  NVIC_SetPriority(BUTB_PIO_ID, 4); // Prioridade 4

  // **************************************************************

  // Inicializa clock do perif�rico PIO responsavel pelo botao Select
	/ *pmc_enable_periph_clk* /(BUTSELECT_PIO_ID);

  // Configura PIO para lidar com o pino do bot�o como entrada
  // com pull-up
/ *
	pio_configure(BUTSELECT_PIO, PIO_INPUT, BUTSELECT_PIO_IDX_MASK, PIO_PULLUP);
* /

  // Configura interrup��o no pino referente ao botao e associa
  // fun��o de callback caso uma interrup��o for gerada
  // a fun��o de callback � a: but_callback()
/ *
  pio_handler_set(BUTSELECT_PIO,
                  BUTSELECT_PIO_ID,
                  BUTSELECT_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butStart_callback());* /

  // Ativa interrup��o
 // pio_enable_interrupt(BUTSELECT_PIO, BUTSELECT_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais pr�ximo de 0 maior)
  //NVIC_EnableIRQ(BUTSELECT_PIO_ID);
  //NVIC_SetPriority(BUTSELECT_PIO_ID, 4); // Prioridade 4

  // **************************************************************

  // Inicializa clock do perif�rico PIO responsavel pelo botao Start
	pmc_enable_periph_clk(BUTSTART_PIO_ID);

  // Configura PIO para lidar com o pino do bot�o como entrada
  // com pull-up
	pio_configure(BUTSTART_PIO, PIO_INPUT, BUTSTART_PIO_IDX_MASK, PIO_PULLUP);

  // Configura interrup��o no pino referente ao botao e associa
  // fun��o de callback caso uma interrup��o for gerada
  // a fun��o de callback � a: but_callback()
  pio_handler_set(BUTSTART_PIO,
                  BUTSTART_PIO_ID,
                  BUTSTART_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butStart_callback);

  // Ativa interrup��o
  pio_enable_interrupt(BUTSTART_PIO, BUTSTART_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais pr�ximo de 0 maior)
  NVIC_EnableIRQ(BUTSTART_PIO_ID);
  NVIC_SetPriority(BUTSTART_PIO_ID, 4); // Prioridade 4
	

  // Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);*/
}

void usart_put_string(Usart *usart, char str[]) {
  usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
  uint timecounter = timeout_ms;
  uint32_t rx;
  uint32_t counter = 0;
  
  while( (timecounter > 0) && (counter < bufferlen - 1)) {
    if(usart_read(usart, &rx) == 0) {
      buffer[counter++] = rx;
    }
    else{
      timecounter--;
      vTaskDelay(1);
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
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100); printf("AT\n");
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100);
  usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEPedro", 100);
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100);
  usart_send_command(USART0, buffer_rx, 1000, "AT+PIN5555", 100);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_bluetooth(void){
	
		
		xQueueBUTA = xQueueCreate( 10, sizeof( int32_t ) );
		char buttonA = '0';
		char buttonB = '0';
		char buttonStart = '0';
		char eof = 'X';
		char buffer[1024];
  
  printf("Bluetooth initializing \n");
  hc05_config_server();
  hc05_server_init();
  io_init();
  
  while(1){
	if (xQueueReceive( xQueueBUTA, &(buttonA), ( TickType_t ) 10 / portTICK_PERIOD_MS)) {
		send_command(0,buttonA,0,eof);
		printf("A\n");
		
	}
   // vTaskDelay( 10 / portTICK_PERIOD_MS);
  }
}
void task_buttons(void *pvParameters)
{
	xSemaphoreA = xSemaphoreCreateBinary();
	xSemaphoreB = xSemaphoreCreateBinary();
	xSemaphoreZ = xSemaphoreCreateBinary();
	xSemaphoreStart = xSemaphoreCreateBinary();
	
	
	char buttonA = '0';
	char buttonB = '0'; 
	char buttonStart = '0';
	char eof = 'X';
	char buffer[1024];
	
	

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
		if( xSemaphoreTake(xSemaphoreA, ( TickType_t ) 500) == pdTRUE){
			pisca_led(LEDA_PIO,LEDA_PIO_IDX_MASK);
			
			
			
			xQueueSend(xQueueBUTA, '1',0);
		}
		if( xSemaphoreTake(xSemaphoreB, ( TickType_t ) 500) == pdTRUE ){
			
			
		}
		
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
  
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}