#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* prototypes and types                                                 */
/************************************************************************/
int genius_get_sequence(int level, int *sequence);
void pin_toggle(Pio *pio, uint32_t mask);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) ;



#define LEDBUZZER_PIO         PIOC
#define LEDBUZZER_PIO_ID      ID_PIOC
#define LEDBUZZER_PIO_IDX	13
#define LEDBUZZER_IDX_MASK    (1<<LEDBUZZER_PIO_IDX)

#define LED_1_PIO PIOA
#define LED_1_PIO_ID ID_PIOA
#define LED_1_IDX 0
#define LED_1_IDX_MASK (1 << LED_1_IDX)

#define LED_2_PIO PIOC
#define LED_2_PIO_ID ID_PIOC
#define LED_2_IDX 30
#define LED_2_IDX_MASK (1 << LED_2_IDX)

#define LED_3_PIO PIOB
#define LED_3_PIO_ID ID_PIOB
#define LED_3_IDX 2
#define LED_3_IDX_MASK (1 << LED_3_IDX)

#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO PIOA
#define BUT_2_PIO_ID ID_PIOA
#define BUT_2_IDX 19
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO PIOC
#define BUT_3_PIO_ID ID_PIOC
#define BUT_3_IDX 31
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)


//CRIANDO FILA
QueueHandle_t  xQueueBtn;
SemaphoreHandle_t xSemaphorebut;

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
void but1_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//Se for usar fila
	uint32_t id1 = 1;
	xQueueSendFromISR(xQueueBtn, (void *)&id1, &xHigherPriorityTaskWoken);
	
}

void but2_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//Se for usar fila
	uint32_t id1 = 2;
	xQueueSendFromISR(xQueueBtn, (void *)&id1, &xHigherPriorityTaskWoken);
}

void but3_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//Se for usar fila
	uint32_t id1 = 3;
	xQueueSendFromISR(xQueueBtn, (void *)&id1, &xHigherPriorityTaskWoken);
 }
	
	
void io_init(void) {
	pmc_enable_periph_clk(LEDBUZZER_PIO_ID);
	pmc_enable_periph_clk(LED_1_PIO_ID);
	pmc_enable_periph_clk(LED_2_PIO_ID);
	pmc_enable_periph_clk(LED_3_PIO_ID);
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pmc_enable_periph_clk(BUT_3_PIO_ID);

	pio_configure(LED_1_PIO, PIO_OUTPUT_0, LED_1_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED_2_PIO, PIO_OUTPUT_0, LED_2_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED_3_PIO, PIO_OUTPUT_0, LED_3_IDX_MASK, PIO_DEFAULT);
	pio_configure(LEDBUZZER_PIO, PIO_OUTPUT_0, LEDBUZZER_IDX_MASK, PIO_DEFAULT);


	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);

	pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE,
	but1_callback);
	pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE,
	but3_callback);
	pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE,
	but2_callback);

	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);

	pio_get_interrupt_status(BUT_1_PIO);
	pio_get_interrupt_status(BUT_2_PIO);
	pio_get_interrupt_status(BUT_3_PIO);

	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void TC0_Handler(void) {
	
	volatile uint32_t status = tc_get_status(TC0, 0);
	//pin_toggle(LED_PIO, LED_IDX_MASK);
	pin_toggle(LED_1_PIO, LED_1_IDX_MASK);
	pin_toggle(LEDBUZZER_PIO, LEDBUZZER_IDX_MASK);
}

void TC1_Handler(void) {
	
	volatile uint32_t status = tc_get_status(TC0, 1);
	//pin_toggle(LED_PIO, LED_IDX_MASK);
	pin_toggle(LED_2_PIO, LED_2_IDX_MASK);
	pin_toggle(LEDBUZZER_PIO, LEDBUZZER_IDX_MASK);
}

void TC3_Handler(void) {
	
	volatile uint32_t status = tc_get_status(TC1, 0);
	//pin_toggle(LED_PIO, LED_IDX_MASK);
	pin_toggle(LED_3_PIO, LED_3_IDX_MASK);
	pin_toggle(LEDBUZZER_PIO, LEDBUZZER_IDX_MASK);
}



/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}


static void task_game(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Level: 0", 0, 0, &sysfont);
	io_init();
	int id;
	
	TC_init(TC0, ID_TC0, 0, 1000);
	tc_start(TC0, 0);
	int freq;
	
	
	int nSequence = 0;
	int sequence[512];
	int level = 0;
	
	
	

	for (;;)  {
		
		nSequence = genius_get_sequence(level, sequence);
		
		
		
		 if(xQueueReceive(xQueueBtn,&id,0)){
			  if(id==1){
				//tc_stop(TC0, 0);
				freq=1000;
				TC_init(TC0, ID_TC0, 0, freq);
				tc_start(TC0, 0);
			  }
			  else if(id==2){
				  //tc_stop(TC0, 0);
				  freq=1500;
				  TC_init(TC0, ID_TC1, 1, freq);
				  tc_start(TC0, 1);
			  }
			  else if(id==3){
				   //tc_stop(TC0, 0);
				   freq=2000;
				   TC_init(TC1, ID_TC3, 0, freq);
				   tc_start(TC1, 0);
			  }
		}
		
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
int genius_get_sequence(int level, int *sequence){
	int n = level + 3;

	for (int i=0; i< n ; i++) {
		*(sequence + i) = rand() % 3;
	}

	return n;
}





static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	
	xQueueBtn=xQueueCreate(100, sizeof(int));
	xSemaphorebut=xSemaphoreCreateBinary();

	/* Create task to control oled */
	if (xTaskCreate(task_game, "game", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create game task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
