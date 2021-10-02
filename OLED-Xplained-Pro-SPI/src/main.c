/************************************************************************
 * 5 semestre - Eng. da Computao - Insper
 * Rafael Seicali Malcervelli - rafaelsm9@al.insper.edu.br
 *
 * Material:
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 *
 * Objetivo:
 *  - Simulado para a Prova Intermediária
 *
 * Log:
 *  - 10/2021: Criação
 ************************************************************************/

/************************************************************************/
/* includes                                                             */
/************************************************************************/
#include <asf.h>
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/
// LED1
#define LED1_PIO      PIOA
#define LED1_PIO_ID   ID_PIOA
#define LED1_IDX      0
#define LED1_IDX_MASK (1 << LED1_IDX)

// LED2
#define LED2_PIO      PIOC
#define LED2_PIO_ID   ID_PIOC
#define LED2_IDX      30
#define LED2_IDX_MASK (1 << LED2_IDX)

// LED3
#define LED3_PIO      PIOB
#define LED3_PIO_ID   ID_PIOB
#define LED3_IDX      2
#define LED3_IDX_MASK (1 << LED3_IDX)

// Botao 1
#define BUT1_PIO      PIOD
#define BUT1_PIO_ID   ID_PIOD
#define BUT1_IDX      28
#define BUT1_IDX_MASK (1 << BUT1_IDX)

// Botao 2
#define BUT2_PIO      PIOC
#define BUT2_PIO_ID   ID_PIOC
#define BUT2_IDX      31
#define BUT2_IDX_MASK (1 << BUT2_IDX)

// Botao 3
#define BUT3_PIO      PIOA
#define BUT3_PIO_ID   ID_PIOA
#define BUT3_IDX      19
#define BUT3_IDX_MASK (1 << BUT3_IDX)

/*  Interrupt Edge detection is active. */
#define PIO_IT_EDGE             (1u << 6)

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile char but1_flag;
volatile char but2_flag;
volatile char but3_flag;
volatile char flag_tc0 = 0;
volatile char flag_tc1 = 0;
volatile char flag_tc2 = 0;

/************************************************************************/
/* prototype                                                            */
/************************************************************************/
void LED1_init(int estado);
void LED2_init(int estado);
void LED3_init(int estado);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
void io_init(void);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/
void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc0 = 1;
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc1 = 1;
}

void TC2_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc2 = 1;
}

/************************************************************************/
/* fun??es                                                              */
/************************************************************************/
void but1_callback(void){
	if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		but1_flag = 1;
	}
}

void but2_callback(void){
	if (!pio_get(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK)) {
		// PINO == 0 --> Borda de descida
		but2_flag = 1;
	}
}

void but3_callback(void){
	if (pio_get(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		but3_flag = 1;
	}
}

/**
* @Brief Inicializa o pino do LED1
*/
void LED1_init(int estado){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_IDX_MASK, estado, 0, 0);
};

/**
* @Brief Inicializa o pino do LED2
*/
void LED2_init(int estado){
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_IDX_MASK, estado, 0, 0);
};

/**
* @Brief Inicializa o pino do LED3
*/
void LED3_init(int estado){
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_IDX_MASK, estado, 0, 0);
};

/*
 * @Brief Pisca LED1 placa
 */
void pisca_led1(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED1_PIO, LED1_IDX_MASK);
    delay_ms(t);
    pio_set(LED1_PIO, LED1_IDX_MASK);
    delay_ms(t);
  }
}

/*
 * @Brief Pisca LED2 placa
 */
void pisca_led2(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED2_PIO, LED2_IDX_MASK);
    delay_ms(t);
    pio_set(LED2_PIO, LED2_IDX_MASK);
    delay_ms(t);
  }
}

/*
 * @Brief Pisca LED3 placa
 */
void pisca_led3(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED3_PIO, LED3_IDX_MASK);
    delay_ms(t);
    pio_set(LED3_PIO, LED3_IDX_MASK);
    delay_ms(t);
  }
}

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void io_init(void){

	// Configura led1
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);
	
	// Configura led2
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT);
	
	// Configura led3
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_IDX_MASK, PIO_DEFAULT);

	// ------ BOTAO 1------
	// Inicializa clock do perif?rico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_IDX_MASK, 60);

	// Configura interrup??o no pino referente ao botao e associa
	// fun??o de callback caso uma interrup??o for gerada
	// a fun??o de callback ? a: but_callback()
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_IDX_MASK,
	PIO_IT_EDGE,
	but1_callback);

	// Ativa interrup??o
	pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	
	// ------ BOTAO 2------
	// Inicializa clock do perif?rico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_IDX_MASK, 60);

	// Configura interrup??o no pino referente ao botao e associa
	// fun??o de callback caso uma interrup??o for gerada
	// a fun??o de callback ? a: but_callback()
	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_IDX_MASK,
	PIO_IT_EDGE,
	but2_callback);

	// Ativa interrup??o
	pio_enable_interrupt(BUT2_PIO, BUT2_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4
	
	// ------ BOTAO 3------
	// Inicializa clock do perif?rico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_IDX_MASK, 60);

	// Configura interrup??o no pino referente ao botao e associa
	// fun??o de callback caso uma interrup??o for gerada
	// a fun??o de callback ? a: but_callback()
	pio_handler_set(BUT3_PIO,
	BUT3_PIO_ID,
	BUT3_IDX_MASK,
	PIO_IT_EDGE,
	but3_callback);

	// Ativa interrup??o
	pio_enable_interrupt(BUT3_PIO, BUT3_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4
}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();

	// Init OLED
	gfx_mono_ssd1306_init();
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/* Configura Leds */
	LED1_init(0);
	LED2_init(1);
	LED3_init(0);
	
	/** Configura timer TC0, canal 0 */
	TC_init(TC0, ID_TC0, 0, 5);
	
	/** Configura timer TC0, canal 1 */
	TC_init(TC0, ID_TC1, 1, 10);
	
	/** Configura timer TC0, canal 2 */
	TC_init(TC0, ID_TC2, 2, 1);

	/* Insert application code here, after the board has been initialized. */
	while(1) {
		if(flag_tc0){
			pisca_led1(1,10);
			flag_tc0 = 0;
		}
		if(flag_tc1){
			pisca_led2(1,10);
			flag_tc1 = 0;
		}
		if(flag_tc2){
			pisca_led3(1,10);
			flag_tc2 = 0;
		}
	}
}
