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
#include "asf.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/
/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

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
volatile Bool but1_flag;
volatile Bool but2_flag;
volatile Bool but3_flag;
volatile char flag_tc0;
volatile char flag_tc1;
volatile char flag_tc2;
volatile char flag_rtc;
volatile char flag_sec;
volatile char f_rtt_alarme;

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
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void inverte_led(Pio *PIO, const uint32_t MASK, volatile Bool flag);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc0 = 1;
}

void TC4_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC1, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc1 = 1;
}

void TC7_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC2, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc2 = 1;
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		delay_s(5);
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		// pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
		f_rtt_alarme = 1;                  // flag RTT alarme
	}
}

/**
* \brief Interrupt handler for the RTC. Refresh the display.
*/
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* seccond tick	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		flag_sec = 1;
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		flag_rtc = 1;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

/************************************************************************/
/* fun??es                                                              */
/************************************************************************/
void but1_callback(void){
	if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		but1_flag = !but1_flag;
	}
}

void but2_callback(void){
	if (!pio_get(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK)) {
		// PINO == 0 --> Borda de descida
		but2_flag = !but2_flag;
	}
}

void but3_callback(void){
	if (pio_get(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		but3_flag = !but3_flag;
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

void inverte_led(Pio *PIO, const uint32_t MASK, volatile Bool flag){
	if (flag){
		if (pio_get(PIO, PIO_DEFAULT, MASK)){
			pio_clear(PIO, MASK);
			} else{
			pio_set(PIO, MASK);
		}
	}
	else {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
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

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
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
	io_init();
	
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	delay_init();

	f_rtt_alarme = 1;

	// Init OLED
	gfx_mono_ssd1306_init();
	
	/* Configura Leds */
	LED1_init(0);
	LED2_init(1);
	LED3_init(0);
	
	/** Configura timer TC0, canal 1 */
	TC_init(TC0, ID_TC1, 1, 5);
	/** Configura timer TC1, canal 1 */
	TC_init(TC1, ID_TC4, 1, 10);
	/** Configura timer TC2, canal 1 */
	TC_init(TC2, ID_TC7, 1, 1);
	
	calendar rtc_initial = {2021, 3, 19, 12, 20, 49, 20};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	
	char buffer[50];
	gfx_mono_draw_string("5", 1, 15, &sysfont);
	gfx_mono_draw_string("10", 60, 15, &sysfont);
	gfx_mono_draw_string("1", 120, 15, &sysfont);
	
	/* Insert application code here, after the board has been initialized. */
	while(1) {
		if(flag_tc0){
			inverte_led(LED1_PIO, LED1_IDX_MASK, but1_flag);
			flag_tc0 = 0;
		}
		if(flag_tc1){
			inverte_led(LED2_PIO, LED2_IDX_MASK, but2_flag);
			flag_tc1 = 0;
		}
		if(flag_tc2){
			inverte_led(LED3_PIO, LED3_IDX_MASK, but3_flag);
			flag_tc2 = 0;
		}
		if(flag_sec){
			uint32_t h, m, s;
			rtc_get_time(RTC, &h, &m, &s);
			
			sprintf(buffer, "%lu:%lu:%lu", h, m, s);
			gfx_mono_draw_string("            ", 35, 2, &sysfont);
			gfx_mono_draw_string(buffer, 35, 2, &sysfont);
			
			flag_sec = 0;
		}
		if(f_rtt_alarme){      
			/*
			* IRQ (interrup??o ocorre) apos 5s => 5 pulsos por sengundo (0,2s) -> 25 pulsos s?o necess?rios para dar 5s
			* tempo[s] = 0,25 * 25 = 5s
			*/
			uint16_t pllPreScale = (int) (((float) 32768) / 5.0);
			uint32_t irqRTTvalue = 25;
	
			// reinicia RTT para gerar um novo IRQ
			RTT_init(pllPreScale, irqRTTvalue);         
      
			f_rtt_alarme = 0;
		}

		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
