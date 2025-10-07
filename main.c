//======================================================================= Controle Motor DC com Ponte H ===================================================================================================================================================================
/*
Firmware para controle de motor DC utilizando m�dulo H-Brigde L298N

Autor: Jo�o Pedro Rey

Data: Setembro 2025
*/
//====================================================================================================================================================================================================================================================

/*Crit�rios de Projeto:

Hardware:
- 02x Motores DC 12V
- 01x Ponte H L298N
- 03x LEDs para indica��o de status
- 03x switchs // Partida/Dire��o // Incremento de PWM // Reset

Firmware:
- L�gica para partida/parada suave dos motores (incremento de duty cycle de 0% a 30%).
- Lg�gica de incremento de PWM: se pressionado continuamenta incrementa o duty cycle dos sinais PWM com steps ~2%, se pressionado e solto incrementa o PWM com step de ~10%.
- L�gica para invers�o de sentido de giro dos motores.

*Obs: Quando ligados os motores devem iniciar o seu movimento em sentido direto

====================================================================================================================================================================================================================================================
Tabela-Verdade para as entradas de habilita��o de dire��o da ponte H

Motor A:
-------------------------------------
IN1		|		IN2		|	Sentido	|
-------------------------------------
1		|		 0		|	Direto	|
-------------------------------------
0		|		 1		|	Reverso	|
-------------------------------------

Motor B:
------------------------------------
IN3	    |		IN4		|	Sentido	|
------------------------------------
1		|		 0		|	Direto	|
------------------------------------
0		|		 1		|	Reverso	|
------------------------------------
/*
//=========================================================================================================================================================================================================================================================================

/*Mapeamento de Hardware:
- Bot�o de partida/dire��o		-> PC5 (A5)
- Bot�o de Incremento de PWM	-> PC4 (A4)
- Bot�o de Reset				-> PC6 (RESET)
- LED verde						-> PC0 (A0)
- LED amarelo					-> PC1 (A1)
- LED vermelho					-> PC2 (A2)
- PWM A // OC0A // Timer0		-> PD6 (D6)
- PWM B // OC0B // Timer		-> PD5 (D5)
- Habilita��o de Dire��o IN1	-> PB4 (D12)
- Habilita��o de Dire��o IN2	-> PB3 (D11)
- Habilita��o de Dire��o IN3	-> PB2 (D10)
- Habilita��o de Dire��o IN4	-> PB1 (D9)
*/

//========================================================================== Frequ�ncia de Clock do MCU =====================================================================================================================================================================

#define F_CPU 16000000

//============================================================================ Bibliotecas Utilizadas =======================================================================================================================================================================

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//============================================================================ Mapeamento de Hardware =======================================================================================================================================================================

#define duty_in 100		//Valor de duty cycle inicial
#define duty_max 255	//Valor m�ximo de duty cycle
#define G_LED (1<<PC0)	//LED Verde
#define Y_LED (1<<PC1)	//LED Amarelo
#define R_LED (1<<PC2)	//LED Vermelho
#define BT1	  (1<<PC5)	//Bot�o de acionamento/sentido dos motores
#define	BT2	  (1<<PC4)	//Bot�o de incremento de PWM
#define IN1	  (1<<PB4)	//Sinal de habilita��o de dire��o IN1
#define IN2	  (1<<PB3)	//Sinal de habilita��o de dire��o IN2
#define IN3	  (1<<PB2)	//Sinal de habilita��o de dire��o IN3
#define IN4	  (1<<PB1)	//Sinal de habilita��o de dire��o IN4
#define PWMA  (1<<PD6)	//Sinal PWM motor A
#define PWMB  (1<<PD5)	//Sinal PWM motor B

//============================================================================ Prot�tipos de Fun��es =======================================================================================================================================================================

void READ_BT1 ();		//Fun��o de leitura do bot�o de acionamento/sentido
void READ_BT2 ();		//Fun��o de leitura do bot�o de incrementp de PWM
void dir_enable();		//Fun��o de invers�o de sentido
void PWM_enable();		//Fun��o de incremento de PWM
void motor_enable();	//Fun��o de liga/desliga motores

//============================================================================ Vari�veis Globais =======================================================================================================================================================================


/*Flags para indicar que o bot�o foi pressionado*/

volatile uint8_t	BT1_f		= 0x00,
					BT2_f		= 0x00;

/*Flags para indicar que o bot�o foi pressionado e mantido por 1 segundo*/

volatile uint8_t	BT1_hold_f	= 0x00,
					BT2_hold_f	= 0x00,
					RLED_blink	= 0x00;

/*Contadores*/

volatile uint16_t	BT1_timer	= 0x00,
					BT2_timer	= 0x00,
					G_LED_timer = 0x00,
					blink_timer	= 0x00,
					duty_timer	= 0x00,
					dir_timer	= 0x00,
					debounce	= 0x00,
					duty_inc	= 0x00;

/*Flag de habilita��o*/

volatile uint8_t	PWM_f1		= 0x00,		//Flag 1 de habilita��o de incremento de PWM
					PWM_f2		= 0x00,		//Flag 2 de habilita��o de incremento de PWM
					duty_f		= 0x00,		//Flag de tempo de intervalo de incremento de duty
					dir_deadt	= 0X00,		//Flag para garantir um dead-time seguro na iners�o de sentido
					db_flag		= 0x00,		//Flag de debounce
					duty_inc_f	= 0x00;

/*Vari�veis de Habilita��o*/

volatile uint8_t	motor_en	= 0x00,		//Vari�vel de habilita��o dos motores
					dir_en		= 0x00,		//Vari�vel de habilita��o de dire��o
					PWM_en		= 0x00,		//Vari�vel de habilita��o de PWM
					duty		= 0x00;		//Vari�vel para armazenar o valor de duty cycle
					
//============================================================================ Rotina de Interrup��o =======================================================================================================================================================================					
									
ISR(TIMER2_OVF_vect)
{
	TCNT2 = 0x06;
	
	G_LED_timer++;
	duty_timer++;
	dir_timer++;
	duty_inc++;

	
	/*L�gica para piscar o LED verde*/
	
	if(G_LED_timer == 500)
	{
		G_LED_timer = 0x00;
		PORTC ^= G_LED;
		
	}/*End if*/
	
	if(debounce < 20)
	{
		debounce++;
				
	}/*End if*/
	
	else if(debounce == 20)
	{
		debounce	= 0x00;
		db_flag		= 0x01;
		
	}/*End else*/
	
	if(duty_timer == 20)
	{
		duty_timer	= 0x00;
		duty_f		= 0x01;
			
	}/*End if duty timer*/
	
	if(duty_inc == 15)
	{
		duty_inc   = 0x00;
		duty_inc_f = 0x01;
		
	}/*End if duty inc*/
			
	if(dir_timer == 100)
	{
		dir_timer	= 0x00;
		dir_deadt	= 0x01;
				
	}/*End timer*/
				
	/*Quando BT1 � pressionado*/
				
	if(BT1_f)
	{
		/*Incrementa o contador*/
					
		BT1_timer++;
					
		/*Quando o contador atinge 1 segundo, caso o bot�o permane�a pressionado*/
					
		if(BT1_f && BT1_timer == 1500)
		{
			/*Reseta o contador*/
						
			BT1_timer	= 0x00;
						
			/*Habilita flag de hold do BT1*/
						
			BT1_hold_f	= 0x01;
						
			PORTC |= Y_LED;
						
		}/*End if*/
						
	}/*End if*/
						
	if(BT2_f)
	{
		BT2_timer++;
							
		// Ativa modo hold ap�s 200ms
		if(BT2_f && BT2_timer == 1000)
		{
			BT2_hold_f	= 0x01;
								
		}/*End if*/
								
	}/*End if*/
							
	/*L�gica para piscar o  LED vermelho enquanto no modo incremento cont�nuo de PWM*/
	if(RLED_blink)
	{
		blink_timer++;
									
		if(blink_timer >= 200) // Pisca a cada 200ms
		{
			blink_timer = 0x00;
			PORTC ^= Y_LED;
										
		}/*End if*/
										
	}/*End if*/
										
}/*End ISR*/

//=========================================================================================================================================================================================================================================================================
/*Fun��o READ_BT1*/

void READ_BT1()
{
	volatile uint8_t press_button		= !(PINC&BT1);
	static uint8_t last_button_state	= 0x00;
	
	if(press_button && last_button_state == 0 && db_flag)
	{
		BT1_f		= 0x01;
		BT1_timer	= 0x00;
		db_flag		= 0x00;
		last_button_state = 0x01;
		
	}/*End if*/
	
	else if(!press_button && last_button_state == 1 && !BT1_hold_f)
	{		
		if(!press_button && db_flag)
		{	
			BT1_f	= 0x00;
		
			motor_en = !motor_en;	
		
			last_button_state = 0x00;	
			
			db_flag = 0x00;
		
		}/*End if*/
		
	}/*End else if*/
	
	else if(!press_button && last_button_state == 1	&& BT1_hold_f && motor_en)
	{
		if(!press_button && db_flag)
		{
			PORTC &= ~ Y_LED;
			PORTC |= R_LED;
		
			BT1_f		= 0x00;
			BT1_hold_f	= 0x00;
		
			last_button_state = 0x00;
		
			dir_en = !dir_en;		
			
			db_flag = 0x00;
			
		}/*End if*/
		
	}/*End else if*/
	
}/*End READ_BT1*/

//=========================================================================================================================================================================================================================================================================
/*Fun��o motor_enable*/

void motor_enable()
{
	volatile uint8_t local_duty		= duty_f;
	
	duty_f = 0x00;
		
	if(motor_en)
	{
		PORTC |= R_LED;
		
		if(duty < duty_in)
		{
			if(local_duty)
			{
				duty +=5;
				
				if(duty > duty_in) duty = duty_in;
				
				OCR0A = duty;
				OCR0B = duty;
				
				duty_f = 0x00;				
				
			}/*End if*/
			
		}/*End if*/

	}/*End if*/
	
	else if(!motor_en)
	{
		PORTC &= ~ R_LED;
		
		if(duty > 0)
		{
			if(local_duty)
			{
				duty --;
				
				OCR0A = duty;
				OCR0B = duty;
				
				duty_f = 0x00;
				
			}/*End if*/
				
		}/*End if*/
		
	}/*End else if*/
	
}/*End motor enable*/

//=========================================================================================================================================================================================================================================================================
/*Fun��o dir_enable*/

void dir_enable()
{
	volatile uint8_t duty_state = 0x00;

	if(dir_en)
	{
		duty_state = duty;
		
		OCR0A = 0x00;
		OCR0B = 0x00;
		
		PORTB ^=   (IN1 | IN3);
		
		if(dir_timer)
		{			
			PORTB ^=   (IN2 | IN4);
		
			dir_en = 0x00;
			
			duty_f = 0x00;
		
		}/*End duty f*/
		
		OCR0A = duty_state;
		OCR0B = duty_state;
		
		
	}/*End if*/
		
}/*End dir enable*/

//=========================================================================================================================================================================================================================================================================
/*Fun��o READ_BT2*/

void READ_BT2()
{
	volatile uint8_t button_pressed		= !(PINC&BT2);
	
	if(button_pressed && !BT2_f && motor_en)
	{
		BT2_f		= 0x01;
		BT2_timer	= 0x00;
		
		PORTC |=  Y_LED;		
		
		db_flag				= 0x00;
				
	}/*End if*/
	
	else if(button_pressed && BT2_hold_f)
	{
		PWM_f1				= 0x01;
		RLED_blink			= 0x01;
		
	}/*End if*/
	
	if(!button_pressed && BT2_f)
	{
		if(!button_pressed && db_flag)
		{
			PWM_f2				= 0x01;
			BT2_f				= 0x00;
			db_flag				= 0x00;
		
			PORTC |= Y_LED;
			
			if(G_LED_timer)
			{
				PORTC &= ~ Y_LED;
				
			}/*End if*/
			
		}/*End if*/
		
	}/*End if*/
	
	else if(!button_pressed && BT2_hold_f)
	{
		if(!button_pressed && db_flag)
		{
			PWM_f1				= 0x00;
			RLED_blink			= 0x00;
			BT2_timer			= 0x00;
			BT2_f				= 0x00;
			db_flag				= 0x00;
		
			PORTC &= ~ Y_LED;
			
		}/*End if*/
		
	}/*End if*/
	
}/*End READ_BT2*/

//=========================================================================================================================================================================================================================================================================
/*Fun��o PWM_enable*/

void PWM_enable()
{
	if(PWM_f1)
	{		
		if(duty > 0)
		{			
			if(duty_inc_f)
			{
				duty_inc_f = 0x00;
				
				duty ++;
				
				if(duty >= duty_max) duty = duty_max;
				
				OCR0A = duty;
				OCR0B = duty;
				
			}/*End if*/			
			
		}/*End if*/
		
	}/*End if*/
	
	if(PWM_f2)
	{		
		if(duty > 0)
		{	
			duty += 25;
				
			if(duty > duty_max) duty = duty_in;
				
			OCR0A = duty;
			OCR0B = duty;
				
			PWM_f2 = 0x00;
				
		}/*End if*/
				
	}/*End if*/

}/*End PWM enable*/
				
//============================================================================== Fun��o main =======================================================================================================================================================================

int main(void)
{
	//----------------------------------------------------------------- Configura��o dos Registradores ------------------------------------------------------------------------------------------------
	
	DDRC	|=		(G_LED | Y_LED | R_LED);
	PORTC	&= ~	(G_LED | Y_LED | R_LED);
	
	DDRC	&= ~	(BT1 | BT2);
	PORTC   |=		(BT1 | BT2);
	
	/*Inicializa movimento em sentido direto*/
	DDRB	|=		(IN1 | IN2 | IN3 | IN4);
	PORTB   |=		(IN1 | IN3);
	PORTB	&= ~	(IN2 | IN4);
	
	DDRD	|=		(PWMA | PWMB);
	PORTD	&= ~	(PWMA | PWMB);
	
	//------------------------------------------------------------------- Configura��o do Timer 2 -----------------------------------------------------------------------------------------------------
	
	cli();														//Desabilita todas as interrup��es
	TCNT2 = 0x06;												//Seta o Timer2 para iniciar a contagem em 6 (intervalo de 0 a 250)
	TCCR2A = 0x00;												//Configura o Timer2 como modo de opera��o non-PWM
	TCCR2B |= (1<<CS22);										//Pre-Scaler de 64
	TIMSK2 |= (1<<TOIE2);										//Habilita a interrup��o gerada pela flag de overflow do Timer 2 indicada pelo bit TOV2 do registrador TIFR2
	sei();														//Seta a rotina de interrup��o para ser tratada
	
	
	//------------------------------------------------------------------- Configura��o do Timer 0 -----------------------------------------------------------------------------------------------------
	
	TCCR0A |= (1<<COM0A1 | 1<<COM0B1 | 1<<WGM01 | 1<<WGM00);	//Configura o Timer0 como modo Fast PWM
	TCCR0B |= (1<<CS01 | 1<<CS00);								//Pre-scaler de 64 (frequ�ncia aproximadamente de 2kHz)
	OCR0A  = 0x00;												//Inicializa o comparador OCR0A;
	OCR0B  = 0x00;												//Inicializa o comparador OCR0B;
	
	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	while (1)
	{
		READ_BT1();
		motor_enable();
		dir_enable();
		READ_BT2();
		PWM_enable();

	}/*End while*/
	
	return 0;
		
}/*End main*/
