#include "lpc17xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "GLCD.h"
#include "audio_muestras.h"

//Relacionado al sistema

#define MANUAL 0 //Modo manual
#define AUTO   1 //Modo automatico

//Relacionado al motor

#define Fpclk 25e6 //Fcpu/4. Valor visto en teoria (Fcpu=100MHz)
#define Tpwm 15e-3 //Periodo de nuestra señal PWM=15ms

#define Pos_Max_Motor 180 //Posicion maxima del motor 90 a la izquierda
#define Pos_Min_Motor 0 //Posicion minima del motor 90 a la derecha
#define T_Pos_Max 2.5e-3 //Pulso posicion maxima
#define T_Pos_Min 0.5e-3 //Pulso posicion minima
#define DERECHA 0 //Sentido de movimiento hacia el minimo
#define IZQUIERDA 1 //Sentido de movimiento hacia el maximo

//Constantes sonar

#define v_sonido 343 //velocidad del sonido en m/s a 20ºC
#define Periodo_sonar 37e-3 //Periodo del pulso del sonar
#define Periodo_trigger 12e-6 //Periodo pulso trigger
#define dist_min 3 //Distancia minima en cm
#define dist_max 150 //Distancia maxima en cm

//Constantes Audio

#define N_muestras 32	// 32 muestras/ciclo
#define Frecuencia2 8000
#define N_muestras2 15000 // igual a la longitud del array generado en Matlab
#define Frecuencia3 8000
#define N_muestras3 16000 //Numero de muestras de audio grabado
#define V_refp 3.3
#define V_refn 0 
#define Pi 3.1415
#define pitido 0 //Valor para que se reproduzca pitido
#define audio 1 //Valor para que se reproduzca mensaje
#define grabacion 2 //Valor para que se reproduzca la grabacion
#define ninguno 3

#define UART_ACCEPTED_BAUDRATE_ERROR    3

//Variables Globales

uint8_t modo=MANUAL; 
uint8_t sentido=DERECHA;
uint8_t grados=90;
uint8_t salto=10;
uint16_t Frecuencia=5000;
uint16_t periodo=500; //Para poder configurar la frecuencia

float distancia=0, cuenta_dist1=0, cuenta_dist2=0, dist=0;

uint8_t token=0, Tim1=0; 

float temperatura=20; //Fijo temperatura a 20º

uint16_t audio_grabado[N_muestras3]; //Vector que contendra el audio grabado
uint16_t i_muestras3=0; //Valor para el vector que almacenara el audio grabado

uint16_t muestreo[N_muestras]; //Vector con las muestras para el pitido
uint16_t i_muestras=0; //Para desplazarme por el vector de muestras
uint16_t i_muestras2=1000; 
uint8_t sonido=pitido;
uint8_t mensaje=audio;


//Variables UART0

uint8_t contUART0=1;
char buffer[30]; //Buffer de recepcion de 30 caracteres
char *ptr_rx; //Puntero de recepcion
char rx_completa; //Flag de recepcion de cadena, se pone a 1 al recibir enter (ascii=13)
char *ptr_tx; //Puntero transmision
char tx_completa; //Flag de transmision de cadena, se pone a 1 al transmitir null (fin de cadena)


void lee_key1() 
{
	if(LPC_GPIO2->FIOPIN==0x000037FF) //Si esta pulsado Key1
	{
		modo=AUTO; //Cambio el modo
		SysTick->CTRL=0x00000007; //Activo SysTick para que empiece a interrumpir
		LPC_TIM0->TCR=1; //Activo Timer0 para pitidos
	}
	else //No esta pulsado
	{
		modo=MANUAL;
	}
}
void configIRQ()
{
	LPC_PINCON->PINSEL4|=(0x01<<20); 		 // P2.10 es entrada interrup. EXT 0
	LPC_PINCON->PINSEL4|=(0x01<<22);	 	 // P2.11 es entrada interrup. EXT 1
	LPC_PINCON->PINSEL4|=(0x01<<24);	 	// P2.12 es entrada interrup. EXT 2
	
	LPC_PINCON->PINSEL4|=0x0550000; //Configuro Key1, Key2 y ISP como IRQ y EINT3
	LPC_SC->EXTMODE=0X0000000F; //Configuro ISP, Key1 y Key2 a flanco
	LPC_SC->EXTPOLAR=0x00000000; //Configuro por flanco de bajada
	
	NVIC_SetPriorityGrouping(2); //Configuro subprioridad a 0
	
	NVIC_SetPriority(18,3); //Fijo prioridad ISP
	NVIC_SetPriority(19,4); //Fijo prioridad Key1
	NVIC_SetPriority(20,4); //Fijo prioridad Key2
	NVIC_SetPriority(21,1);
	
	NVIC->ISER[0]|=0x003C0000; //Habilito interrupciones EINT0, EINT1 y EINT2 y EINT3

}
void config_SysTick()
{
	SysTick->LOAD=100e6*(0.5*periodo/5)-1; //Pulsos para que pasen 0,1s y que no haya desbordamientos
	SysTick->VAL=0; //Limpio
	SysTick->CTRL=0x00000006; //SysTick desactivado
	
	NVIC_SetPriority(-1,0); //Fijo prioridad SysTick
}
void configPWM()
{
	LPC_PINCON ->PINSEL3 |= (2<<4);//seleccionamos el PWM1.1 en la patilla 1.18
	LPC_PWM1->MCR = 0x2;//resetea MR0
	LPC_PWM1->PCR |= (1<<9);//habilita el canal PWM1
	LPC_PWM1->MR0 = Fpclk*Tpwm-1;//fijamos el valor al que se produce el match
	LPC_PWM1->MR1=Fpclk*((T_Pos_Max-T_Pos_Min)*grados/180+T_Pos_Min)-1; //Para que comience a 90
	LPC_PWM1->TCR = 0x2;//reseta TC y vuelve a iniciarlo
	LPC_PWM1->TCR = 0x1;
	LPC_PWM1->LER|=(1<<0)|(1<<1); //Se validan MR0 y MR1
}
void config_Timer2()
{
	LPC_PINCON->PINSEL9|=(1<<27); //Configuro P4.29 como Mat2.1
	LPC_PINCON->PINSEL0|=(0x3<<8); //Configuro P0.4 como Cap2.0
	
	LPC_SC->PCONP|=(1<<22); //Alimento Timer2
	LPC_TIM2->PR=0x0; //Ftick=Fpclk/(PR+1)
	LPC_TIM2->MCR|=(1<<3); //Cuando llegue a MR1 interrumpe
	LPC_TIM2->MCR|=(1<<1); //Cuando llegue a MR0 se reinicia TC
	LPC_TIM2->CCR|=0; //Se configurara en la interrupcion
	LPC_TIM2->EMR|=(1<<6); //Cuando acaba MR1 pongo el pin de salida a 0
	LPC_TIM2->MR0=Fpclk*Periodo_sonar-1; //Periodo sonar
	LPC_TIM2->MR1=Fpclk*Periodo_trigger-1; //Periodo pulso de activacion
	NVIC_SetPriority(TIMER2_IRQn,1); //Prioridad timer 2
	NVIC_EnableIRQ(TIMER2_IRQn); //habilito interrupcion timer2
	LPC_TIM2->TCR=2; //Contador reseteado
}
void TIMER2_IRQHandler()
{
	if (LPC_TIM2->IR==0x2) //Detecto fin de cuenta MR1
	{
		LPC_TIM2->IR|=(1<<1); //Borro flag asociado
		LPC_TIM2->MCR=0x2; //Dejo de interrumpir por Mat2.1
		LPC_TIM2->CCR|=5; //Interrumpira en el flanco de subida de Cap2.0
	}
	else if ((LPC_TIM2->IR==0x10) && (Tim1==0)) //Detecto interrupcion por flanco Cap de subida
	{
		LPC_TIM2->IR|=(0<<4); //Borro flag asociado
		cuenta_dist1=LPC_TIM2->CR0; //Guardo cuentas cuando se produce un flanco
		LPC_TIM2->CCR=6; //Cambio para que interrumpa cuando llegue el flanco de bajada
		Tim1=1; //Para saber en que tipo estoy
	}
	else if ((LPC_TIM2->IR==0x10) && (Tim1==1)) //Detecto interrupcion por flanco Cap de bajada
	{
		LPC_TIM2->IR|=(0<<4); //Borro flag asociado
		cuenta_dist2=LPC_TIM2->CR0; //Guardo cuentas cuando se produce otro flanco
		dist=((v_sonido*((cuenta_dist2-cuenta_dist1)/Fpclk))/2)*(temperatura/20)*100; //Formula para obtener la distancia en cm dependiendo de la temperatura
		LPC_TIM2->CCR=0; //Apago Capture
		LPC_TIM2->TCR=2; //Reseteo y apago timer
		Tim1=0; 
		token=1;
	}
}
void set_servo(uint16_t grados1) //Ajustar el servo a los grados
{
	LPC_PWM1->MR1=Fpclk*((T_Pos_Max-T_Pos_Min)*grados1/180+T_Pos_Min)-1; //Actualizar posicion servo
	LPC_PWM1->LER|=(1<<1); //Se valida MR1
}
void on_manual()
{
	NVIC->ICER[0]|=0x00180000; //Desactivo todas las IRQ
  NVIC->ISER[0]|=0x0024000A; //Activo IRQ solo ISP, TIM2 y Tim0
	NVIC_EnableIRQ(UART0_IRQn);
	LPC_TIM0->TCR=1; //Activo Timer0 para pitidos
	SysTick->CTRL=0x00000007; //Activo SysTick para que empiece a interrumpir
}
void off_manual()
{
	LPC_TIM0->TCR=2; //Reseteo Timer0 para que no pita
	SysTick->CTRL=0x00000006; //Apago SysTick
	NVIC_ClearPendingIRQ (EINT0_IRQn); //Elimino pending de las interrupciones
	NVIC_ClearPendingIRQ (EINT1_IRQn);
	NVIC_ClearPendingIRQ (EINT2_IRQn);
	NVIC->ISER[0]|=0x003C000A; //Vuelvo a activar todas las IRQ
	NVIC_EnableIRQ(UART0_IRQn);
}
void on_auto()
{
	LPC_TIM0->TCR=1; //Activo Timer0 para pitidos
	SysTick->CTRL=0x00000007; //Reanudo SysTick
}
void off_auto()
{
	SysTick->CTRL=0x00000006; //Apago SysTick
	LPC_TIM0->TCR=2; //Apago Tim0 pitidos
}
void EINT0_IRQHandler() //ISP, Activa sonar en modo manual
{
	static uint8_t contISP=0; //contador para interrupciones ISP
	
	LPC_SC->EXTINT|=0x00000001;  //Borro flag para que se pueda repetir
	
	contISP++;
	
	if (modo==MANUAL)
	{
		if (contISP==1)
		{
			on_manual();
		}
		else if (contISP==2)
		{
			contISP=0;
			off_manual();
		}
	}
	else
	{
		if (contISP==1)
		{
			off_auto();
		}
		else if (contISP==2)
		{
			on_auto();
			contISP=0;
		}
	}
}
void EINT1_IRQHandler() //Key1, Movimiento hacia la izquierda 
{	
	uint16_t grados2=grados;
	LPC_SC->EXTINT|=0x00000002;  //Borro flag para que se pueda repetir
	NVIC_ClearPendingIRQ (EINT1_IRQn);
	NVIC_ClearPendingIRQ (EINT2_IRQn);
	
	if (modo==MANUAL)
	{
		if (Pos_Max_Motor<=(grados2+=salto))
		{
			grados=Pos_Max_Motor;
			sentido=DERECHA; //Para que continue en automatico por donde lo he dejado
		}
		else
		{
			grados+=salto;
		}
		
		set_servo(grados);
	}
}
void EINT2_IRQHandler() //Key2, Movimiento hacia la derecha
{
	int16_t grados2=grados;
	LPC_SC->EXTINT|=0x00000004;  //Borro flag para que se pueda repetir
	NVIC_ClearPendingIRQ (EINT1_IRQn);
	NVIC_ClearPendingIRQ (EINT2_IRQn);
	
	if (modo==MANUAL)
	{
		if (Pos_Min_Motor>=(grados2-=salto))
		{
			grados=Pos_Min_Motor;
			sentido=IZQUIERDA; //Para que continue en automatico por donde lo he dejado
		}
		else
		{
			grados-=salto;
		}
		
		set_servo(grados);
	}
}
void SysTick_Handler() //Interrupcion SysTick modo AUTO
{
	static uint8_t cont=0; //Inicializo variable count para contar hasta 5->0,5s
	int16_t grados1=grados, grados2=grados; 
	
	if(modo==AUTO) //Si estoy en modo automatico
	{
		cont++; //Incremento el contador
		
		if(cont==2)
		{
			if (sentido==IZQUIERDA)
			{
				if (Pos_Max_Motor<(grados2+=salto))
				{
					sentido=DERECHA; //Cambio de sentido
				}
				else
				{
					grados+=salto;
				}
				set_servo(grados);
			}
			if (sentido==DERECHA)
			{
				if (Pos_Min_Motor>(grados1-=salto))
				{
					sentido=IZQUIERDA; //Cambio de sentido
				}
				else
				{
					grados-=salto;
				}
				set_servo(grados);
			}
		}
		if(cont==3)
		{
			cont=0; //Se reinicia el contador
			
			 //Mido distancia
			LPC_TIM2->MCR=0xA; //Cuando llegue a MR1 interrumpe
			LPC_TIM2->EMR|=(1<<1); //Pongo bit manualmente a 1
			LPC_TIM2->CCR=0; //Funcion capture desactivada
			LPC_TIM2->TCR=1; //Prescaler y contador sincronizados y activados
		}
	}
	else
	{
		 //Mido distancia
			LPC_TIM2->MCR=0xA; //Cuando llegue a MR1 interrumpe
			LPC_TIM2->EMR|=(1<<1); //Pongo bit manualmente a 1
			LPC_TIM2->CCR=0; //Funcion capture desactivada
			LPC_TIM2->TCR=1; //Prescaler y contador sincronizados y activados
	}
}
void config_DAC_sonido()
{
	LPC_PINCON->PINSEL1|=(1<<21); //Configuro patilla P0.26 como AOUT(DAC)
	LPC_PINCON->PINMODE1|=(1<<21); //El pin no tiene ni Pull-up ni Pull-dows
	LPC_DAC->DACCTRL=0; //DMA desactivado
}
void config_Timer0_sonido()
{
	LPC_SC->PCONP|=(1<<1); //Alimento Timer0
	LPC_TIM0->PR=0; //Ftick=Fpclk/(PR+1)
	LPC_TIM0->MCR|=3; //Me resetea el TC e interrumpe cuando llega a MR0
	LPC_TIM0->CCR=0; //Deshabilitado
	LPC_TIM0->EMR=0; //no me cambia ningun pin, solo necesito contador
	LPC_TIM0->MR0=(Fpclk/Frecuencia/N_muestras)-1; //Para que me genere las muestras
	NVIC_SetPriority(TIMER0_IRQn,1); //Prioridad 
	NVIC_EnableIRQ(TIMER0_IRQn); //habilito interrupcion timer0
	LPC_TIM0->TCR=2; //Contador reseteado
}
void TIMER0_IRQHandler()
{
	LPC_TIM0->IR|=(1<<0); //Borro flag Timer0
	
	if(sonido==pitido)
	{
		LPC_DAC->DACR=muestreo[i_muestras]<<6; //Saco valor de las muestras al DAC
		i_muestras++; //Incremento contados
	
		if(i_muestras==N_muestras)
		{
			i_muestras=0;
			LPC_DAC->DACR=0;
		}
	}
	else if(sonido==audio)
	{
		LPC_DAC->DACR=muestras[i_muestras2++]<<6; //Saco valor de las muestras al DAC
	
		if(i_muestras2==N_muestras2-7400)
		{
			i_muestras2=1000;
			LPC_DAC->DACR=0;
		}
	}
}
void genera_muestras()
{
	for(i_muestras=0;i_muestras<N_muestras;i_muestras++)
	{
		muestreo[i_muestras]=(uint32_t)(511+511*sin(2*Pi*i_muestras/N_muestras)); //Genera señal senoidal
	}
	i_muestras=0;
}
void tx_cadena_UART0(char *cadena)
{
 ptr_tx=cadena;
 tx_completa=0;
 LPC_UART0->THR=*ptr_tx++;	 // IMPORTANTE: Introducir un carácter al comienzo para iniciar TX o
                             // activar flag interrupción por registro transmisor vacio
}	
void menu_UART0()
{
	tx_cadena_UART0("Configuracion de parametros\n\r"
	                "Comandos:\n\r"
	                "on-activar mediciones o movimiento segun el modo\n\r"
	                "off-desactivar mediciones o movimiento segun el modo\n\r"
	                "Configuracion de parametros\n\r" 
	                "1-Resolucion de grados\n\r"
	                "2-Periodo de movimiento en mS\n\r");
}
void UART0_IRQHandler() 
{
	
 switch(LPC_UART0->IIR&0x0E) 
	{
	 case 0x04:								 /* RBR, Receiver Buffer Ready */
						 *ptr_rx=LPC_UART0->RBR; /* lee el dato recibido y lo almacena */
	 
							if (*ptr_rx++ ==13) 				// Caracter return --> Cadena completa
	    				{
								*ptr_rx=0;		/* Añadimos el caracter null para tratar los datos recibidos como una cadena*/ 
								rx_completa = 1; /* rx completa */
								ptr_rx=buffer;	/* puntero al inicio del buffer para nueva recepción */
	    				}	
							break;
	
    
   case 0x02:								/* THRE, Transmit Holding Register empty */
						 if (*ptr_tx!=0) {LPC_UART0->THR=*ptr_tx++;}	/* carga un nuevo dato para ser transmitido */
						 else {tx_completa=1;}
						 break;

	}
}	
int set_baudrate() 
{
   
 int errorStatus = -1; //< Failure

 unsigned int baudrate=9600;
 unsigned int uClk =Fpclk;
 unsigned int calcBaudrate = 0;
 unsigned int temp = 0;

 unsigned int mulFracDiv, dividerAddFracDiv;
 unsigned int divider = 0;
 unsigned int mulFracDivOptimal = 1;
 unsigned int dividerAddOptimal = 0;
 unsigned int dividerOptimal = 0;

 unsigned int relativeError = 0;
 unsigned int relativeOptimalError = 100000;

 uClk = uClk >> 4; /* div by 16 */

    /*
     *  The formula is :
     * BaudRate= uClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * DLL)
     *
     * The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
     * 0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15
     */
    for (mulFracDiv = 1; mulFracDiv <= 15; mulFracDiv++) {
        for (dividerAddFracDiv = 0; dividerAddFracDiv <= 15; dividerAddFracDiv++) {
            temp = (mulFracDiv * uClk) / (mulFracDiv + dividerAddFracDiv);

            divider = temp / baudrate;
            if ((temp % baudrate) > (baudrate / 2))
                divider++;

            if (divider > 2 && divider < 65536) {
                calcBaudrate = temp / divider;

                if (calcBaudrate <= baudrate) {
                    relativeError = baudrate - calcBaudrate;
                } else {
                    relativeError = calcBaudrate - baudrate;
                }

                if (relativeError < relativeOptimalError) {
                    mulFracDivOptimal = mulFracDiv;
                    dividerAddOptimal = dividerAddFracDiv;
                    dividerOptimal = divider;
                    relativeOptimalError = relativeError;
                    if (relativeError == 0)
                        break;
                }
            }
        }

        if (relativeError == 0)
            break;
    }

    if (relativeOptimalError < ((baudrate * UART_ACCEPTED_BAUDRATE_ERROR) / 100)) {

        LPC_UART0->LCR |= (1 << 7); 	// importante poner a 1
        LPC_UART0->DLM = (unsigned char) ((dividerOptimal >> 8) & 0xFF);
        LPC_UART0->DLL = (unsigned char) dividerOptimal;
        LPC_UART0->LCR &= ~(1 << 7);	// importante poner a 0

        LPC_UART0->FDR = ((mulFracDivOptimal << 4) & 0xF0) | (dividerAddOptimal & 0x0F);

        errorStatus = 0; //< Success
    }

    return errorStatus;
}
void config_UART0() 
{
  LPC_PINCON->PINSEL0|=(1<<4)|(1<<6); //Configuro patilla 0.2 como TXD0 y 0.3 como RXD0
	LPC_SC->PCONP|=(1<<3); //Alimento UART0
	LPC_SC->PCLKSEL0&=(0<<6)&(0<<7); //Pclk=Cclk/4 de fabrica
	
  LPC_UART0->LCR&=(0<<2); // 1 bit de stop
	LPC_UART0->LCR&=(0<<3); //Sin paridad
	LPC_UART0->LCR|=(0x3<<0); // 8 bits de longitud

  set_baudrate(); // Set the baudrate
    
     
  LPC_UART0->IER = (1 << 1)|(1 << 0);// Enable UART TX and RX interrupt (for LPC17xx UART)   
  NVIC_SetPriority(UART0_IRQn,10); //Fijo prioridad UART0
	NVIC_EnableIRQ(UART0_IRQn); //Habilito UART0
}
int main()
{
	lee_key1(); //Leer dato Key1 para saber en que modo entrar
	
	LCD_Initialization();
  LCD_Clear(Black);
	
	config_SysTick(); //configSysTick();
	configPWM(); //Configuro 
	configIRQ(); //Configuro las IRQ
	config_Timer2(); //Configuro Timer Ultrasonidos

	
	genera_muestras(); //Funcion que me genera las muestras
	config_DAC_sonido(); //DAC para reproducir audio
	config_Timer0_sonido(); //Timer para DAC
	
	ptr_rx=buffer; //Inicializo puntero de recepcion al comienzo del buffer
	config_UART0(); //Configuro UART0
	
	tx_cadena_UART0("UART0 INICIADA\n\r");
	
	while (1)
	{
		if (token==1)
		{
			
			if((dist>=dist_min) && (dist<=dist_max)) //Si esta dentro del rango establecido
			{
				distancia=dist; //Obtengo la distancia
				Frecuencia=5000-(distancia*10); //Para que varie la frecuencia en funcion de la distancia
				LPC_TIM0->TCR=2; //Activo Timer0 audio
				
				if ((dist>6) && (dist<=dist_max)) //En este rango solo pita
				{
					sonido=pitido; //Se reproducira pitido
					LPC_TIM0->MR0=(Fpclk/Frecuencia/N_muestras)-1; //Ajusto la frecuencia del pitido
					LPC_TIM0->TCR=1; //Activo Timer0 audio
				}
				else if((dist<=6) && (dist>=dist_min) && (mensaje==audio)) //En este rango reproduze audio
				{
					sonido=audio; //Se reproducira audio
					LPC_TIM0->MR0=(Fpclk/Frecuencia2)-1;
					LPC_TIM0->TCR=1; //Activo Timer0 audio
				}
			}
			else //Si estoy fuera de esos rangos
			{
				distancia=0;
				LPC_TIM0->TCR=2; //Reseteo Timer0 para que no pita
			}
		  sprintf(buffer, "Distancia= %d\n\r", distancia);
			tx_cadena_UART0(buffer);
			GUI_Text(10,40,(uint8_t *)buffer, White, Black); //Representacion en LCD
			token=0;
			
		}
		if(rx_completa) //Si esta algo almacenado en el buffer
		{
			rx_completa=0; //Borro flag para otra recepcion


			if(strcmp(buffer,"h\r")==0){menu_UART0();} //Si he pulsado h
			
			else if(strcmp(buffer,"on\r")==0) //Si se ha escrito on
			{
				if(modo==MANUAL)
				{
					on_manual(); //Funcion que activa medidas
				}
				else
				{
					on_auto(); //Funcion que pone en marcha modo automatico
				}
			}
			
			else if(strcmp(buffer,"off\r")==0) //Si se ha escrito off
			{
				if(modo==MANUAL)
				{
					off_manual(); //Funcion que para medidas
				}
				else
				{
					off_auto(); //Funcion que detiene modo automatico
				}
			}
			
			else if(contUART0==1) //Si configuro el primer parametro
			{
				tx_cadena_UART0("1-\n\r"); //Imprimo que paramentro he acrualizado
				contUART0++; //Incremento contador para el siguiente parametro
				salto=atoi(buffer); //Guardo el valor
			}
			
			else if(contUART0==2) //Si configuro el segundo parametro
			{
				tx_cadena_UART0("2-\n\r"); //Imprimo que paramentro he acrualizado
				contUART0=1;
				periodo=atoi(buffer); //Guardo el valor
			}
		}
	}
}
