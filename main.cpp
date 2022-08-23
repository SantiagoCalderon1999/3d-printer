#include <stdio.h>
#include <math.h>
#include "STM32F7xx.h"

typedef union{
		float number;
		uint8_t bytes[4];
	}valor;

	valor px;
	valor py;
	valor pz;
	valor tiempo;
	
	valor olv;
	valor rxd;
	valor ryd;
	valor rzd;
	valor rext;	
	int paso=0;
bool inicialXY=1, inicialZ=1;
double pi= 3.1416;
int	posicionInicial=0;
int	posicionInicialZ=0;
bool leyendoValores=0;
int time=500000;      //n ciclos de maquina
int dato=0;
bool bandera=0;
	double voltajeSalida=0;
int giro=0;
int contador=0;
	int contadorY=0;
	int contadorZ=0;
int contadorRecepcion=0;
	int contadorTest=0;
	bool Test=0;
double angulo_x=0;
int contador_tiempo=0;
int cantidadPasos=0;
float distanciaExtrusor=0;
float distanciaExtrusorAcumulada=0;
int contadorPaso=0;
//Temperatura
int adc_extrusor=0;
int adc_cama_caliente=0;
float logR2, R2, TEMPERATURA,logR22,TEMPERATURAprev=0;
float temperaturaExtrusor=0,temperatura_cama=0;;
float c1=0.827206948e-3, c2=2.087897328e-4, c3=0.8062131944e-7, Vcc=4095,R1=10000;
	void enviarMatlab()
	{
		
		//py.number=120.3f;
		//pz.number=510.3f;
			UART4->TDR = 'V'; 
			while ((UART4->ISR &= 0x80)==0);
			for	(int i=0; i<4; i++)
			{
			UART4->TDR = px.bytes[i]; 
			while ((UART4->ISR &= 0x80)==0);
			}
			for	(int i=0; i<4; i++)
			{
			UART4->TDR = py.bytes[i]; 
			while ((UART4->ISR &= 0x80)==0);
			}
			for	(int i=0; i<4; i++)
			{
			UART4->TDR = pz.bytes[i]; 
			while ((UART4->ISR &= 0x80)==0);
			}
			for	(int i=0; i<4; i++)
			{
			UART4->TDR = tiempo.bytes[i]; 
			while ((UART4->ISR &= 0x80)==0);
			}
			
			UART4->TDR = '\n'; //Nueva línea
			while ((UART4->ISR &= 0x80)==0);
	}
	
	void conv_adc(){
		ADC1->CR2 |= (1UL << 30);        //inicio conversión (set el bit SWSTART)
	while ((ADC1->SR &= 0x2)==1);   //esperar hasta que EOC sea 1 (termino la conversión canal 1)		
		
	ADC3->CR2 |= (1UL << 30);        //inicio conversión (set el bit SWSTART)
	while ((ADC3->SR &= 0x2)==1);   //esperar hasta que EOC sea 1 (termino la conversión canal 1)

}
	void movimientoExtrusor()
	{
		
	}
extern "C" {
		void SysTick_Handler ( void )
		{
				
									
			
		
		}//cierra la interrupcion por Systick
		
		void ADC_IRQHandler(void){         //Interrupcion por final de conversion.
				adc_extrusor = ADC3->DR;	   //lee el dato
				adc_cama_caliente = ADC1->DR;	   //lee el dato
			}
	
		void TIM3_IRQHandler(void)
		{
			TIM3->SR&=~(0x1);
			contador_tiempo++;
		}
		
		void TIM4_IRQHandler(void)
		{
			TIM4->SR&=~(0x1);
			
		}
		
		
	void EXTI0_IRQHandler(void)
		{
			EXTI->PR |= (1UL<<0);      //limpia la bandera de la interrupcion
			if( (GPIOD->IDR & 0x1) == 0x1)
			{
				contador++;
			}
			else{
				contador--;
			}
		}

	void EXTI1_IRQHandler(void)
		{
			EXTI->PR |= (1UL<<1);      //limpia la bandera de la interrupcion
			if( (GPIOD->IDR & 0x2) == 0x2)
			{
				contadorY++;
			}
			else{
				contadorY--;
			}
		}
		
		void EXTI2_IRQHandler(void)
		{
			EXTI->PR |= (1UL<<2);      //limpia la bandera de la interrupcion
			if( (GPIOD->IDR & 0x4) == 0x4)
			{
				contadorZ++;
			}
			else{
				contadorZ--;
			}
		}
		void EXTI3_IRQHandler(void) // Eje X final de carrera
		{
			EXTI->PR |= (1UL<<3);   
			if (posicionInicial==0)
			{
				posicionInicial=1;
			}
		}
		void EXTI4_IRQHandler(void)
		{
			EXTI->PR |= (1UL<<4);      //Eje Y final de carrera
			if (posicionInicial==1)
			{
				posicionInicial=2;
			}
		}
		void EXTI9_5_IRQHandler(void)
		{
			EXTI->PR |= (1UL<<5);      //Eje Z final de carrera
			if (posicionInicialZ==0)
			{
				posicionInicialZ=1;
			}
		}
	void EXTI15_10_IRQHandler(void)
		{
			EXTI->PR |= (1UL<<13);      //limpia la bandera de la interrupcion
			contador=0;
			contadorY=0;
			contadorZ=0;
			
			posicionInicial=0;
			posicionInicialZ=0;
		}
		void UART4_IRQHandler(void){
			
	if (UART4->ISR & 0x20) {
		dato = UART4->RDR;
		if (!leyendoValores)
		{
			if (dato=='V')
			{
			leyendoValores=1;
			}
		}
		else{
			if (contadorRecepcion<4)
			{
			rxd.bytes[contadorRecepcion]=dato;
			}
			else
			{		
				if (contadorRecepcion<8 )
				{
				ryd.bytes[contadorRecepcion-4]=dato;
				}
				else
				{
					if (contadorRecepcion<12)
					{
					rzd.bytes[contadorRecepcion-8]=dato;
					}
					else
					{
						if (contadorRecepcion<16)
						{
						rext.bytes[contadorRecepcion-12]=dato;
						}
					}
				}
			}
			contadorRecepcion++;
			if(contadorTest>10000)
			{				
					Test=1;
			}
			else{
				if(contadorTest<0)
				{
					Test=0;
				}
			}
			if (Test)
			{
				contadorTest--;
			}
			else{
			contadorTest++;
			}
			
			switch	 (contadorRecepcion)
			{
				case 4:
					
																					//Como es de 12 bits el máximo es 4095
				//	voltajeSalida=(-0.165*rxd.number+1.65);
					break;
				case 8:
				//	DAC->DHR12R2=(-0.165*ryd.number+1.65)*4095/3.3;//Canal 1 con 12 bits alineado a la derecha
																					//Como es de 12 bits el máximo es 4095
				//	voltajeSalida=(-0.165*rxd.number+1.65);
					break;
				case 12:
			/*		contadorRecepcion=0;
					leyendoValores=0;
					
					px.number=contador*2*3.1416/1440;	
					py.number=contadorY*2*3.1416/1440;	
					pz.number=contadorZ*2*3.1416/1440;	
					tiempo.number=((float)(TIM3->ARR))/(16000000/((float)(TIM3->PSC)+1))*(float)contador_tiempo;
					enviarMatlab();*/
					break;
				case 16:
					contadorRecepcion=0;
					leyendoValores=0;
					
					px.number=contador*2*3.1416/1440;	
					py.number=contadorY*2*3.1416/1440;	
					pz.number=contadorZ*2*3.1416/1440;	
					tiempo.number=((float)(TIM3->ARR))/(16000000/((float)(TIM3->PSC)+1))*(float)contador_tiempo;
					enviarMatlab();
					break;
			}
			
		}
		
	}
}
	
}


int main(void)
{
	//**********************************************************
	//CONFIGURACION "CLOCK"5
	RCC->AHB1ENR |= (1UL << 0);    //PRENDER EL CLOCK DEL PTA
	RCC->AHB1ENR |= (1UL << 1);    //PRENDER EL CLOCK DEL PTB
	RCC->AHB1ENR |= (1UL << 2);    //PRENDER EL CLOCK DEL PTC
	RCC->AHB1ENR |= (1UL << 3);    //PRENDER EL CLOCK DEL PTD
	RCC->AHB1ENR |= (1UL << 4);    //PRENDER EL CLOCK DEL PTE
	RCC->AHB1ENR |= (1UL << 5);    //PRENDER EL CLOCK DEL PTF
	RCC->APB2ENR |= (1UL << 14);   //System configuration controller clock enable
	RCC->APB1ENR |= (1UL<<3);//HABILITA TIMER 5
	RCC->APB1ENR |= (1UL<<2);//HABILITA TIMER 4
	RCC->APB1ENR |= (1UL<<1);//HABILITA TIMER 3
	//**********************************************************
	//CONFIGURACION DE PINES
	GPIOB->MODER |= 0x10004001;       //PTB0, PTB7 y PTB 14 -> OUTPUT
	GPIOB->OTYPER = 0;               //PUSH PULL -> PTB0, PTB7 y PTB 14
	GPIOB->OSPEEDR |= 0x10004001;     //MEDIUM SPEEED -> PTB0, PTB7 y PTB 14
	GPIOB->PUPDR |= 0x10004001;       //PULL-UP -> PTB0, PTB7 y PTB 14
	//**********************************************************
	//CONFIGURACION DE PINES
	GPIOE->MODER |= 0x10004001;       //PTB0, PTB7 y PTB 14 -> OUTPUT
	GPIOE->OTYPER = 0;               //PUSH PULL -> PTB0, PTB7 y PTB 14
	GPIOE->OSPEEDR |= 0x10004001;     //MEDIUM SPEEED -> PTB0, PTB7 y PTB 14
	GPIOE->PUPDR |= 0x10004001;       //PULL-UP -> PTB0, PTB7 y PTB 14
	//**********************************************************
	//CONFIGURACION DE PINES
	GPIOF->MODER |= 0x10004005 | (1UL<<8*2) | (1UL<<9*2) | (1UL<<10*2) | (1UL<<11*2);       //PTC0 -> OUTPUT
	GPIOF->OTYPER = 0;               //PUSH PULL -> PTC0
	GPIOF->OSPEEDR |= 0x10004005| (1UL<<8*2) | (1UL<<9*2) | (1UL<<10*2) | (1UL<<11*2);     //MEDIUM SPEEED -> PTC0
	GPIOF->PUPDR |= 0x10004005| (1UL<<8*2) | (1UL<<9*2) | (1UL<<10*2) | (1UL<<11*2);       //PULL-UP -> PTC0
	//*********************************************************
	//CONFIGURACION PIN PTC13 COMO ENTRADA (pulsador)
	GPIOC->MODER &=  ~(3UL << 2*13); //pulsador como entrada (PC13)
  GPIOC->MODER &=  ~(3UL << 2*0); //pulsador como entrada (PC0)
	GPIOD->MODER &=  ~(3UL << 2*0); //pulsador como entrada (PD0)
	GPIOC->MODER &=  ~(3UL << 2*1); //pulsador como entrada (PC1)
	GPIOD->MODER &=  ~(3UL << 2*1); //pulsador como entrada (PD1)
	GPIOC->MODER &=  ~(3UL << 2*2); //pulsador como entrada (PC2)
	GPIOD->MODER &=  ~(3UL << 2*2); //pulsador como entrada (PD2)
	GPIOC->MODER &=  ~(3UL << 2*3); //pulsador como entrada (PC3)
	GPIOC->MODER &=  ~(3UL << 2*4); //pulsador como entrada (PC4)
	GPIOC->MODER &=  ~(3UL << 2*5); //pulsador como entrada (PC5)
	//********************************************************************************************
	//CONFIGURACION EXTI_13 (para usar el PTC13 como interrupción externa)
	EXTI->IMR |= (1UL<<13);
	EXTI->RTSR |= (1UL<<13);
   SYSCFG ->EXTICR[3]=0x20;
	NVIC_EnableIRQ(EXTI15_10_IRQn);         //Habilitar la interrupción

	//********************************************************************************************
	//CONFIGURACION EXTI_13 (para usar el PTC13 como interrupción externa)
	EXTI->IMR |= (1UL<<0);
	EXTI->RTSR |= (1UL<<0);
  SYSCFG ->EXTICR[0]|=0x2;
	NVIC_EnableIRQ(EXTI0_IRQn);         //Habilitar la interrupción
	//********************************************************************************************
	//CONFIGURACION EXTI_13 (para usar el PTC13 como interrupción externa)
	EXTI->IMR |= (1UL<<1);
	EXTI->RTSR |= (1UL<<1);
  SYSCFG ->EXTICR[0]|=0x20;
	NVIC_EnableIRQ(EXTI1_IRQn);         //Habilitar la interrupción
	//********************************************************************************************
	//CONFIGURACION EXTI_13 (para usar el PTC13 como interrupción externa)
	EXTI->IMR |= (1UL<<2);
	EXTI->RTSR |= (1UL<<2);
  SYSCFG ->EXTICR[0]|=0x200;
	NVIC_EnableIRQ(EXTI2_IRQn);         //Habilitar la interrupción
	//********************************************************************************************
	//CONFIGURACION EXTI_13 (para usar el PTC13 como interrupción externa)
	EXTI->IMR |= (1UL<<3);
	EXTI->FTSR |= (1UL<<3);
  SYSCFG ->EXTICR[0]|=0x2000;
	NVIC_EnableIRQ(EXTI3_IRQn);         //Habilitar la interrupción
	//**********************************************************	
		//CONFIGURACION EXTI_13 (para usar el PTC13 como interrupción externa)
	EXTI->IMR |= (1UL<<4);
	EXTI->FTSR |= (1UL<<4);
  SYSCFG ->EXTICR[1]|=0x2;
	NVIC_EnableIRQ(EXTI4_IRQn);         //Habilitar la interrupción
		//**********************************************************	
		//CONFIGURACION EXTI_13 (para usar el PTC13 como interrupción externa)
	EXTI->IMR |= (1UL<<5);
	EXTI->FTSR |= (1UL<<5);
  SYSCFG ->EXTICR[1]|=0x20;
	NVIC_EnableIRQ(EXTI9_5_IRQn);         //Habilitar la interrupción
	//**********************************************************	
	//**********************************************************	
	// DAC
	RCC->APB1ENR|=0X20000000;     //Reloj del DAC 
	GPIOA->MODER|=0XF00;          //DAC Puerto A4 y A5 en analógico
  DAC->CR=0x00010001;             		  //Habilita el Canal 1 del DAC 
//********************************************************************************************
	//UART4 CONFIGURATION
	
	RCC->APB1ENR |= 0x80000; // Enable clock for UART4 (set pin 19 )
													 // 8N1-> M=00
	UART4->BRR = 16000000/500000;      // 1M Baudrate, fclk=16Mhz
	UART4->CR3 |=(1UL<<12); 
  UART4->CR1 |= 0x2C;      // Tx enabled, Rx enabled, Rx interruption enabled, 8N1 
		
	UART4->CR1 |= 0x1;       //Enables UART, UE=1 
	
	
	NVIC_EnableIRQ(UART4_IRQn);      //Enables UART interruption 
	
	GPIOC->MODER |=  (2UL << 2*10);   //Alternate mode PC10 (TX)
  GPIOC->MODER |=  (2UL << 2*11);   //Alternate mode PC11 (RX)
	GPIOC->AFR[0] |= 0x8;            // PC10 -> AF8=UART4 TX, AF8=1000 en AFR10
	GPIOC->AFR[1] |= 0x8800;         // PC11 -> AF8=UART4 RX, AF8=1000 en AFR11
	
		///////////////////////////////
	//TIMER 5-CH1,CH2, CH3, CH4	PUERTO PA0,PA1,PA2,PA3
	GPIOA->MODER&=~((3UL<<0)|(3UL<<2*1)|(3UL<<2*2)|(3UL<<2*3));// ALTERNO //LIMPIA
	GPIOA->MODER|=(2UL<<0)|(2UL<<2*1)|(2UL<<2*2)|(2UL<<2*3);//METE LOS UNOS
	GPIOA->OTYPER=0;
	GPIOA->OSPEEDR&=~((3UL<<0)|(3UL<<2*1)|(3UL<<2*2)|(3UL<<2*3));// MEDIUM SPEED/LIMPIA
	GPIOA->OSPEEDR|=(1UL<<0)|(1UL<<2*1)|(1UL<<2*2)|(1UL<<2*3);//METE LOS UNOS
	GPIOA->PUPDR&=~((3UL<<0)|(3UL<<2*1)|(3UL<<2*2)|(3UL<<2*3));//PULL-UP //LIMPIA
	GPIOA->PUPDR|=(1UL<<0)|(1UL<<2*1)|(1UL<<2*2)|(1UL<<2*3);//METE LOS UNOS
	GPIOA->AFR[0]|=0x2222;//	FUNCIÓN ALTERNA AF2=TIM5_CH1
	
	
	//CONFIGURACIÓN DEL TIM5:CH1,CH1,CH3,CH4
	TIM5->EGR|=(1UL<<0);//REINICIALIZA EL CONTADOR
	TIM5->PSC=0;  //PRESCALER, GENERA 1MHz
	TIM5->DIER|=(1UL<<0);//UIE=1, ACTIVA LA INTERRUPCIÓN POR ACTUALIZACIÓN(CADA VEZ QUE SE REALIZA UN CONTEO)
	TIM5->CR1|=(1UL<<0);//ACTIVA EL CONTADOR
	TIM5->CCER|=(1UL<<0)|(1UL<<4)|(1UL<<8)|(1UL<<12);//PONE EL PIN COMO SALIDA 
	TIM5->CCMR1=0x6060; //ESTÁ ACTIVADO EN CONTEO ASCENDENTE
  TIM5->CCMR2=0x6060; //ESTÁ ACTIVADO EN CONTEO ASCENDENTE
	TIM5->ARR=0.00001*16000000; //Cambia periodo a 0.001
	
	//CONFIGURACIÓN DEL TIM3:CH1
	TIM3->EGR|=(1UL<<0);//REINICIALIZA EL CONTADOR
	TIM3->PSC=15;  //PRESCALER, GENERA 1MHz
	TIM3->ARR=20000; //PERIODO DE LA SEÑAL...20000->20000*1uS=20mS que es la señal del servo
	TIM3->DIER|=(1UL<<0);//UIE=1, ACTIVA LA INTERRUPCIÓN POR ACTUALIZACIÓN(CADA VEZ QUE SE REALIZA UN CONTEO)
	TIM3->CR1|=(1UL<<0) | (1UL<<2);//ACTIVA EL CONTADOR
	TIM3->CCER|=(1UL<<0);//PONE EL PIN COMO SALIDA 
	TIM3->CCMR1=0x60; //ESTÁ ACTIVADO EN CONTEO ASCENDENTE
	NVIC_EnableIRQ(TIM3_IRQn);      //Enables TIM3 interruption
	
	//CONFIGURACIÓN DEL TIM3:CH1
	TIM4->EGR|=(1UL<<0);//REINICIALIZA EL CONTADOR
	TIM4->PSC=15;  //PRESCALER, GENERA 1MHz
	TIM4->ARR=20000; //PERIODO DE LA SEÑAL...20000->20000*1uS=20mS que es la señal del servo
	TIM4->DIER|=(1UL<<0);//UIE=1, ACTIVA LA INTERRUPCIÓN POR ACTUALIZACIÓN(CADA VEZ QUE SE REALIZA UN CONTEO)
	TIM4->CR1|=(1UL<<0) | (1UL<<2);//ACTIVA EL CONTADOR
	TIM4->CCER|=(1UL<<0);//PONE EL PIN COMO SALIDA 
	TIM4->CCMR1=0x60; //ESTÁ ACTIVADO EN CONTEO ASCENDENTE
	NVIC_EnableIRQ(TIM3_IRQn);      //Enables TIM3 interruption
	
	//*******************************************************************************************************	
	//CONFIGURACION ADC (en este caso ADC3)
	
	RCC->APB2ENR |= 0x400;   //Enable clock for the ADC3 (set el bit 10 =ADC3EN)

	GPIOF->MODER |= 0xC0;	//configurar los pin como analógico PF3 (analogico = 11)
	
	ADC3->CR1 |= (1UL << 5);   	 //ADC3->CR1 se activa la interrupcion EOCIE
	ADC3->CR2 |= (1UL << 0);   	//ADC3->CR2 - The ADC is powered on by setting the ADON bit in the ADC_CR2 register
	//ADC3->CR2 |= (1UL << 1);   	//ADC3->CR2 - Set Bit CONT in the ADC_CR2 register
	ADC3->CR2 |= (1UL << 10);   //Set to 1 the bit EOCS (The EOC bit is set in the ADC_SR register:
																											//At the end of each regular channel conversion) 
														
	ADC3->SQR3 = 0x9;     //Define el canal de conversión (PF3 - ADC3_IN9)
	
		//*******************************************************************************************************	
	//CONFIGURACION ADC (en este caso ADC1)
	
	RCC->APB2ENR |= (1UL << 8);   //Enable clock for the ADC3 (set el bit 10 =ADC3EN)

	GPIOA->MODER |= (3UL<<6*2);	//configurar los pin como analógico PA7 (analogico = 11)
	
	ADC1->CR1 |= (1UL << 5);   	 //ADC3->CR1 se activa la interrupcion EOCIE
	ADC1->CR2 |= (1UL << 0);   	//ADC3->CR2 - The ADC is powered on by setting the ADON bit in the ADC_CR2 register
	//ADC3->CR2 |= (1UL << 1);   	//ADC3->CR2 - Set Bit CONT in the ADC_CR2 register
	ADC1->CR2 |= (1UL << 10);   //Set to 1 the bit EOCS (The EOC bit is set in the ADC_SR register:
																											//At the end of each regular channel conversion) 
														
	ADC1->SQR3 = 0x6;     //Define el canal de conversión (PA7 - ADC123_IN7)
	
  NVIC_EnableIRQ(ADC_IRQn);      //Habilita interrupción del ADC
	
	//**********************************************************************
	//CONFIGURACION SYSTICK
		SystemCoreClockUpdate();
		SysTick_Config(SystemCoreClock);
	
	while(true){        //bucle infinito
		conv_adc();                //convertir dato analogico a digital
		switch(posicionInicialZ)
		{
			case 0:
				rzd.number=-10;
				break;
			case 1:
				rzd.number=0;
				contadorZ=0;
				posicionInicialZ=2;
				break;
			
		}
		switch (posicionInicial)
		{
			case 0:
				
				rxd.number=10;
				ryd.number=10;
				
				break;
			case 1:
				rxd.number=-10;
				ryd.number=10;
				break;
			case 2:
				contador=0;
				contadorY=0;
				rxd.number=0;
				ryd.number=0;
				rzd.number=0;
				posicionInicial=3;
			
				break;
		/*	case 2:
				if(contadorY>=-3323)
				{
					rxd.number=10;
					ryd.number=-10;
				}
				else{
					posicionInicial=3;
				}
				break;
			case 3:
				if(contadorY>=-3323*2)
				{
					rxd.number=-10;
					ryd.number=-10;
					
				}
				else{
					posicionInicial=4;
					contador=0;
					contadorY=0;
					contadorZ=0;
					rxd.number=0;
					ryd.number=0;
				}
				break;*/
		}
		if(~((GPIOC->IDR & 0x8) == 0x8 || (GPIOC->IDR & 0x10) == 0x10) && inicialXY)
				{
					posicionInicial=3;
					rxd.number=0;
					ryd.number=0;
					inicialXY=0;
				}
				
				if(~((GPIOC->IDR & 0x20) == 0x20) && inicialZ)
				{
					rzd.number=0;
					inicialZ=0;
					posicionInicialZ=2;
				}
				DAC->DHR12R1=(-0.165*rxd.number+1.65)*4095/3.3;//Canal 1 con 12 bits alineado a la derecha
				DAC->DHR12R2=(-0.165*ryd.number+1.65)*4095/3.3;//Canal 1 con 12 bits alineado a la derecha
				if (rzd.number>=0 && rzd.number<=10)
				{
					TIM5->CCR1=(rzd.number/10)*(TIM5->ARR);
					TIM5->CCR2=0;
				}
				else{
					if (rzd.number>=-10 && rzd.number<=0)
					{
					TIM5->CCR2=rzd.number*(TIM5->ARR)/(-12);
					TIM5->CCR1=0;
					}
				}
				
				//Control temperatura extrusor
				logR22 = log((adc_extrusor*R1 / (Vcc-adc_extrusor)));
				temperaturaExtrusor = abs((1.0 / (c1 + c2*logR22 + c3*logR22*logR22*logR22))-273.15);
				if (temperaturaExtrusor<220)//Transistor TIP122 conectado pull up
				{
					GPIOF->ODR&=~(1UL<<0);
				}
				else{
					GPIOF->ODR|=(1UL<<0);
				}
				
				//Control temperatura cama caliente
				logR2 = log((adc_cama_caliente*R1 / (Vcc-adc_cama_caliente)));
				temperatura_cama = abs((1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2))-273.15);
				
				if (temperatura_cama<40) 
				{
					GPIOF->ODR&=~(1UL<<7);
				}
				else{
					GPIOF->ODR|=(1UL<<7);
				}
				
				if(distanciaExtrusorAcumulada<rext.number)
				{
cantidadPasos++;
		distanciaExtrusorAcumulada=cantidadPasos*0.15708;
		switch(contadorPaso)
					{
						case 0:	//1010
							GPIOF->ODR|=(1UL<<8);
							GPIOF->ODR&=~(1UL<<9);
							GPIOF->ODR|=(1UL<<10);
							GPIOF->ODR&=~(1UL<<11);
							contadorPaso=1;
							break;
						case 1:	//0110
							GPIOF->ODR&=~(1UL<<8);
							GPIOF->ODR|=(1UL<<9);
							GPIOF->ODR|=(1UL<<10);
							GPIOF->ODR&=~(1UL<<11);
							contadorPaso=2;
							break;
						case 2:	//0101
							GPIOF->ODR&=~(1UL<<8);
							GPIOF->ODR|=(1UL<<9);
							GPIOF->ODR&=~(1UL<<10);
							GPIOF->ODR|=(1UL<<11);
							contadorPaso=3;
							break;
						case 3:	//1001
							GPIOF->ODR|=(1UL<<8);
							GPIOF->ODR&=~(1UL<<9);
							GPIOF->ODR&=~(1UL<<10);
							GPIOF->ODR|=(1UL<<11);
							contadorPaso=0;
							break;
					}
				}
			
				
	}
}