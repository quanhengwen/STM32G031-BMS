//--------------------------ADC peripheral explanation--------------------------------------
//PA0 and PA1 are analog input pins connected to measure battery voltage
//We assume that at PA0 pin (ADC_VOLT point) will be 16 bit battery voltage value
//at PA1 (ADC_VOLT_IN) will be 12 bit battery voltage value
//PA4 pin (ADC_TEMP1 point) is connected to voltage divider made by 10k resistor and thermistor in series
//PA5 pin (ADC_TEMP2 point) is connected aslo to simmilar voltage divider
//F_CPU is 16 MHz and we need to divide that value to cca 125 kHz to form ADCCLK
//for this purpose, we will use ADCx_SMPRx register (SMPx [2:0]) that holds the value how many cycles will th system sample
//any value and ADCx_CCR register (ADCPRE[17:16]) SMPx = 001 (15 cycles), ADCPRE = 11 ( PCK2 / 8) 
//ADCCLK -> 16 MHz / (15*8) = 133.3 kHz 
//We will allign data to the right so ALIGH bit in ADCx_CR2 will be cleared to 0, VREF is 2.56 V internally

//--------------------------System Nofitications----------------------------------------------
//type 0 for false ondition and 1 for true condition

//--------------------------TIM1 peripheral explanation---------------------------------------
//TIM1 is used to generate clock signal with frquency about 8 Hz, timer mode will be clear at compare match 

#include "stm32f4xx.h"                  // Device header
#include "RTE_Components.h"             // Component selection

//system predirectives
//Number of voltage readings to take before we take a temperature reading
#define TEMP_READING_LOOP_FREQ 16
#define OVERSAMPLE_LOOP 32
#define PIN0 0x1
#define PIN1 0x2
#define PIN4 0x4
#define PIN5 0x5
uint8_t PINS [4] = {PIN0, PIN1, PIN4, PIN5};

//forward method declarations
void initADC(void);
void ADC1_IRQHandler(void);
void initTimer1(void);

//system global variable definitions
volatile uint32_t analogVal[OVERSAMPLE_LOOP];
volatile uint8_t analogValIndex;
volatile uint8_t buffer_ready = 0;
volatile int skipNextADC = 0;
volatile uint8_t reading_count = 0;
volatile uint32_t temperature_probe = 0;
volatile uint32_t last_raw_adc = 0;

void initADC(void){
	//Enable clock acces to GPIOA module
	RCC->AHB1ENR |= 0x1;
	//Declare PA0, PA1, PA4 and PA5 as analog input mode pins
	GPIOA->MODER |= 0xF0F;
	//Enable clock access to ADC1 module (represented by 8th bit in APB2ENR reg)
	RCC->APB2ENR |= 0x100;
	//Disable the ADC module
	ADC1->CR2 = 0;
 //Set the ADCCLK value, 133.3 kHz
	ADC1->SMPR1 |= 0x1;
	ADC->CCR |= 0x30000;
	//Channel number CH1 assigned as the 1st in the sequence to be converted
	//PA0 is connected to ADC1 input module
	ADC1->SQR3 = PINS[0];
	//Interrupt enable for ADC1 end of conversion
	ADC1->CR1 |= 0x10;
	//Enable ADC1 module and set continouns sequence mode
	ADC1->CR2 |= 0x3;
}

void ADC1_IRQHandler(void){
	//Interrupt servie routine for ADC completion
	uint32_t value = 0;
	//If we skip this ADC reading, quit ISR here
  if (skipNextADC) {
    skipNextADC = 0;
    return;
  }
		if (reading_count == TEMP_READING_LOOP_FREQ ) {
    reading_count = 0;

    //We reduce the value by 512 as we have a DC offset we need to remove
    temperature_probe = value;

    //Connect the next pin to take the value
    //ADMUX = B10010011
			//Connect GPIOA PIN 0 to th ADC1 module input
				ADC1->SQR3 = PINS[0];
			
				/*if(ADC1->SQR3 == PINS[0]){
					ADC1->SQR3 = PINS[1];
				}
				else if(ADC1->SQR3 == PINS[1]){
					ADC1->SQR3 = PINS[2];
				}
				else if(ADC1->SQR3 == PINS[2]){
					ADC1->SQR3 = PINS[3];
				}
				else if(ADC1->SQR3 == PINS[3]){
					ADC1->SQR3 = PINS[0];
				}
				*/
			
    //Set skipNextADC to delay the next TIMER2 call to ADC reading to allow the ADC to settle after changing MUX
    skipNextADC = 1;

  }
		 else {

    //Populate the rolling buffer with values from the ADC
    last_raw_adc = value;
    analogVal[analogValIndex] = value;

    analogValIndex++;

    if (analogValIndex == OVERSAMPLE_LOOP) {
      analogValIndex = 0;
      buffer_ready = 1;
    }

    reading_count++;

    if (reading_count == TEMP_READING_LOOP_FREQ) {
      //use ADC0 for temp probe input on next ADC loop
						
					
      //ADMUX = B10010000;
							//Assuming PA0 is connected to ADC1 module input and we need to connect PA1 to the input and so on
					ADC1->SQR3 = PINS[2];
					
					/*
							if(ADC1->SQR3 == PINS[0]){
								ADC1->SQR3 = PINS[1];
							}
							else if(ADC1->SQR3 == PINS[1]){
								ADC1->SQR3 = PINS[2];
							}
							else if(ADC1->SQR3 == PINS[2]){
								ADC1->SQR3 = PINS[3];
							}
							else if(ADC1->SQR3 == PINS[3]){
								ADC1->SQR3 = PINS[0];
							}
					*/
					
      //Set skipNextADC to delay the next TIMER1 call to ADC reading to allow the ADC to settle after changing MUX
      skipNextADC = 1;
    }
  }
}

void initTimer1(void){
	//Enable clock access to TIM1
	RCC->APB2ENR |= 0x1;
	//Define clock signal prescaler, generate 8 Hz signal 
	//F_OSC = 16 MHz, prescaler = 16000 * 125 = 2 MHz equals 8 ticks per second
	TIM1->PSC = 16000 - 1;
	TIM1->ARR = 125 - 1;
}

