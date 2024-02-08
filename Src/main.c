/* Set correct USART GPIO pins to their correct functions,
 * done in GPIO Mode Register, AHB1EN Register and Alternate Function Register
 * Then make the necessary configurations in UART registers (USARTx CR1, USARTx BRR for baud rate)
 * where x is the number of the USART in use
 * Status register is USART SR, data register USART DR
 */

//GPIO pins used in this code are PA2 and PA3 for USART transmission
//When transmitting data, blink onboard led LD4 (PD12)

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "bitfields.h"

#define Adc1r ((uint32_t*)0x40012000) //ADC1 register
#define Adc1Sqr3 ((uint32_t*)0x40012034) //ADC1 sequence register 3
#define Adc1Sqr1 ((uint32_t*)0x4001202C) //ADC1 sequence register 1, for amount of conversions
#define Adc1Cr2 ((uint32_t*)0x40012008) //ADC1 control register to enable the adc
#define Adc1Sr ((uint32_t*)0x40012000) //ADC1 status register
#define Adc1Dr ((uint32_t*)0x4001204C) //ADC1 data register
#define AdcClk ((uint32_t*)0x40023844) //ADC clock control register

typedef struct { //USART baud rate register

	uint32_t DIV_fraction : 4;
	uint32_t DIV_mantissa : 12;
	uint32_t RESERVED : 16;

}USART_BRR_t;




AHB1ENR_t* ClkReg = (AHB1ENR_t*) 0x40023830; //Clock register
GPIOx_MODER_t* ModeReg = (GPIOx_MODER_t*) 0x40020000; //GPIOA mode register
GPIOx_MODER_t* ModeRegD = (GPIOx_MODER_t*) 0x40020C00; //GPIOD mode register
GPIOx_ODR_t* OutDataRegD = (GPIOx_ODR_t*) 0x40020C14; //GPIOD output data register
GPIOx_AFRL_t* AltFuncReg = (GPIOx_AFRL_t*) 0x40020020; //GPIOA alternate function register
USART_BRR_t* UsartBaudReg = (USART_BRR_t*) 0x40004408; //USART baud rate register for USART2

uint32_t* UsartCntlReg1 = (uint32_t*) 0x4000440C; //USART control register 1 for USART2
uint32_t* UsartCntlReg2 = (uint32_t*) 0x40004410; //USART control register 2 for USART2
uint32_t* Usart_dr = (uint32_t*)0x40004404; //USART data register for USART2
uint32_t* Usart_clk = (uint32_t*)0x40023840; //Clock register for USART
uint32_t* UsartStatusReg = (uint32_t*) 0x40004400; //USART status register for USART2
uint32_t* PinSpeedReg = (uint32_t*) 0x40020008; //GPIO pin output speed register

uint32_t* ClkCntlReg = (uint32_t*) 0x40023800; //Clock control register
uint32_t* ClkConfReg = (uint32_t*) 0x40023808; //Clock configuration register
uint32_t* PllConfReg = (uint32_t*) 0x40023804; //Pll configuration register
uint32_t* PwrCntlReg = (uint32_t*) 0x40007000; //Power control register
uint32_t* FlashArcReg = (uint32_t*) 0x40023C00; //Flash interface register
uint32_t* FlashAccReg = (uint32_t*) 0x40023C04; //Flash interface access register

void init_clock();
void init_board();
void send_char(uint8_t c);
void send_str(char* str);
uint32_t ADC_read();
void ADC_init();

int main(void) {
	char msg[50];
	float voltage = 0;
	//uint8_t test = 'a';
	init_clock();
	init_board();
	ADC_init();
	while(1){
		voltage = (float)ADC_read() * (3.0/4096.0);
		sprintf(msg, "Voltage readign: %f\n", voltage);
		send_str(msg);
		for(int i = 0; i < 0xFFFFFF; i++);
	}
}

void init_clock(){
	/* Turn HSE on and wait it to be ready
	 * then configure PLL to generate 168MHz signal
	 * then PLL on and ready
	 * then divide APB1 clock by 4 to get it to run 42MHz
	 */


	*ClkCntlReg |= (1 << 16); //Turn HSE on
	while(!(*ClkCntlReg & (1 << 17))); //Wait for HSE clock to be ready

	*Usart_clk |= (1 << 28); //Enable power interface clock
	*PwrCntlReg |= (1 << 14); //Voltage regulator scaling 1

	*FlashAccReg = 0x45670123; //Access key 1
	*FlashAccReg = 0xCDEF89AB; //Access key 1

	*FlashArcReg |= (1 << 8) | (1 << 9) | (1 << 10) | (5 << 0); //Configure the flash register

	*ClkConfReg &= ~(1 << 7); //AHB prescaler as 1
	*ClkConfReg |= (5 << 10); //Divide AHB by 4 for APB1

	//M factor for PLL
	*PllConfReg &= ~(1 << 4);
	*PllConfReg |= (8 << 0);

	//N factor for PLL
	*PllConfReg &= ~(1 << 13);
	*PllConfReg |= (336 << 6);

	*PllConfReg &= ~(3 << 16); //P factor for PLL
	*PllConfReg |= (7 << 24); //Q factor for PLL
	*PllConfReg |= (1 << 22); //HSE as PLL source

	*ClkCntlReg |= (1 << 24); //Turn PLL on
	while(!(*ClkCntlReg & (1 << 25))); //Wait for PLL to be ready

	*ClkConfReg |= (2 << 0); //PLL as system clock
	while(!(*ClkConfReg & (2 << 2))); //Wait for PLL to be selected as system clock



}

void init_board(){

	//Reset the USART2 control register 1
	*UsartCntlReg1 = 0;

	//Configure stop bits (1)
	*UsartCntlReg2 &= ~(3 << 12);

	//Enable clock for GPIOA, USART2 and GPIOD
	ClkReg -> GPIOAEN = 1;
	ClkReg -> GPIODEN = 1;
	*Usart_clk |= (1 << 17);

	//Alternate function mode for PA2 and PA3 and analog input for PA1
	ModeReg -> PIN2 = 2;
	ModeReg -> PIN3 = 2;
	ModeReg -> PIN1 = 3;

	//Speed for the GPIO pins
	*PinSpeedReg |= (2 << 4);
	*PinSpeedReg |= (2 << 6);

	//Choose the correct alt function for the pins
	AltFuncReg -> PIN2 = 7; //Write 0111 to choose USART for the pins
	AltFuncReg -> PIN3 = 7;

	//Enable USART and configure the baud rate
	*UsartCntlReg1 |= (1 << 13); //Enable USART
	*UsartCntlReg1 &= ~(1 << 12);

	/* For baud rate of 115200, the USARTDIV is 22,78
	 * calculated with: 42Mhz / (16*115200)
	 * The formula is: clock frequency / (8 * (2 - OVER8bit) * baudrate)
	 * Then we need to check with the fractional part, what fraction bits we need to program
	 * This is done with: 16*0,78 = 12,48 (Amount of bits in register * the fraction of USARTDIV)
	 * So we write 13 to the fraction part of the register and 22 to the mantissa part
	 */
	UsartBaudReg -> DIV_fraction = 13;
	UsartBaudReg -> DIV_mantissa = 22;

	//Then just enable the transmitter and receiver
	*UsartCntlReg1 |= (1 << 2);
	*UsartCntlReg1 |= (1 << 3);

	//Make PD12 as output
	ModeRegD -> PIN12 = 1;

}
void ADC_init(){

	*AdcClk |= (1 << 8); //Enable ADC1 clock
	*Adc1Sqr3 |= (1 << 0); //Only 1 sequence needed
	*Adc1Sqr1 = 0; //Amount of conversions is 1 so all bits to 0
	*Adc1Cr2 |= (1 << 0); //Enable the adc

}

uint32_t ADC_read(){

	*Adc1Cr2 |= (1 << 30); //Start the conversion

	while(!(*Adc1Sr & (1 << 1))); //Wait for the conversion to complete

	return *Adc1Dr;

}

void send_char(uint8_t c){
	*Usart_dr = c; //Copy the character to the USART data register
	//When transmitting turn led on and off after transmitting
	OutDataRegD -> PIN12 = 1;
	while(!(*UsartStatusReg & (1 << 6))); //Wait for the transmission complete bit to be set
	OutDataRegD -> PIN12 = 0;
}

void send_str(char* str){
	while(*str){
		send_char(*str++);
	}
}
