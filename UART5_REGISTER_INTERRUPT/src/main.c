#include "stm32f4xx.h"

/*
 * 							Dogukan AKCA
 *
 * 		Tx/Rx baud = Fclk / ( ( 8 x (2 - OVER8) x USARTDIV ) )
 *
*/

#define  USART_TXE     ((uint16_t)0x0080)  /*Transmit Data Register Empty*/
#define  USART_RXNE    ((uint16_t)0x0020)  /*Read Data Register Not Empty*/
#define  USART_TC      ((uint16_t)0x0040)  /*Transmission Complete*/


void CLK_Config(void);
void GPIO_Config(void);
void UART5_Config(void);
void Send_Data(uint8_t *data, uint8_t size);


char rx_data[6];
char key1[6] = "led on";
char key2[6] = "ledoff";
uint8_t counter = 0;


int main(void)
{
	CLK_Config();
	GPIO_Config();
	UART5_Config();


	Send_Data("mrb",5);
	Send_Data('\n',1);

  while (1)
  {

  }
}


void CLK_Config() // Clock speed for 168MHz
{
	RCC->CR |= 0x00010000;                 // HSEON ENABLE
	while(!(RCC->CR & 0x00020000));        // HSEON Ready Flag wait
	RCC->CR |= 0x00080000;              // CSS ENABLE
	RCC->CR |= 0x01000000;				// PLL ON
	RCC->PLLCFGR |= 0x00400000;        // PLL SRC HSE is selected
	RCC->PLLCFGR |= 0x00000004;       // PLL M 4
	RCC->PLLCFGR |= 0x00005A00;        // PLL N 168
	RCC->PLLCFGR |= 0x00000000;       // PLL P 2
	RCC->CFGR |= 0x00000000;          // AHB PRESCALER 1
	RCC->CFGR |= 0x00080000;          // APB2 PRESCALER 2
	RCC->CFGR |= 0x00001400;          // APB1 PRESCALER 4
	RCC->CIR |= 0x00080000;             // HSE READY FLAG CLEAR
	RCC->CIR |= 0x00800000;             // CSS FLAG CLEAR
}


void GPIO_Config(void)  // User led configuration
{
	RCC->AHB1ENR |= 0x1U << 3U; // D port clock enable

	GPIOD->MODER |= 0x55000000; // pins D12, D13, D14, D15 is selected output mode
	GPIOD->OSPEEDR |= 0xFF000000; // very high speed is selected
	GPIOD->PUPDR |= 0x00000000; // no pull up, pull down



	// UART5_TX --> PC12  AF8 ,      UART5_RX --> PD2  AF8

	RCC->AHB1ENR |= 0x1U << 2U;  // C port clock enable

	GPIOD->MODER |= 2U << 4U;	// PD2 alternate function mode
	GPIOD->OSPEEDR |= 3U << 4U; // PD2 very high speed is selected

	GPIOC->MODER |= 2U << 24U;	// PC12 alternate function mode
	GPIOC->OSPEEDR |= 3U << 24U; // PC12 very high speed is selected

	GPIOC->AFR[1] |= 8U << 16U;  // PC12 alternate function selection
	GPIOD->AFR[0] |= 8U << 8U;  // PC12 alternate function selection
}


void UART5_Config()
{
	RCC->APB1ENR |= 0x1U << 20U; // UART5 clock enable

	UART5->BRR = 0x16D; 	 /* 115200
	 	 	 	 	 	 	 DIV_Mantissa[11:0]: mantissa of USARTDIV
							 These 12 bits define the mantissa of the USART Divider (USARTDIV)

							 DIV_Fraction[3:0]: fraction of USARTDIV
							 These 4 bits define the fraction of the USART Divider (USARTDIV). When OVER8=1, the DIV_Fraction3 bit is not considered and must be kept cleared.
	 	 	 	 	 	 	 */
	UART5->CR1 |= 1U << 2U;  // Receiver Enable
	UART5->CR1 |= 1U << 3U;  // Transmitter Enable
	UART5->CR1 |= 1U << 5U;  // RXNE interrupt enable
	UART5->CR1 |= 0U << 12U; // 1 Start bit, 8 Data bits, n Stop bit
	UART5->CR1 |= 1U << 13U; // USART enable

	NVIC_EnableIRQ(UART5_IRQn);  // enable UART5 interrupt in NVIC(nested vector interrupt controller)

}


void Send_Data(uint8_t *data, uint8_t size)
{
	const uint8_t  *pdata8bits;

	while(size > 0)
	{
		pdata8bits  = data;
		size--;
		while((UART5->SR & USART_TXE) != USART_TXE);	// wait for transmitter empty

		UART5->DR = (uint8_t)(*pdata8bits);	// write the data address in the UART5's data register and plus one (1 byte plus)
		pdata8bits++;

		while((UART5->SR & USART_TC) != USART_TC);      // wait for transmitter complete
	}
}


void UART5_IRQHandler()
{

	rx_data[counter++] = UART5->DR;		// to get received data

	if(counter == 6) counter = 0;

	if(rx_data[0] == key1[0] && rx_data[1] == key1[1] && rx_data[2] == key1[2] && rx_data[3] == key1[3] && rx_data[4] == key1[4] && rx_data[5] == key1[5])
	{
		GPIOD->ODR = 0x0000F000;   // pins set
	}
	else if(rx_data[0] == key2[0] && rx_data[1] == key2[1] && rx_data[2] == key2[2] && rx_data[3] == key2[3] && rx_data[4] == key2[4] && rx_data[5] == key2[5])
	{
		GPIOD->ODR = 0x00000000;   // pins reset
	}


	Send_Data("ok",2);
	Send_Data('\n',1);
}

