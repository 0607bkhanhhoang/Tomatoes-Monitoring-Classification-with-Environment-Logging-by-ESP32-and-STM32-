# STM32_Configure And Main Interface
Firstly, to code on STM32, IOC file must be configured as the figure below. 
![IOC_Config](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/IOC_STM32_Config_View.png)

## Clock Configuration and TIMER selection
I use Timer 1, due to its extension in bits ( 16 bits) with another timer. The value of the timer is calculated is below and the reason to choose this timer is that I need a milis delay function for running DHT11. 

With the value of Prescaler is 64 - 1, counter period is 0xffff-1. The clock source is internal clock
![Timer_configi](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/Timer_config.png)

Code for create delay function to use on DHT11 
```bash
void delay (uint16_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
}
```
## ADC Configuration to read data from soil 
ADC on STM32 is 11 bits resolution, this is the configuration and function to read configure on ADC.

![ADC_Configure](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/adc_config.png)

:pushpin: Function to read ADC, R: 1-> 1000, voltage : 3.3V

```bash
uint8_t readSoil(void)
{
	uint16_t soil = 0;
	var = val*4096/3.3;
	val+=0.1;
	if (val>=5) val=0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	soil = ((3.3*ADC_VAL/4095 - V25)/Avg_Slope)+25;
	HAL_Delay (100);

	return soil;
}
```

## UART Configuration on Interrupt mode 
For UART, we already have 3 mode on UART (Polling, Interrupt, DMA) but the data is declared on uint8_t, meanwhile, we can send it respectively that ESP32 need an delay for the transmition, but it must have a signal to indicate that the data is sent -> so that i use UART in interrupt mode 

:email: To do this, global interrupt is turn on, that this is the 1st interrupt priority.

![UART_Send](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/UART_Sender.png)

:pen: **Script for UART Sending Data :** For this, I type casting my data from float -> integer value and send to ESP32

Buffer containing Data
```bash
/**** Create a buffer for UART****/
uint8_t UART1_rxBuffer[12] = {2};
uint8_t index;
uint8_t character_stop = 5;
//NULL Character buffer//
uint8_t TxData[32] = {'\0'};
uint8_t MSG[32] = {'\0'};
```

Sending Data in UART IT Mode
```bash
/**** Typecasting Data**********/
soilvalue = (uint8_t)readValue;
temp =(uint8_t)Temperature;
humid = (uint8_t)Humidity;
/***** UART Transmittion ***********/
sprintf(TxData, "%d%d%d\r\n ",temp,humid,soilvalue);
HAL_UART_Transmit_IT(&huart1,TxData, sizeof(TxData));
HAL_Delay(500);
```

## SPI Configure to ILI9341

:pushpin: **For more information, visit :** Page (https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf) 

SPI is configured on DMA mode to drive ILI9341 signal, the configure is as below 

![SPI_ILI9341](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/ILI9341_Config.png)

:pen: **Sample script ILI9341**

```bash
ILI9341_Init();
ILI9341_FillScreen(WHITE);
ILI9341_SetRotation(SCREEN_HORIZONTAL_2);
ILI9341_DrawText("SMART FARM", FONT4, 90, 110, BLACK, WHITE);
HAL_Delay(500);
// Horizontal Line (X, Y, Length, Color)
HAL_Delay(500);
ILI9341_FillScreen(DARKCYAN);
/*****DRAW DEFAULT DISPLAY FOR ILI9341 ******/
ILI9341_DrawHollowCircle(50, 50, 40, GREEN);
ILI9341_FillScreen(WHITE);
```
