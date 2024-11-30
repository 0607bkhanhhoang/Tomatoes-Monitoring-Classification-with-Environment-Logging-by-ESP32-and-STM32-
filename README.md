# Tomatoes Monitoring and Classification with Environment Logging by ESP32S3 and STM32 

Hi Everyone, this is the project from my course on Ho Chi Minh University of Technology as the course is on **EE3103 : Embedded System Programming**. In this project, on abstraction view, this project will measure temperature, humidity,soil moisture and rain drops state from a microcontroller : **STM32F103C8T6**. This MCU will connect to TFT ILI9341 for displaying temperature, humidity, soil moisture and LCD1602 will display Rain drops state and relay state. 

All the data will be transmitted to **ESP32-S3** and logged into a local webserver. This web will contains data collected from **Raspberry Pi 3B**, data concludes **confidence score** and **counting class ID** from YOLOv8 runned on Raspberry Pi 3B. This model can be runned on Window but have to ensuring that it can run your model. I have just pushed on script, for more flexible installation, you must install VNC Viewer and set up for your Raspberry or any embedded computer.

# IDE Installation and Set up Anaconda
:pushpin: STM32CubeIDE : This IDE is used for STM32 Programming https://www.st.com/en/development-tools/stm32cubeide.html 

:pushpin: ESP32- ESPIDF IDE : Use for esp32 https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html

:pushpin: Anaconda Installation : Install anaconda on this website https://www.anaconda.com/download . Anaconda or any conda distributions will be avoiding conflict when you install Machine Learning or python packages. This is some of my package I used, command for download :
```bash
pip install ultralytics==8.0.196
```
For transmitting data from PC(Raspberry, Jetson,...) to ESP32-S3, use serial library in python:
```bash
pip install pyserial
```
For capturing data from terminal that counts the value of real-time class, use logging to log data from terminal and tracking it by logging:
```bash
pip install logging
```
OpenCV library for draw bounding box 
```bash
pip install opencv-python
```
# Main Hardware Diagram
![Hardware Diagram](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/Brief-%20Diagram.png)

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

Function to read ADC, R: 1-> 1000, voltage : 3.3V

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


