# Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32 -S3 -and-STM32-

Hi Everyone, this is the project from my course on Ho Chi Minh University of Technology as the course is on **EE3103 : Embedded System Programming**. In this project, on abstraction view, this project will measure temperature, humidity,soil moisture and rain drops state from a microcontroller : **STM32F103C8T6**. This MCU will connect to TFT ILI9341 for displaying temperature, humidity, soil moisture and LCD1602 will display Rain drops state and relay state. 

All the data will be transmitted to **ESP32-S3** and logged into a local webserver. This web will contains data collected from **Raspberry Pi 3B**, data concludes **confidence score** and **counting class ID** from YOLOv8 runned on Raspberry Pi 3B.
