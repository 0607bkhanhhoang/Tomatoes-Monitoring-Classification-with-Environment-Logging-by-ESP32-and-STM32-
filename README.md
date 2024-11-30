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




