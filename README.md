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

## Schematics
![Schematics_Diagram](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/PCB_and_Hardware/Schematics.jpg)

## PCB View

:pushpin: Top Layer View by Altium

![Top_Layer](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/PCB_and_Hardware/Top_Layer_PCB.jpg)

:pushpin: Bottom Layer View by Altium

![Bottom_layer](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/PCB_and_Hardware/Bottom_Layer_PCB.jpg)

:pushpin: 3D View

![3D](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/PCB_and_Hardware/3D_View.png)

:pushpin: Realistic

![Real_PCB](https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/blob/main/PCB_and_Hardware/Real_Circuit_2.jpg)


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

# ESP32 Script and web page full stack

This script is for an ESP32-based web server that handles real-time environmental data collection and visualization. It uses UART communication to receive sensor data (temperature, humidity, and soil moisture) from two UART ports, processes the data, and serves it on a web interface. Here's a breakdown of the key features:

:pushpin: **Wi-Fi Setup**: The ESP32 connects to a Wi-Fi network using predefined SSID and password.

:pushpin: **UART Communication:**

The script reads environmental data from two UART devices (UART_0 and UART_1), extracting temperature, humidity, and soil moisture values.
It also reads detection data for plant health (e.g., early blight, mold leaf, tomato health) and logs this data.
Data Parsing: The incoming data from UART is parsed and stored in arrays. The parsed values (temperature, humidity, soil moisture, and plant health data) are logged.

:pushpin: **Web Server:**

A simple HTTP server is set up to serve a webpage that visualizes the collected data.
The webpage includes line charts to display real-time temperature, humidity, and soil moisture, as well as pie charts and a switch to toggle the status.
Data is fetched every second and dynamically updates the charts on the page.
Real-Time Updates: The ESP32 continuously updates the webpage with the latest environmental and plant health data, which is fetched via a /data endpoint.

**Web Interface**: The HTML page includes JavaScript (with Chart.js) to create interactive charts for visualizing data and a switch button to interact with the system.

**Task Management**: The script uses FreeRTOS tasks for UART data reception and logging, ensuring that the ESP32 can handle multiple operations concurrently without blocking.

:pen: **UART Task 1 :** Receive from STM32 and **parsering data**

Receive from STM32 UART
```bash
static void echo_task_uart(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    char *data = (char *) malloc(BUF_SIZE);
    while(1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
        
        // Parse the received data
        int temp_val, humid_val, soil_val;
        for(int j =0;j<10;j++){
           sscanf(data, "%d %d %d\r\n", &temp_val, &humid_val, &soil_val);
	       
	        // Extract the first two characters and convert them to an integer
		    char temp_str[3]; 
		    strncpy(temp_str, data, 2); 
		    temp_str[2] = '\0'; 
		    temp_val = atoi(temp_str);
		
		    char humid_str[3];
		    strncpy(humid_str, data + 2, 2); 
		    humid_str[2] = '\0'; 
		    humid_val = atoi(humid_str);
		
		    char soil_str[3];
		    strncpy(soil_str, data + 4, 2); 
		    soil_str[2] = '\0'; 
		    soil_val = atoi(soil_str);	
		        
            temp[j] = temp_val;
            humid[j] = humid_val;
            soil[j] = soil_val;
                
           	ESP_LOGI("UART","data : %s", data);
	       	ESP_LOGI("UART", "Received - Temp: %d, Humidity: %d, Soil: %d", temp_val, humid_val, soil_val);
	       	
            vTaskDelay(500 / portTICK_PERIOD_MS);
          }
        }
            free(data);
     }
```

:pen: **UART TASK 2 :** Receive from PC and logging

:email:Create functiong for **Parsering Data** on ESP
```bash
void parse_detection_data(const char *data) {
    int early_blight = 0;
    int mold_leaf = 0;
    int tomato_healthy = 0;

    // Check for early_blight, mold_leaf, and tomato_healthy in the received string
    if (strstr(data, "early_blight:") != NULL) {
        early_blight = atoi(strstr(data, "early_blight:") + 13);  
    }
    if (strstr(data, "mold_leaf:") != NULL) {
        mold_leaf = atoi(strstr(data, "mold_leaf:") + 10);  
    }
    if (strstr(data, "tomato_healthy:") != NULL) {
        tomato_healthy = atoi(strstr(data, "tomato_healthy:") + 15);  
    }

    // Log the parsed values
    ESP_LOGI(TAG, "Parsed detection data: early_blight=%d, mold_leaf=%d, tomato_healthy=%d",
             early_blight, mold_leaf, tomato_healthy);
}
```
Task 2 on UART
```bash
static void second_uart_task(void *arg) {
    // Configure the second UART
    uart_config_t uart_config = {
        .baud_rate = SECOND_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(SECOND_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(SECOND_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SECOND_UART_PORT_NUM, SECOND_UART_TXD, SECOND_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Buffer for receiving data
    uint8_t data_2[BUF_SIZE];
    int length = 0;
    
    while (1) {
        length = uart_read_bytes(SECOND_UART_PORT_NUM, data_2, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (length > 0) {

            data_2[length] = '\0';

            ESP_LOGI(TAG, "Received data: %s", data_2);

            parse_detection_data((const char *)data_2);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); 
    }
    free(data_2);
}
```

:pushpin: **WEBSERVER:** Configuring ESP32-S3 on Station mode and create Web based on local IP
:pushpin:**Data Handling:**

The server processes sensor data (temperature, humidity, soil moisture) and plant health data (early blight, mold leaf, tomato health) received through UART communication.
Real-Time Data Visualization:

The web page served by the ESP32 features real-time line charts and pie charts to display environmental data (temperature, humidity, soil moisture) and the detection results of plant health.
The data is updated every second using JavaScript (with the Chart.js library), so the user can monitor changes live.
HTML and JavaScript Interface:

The web interface includes a switch button to toggle a setting, displaying real-time updates on the device's status.
The page uses AJAX to fetch the latest data from the server and update the charts dynamically without needing to reload the page.

:pen: **WEB PAGE**

```bash
const char *html_page = 
    "<!DOCTYPE html>"
    "<html>"
    "<head><title>ESP32 WEB SERVER</title>"
    "<script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>"
    "<style>"
        ".switch {"
            "position: relative;"
            "display: inline-block;"
            "width: 60px;"
            "height: 34px;"
        "}"
        ".switch input {"
            "opacity: 0;"
            "width: 0;"
            "height: 0;"
        "}"
        ".slider {"
            "position: absolute;"
            "cursor: pointer;"
            "top: 0;"
            "left: 0;"
            "right: 0;"
            "bottom: 0;"
            "background-color: #ccc;"
            "transition: 0.4s;"
            "border-radius: 34px;"
        "}"
        ".slider:before {"
            "position: absolute;"
            "content: \"\";"
            "height: 26px;"
            "width: 26px;"
            "left: 4px;"
            "bottom: 4px;"
            "background-color: white;"
            "transition: 0.4s;"
            "border-radius: 50%;"
        "}"
        "input:checked + .slider {"
            "background-color: #2196F3;"
        "}"
        "input:checked + .slider:before {"
            "transform: translateX(26px);"
        "}"
    "</style>"
    "</head>"
    "<body>"
    "<h2>Real-time Data Charts</h2>"
    "<h3>Line Charts</h3>"
    "<canvas id=\"lineChart1\" width=\"400\" height=\"200\"></canvas><br>"
    "<canvas id=\"lineChart2\" width=\"400\" height=\"200\"></canvas><br>"
    "<canvas id=\"lineChart3\" width=\"400\" height=\"200\"></canvas>"
    "<h3>Pie Charts</h3>"
    "<canvas id=\"pieChart1\" width=\"400\" height=\"200\"></canvas><br>"
    "<canvas id=\"pieChart2\" width=\"400\" height=\"200\"></canvas><br>"
    "<canvas id=\"pieChart3\" width=\"400\" height=\"200\"></canvas>"
    "<h4>Detection Data</h4>"
    "<p id=\"early_blight\">Early Blight Count: 0</p>"
    "<p id=\"mold_leaf\">Mold Leaf Count: 0</p>"
    "<p id=\"tomato_healthy\">Tomato Healthy Count: 0</p>"
    "<h2>Switch Button Example</h2>"
    "<label class=\"switch\">"
        "<input type=\"checkbox\" id=\"toggleSwitch\">"
        "<span class=\"slider\"></span>"
    "</label>"
    "<p id=\"status\">Switch is off</p>"
    "<script>"
        "let a_switch = \"off\";"
        "const toggleSwitch = document.getElementById(\"toggleSwitch\");"
        "const statusText = document.getElementById(\"status\");"
        "toggleSwitch.addEventListener(\"change\", function () {"
            "if (toggleSwitch.checked) {"
                "a_switch = \"on\";"
                "statusText.textContent = \"Switch is on\";"
            "} else {"
                "a_switch = \"off\";"
                "statusText.textContent = \"Switch is off\";"
            "}"
            "console.log(\"Switch status:\", a_switch);"
        "});"
        "const lineCtx1 = document.getElementById('lineChart1').getContext('2d');"
        "const lineCtx2 = document.getElementById('lineChart2').getContext('2d');"
        "const lineCtx3 = document.getElementById('lineChart3').getContext('2d');"
        "const pieCtx1 = document.getElementById('pieChart1').getContext('2d');"
        "const pieCtx2 = document.getElementById('pieChart2').getContext('2d');"
        "const pieCtx3 = document.getElementById('pieChart3').getContext('2d');"
        "const lineChart1 = new Chart(lineCtx1, {"
            "type: 'line',"
            "data: {"
                "labels: ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10'],"
                "datasets: [{"
                    "label: 'Temperature',"
                    "data: [],"
                    "borderColor: 'rgba(75, 192, 192, 1)',"
                    "fill: false"
                "}]"
            "},"
            "options: {"
                "responsive: true,"
                "scales: {"
                    "x: { grid: { display: true, color: 'rgba(200, 200, 200, 0.8)' } },"
                    "y: { grid: { display: true, color: 'rgba(200, 200, 200, 0.8)' }, beginAtZero: true }"
                "}"
            "}"
        "});"
        "const lineChart2 = new Chart(lineCtx2, {"
            "type: 'line',"
            "data: {"
                "labels: ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10'],"
                "datasets: [{"
                    "label: 'Humidity',"
                    "data: [],"
                    "borderColor: 'rgba(153, 102, 255, 1)',"
                    "fill: false"
                "}]"
            "},"
            "options: {"
                "responsive: true,"
                "scales: {"
                    "x: { grid: { display: true, color: 'rgba(200, 200, 200, 0.8)' } },"
                    "y: { grid: { display: true, color: 'rgba(200, 200, 200, 0.8)' }, beginAtZero: true }"
                "}"
            "}"
        "});"
        "const lineChart3 = new Chart(lineCtx3, {"
            "type: 'line',"
            "data: {"
                "labels: ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10'],"
                "datasets: [{"
                    "label: 'Soil Moisture',"
                    "data: [],"
                    "borderColor: 'rgba(255, 159, 64, 1)',"
                    "fill: false"
                "}]"
            "},"
            "options: {"
                "responsive: true,"
                "scales: {"
                    "x: { grid: { display: true, color: 'rgba(200, 200, 200, 0.8)' } },"
                    "y: { grid: { display: true, color: 'rgba(200, 200, 200, 0.8)' }, beginAtZero: true }"
                "}"
            "}"
        "});"
        "function updateChartData() {"
            "fetch('/data').then(response => response.json()).then(data => {"
                "lineChart1.data.datasets[0].data = data.type1;"
                "lineChart2.data.datasets[0].data = data.type2;"
                "lineChart3.data.datasets[0].data = data.type3;"
                "document.getElementById('early_blight').textContent = 'Early Blight Count: ' + data.early_blight;"
                "document.getElementById('mold_leaf').textContent = 'Mold Leaf Count: ' + data.mold_leaf;"
                "document.getElementById('tomato_healthy').textContent = 'Tomato Healthy Count: ' + data.tomato_healthy;"
                "lineChart1.update();"
                "lineChart2.update();"
                "lineChart3.update();"
            "}).catch(error => console.error('Error fetching data:', error));"
        "}"
        "setInterval(updateChartData, 1000);"
    "</script>"
    "</body>"
    "</html>";
    

esp_err_t html_handler(httpd_req_t *req) {
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}
```

# Machine Learning Model Visualization 
:pushpin: **View on this :** https://github.com/0607bkhanhhoang/Tomatoes-Monitoring-Classification-with-Environment-Logging-by-ESP32-and-STM32-/tree/main/model_visualization

This script is designed to perform real-time object detection using YOLO (You Only Look Once) and communicate detection results to an ESP32 via a serial connection. The overall goal is to detect specific tomato deseases using a webcam, log the detection results, and periodically send these results to an ESP32 device. Here is a breakdown of the script’s key components:

# Key Components:
## Logging Setup:

The script uses Python's logging module to log detection events into a file called capture.log. This allows for tracking YOLO inference results and error messages.

## StreamToLogger Class:

Captures print statements (stdout) and error messages (stderr) and sends them to the logger instead of printing to the terminal. This ensures that all important messages are logged for review.
send_to_esp32 Function:

Sends detection data (like the count of specific detected plant diseases) to the ESP32 via a serial connection. The data is formatted as a string and sent through the serial interface.
YOLO Inference (run_yolo_inference Function):

Initializes the YOLO model (best.pt) and captures frames from the webcam.
Each frame is processed by YOLO for object detection, and bounding boxes, confidence scores, and class IDs are logged.
Detected objects are annotated in the webcam feed (bounding boxes displayed), and the feed is shown on the screen.
Processing and Sending Detection Counts (process_and_send_counts Function):

Reads the capture.log file where YOLO detections are logged.
Counts the occurrences of specific class IDs corresponding to plant diseases (early_blight, mold_leaf, and tomato_healthy).
Sends these counts to the ESP32 device over the serial connection.
If any data is received from the ESP32, it is logged.
## Multithreading:

The script uses Python's threading module to run two tasks concurrently:
YOLO inference and webcam processing.
Processing log data, counting detections, and sending this information to the ESP32.
## Main Function:

Initializes and starts two threads: one for running YOLO inference (run_yolo_inference) and one for processing log data and sending counts to the ESP32 (process_and_send_counts).
Serial Communication:

The ESP32 device is connected via serial port (COM10 with baud rate 115200).
The script sends detection counts over the serial link and waits for responses from the ESP32.
## Execution Flow:
YOLO Detection: The webcam feed is continuously captured and processed by the YOLO model for object detection. The results are logged.
Counting and Sending: Detection counts (based on specific class IDs) are periodically computed from the log file and sent to the ESP32 via serial communication.
Real-time Interaction: Detection data is updated regularly, and communication with the ESP32 allows for real-time feedback.

### Function to send to ESP32
```bash
def send_to_esp32(data):
    try:
        message = f"{data}\n"
        ser.write(message.encode())
        print(f"Sent detection data to ESP32: {message}")
    except Exception as e:
        print(f"Error sending data to ESP32: {e}")
```

### Setting log output
```bash
class StreamToLogger:
    """
    Captures the output from terminal and sends it to the logger.
    """
    def __init__(self, logger, log_level):
        self.logger = logger
        self.log_level = log_level

    def write(self, message):
        """
        Captures and logs the message.
        """
        if message.strip():  
            self.logger.log(self.log_level, message.strip())

    def flush(self):
        """
        Flush the stream (not needed for this implementation).
        """
        pass
    
sys.stdout = StreamToLogger(logging.getLogger(), logging.INFO)  
sys.stderr = StreamToLogger(logging.getLogger(), logging.ERROR)
```

### Process Log File 
**Processing log file captured from terminal**
```bash
def process_and_send_counts():
    """
    Processes the log file to count detections and sends the counts over USB to ESP32.
    """
    ser = serial.Serial(port, baudrate, timeout=1)  # Connect to ESP32
    log_file = "capture.log"

    while True:  
        try:
            blight = 0  # early_blight counting value
            mold = 0    # mold_leaf counting value
            tomato = 0  # tomato_healthy counting value

            # Read the log file and count detections
            with open(log_file, "r") as file:
                for line in file:
                    if "Class ID: [2.]" in line:
                        blight += 1
                    if "Class ID: [1.]" in line:
                        mold += 1
                    if "Class ID: [3.]" in line:
                        tomato += 1

            detection_data = f"early_blight:{blight},mold_leaf:{mold},tomato_healthy:{tomato}"

            send_to_esp32(detection_data)

            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                print(f"Received from ESP32: {response}")

            time.sleep(1)

        except Exception as e:
            print(f"Error in USB communication: {e}")
            break

    ser.close()
    print("Serial communication ended.")
```
