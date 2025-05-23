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