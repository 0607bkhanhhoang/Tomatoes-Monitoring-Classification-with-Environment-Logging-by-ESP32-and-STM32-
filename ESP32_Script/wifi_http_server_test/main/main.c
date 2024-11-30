#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define WIFI_SSID "DEN DA COFFEE"
#define WIFI_PASS "thu6freeupsize"
#define TAG "WebServer"

//UART PARAMETER CONFIGURATION - UART_0
#define ECHO_TEST_TXD 43
#define ECHO_TEST_RXD 44
#define ECHO_TEST_RTS UART_PIN_NO_CHANGE
#define ECHO_TEST_CTS UART_PIN_NO_CHANGE

#define ECHO_UART_PORT_NUM      UART_NUM_0
#define ECHO_UART_BAUD_RATE     115200
#define ECHO_TASK_STACK_SIZE    1024

//UART PARAMETER CONFIGURATION - UART_1
#define SECOND_UART_TXD 17
#define SECOND_UART_RXD 18
#define SECOND_UART_PORT_NUM UART_NUM_1
#define SECOND_UART_BAUD_RATE 115200
#define SECOND_UART_TASK_STACK_SIZE 1024

//static const char *TAG = "ESP_IDF LOGGING DATA";

#define BUF_SIZE 1024

// Data buffer for two types of data
int temp[10] = {0};  
int humid[10] = {0};  
int soil[10] = {0}; 
int uart_bufer[30] = {0};
float value1;
float value2;
int check;

int early_blight;
int mold_leaf;
int tomato_healthy;

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
     
//UART2 - LOGGING FILE
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

static httpd_handle_t server = NULL;

esp_err_t data_handler(httpd_req_t *req) {
    char response[500]; 
    snprintf(response, sizeof(response),
             "{\"type1\": [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d],"
             " \"type2\": [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d],"
             " \"type3\": [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d],"
             " \"early_blight\": %d,"
             " \"mold_leaf\": %d,"
             " \"tomato_healthy\": %d}",
             temp[0], temp[1], temp[2], temp[3], temp[4],
             temp[5], temp[6], temp[7], temp[8], temp[9],
             humid[0], humid[1], humid[2], humid[3], humid[4],
             humid[5], humid[6], humid[7], humid[8], humid[9],
             soil[0], soil[1], soil[2], soil[3], soil[4],
             soil[5], soil[6], soil[7], soil[8], soil[9],
             early_blight, mold_leaf, tomato_healthy);  // Include variables
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}


// HTML page to display the chart
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

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_html = { .uri = "/", .method = HTTP_GET, .handler = html_handler };
        httpd_uri_t uri_data = { .uri = "/data", .method = HTTP_GET, .handler = data_handler };
        httpd_register_uri_handler(server, &uri_html);
        httpd_register_uri_handler(server, &uri_data);
    }
    return server;
}


void wifi_init_sta() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}

void app_main(void) {
    nvs_flash_init();
    wifi_init_sta();
    server = start_webserver();
    xTaskCreate(echo_task_uart, "uart_receive_task", 4096, NULL, 10, NULL);
    xTaskCreate(second_uart_task, "second_uart_receive_task", 4096, NULL, 10, NULL);
}
