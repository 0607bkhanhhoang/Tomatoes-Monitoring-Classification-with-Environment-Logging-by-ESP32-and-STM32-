/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body using CMSIS LL drivers with ILI9341 Touch
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "i2c-lcd.h"
#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"
#include "xpt2046.h"
#include <stdio.h>

float Temperature = 0, Humidity = 0, Soil = 0;
uint8_t temp = 0, humid = 0, soilval = 0;

void delay_us(uint32_t us) {
    LL_TIM_SetCounter(TIM1, 0);
    while (LL_TIM_GetCounter(TIM1) < us);
}

void Display_LCD() {
    char str[20];
    lcd_put_cur(0, 0);
    sprintf(str, "T:%.1f H:%.1f", Temperature, Humidity);
    lcd_send_string(str);

    lcd_put_cur(1, 4);
    sprintf(str, "M:%.1f", Soil);
    lcd_send_string(str);
}

void Display_TFT() {
    char buf[30];
    ILI9341_FillScreen(WHITE);

    sprintf(buf, "TEMP: %.2f", Temperature);
    ILI9341_DrawText(buf, FONT4, 10, 40, BLACK, WHITE);

    sprintf(buf, "HUMID: %.2f", Humidity);
    ILI9341_DrawText(buf, FONT4, 10, 80, BLACK, WHITE);

    sprintf(buf, "SOIL: %.2f", Soil);
    ILI9341_DrawText(buf, FONT4, 10, 120, BLACK, WHITE);
}

void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read(void);

void Read_DHT11() {
    uint8_t Rh1, Rh2, T1, T2, sum;
    DHT11_Start();
    if (DHT11_Check_Response()) {
        Rh1 = DHT11_Read(); Rh2 = DHT11_Read();
        T1 = DHT11_Read(); T2 = DHT11_Read();
        sum = DHT11_Read();
        if (sum == (Rh1 + Rh2 + T1 + T2)) {
            Humidity = Rh1;
            Temperature = T1;
        }
    }
}

float Read_Soil() {
    LL_ADC_REG_StartConversionSWStart(ADC1);
    while (!LL_ADC_IsActiveFlag_EOC(ADC1));
    uint32_t val = LL_ADC_REG_ReadConversionData12(ADC1);
    return (3.3 * val / 4095.0) * 100;
}

int main(void) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    SystemClock_Config();
    MX_GPIO_Init_LL();
    MX_ADC1_Init_LL();
    MX_TIM1_Init_LL();
    MX_USART1_UART_Init_LL();

    LL_TIM_EnableCounter(TIM1);
    lcd_init();
    ILI9341_Init();
    ILI9341_SetRotation(SCREEN_HORIZONTAL_2);
    ILI9341_FillScreen(WHITE);

    while (1) {
        Read_DHT11();
        Soil = Read_Soil();

        Display_LCD();
        Display_TFT();

        temp = (uint8_t)Temperature;
        humid = (uint8_t)Humidity;
        soilval = (uint8_t)Soil;

        char TxData[32];
        sprintf(TxData, "%d,%d,%d\r\n", temp, humid, soilval);
        for (uint8_t i = 0; i < strlen(TxData); i++) {
            while (!LL_USART_IsActiveFlag_TXE(USART1));
            LL_USART_TransmitData8(USART1, TxData[i]);
        }

        uint16_t x, y;
        if (XPT2046_Touched()) {
            XPT2046_GetTouch(&x, &y);
            char buf[32];
            sprintf(buf, "Touch: %d,%d", x, y);
            ILI9341_DrawText(buf, FONT4, 10, 160, RED, WHITE);
        }

        LL_mDelay(1000);
    }
}

// Ensure to implement all MX_*.c functions with LL drivers instead of HAL
