#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>

// Определения пинов
#define LED_STATUS_PIN          GPIO_Pin_13
#define LED_STATUS_PORT         GPIOC
#define PWRKEY_PIN              GPIO_Pin_8
#define PWRKEY_PORT             GPIOA

// Таймауты
#define POWER_ON_TIMEOUT_MS     3000
#define AT_COMMAND_TIMEOUT_MS   5000
#define RESPONSE_TIMEOUT_MS     3000

// Буферы для USART
#define RX_BUFFER_SIZE          512
#define TX_BUFFER_SIZE          256

// Состояния модуля
typedef enum {
    SIM_STATE_POWER_OFF = 0,
    SIM_STATE_POWERING_ON,
    SIM_STATE_READY,
    SIM_STATE_REGISTERING,
    SIM_STATE_REGISTERED,
    SIM_STATE_ERROR
} SimState_t;

// Структура для хранения данных
typedef struct {
    SimState_t state;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint16_t rx_index;
    uint8_t tx_buffer[TX_BUFFER_SIZE];
    uint32_t last_tick;
    uint8_t response_received;
    uint8_t network_registered;
    uint8_t gprs_attached;
    char operator_name[32];
    uint8_t signal_strength;
} Sim868_Handler_t;

// Глобальные переменные
Sim868_Handler_t sim868 = {0};
volatile uint32_t systick_counter = 0;

// Прототипы функций
void SystemClock_Config(void);
void GPIO_Init(void);
void USART2_Init(void);
void SysTick_Init(void);
void Delay_ms(uint32_t ms);
void SIM868_PowerOn(void);
void SIM868_PowerOff(void);
void SIM868_SendATCommand(const char* cmd);
void SIM868_SendATCommandWithResponse(const char* cmd, const char* expected_response, uint32_t timeout);
uint8_t SIM868_WaitForResponse(const char* expected_response, uint32_t timeout);
void SIM868_CheckNetwork(void);
void SIM868_GetOperatorInfo(void);
void SIM868_GetSignalStrength(void);
void SIM868_SetupGPRS(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

// Прерывание SysTick
void SysTick_Handler(void) {
    systick_counter++;
}

// Прерывание USART2
void USART2_IRQHandler(void) {
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART2);
        
        if(sim868.rx_index < RX_BUFFER_SIZE - 1) {
            sim868.rx_buffer[sim868.rx_index++] = data;
            
            // Проверка на конец строки
            if(data == '\n') {
                sim868.rx_buffer[sim868.rx_index] = '\0';
                sim868.response_received = 1;
            }
        }
        
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
    
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        USART_ClearITPendingBit(USART2, USART_IT_TXE);
    }
}

// Настройка тактирования
void SystemClock_Config(void) {
    RCC_DeInit();
    
    // Включение HSI
    RCC_HSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
    
    // Настройка AHB, APB1, APB2
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_PCLK2Config(RCC_HCLK_Div1);
    
    // Настройка FLASH
    FLASH_SetLatency(FLASH_Latency_1);
    FLASH_PrefetchBufferCmd(ENABLE);
    
    // Выбор HSI как источника системной частоты
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    while(RCC_GetSYSCLKSource() != 0x00);
}

// Инициализация GPIO
void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // Включение тактирования портов
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    // Настройка LED на PC13
    GPIO_InitStruct.GPIO_Pin = LED_STATUS_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_Init(LED_STATUS_PORT, &GPIO_InitStruct);
    LED_Off();
    
    // Настройка PWRKEY на PA8
    GPIO_InitStruct.GPIO_Pin = PWRKEY_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_Init(PWRKEY_PORT, &GPIO_InitStruct);
    
    // Изначально PWRKEY в низком уровне
    GPIO_ResetBits(PWRKEY_PORT, PWRKEY_PIN);
    
    // Настройка USART2 pins (PA2 - TX, PA3 - RX)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Альтернативные функции
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
}

// Инициализация USART2
void USART2_Init(void) {
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART2, &USART_InitStruct);
    
    // Включение прерывания по приему
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    
    // Настройка NVIC для USART2
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    USART_Cmd(USART2, ENABLE);
}

// Инициализация SysTick
void SysTick_Init(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000); // 1ms interrupts
}

// Задержка в миллисекундах
void Delay_ms(uint32_t ms) {
    uint32_t start = systick_counter;
    while((systick_counter - start) < ms);
}

// Управление LED
void LED_On(void) {
    GPIO_SetBits(LED_STATUS_PORT, LED_STATUS_PIN);
}

void LED_Off(void) {
    GPIO_ResetBits(LED_STATUS_PORT, LED_STATUS_PIN);
}

void LED_Toggle(void) {
    GPIO_ToggleBits(LED_STATUS_PORT, LED_STATUS_PIN);
}

// Включение модуля SIM868
void SIM868_PowerOn(void) {
    LED_Off();
    sim868.state = SIM_STATE_POWERING_ON;
    
    // Удерживаем PWRKEY высоким 1 секунду
    GPIO_SetBits(PWRKEY_PORT, PWRKEY_PIN);
    Delay_ms(1000);
    GPIO_ResetBits(PWRKEY_PORT, PWRKEY_PIN);
    
    // Ждем запуска модуля
    Delay_ms(POWER_ON_TIMEOUT_MS);
    
    // Очищаем буфер приема
    sim868.rx_index = 0;
    memset(sim868.rx_buffer, 0, RX_BUFFER_SIZE);
    
    // Проверяем, отвечает ли модуль
    SIM868_SendATCommand("AT\r\n");
    if(SIM868_WaitForResponse("OK", 2000)) {
        sim868.state = SIM_STATE_READY;
        LED_On();
    } else {
        sim868.state = SIM_STATE_ERROR;
        // Пробуем еще раз
        SIM868_PowerOn();
    }
}

// Выключение модуля SIM868
void SIM868_PowerOff(void) {
    // Отправляем команду на выключение
    SIM868_SendATCommand("AT+CPOWD=1\r\n");
    Delay_ms(3000);
    
    // Принудительно держим PWRKEY для выключения
    GPIO_SetBits(PWRKEY_PORT, PWRKEY_PIN);
    Delay_ms(3000);
    GPIO_ResetBits(PWRKEY_PORT, PWRKEY_PIN);
    
    sim868.state = SIM_STATE_POWER_OFF;
    LED_Off();
}

// Отправка AT команды
void SIM868_SendATCommand(const char* cmd) {
    // Очищаем буфер приема
    sim868.rx_index = 0;
    memset(sim868.rx_buffer, 0, RX_BUFFER_SIZE);
    sim868.response_received = 0;
    
    // Отправляем команду
    while(*cmd) {
        USART_SendData(USART2, *cmd++);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
}

// Отправка AT команды с ожиданием ответа
void SIM868_SendATCommandWithResponse(const char* cmd, const char* expected_response, uint32_t timeout) {
    SIM868_SendATCommand(cmd);
    SIM868_WaitForResponse(expected_response, timeout);
}

// Ожидание ответа
uint8_t SIM868_WaitForResponse(const char* expected_response, uint32_t timeout) {
    uint32_t start_time = systick_counter;
    
    while((systick_counter - start_time) < timeout) {
        if(sim868.response_received) {
            if(strstr((char*)sim868.rx_buffer, expected_response) != NULL) {
                sim868.response_received = 0;
                return 1;
            }
            sim868.response_received = 0;
        }
    }
    return 0;
}

// Проверка регистрации в сети
void SIM868_CheckNetwork(void) {
    SIM868_SendATCommand("AT+CREG?\r\n");
    
    if(SIM868_WaitForResponse("+CREG:", 3000)) {
        // Ищем статус регистрации в ответе
        char* response = (char*)sim868.rx_buffer;
        char* pos = strstr(response, "+CREG:");
        if(pos) {
            int status = 0;
            sscanf(pos, "+CREG: %*d,%d", &status);
            
            if(status == 1 || status == 5) {
                sim868.network_registered = 1;
                sim868.state = SIM_STATE_REGISTERED;
            } else {
                sim868.network_registered = 0;
                sim868.state = SIM_STATE_REGISTERING;
            }
        }
    }
}

// Получение информации об операторе
void SIM868_GetOperatorInfo(void) {
    SIM868_SendATCommand("AT+COPS?\r\n");
    
    if(SIM868_WaitForResponse("+COPS:", 3000)) {
        char* response = (char*)sim868.rx_buffer;
        char* pos = strstr(response, "+COPS:");
        if(pos) {
            // Парсим имя оператора
            // Формат: +COPS: <mode>[,<format>[,<oper>]]
            char* quote_start = strchr(pos, '\"');
            if(quote_start) {
                char* quote_end = strchr(quote_start + 1, '\"');
                if(quote_end) {
                    int len = quote_end - quote_start - 1;
                    if(len > 0 && len < sizeof(sim868.operator_name) - 1) {
                        strncpy(sim868.operator_name, quote_start + 1, len);
                        sim868.operator_name[len] = '\0';
                    }
                }
            }
        }
    }
}

// Получение уровня сигнала
void SIM868_GetSignalStrength(void) {
    SIM868_SendATCommand("AT+CSQ\r\n");
    
    if(SIM868_WaitForResponse("+CSQ:", 3000)) {
        char* response = (char*)sim868.rx_buffer;
        char* pos = strstr(response, "+CSQ:");
        if(pos) {
            int rssi = 99;
            sscanf(pos, "+CSQ: %d", &rssi);
            
            if(rssi >= 0 && rssi <= 31) {
                // Преобразуем в проценты (0-31 -> 0-100%)
                sim868.signal_strength = (rssi * 100) / 31;
            } else if(rssi == 99) {
                sim868.signal_strength = 0; // Неизвестно или нет сигнала
            }
        }
    }
}

// Настройка GPRS (базовая конфигурация)
void SIM868_SetupGPRS(void) {
    // Отключаем GPRS на всякий случай
    SIM868_SendATCommandWithResponse("AT+CGATT=0\r\n", "OK", 5000);
    Delay_ms(1000);
    
    // Устанавливаем APN (замените на свой)
    SIM868_SendATCommandWithResponse("AT+CSTT=\"internet\",\"\",\"\"\r\n", "OK", 5000);
    Delay_ms(1000);
    
    // Включаем GPRS
    SIM868_SendATCommandWithResponse("AT+CIICR\r\n", "OK", 10000);
    Delay_ms(1000);
    
    // Получаем IP адрес
    SIM868_SendATCommandWithResponse("AT+CIFSR\r\n", ".", 5000);
    
    if(strstr((char*)sim868.rx_buffer, ".") != NULL) {
        sim868.gprs_attached = 1;
    }
}

// Основная инициализация модуля
void SIM868_Init(void) {
    uint8_t init_attempts = 0;
    
    while(init_attempts < 3 && sim868.state != SIM_STATE_READY) {
        SIM868_PowerOn();
        init_attempts++;
        Delay_ms(1000);
    }
    
    if(sim868.state == SIM_STATE_READY) {
        // Настраиваем эхо-режим
        SIM868_SendATCommandWithResponse("ATE0\r\n", "OK", 2000);
        
        // Проверяем SIM карту
        SIM868_SendATCommandWithResponse("AT+CPIN?\r\n", "+CPIN: READY", 3000);
        
        // Получаем информацию об операторе
        SIM868_GetOperatorInfo();
        
        // Получаем уровень сигнала
        SIM868_GetSignalStrength();
        
        // Проверяем сеть
        SIM868_CheckNetwork();
    }
}

// Основной цикл
int main(void) {
    // Инициализация
    SystemClock_Config();
    SysTick_Init();
    GPIO_Init();
    USART2_Init();
    
    // Включаем прерывания
    __enable_irq();
    
    // Инициализация SIM868
    SIM868_Init();
    
    // Основной рабочий цикл
    while(1) {
        // Мигаем LED в зависимости от состояния
        switch(sim868.state) {
            case SIM_STATE_READY:
            case SIM_STATE_REGISTERING:
                // Медленное мигание (1 Гц)
                if(systick_counter % 1000 < 500) {
                    LED_On();
                } else {
                    LED_Off();
                }
                break;
                
            case SIM_STATE_REGISTERED:
                // Быстрое мигание (2 Гц)
                if(systick_counter % 500 < 250) {
                    LED_On();
                } else {
                    LED_Off();
                }
                
                // Если GPRS не активирован, пробуем активировать
                if(!sim868.gprs_attached) {
                    SIM868_SetupGPRS();
                }
                break;
                
            case SIM_STATE_ERROR:
                // Три коротких вспышки
                for(int i = 0; i < 3; i++) {
                    LED_On();
                    Delay_ms(100);
                    LED_Off();
                    Delay_ms(100);
                }
                Delay_ms(1000);
                break;
                
            default:
                LED_Off();
                break;
        }
        
        // Периодическая проверка сети каждые 10 секунд
        if(systick_counter % 10000 == 0) {
            if(sim868.state >= SIM_STATE_READY) {
                SIM868_CheckNetwork();
            }
        }
        
        // Периодическое обновление уровня сигнала каждые 30 секунд
        if(systick_counter % 30000 == 0) {
            if(sim868.state >= SIM_STATE_READY) {
                SIM868_GetSignalStrength();
            }
        }
    }
}
