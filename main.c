#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* =========================================================
   COMMANDES ROBOT
========================================================= */
typedef enum {
    CMD_NONE = 0,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_STOP
} RobotCmd_t;

/* =========================================================
   FREE RTOS OBJECTS
========================================================= */
xQueueHandle ADC_Queue;
xQueueHandle Buffer_Queue;
xQueueHandle QUEUE_REC_CMD;
xSemaphoreHandle SEM_STOP;

/* =========================================================
   UART RX BUFFER
========================================================= */
#define RX_BUFFER_SIZE 32
static char RX_Buffer[RX_BUFFER_SIZE];
static uint8_t rx_index = 0;

/* =========================================================
   ADC VARIABLES
========================================================= */
uint16_t ADC_VALUE[3];
uint8_t adc_index = 0;
uint8_t current_channel = 4;

/* =========================================================
   DS1621 I2C DEFINITIONS
========================================================= */
#define DS1621_WRITE_ADDR 0x90
#define DS1621_READ_ADDR  0x91
#define DS1621_START_CONVERSION    0xEE
#define DS1621_READ_TEMP  0xAA
#define DS1621_ACCESS_CONFIG       0xAC
#define DS1621_ADDR       0x48

/* =========================================================
   SYSTEM CLOCK 84 MHz
========================================================= */
void SystemClock_Config(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    RCC->PLLCFGR = (8U<<0) | (336U<<6) | RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    FLASH->ACR = FLASH_ACR_LATENCY_5WS;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void delay(int ncount)
{
	while(ncount--);
}
/* =========================================================
   USART2 INIT
========================================================= */
void USART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER |= (2<<4)|(2<<6);
    GPIOA->AFR[0] |= (7<<8)|(7<<12);

    USART2->BRR = (SystemCoreClock/4)/9600;
    USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(USART2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1);
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_SendString(char *s)
 { while(*s){ while(!(USART2->SR & USART_SR_TXE));
 USART2->DR = *s++; } }



/* =========================================================
   MOTOR PWM + DIRECTION
========================================================= */
void Motor_PWM_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    GPIOD->MODER |= (2<<24)|(2<<26)|(2<<28)|(2<<30);
    GPIOD->AFR[1] |= (2<<16)|(2<<20)|(2<<24)|(2<<28);

    TIM4->PSC = 84-1;
    TIM4->ARR = 1000-1;
    TIM4->CCMR1 = (6<<4)|(6<<12);
    TIM4->CCMR2 = (6<<4)|(6<<12);
    TIM4->CCER = 0x1111;
    TIM4->CR1 |= TIM_CR1_CEN;
}

void Motor_DIR_Init(void)
{
    GPIOD->MODER |= 0x55;
}

void config_I2C1(void)
{
    // Configuration GPIO pour I2C1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  
    GPIOB->OTYPER |= (1<<6) | (1<<7);//GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7;  
    GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; 
    GPIOB->AFR[0] |= (4<<24) | (4<<28);//GPIO_AFRL_AFRL6_2 | GPIO_AFRL_AFRL7_2;  

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; 
    // Configure I2C1 for 100 kHz	
    I2C1->CCR &= ~I2C_CCR_FS;  
    I2C1->CR2 = 42;  // APB1 = 42 MHz frequency in MHz
    I2C1->CCR = 210; // 42MHz / (2 * 100kHz) = 210
    I2C1->TRISE = 43;  // (1000ns / 23.8ns) + 1 = 43
    I2C1->CR1 |= I2C_CR1_PE; 
}
/* ========== DS1621 Functions ========== */
/**
  *   Envoi d'une commande I2C au DS1621
  */
void I2C_COMMAND(int command)
{
    I2C1->CR1 |= I2C_CR1_START; 
    while (!(I2C1->SR1 & I2C_SR1_SB));  
    I2C1->DR = DS1621_WRITE_ADDR;  
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); 
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    while (!(I2C1->SR1 & I2C_SR1_TXE));  
    I2C1->DR = command; 

    I2C1->CR1 |= I2C_CR1_STOP;
    while(I2C1->CR1 & I2C_CR1_STOP);
}
/**
  *   Démarrage de la conversion DS1621
  */

void Start_DS1621_Conv(void) 
{
    I2C_COMMAND(DS1621_START_CONVERSION); 
    delay(10);
}
/**
  *   Initialisation du DS1621
  */
void DS1621_Init(void) 
{
    // Send Access Config command
    I2C1->CR1 |= I2C_CR1_START; 
    while (!(I2C1->SR1 & I2C_SR1_SB));  
    I2C1->DR = DS1621_WRITE_ADDR;  
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); 
    (void)I2C1->SR1;  
    (void)I2C1->SR2;

    while (!(I2C1->SR1 & I2C_SR1_TXE));  
    I2C1->DR = DS1621_ACCESS_CONFIG; 
    
    while (!(I2C1->SR1 & I2C_SR1_TXE));  
    I2C1->DR = 0x00;  // Set continuous conversion mode

    I2C1->CR1 |= I2C_CR1_STOP;
    while(I2C1->CR1 & I2C_CR1_STOP);
    
    delay(10);
    
    // Start conversion
    Start_DS1621_Conv();
}

/*
  *  Lecture de la température du DS1621
  *  Température en °C
 */
float I2C_ReadTemperature(void) 
{
    int Msb, Lsb;
    float Temp;

    I2C_COMMAND(DS1621_READ_TEMP);  
    delay(5);
    
    I2C1->CR1 |= I2C_CR1_START;  
    while (!(I2C1->SR1 & I2C_SR1_SB));  

    I2C1->DR = DS1621_READ_ADDR;  
    while (!(I2C1->SR1 & I2C_SR1_ADDR));  
    (void)I2C1->SR1;  
    (void)I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_ACK; 
    while (!(I2C1->SR1 & I2C_SR1_RXNE));  
    Msb = I2C1->DR;  

    I2C1->CR1 &= ~I2C_CR1_ACK; 
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));  
    Lsb = I2C1->DR; 

    if (Msb & 0x80) {  // Negative temperature
        Temp = Msb - 256;
    } else {            // Positive temperature
        Temp = Msb;
    }
    
    // Add 0.5 degrees if bit 7 of LSB is set
    if (Lsb & 0x80) {
        Temp += 0.5;
    }
    
    while(I2C1->CR1 & I2C_CR1_STOP);
    
    return Temp;
}

void Motor_Set(RobotCmd_t cmd, uint32_t speed)
{
  
    // Stopper toutes les roues
    GPIOD->ODR &= ~0x0F;
    TIM4->CCR1 = TIM4->CCR2 = TIM4->CCR3 = TIM4->CCR4 = 0;

    switch(cmd)
    {
        case CMD_FORWARD:  // AVANCE ? 2 moteurs ensemble
            GPIOD->ODR |= (1<<0)|(1<<2); // Moteur gauche avant + droit avant
            TIM4->CCR1 = TIM4->CCR3 = 500; // PWM vitesse
            TIM4->CCR2 = TIM4->CCR4 = 0;
            break;

        case CMD_BACKWARD: // RECULE ? 2 moteurs ensemble
            GPIOD->ODR |= (1<<1)|(1<<3); // Moteur gauche arrière + droit arrière
            TIM4->CCR2 = TIM4->CCR4 = 500; // PWM vitesse
            TIM4->CCR1 = TIM4->CCR3 = 0;
            break;

        case CMD_RIGHT:    // DROITE ? seulement moteur droit
            GPIOD->ODR |= (1<<2);   // roue droite avant
            TIM4->CCR3 = 500;       // PWM moteur droit
            TIM4->CCR1 = TIM4->CCR2 = TIM4->CCR4 = 0;
            break;

        case CMD_LEFT:     // GAUCHE ? seulement moteur gauche
            GPIOD->ODR |= (1<<0);   // roue gauche avant
            TIM4->CCR1 = 500;       // PWM moteur gauche
            TIM4->CCR2 = TIM4->CCR3 = TIM4->CCR4 = 0;
            break;

        case CMD_STOP:     // STOP ? tout s’arrête
            TIM4->CCR1 = TIM4->CCR2 = TIM4->CCR3 = TIM4->CCR4 = 0;
            break;
    }
}


/* =========================================================
   ADC + TIMER + IRQ
========================================================= */
void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 8399;
    TIM2->ARR = 149999;
	  TIM2->CR2 &= ~TIM_CR2_MMS;

    TIM2->CR2 |= TIM_CR2_MMS_1;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/*void ADC1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= (3<<8)|(3<<10)|(3<<12);

    ADC1->CR1 |= ADC_CR1_SCAN | ADC_CR1_EOCIE;
    ADC1->CR2 |= ADC_CR2_EXTEN_0 | (6<<24);
    ADC1->SMPR2 = 0;
    ADC1->SQR3 = 4;
    ADC1->CR2 |= ADC_CR2_ADON;

    NVIC_EnableIRQ(ADC_IRQn);
}
*/
void ADC1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA4 PA5 PA6 analog
    GPIOA->MODER |= (3<<8)|(3<<10)|(3<<12);

    ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_EOCIE;

    ADC1->CR2 = ADC_CR2_EXTEN_0;     // Trigger rising edge
    ADC1->CR2 |= (6 << 24);          // TIM2 TRGO

    ADC1->SQR1 |= (2 << 20);         // 3 conversions
    ADC1->SQR3 = (4 << 0) | (5 << 5) | (6 << 10);

    ADC1->CR2 |= ADC_CR2_ADON;

    NVIC_EnableIRQ(ADC_IRQn);

    ADC1->CR2 |= ADC_CR2_SWSTART;
}


void ADC_IRQHandler(void)
{
    if(ADC1->SR & ADC_SR_EOC){
        ADC_VALUE[adc_index++] = ADC1->DR;
        current_channel++;
        if(current_channel>6) current_channel=4;
        ADC1->SQR3 = current_channel;
        if(adc_index>=3) adc_index=0;
    }
}

/* =========================================================
   USART IRQ
========================================================= */
void USART2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    char c = USART2->DR;
    RX_Buffer[rx_index++] = c;

    if(c=='\n'){
        RX_Buffer[rx_index]=0;
        xQueueSendFromISR(QUEUE_REC_CMD,RX_Buffer,&xHigherPriorityTaskWoken);
        rx_index=0;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* =========================================================
   TASKS
========================================================= */
void vTask_ADC_CH1(void *p){uint16_t v;while(1){v=ADC_VALUE[0];xQueueSend(ADC_Queue,&v,0);vTaskDelay(1000);}}
void vTask_ADC_CH2(void *p){uint16_t v;while(1){v=ADC_VALUE[1];xQueueSend(ADC_Queue,&v,0);vTaskDelay(1000);}}
void vTask_ADC_CH3(void *p){uint16_t v;while(1){v=ADC_VALUE[2];xQueueSend(ADC_Queue,&v,0);vTaskDelay(1000);}}


void vTask_Receive_Data(void *p)
{
    uint16_t buf[4], val;
    uint8_t i = 0;
    float temp;

    while(1)
    {
        if(xQueueReceive(ADC_Queue, &val, portMAX_DELAY))
        {
            buf[i++] = val;

            if(i == 3)
            {
                temp = I2C_ReadTemperature();   // ? VRAIE LECTURE
                buf[3] = (uint16_t)(temp * 10); // ex: 25.5 ? 255

                xQueueSend(Buffer_Queue, buf, portMAX_DELAY);
                i = 0;
            }
        }
    }
}

/*


*/
void vTask_Send_To_Smartphone(void *arg)
{
    uint16_t b[4];
    char msg[80];

    while(1)
    {
        if(xQueueReceive(Buffer_Queue, b, portMAX_DELAY))
        {
            sprintf(msg,
                "ADC:%u %u %u TEMP:%u.%u C\r\n",
                b[0], b[1], b[2],
                b[3] / 10, b[3] % 10
            );

            USART2_SendString(msg);
        }
    }
}

//////////////////////////////////////




/*
void TASK_RECEIVE_COMMAND(void *p)
{
    char cmd[RX_BUFFER_SIZE];
    while(1){
        xQueueReceive(QUEUE_REC_CMD,cmd,portMAX_DELAY);
        if(!strcmp(cmd,"STOP")) xSemaphoreGive(SEM_STOP);
        else if(!strcmp(cmd,"AVANCE")) Motor_Set(CMD_FORWARD,800);
        else if(!strcmp(cmd,"RECULE")) Motor_Set(CMD_BACKWARD,800);
        else if(!strcmp(cmd,"GAUCHE")) Motor_Set(CMD_LEFT,800);
        else if(!strcmp(cmd,"DROITE")) Motor_Set(CMD_RIGHT,800);
    }
}
*/

void TASK_RECEIVE_COMMAND(void *arg)
{
    char cmd[RX_BUFFER_SIZE];

    while(1)
    {
        xQueueReceive(QUEUE_REC_CMD, cmd, portMAX_DELAY);

        // ?? LIGNE À AJOUTER (TRÈS IMPORTANTE)
        cmd[strcspn(cmd, "\r\n")] = 0;

        if(strcmp(cmd,"STOP") == 0)
        {
            xSemaphoreGive(SEM_STOP);
        }
        else if(strcmp(cmd,"AVANCE") == 0)
        {
            Motor_Set(CMD_FORWARD, 500);
        }
        else if(strcmp(cmd,"RECULE") == 0)
        {
            Motor_Set(CMD_BACKWARD, 500);
        }
        else if(strcmp(cmd,"GAUCHE") == 0)
        {
            Motor_Set(CMD_LEFT, 500);
        }
        else if(strcmp(cmd,"DROITE") == 0)
        {
            Motor_Set(CMD_RIGHT, 500);
        }
    }
}
void TASK_STOP_ROBOT(void *p)
{
    while(1){
        xSemaphoreTake(SEM_STOP,portMAX_DELAY);
        Motor_Set(CMD_STOP,0);
    }
}

/* =========================================================
   MAIN
========================================================= */
int main(void)
{
    SystemClock_Config();
    USART2_Init(); //command 
    Motor_PWM_Init();
    Motor_DIR_Init();
    TIM2_Init();
    ADC1_Init();
    config_I2C1();      // Configuration I2C
    DS1621_Init();      // Initialisation DS1621
	
	
    ADC_Queue = xQueueCreate(3,sizeof(uint16_t));
    Buffer_Queue = xQueueCreate(1,sizeof(uint16_t)*4);
    QUEUE_REC_CMD = xQueueCreate(5,RX_BUFFER_SIZE);
    vSemaphoreCreateBinary(SEM_STOP);
    xSemaphoreTake(SEM_STOP,0);

    xTaskCreate(vTask_ADC_CH1,"ADC1",128,NULL,1,NULL);
    xTaskCreate(vTask_ADC_CH2,"ADC2",128,NULL,1,NULL);
    xTaskCreate(vTask_ADC_CH3,"ADC3",128,NULL,1,NULL);
    xTaskCreate(vTask_Receive_Data,"RX_ADC",256,NULL,2,NULL);
    xTaskCreate(vTask_Send_To_Smartphone,"TX",256,NULL,2,NULL);
    xTaskCreate(TASK_RECEIVE_COMMAND,"CMD",256,NULL,3,NULL);
    xTaskCreate(TASK_STOP_ROBOT,"STOP",128,NULL,4,NULL);

    vTaskStartScheduler();
    while(1);
}
