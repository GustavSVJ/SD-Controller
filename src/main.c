#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"
#include "flash.h"
#include <stdio.h>
#include <string.h>

#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))

volatile uint8_t MeasureFlag;
volatile uint8_t JoystickLeftFlag;
volatile uint8_t JoystickRightFlag;

void ADC_setup_PA(){
    RCC->AHBENR |= RCC_AHBPeriph_GPIOA;

    //PA0
    GPIOA->MODER &= ~(0x00000003 << (0 * 2)); // Clear mode register
    GPIOA->MODER |= (0x00000000 << (0 * 2)); // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOA->PUPDR &= ~(0x00000003 << (0 * 2)); // Clear push/pull register
    GPIOA->PUPDR |= (0x00000000 << (0 * 2)); // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    //PA1
    GPIOA->MODER &= ~(0x00000003 << (1 * 2)); // Clear mode register
    GPIOA->MODER |= (0x00000000 << (1 * 2)); // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOA->PUPDR &= ~(0x00000003 << (1 * 2)); // Clear push/pull register
    GPIOA->PUPDR |= (0x00000000 << (1 * 2)); // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    RCC->CFGR2 &= ~RCC_CFGR2_ADCPRE12; // Clear ADC12 prescaler bits
    RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV8; // Set ADC12 prescaler to 8
    RCC->AHBENR |= RCC_AHBPeriph_ADC12; // Enable clock for ADC12

    ADC1->CR = 0x00000000; // Clear CR register
    ADC1_2->CCR |= ADC12_CCR_VREFEN;
    ADC1->CFGR &= 0xFDFFC007; // Clear ADC1 config register

    ADC1->SQR1 &= ~ADC_SQR1_L; // Clear regular sequence register 1

    //Verify ADVREG state (ADC voltage regulator)
    //1. Set ADVREGEN from '10' (disabled state) to '00' and then to '01' (enabled)
    //2. Wait 10uS (worst case) before performing calibration and/or
    ADC1->CR &= ~ADC_CR_ADVREGEN;
    ADC1->CR |= ADC_CR_ADVREGEN_0;
    //Wait for at least 10uS before continuing...
    for(uint32_t i = 0; i < 10000; i++);

    ADC1->CR &= ~ADC_CR_ADEN; //Make sure ADC is disabled
    ADC1->CR &= ~ADC_CR_ADCALDIF; //Use single ended calibration
    ADC1->CR |= ADC_CR_ADCAL; //Start calibration
    while((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL){} //Wait for calibration to finish
    //Wait > 4 ADC clock cycles after ADCAL bit is cleared
    for(uint32_t i = 0; i < 100; i++);

    ADC1->CR |= ADC_CR_ADEN;
    //wait for ADC1 to be ready to start conversion
    while (!ADC1->ISR & ADC_ISR_ADRD){}

}

void JoystickInit(){
     // Set pin PC0 to input
     GPIOC->MODER &= ~(0x00000003 << (0 * 2)); // Clear mode register
     GPIOC->MODER |= (0x00000000 << (0 * 2)); // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
     GPIOC->PUPDR &= ~(0x00000003 << (0 * 2)); // Clear push/pull register
     GPIOC->PUPDR |= (0x00000000 << (0 * 2)); // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

     // Set pin PC1 to input
     GPIOC->MODER &= ~(0x00000003 << (1 * 2)); // Clear mode register
     GPIOC->MODER |= (0x00000000 << (1 * 2)); // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
     GPIOC->PUPDR &= ~(0x00000003 << (1 * 2)); // Clear push/pull register
     GPIOC->PUPDR |= (0x00000000 << (1 * 2)); // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

     //Enable SYSCFG clock to enable interrupts
     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

     //Enable interrupt on PB5
     SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0 | SYSCFG_EXTICR1_EXTI1);
     SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PC | SYSCFG_EXTICR1_EXTI1_PC);

     //Select interrupt trigger
     EXTI->FTSR |= (EXTI_FTSR_TR0 | EXTI_FTSR_TR1);
     EXTI->RTSR &= ~(EXTI_RTSR_TR0 | EXTI_RTSR_TR1);

     //Unmask interrupt
     EXTI->IMR |= (EXTI_IMR_MR0 | EXTI_IMR_MR1);

     //Set interrupt priority
     NVIC_SetPriority(EXTI0_IRQn, 1);
     NVIC_EnableIRQ(EXTI0_IRQn);

     NVIC_SetPriority(EXTI1_IRQn, 1);
     NVIC_EnableIRQ(EXTI1_IRQn);
}


uint16_t ADC_measure_PA(uint8_t ch){

    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_1Cycles5);

    ADC_StartConversion(ADC1); // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read

    return ADC_GetConversionValue(ADC1);
}

float ADC_Stepsize_Calc(){
    ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_19Cycles5);

    ADC_StartConversion(ADC1); // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read

    return (3.3 * (VREFINT_CAL/ADC_GetConversionValue(ADC1))) / 4096;
}

float ADC_Manual_Calibration(uint8_t ch, float stepSize){
    uint16_t meas = 0;

    for (uint8_t i = 0; i < 16; i++){
        ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_1Cycles5);

        ADC_StartConversion(ADC1); // Start ADC read
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read

        meas += ADC_GetConversionValue(ADC1);
    }

    float avgMeas = meas / 16;
    float absMeas = stepSize * avgMeas;

    float correction = 3.2 / absMeas;

    return correction;
}

void ADC_SaveCor(uint8_t ch, float CorValue){
    float flashValues[512];

    for (uint16_t i = 0; i < 512 ; i++){
        read_float_flash(PG31_BASE, i);
    }

    flashValues[ch-1] = CorValue;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    FLASH_ErasePage(PG31_BASE);
    for (uint16_t i = 0; i < 512 ; i++){
        write_float_flash(PG31_BASE, i, flashValues[i]);
    }

    FLASH_Lock();
}

float ADC_LoadCor(uint8_t ch){
    return read_float_flash(PG31_BASE, ch-1);
}

void initTimer(){

    //RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR |= RCC_APB1Periph_TIM2;
    TIM2->CR1 &= ~(0xffff); //Disable the counter
    TIM2->ARR = 39999;  //Top value?
    TIM2->PSC = 159; //Prescaler value

    TIM2->CR1 |= 0x0001; //Start counting

    TIM2->DIER |= TIM_IT_Update; //enable interrupts

    NVIC_SetPriority(TIM2_IRQn, 0); // Set interrupt priority (The higher the number the lower the priority)
    NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) {
    MeasureFlag = 1;
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
 }

void EXTI0_IRQHandler(void){
    JoystickRightFlag = 1;
    EXTI->PR |= EXTI_PR_PR0;
}

void EXTI1_IRQHandler(){
    JoystickLeftFlag = 1;
    EXTI->PR |= EXTI_PR_PR1;
}

int main(void){


    init_spi_lcd();
    lcd_reset();
    ADC_setup_PA();
    initTimer();
    JoystickInit();


    float ADC_Ch1Cor = ADC_LoadCor(1);
    if (!(0.5 < ADC_Ch1Cor && ADC_Ch1Cor < 1.5)){
        ADC_Ch1Cor = 1;
    }

    float ADC_Ch2Cor = ADC_LoadCor(2);
    if (!(0.5 < ADC_Ch2Cor && ADC_Ch2Cor < 1.5)){
        ADC_Ch2Cor = 1;
    }

    float ADCStep = ADC_Stepsize_Calc();

    uint8_t sbuffer[512], obuffer[512];

    memset(sbuffer, 0x00, 512);
    memset(obuffer, 0x00, 512);


    while(1){



        if (MeasureFlag == 1){
            MeasureFlag = 0;

            memset(obuffer, 0x00, 512);

            memset(sbuffer, 0x00, 512);

            uint16_t raw = ADC_measure_PA(1);
            float abs = ADCStep * raw * ADC_Ch1Cor;

            sprintf(sbuffer, "Channel 1: %.2f(%i)", abs, raw);
            lcd_write_string(sbuffer, obuffer, 0, 0);

            memset(sbuffer, 0x00, 512);

            raw = ADC_measure_PA(2);
            abs = ADCStep * raw * ADC_Ch2Cor;

            sprintf(sbuffer, "Channel 2: %.2f(%i)", abs, raw);
            lcd_write_string(sbuffer, obuffer, 0, 1);

            lcd_push_buffer(obuffer);

        }

        if (JoystickLeftFlag == 1){


            ADCStep = ADC_Stepsize_Calc();
            ADC_Ch1Cor = ADC_Manual_Calibration(1, ADCStep);

            ADC_SaveCor(1, ADC_Ch1Cor);

            JoystickLeftFlag = 0;
        }
        if (JoystickRightFlag == 1){

            ADCStep = ADC_Stepsize_Calc();
            ADC_Ch2Cor = ADC_Manual_Calibration(2, ADCStep);

            ADC_SaveCor(2, ADC_Ch2Cor);

            JoystickRightFlag = 0;
        }




    }

    return 0;
}
