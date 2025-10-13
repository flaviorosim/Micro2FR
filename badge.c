#include "stm32f10x.h"

/**
 * @brief  Função de inicialização dos pinos para o OneWire e sinais de depuração.
 * @note   Configura os pinos PB4, PB5 e PB0 para suas funções específicas no projeto.
 * - PB4: TIM3_CH1 (Sinal OneWire) -> Saída Alternate Function Open-Drain
 * - PB5: TIM3_CH2 (Sinal Espião)  -> Saída Alternate Function Push-Pull
 * - PB0: Associado ao TIM3_CH3   -> Saída Push-Pull para depuração manual
 */
void init_pin_onewire(void) {

    GPIO_InitTypeDef GPIO_InitStruct; 

    // Habilitar os clocks necessários
    // GPIOs no STM32F1 ficam no barramento APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
    // TIM3 fica no barramento APB1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // É uma boa prática habilitar o clock do AFIO ao usar funções alternadas
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Configurar o pino PB4 como saída do TIM3_CH1 (SIGNAL ONEWIRE) 
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;      // AF_OD = Alternate Function Open-Drain
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    // Velocidade máxima
    GPIO_Init(GPIOB, &GPIO_InitStruct);               // A função de inicialização também muda

    // Configurar o pino PB5 como saída do TIM3_CH2 (SIGNAL ESPION)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;      // AF_PP = Alternate Function Push-Pull
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);


}