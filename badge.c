#include "stm32f10x.h"

/**
 * @brief  Fun��o de inicializa��o dos pinos para o OneWire e sinais de depura��o.
 * @note   Configura os pinos PB4, PB5 e PB0 para suas fun��es espec�ficas no projeto.
 * - PB4: TIM3_CH1 (Sinal OneWire) -> Sa�da Alternate Function Open-Drain
 * - PB5: TIM3_CH2 (Sinal Espi�o)  -> Sa�da Alternate Function Push-Pull
 * - PB0: Associado ao TIM3_CH3   -> Sa�da Push-Pull para depura��o manual
 */
void init_pin_onewire(void) {

    GPIO_InitTypeDef GPIO_InitStruct; 

    // Habilitar os clocks necess�rios
    // GPIOs no STM32F1 ficam no barramento APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
    // TIM3 fica no barramento APB1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // � uma boa pr�tica habilitar o clock do AFIO ao usar fun��es alternadas
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Configurar o pino PB4 como sa�da do TIM3_CH1 (SIGNAL ONEWIRE) 
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;      // AF_OD = Alternate Function Open-Drain
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    // Velocidade m�xima
    GPIO_Init(GPIOB, &GPIO_InitStruct);               // A fun��o de inicializa��o tamb�m muda

    // Configurar o pino PB5 como sa�da do TIM3_CH2 (SIGNAL ESPION)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;      // AF_PP = Alternate Function Push-Pull
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);


}