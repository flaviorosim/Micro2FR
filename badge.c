#include "stm32f10x.h"

/**

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

void init_pin_onewire() {
    
    // --- Habilitar Clocks ---
    // RCC_APB2ENR: [ IOPA | IOPB | IOPC | ... | AFIOEN ]
    // Bit 3 (IOPBEN): Habilita clock do GPIOB
    // Bit 0 (AFIOEN): Habilita clock das Funções Alternadas
    RCC->APB2ENR |= (1 << 3) | (1 << 0);

    // --- Configurar Pinos no GPIOB->CRL ---
    // O registrador CRL controla os pinos 0 a 7.
    // Vamos zerar os bits dos pinos 0, 4 e 5 antes de configurá-los.
    // Isso é feito com uma máscara AND para limpar apenas os bits que queremos mudar.
    GPIOB->CRL &= ~(0x00F0FFF0); // Limpa os bits de configuração dos pinos 0, 4 e 5

    // Agora, configuramos os bits usando uma operação OR.
    // PB0: Output Push-Pull 50MHz (0011b)
    // PB4: Alternate Function Open-Drain 50MHz (1111b)
    // PB5: Alternate Function Push-Pull 50MHz (1011b)
    GPIOB->CRL |= (0x00B0B003); // [PB5: 1011(B)] [PB4: 1111(F)] [PB0: 0011(3)] - A ordem no registrador é diferente!
    
    // Vamos fazer de uma forma mais legível para evitar erros:
    GPIOB->CRL &= ~((0xF << 20) | (0xF << 16) | (0xF << 0)); // Limpa bits de PB5, PB4, PB0
    
    // Configuração para PB0 (bits 3-0): Saída Push-Pull, 50MHz
    GPIOB->CRL |= (0x3 << 0); // MODE=11 (50MHz), CNF=00 (GP Push-Pull) -> 0011b = 0x3
    
    // Configuração para PB4 (bits 19-16): AF Open-Drain, 50MHz
    GPIOB->CRL |= (0xF << 16); // MODE=11 (50MHz), CNF=11 (AF Open-Drain) -> 1111b = 0xF
    
    // Configuração para PB5 (bits 23-20): AF Push-Pull, 50MHz
    GPIOB->CRL |= (0xB << 20); // MODE=11 (50MHz), CNF=10 (AF Push-Pull) -> 1011b = 0xB
}

void init_timer() {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // Assumindo clock do APB1 para Timers é 72MHz.
    // Prescaler para ter uma frequência de contagem de 1MHz (1 tick = 1µs).
    // Prescaler = (72,000,000 / 1,000,000) - 1 = 71
    uint16_t prescalerValue = 71;

    // 1. Configuração da Base de Tempo do Timer 3
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // Período máximo, o controle será pelos canais
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 2. Configuração dos Canais em modo Output Compare
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active; // Modo inicial, será ajustado no init_motif
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // O pulso será definido no init_motif
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    // Configura CH1, CH2, e CH3 com a mesma base
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    
    // Desabilita o timer por enquanto. Ele será ligado pelo init_motif.
    TIM_Cmd(TIM3, DISABLE);
}


// Esta função é chamada toda vez que um "motif" precisa ser gerado.
void init_motif(uint16_t D1, uint16_t D2, uint16_t D3) {
    
    // Garante que o timer esteja parado antes de reconfigurar
    TIM3->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
    
    // Zera o contador
    TIM3->CNT = 0;

    // Carrega os novos valores de duração nos registradores de Capture/Compare
    TIM3->CCR1 = D1; // Duração do pulso baixo do OneWire
    TIM3->CCR2 = D2; // Instante da leitura / pulso espião
    TIM3->CCR3 = D3; // Fim do motif
    
    // Limpa as flags de interrupção pendentes
    TIM3->SR = 0;
    
    // Habilita as interrupções para o canal 2 (leitura) e canal 3 (fim do motif) 
    TIM3->DIER |= TIM_DIER_CC2IE | TIM_DIER_CC3IE;
    
    // Configura o modo de Output Compare: Força nível baixo na partida e fica ativo até o compare.
    // TIM_OCMode_Inactive: Força um nível (alto, pois polaridade é High).
    // TIM_OCMode_Active:   Força o nível oposto (baixo).
    // Queremos que o pino fique BAIXO no início e suba em D1. O modo "PWM2" faz isso:
    // Inativo (Alto) enquanto CNT < CCRx, Ativo (Baixo) quando CNT >= CCRx.
    // Para obter o efeito contrário (Baixo no início, Alto em D1), usamos "PWM1".
    TIM_SelectOCxM(TIM3, TIM_Channel_1, TIM_OCMode_PWM1);
    TIM_SelectOCxM(TIM3, TIM_Channel_2, TIM_OCMode_PWM1); // Para o pino espião
    
    // Habilita a saída dos canais
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    // Liga o contador para iniciar a geração do "motif" 
    TIM3->CR1 |= TIM_CR1_CEN;
}
