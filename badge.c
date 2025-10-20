#include "stm32f10x.h"

/******************************************************************************/
/* 1. DEFINIÇÕES E VARIÁVEIS GLOBAIS                                          */
/******************************************************************************/

//--- Defines para os tempos dos "motifs" em microssegundos (µs) ---
#define RESET_PULSE_DURATION      480
#define RESET_READ_TIME           570
#define RESET_MOTIF_DURATION      960

#define READ_PULSE_DURATION       5
#define READ_READ_TIME            10
#define READ_MOTIF_DURATION       70

#define WRITE_0_PULSE_DURATION    60
#define WRITE_1_PULSE_DURATION    10
#define WRITE_MOTIF_DURATION      70 // Tempo de fim para os dois estados da escrita

// Para escrita, o tempo de leitura é inútil. Colocamos um valor > D3 para ser ignorado.
#define WRITE_READ_TIME_DUMMY     80 


//--- Defines para os bits dos registradores ---
#define RCC_APB2ENR_AFIOEN      (1U << 0)
#define RCC_APB2ENR_IOPBEN      (1U << 3)
#define RCC_APB1ENR_TIM3EN      (1U << 1)

#define GPIO_MODE_OUTPUT_50MHZ  (0x3)
#define GPIO_CNF_GPOUT_PP       (0x0)
#define GPIO_CNF_AFOUT_PP       (0x2)
#define GPIO_CNF_AFOUT_OD       (0x3)

#define ONEWIRE_PIN_POS         (4) // PB4
#define ONEWIRE_PIN_MASK        (1U << ONEWIRE_PIN_POS)

#define TIM_CR1_CEN             (1U << 0)
#define TIM_DIER_CC2IE          (1U << 2)
#define TIM_DIER_CC3IE          (1U << 3)
#define TIM_CCER_CC1E           (1U << 0)
#define TIM_CCER_CC2E           (1U << 4)
#define TIM_SR_CC2IF            (1U << 2)
#define TIM_SR_CC3IF            (1U << 3)
#define TIM_OCM_PWM1            (0x6)

//--- Variáveis Globais de Estado ---
// 'volatile' é ESSENCIAL aqui. Ele informa ao compilador que essas variáveis
// podem mudar a qualquer momento (por uma interrupção), então ele não deve
// otimizar a leitura delas.
volatile uint8_t motif_one_wire_fini = 0;
volatile uint8_t etat_one_wire = 0;


/******************************************************************************/
/* 2. FUNÇÕES DE HARDWARE (Seções 2.1 e 2.2)                                  */
/******************************************************************************/

void init_pin_onewire_manual_definido(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    
    GPIOB->CRL &= ~((0xF << (0*4)) | (0xF << (4*4)) | (0xF << (5*4))); // Limpa PB0, PB4, PB5
    
    uint32_t config_pb0 = (GPIO_CNF_GPOUT_PP << 2) | GPIO_MODE_OUTPUT_50MHZ;
    uint32_t config_pb4 = (GPIO_CNF_AFOUT_OD << 2) | GPIO_MODE_OUTPUT_50MHZ;
    uint32_t config_pb5 = (GPIO_CNF_AFOUT_PP << 2) | GPIO_MODE_OUTPUT_50MHZ;
    
    GPIOB->CRL |= (config_pb0 << (0*4)) | (config_pb4 << (4*4)) | (config_pb5 << (5*4));
}

void init_timer_manual_definido(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    TIM3->PSC = 71; // Assume clock de 72MHz para ter 1 tick = 1µs
    TIM3->ARR = 0xFFFF;

    TIM3->CCMR1 = 0;
    TIM3->CCMR1 |= (TIM_OCM_PWM1 << 4);  // CH1
    TIM3->CCMR1 |= (TIM_OCM_PWM1 << 12); // CH2
    
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->CR1 &= ~TIM_CR1_CEN;
}

void init_motif_manual_definido(uint16_t D1, uint16_t D2, uint16_t D3) {
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->CNT = 0;
    TIM3->SR = 0;

    TIM3->CCR1 = D1;
    TIM3->CCR2 = D2;
    TIM3->CCR3 = D3;
    
    TIM3->DIER |= TIM_DIER_CC2IE | TIM_DIER_CC3IE;
    
    TIM3->CR1 |= TIM_CR1_CEN;
}

/******************************************************************************/
/* 3. ROTINA DE INTERRUPÇÃO (Seção 2.3)                                       */
/******************************************************************************/

/**
 * @brief  Handler de interrupção para o Timer 3.
 * @note   Esta função é chamada automaticamente pelo hardware quando o TIM3
 * gera uma interrupção (nos instantes D2 e D3). O nome "TIM3_IRQHandler"
 * é Padrão e não deve ser mudado.
 */
void TIM3_IRQHandler(void) {
    // --- Verifica se a interrupção foi causada pelo Canal 2 (TIMx_CH2) ---
    // Este é o nosso "instant de lecture" (instante de leitura)
    if ((TIM3->SR & TIM_SR_CC2IF) != 0) {
        // Lê o estado do pino OneWire (PB4)
        // GPIOB->IDR é o registrador de Input Data. Verificamos se o bit 4 está em 1.
        if ((GPIOB->IDR & ONEWIRE_PIN_MASK) != 0) {
            etat_one_wire = 1; // Linha está em alta
        } else {
            etat_one_wire = 0; // Linha está em baixa
        }
        
        // LIMPA a flag de interrupção do Canal 2.
        // **MUITO IMPORTANTE**: Se não limpar a flag, o programa
        // ficará preso em um loop infinito nesta interrupção.
        TIM3->SR &= ~TIM_SR_CC2IF;
    }

    // --- Verifica se a interrupção foi causada pelo Canal 3 (TIMx_CH3) ---
    // Este é o nosso sinal de "fin de motif" (fim do padrão)
    if ((TIM3->SR & TIM_SR_CC3IF) != 0) {
        
        TIM3->CR1 &= ~TIM_CR1_CEN; // Para o timer
        
        // Levanta o "semáforo" para avisar o programa principal que o motif terminou
        motif_one_wire_fini = 1;
        
        // LIMPA a flag de interrupção do Canal 3
        TIM3->SR &= ~TIM_SR_CC3IF;
    }
}


/******************************************************************************/
/* 4. FUNÇÕES DE ALTO NÍVEL E TESTES (Seção 2.4)                              */
/******************************************************************************/

/**
 * @brief Gera um pulso de reset e detecta a presença de um dispositivo.
 * @return 1 se um dispositivo está presente, 0 caso contrário.
 */
uint8_t RESET_ONEWIRE(void) {
    motif_one_wire_fini = 0;
    init_motif_manual_definido(RESET_PULSE_DURATION, RESET_READ_TIME, RESET_MOTIF_DURATION);
    
    // Espera bloqueante: Fica neste loop até a interrupção (CH3)
    // sinalizar que o motif terminou.
    while (motif_one_wire_fini == 0) {}
    
    // A interrupção (CH2) já salvou o estado da linha em 'etat_one_wire'.
    // Se a linha estava em BAIXO (0) no instante da leitura, significa
    // que um dispositivo respondeu com um "pulso de presença".
    if (etat_one_wire == 0) {
        return 1; // Dispositivo presente
    } else {
        return 0; // Nenhum dispositivo
    }
}

/**
 * @brief Envia um único bit (0 ou 1) pelo barramento OneWire.
 * @param bit_a_envoyer: O bit a ser enviado (0 ou 1).
 */
void ENVOI_BIT_ONEWIRE(uint8_t bit_a_envoyer) {
    motif_one_wire_fini = 0;
    
    if (bit_a_envoyer == 1) {
        init_motif_manual_definido(WRITE_1_PULSE_DURATION, WRITE_READ_TIME_DUMMY, WRITE_MOTIF_DURATION);
    } else {
        init_motif_manual_definido(WRITE_0_PULSE_DURATION, WRITE_READ_TIME_DUMMY, WRITE_MOTIF_DURATION);
    }
    
    while (motif_one_wire_fini == 0) {}
}

/**
 * @brief Lê um único bit do barramento OneWire.
 * @return O valor do bit lido (0 ou 1).
 */
uint8_t LECTURE_BIT_ONEWIRE(void) {
    motif_one_wire_fini = 0;
    init_motif_manual_definido(READ_PULSE_DURATION, READ_READ_TIME, READ_MOTIF_DURATION);
    
    while (motif_one_wire_fini == 0) {}
    
    return etat_one_wire;
}

//--- Funções Bônus (baseadas no fim da seção 2.4) ---
void ENVOI_OCTET_ONEWIRE(uint8_t octet) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        ENVOI_BIT_ONEWIRE(octet & 0x01);
        octet >>= 1; // Desloca para o próximo bit
    }
}

uint8_t LECTURE_OCTET_ONEWIRE(void) {
    uint8_t octet = 0;
    uint8_t i;
    for (i = 0; i < 8; i++) {
        octet >>= 1; // Abre espaço para o novo bit
        if (LECTURE_BIT_ONEWIRE() == 1) {
            octet |= 0x80; // Coloca o bit 1 na posição mais significativa
        }
    }
    return octet;
}

// Adicione esta função junto com as outras de inicialização
void init_led_placa(void) {
    // O clock do GPIOA já deve estar habilitado pela outra função,
    // mas é uma boa prática garantir.
    RCC->APB2ENR |= (1U << 2); // Habilita clock do GPIOA (IOPAEN)

    // Limpa a configuração atual do pino PA5
    GPIOA->CRL &= ~((0xF << (5*4)));
    
    // Configura PA5 como Saída Push-Pull, 50MHz
    uint32_t config_pa5 = (GPIO_CNF_GPOUT_PP << 2) | GPIO_MODE_OUTPUT_50MHZ;
    GPIOA->CRL |= (config_pa5 << (5*4));
}


int main(void) {
    // 1. Inicializa o hardware
    init_pin_onewire_manual_definido();
    init_timer_manual_definido();
		init_led_placa();
	
    // 2. Habilita a interrupção do TIM3 no controlador de interrupções (NVIC)
    // **PASSO CRUCIAL**: Sem isso, a função TIM3_IRQHandler nunca será chamada.
    NVIC_EnableIRQ(TIM3_IRQn);
    
    volatile uint32_t i; // Para um pequeno delay
    
    while (1) {
        // --- Teste do Reset OneWire ---
        // Tente com e sem o sensor DS18B20 conectado para ver a diferença.
        uint8_t presenca = RESET_ONEWIRE();
        if (presenca) {
					
						// 1. Acende a LED para feedback visual
            GPIOA->BSRR = (1U << 5);
					
            // Se houver um dispositivo, podemos tentar ler sua ROM
            ENVOI_OCTET_ONEWIRE(0x33); // Comando "Read ROM"
            
            // Lê os 8 bytes do endereço de 64 bits
            uint8_t rom_code[8];
            for (int j = 0; j < 8; j++) {
                rom_code[j] = LECTURE_OCTET_ONEWIRE();
            }
            // Neste ponto, 'rom_code' contém o endereço único do seu DS18B20.
            // Você pode colocar um breakpoint aqui e ver o valor no debugger.
        }else {
            // --- Ação quando o badge ESTÁ AUSENTE ---

            // 1. Apaga a LED
            GPIOA->BSRR = (1U << (5 + 16));
        }
        
        // --- Testes individuais de escrita e leitura podem ser feitos aqui ---
        // Ex: ENVOI_BIT_ONEWIRE(1);
        // Ex: ENVOI_BIT_ONEWIRE(0);
        // Ex: uint8_t bit_lido = LECTURE_BIT_ONEWIRE();
        
        // Delay simples para não sobrecarregar o barramento
        for (i = 0; i < 500000; i++) {}
    }
}
