#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>
#include "ADXL345.h"

// Bits 0-1: Modo Principal
#define PIN_MODE_INPUT          (1U << 0) // 0b0001
#define PIN_MODE_OUTPUT_PP      (1U << 1) // 0b0010
#define PIN_MODE_OUTPUT_OD      (1U << 2) // 0b0100
// Bits 4-5: Estado Secundário
#define PIN_STATE_LOW_PD        (1U << 4) // 0b00010000 (Low ou Pull-Down)
#define PIN_STATE_HIGH_PU       (1U << 5) // 0b00100000 (High ou Pull-Up)


// Estes são os defines que você realmente usará ao chamar a função.
#define MODE_INPUT_PULLDOWN     (PIN_MODE_INPUT | PIN_STATE_LOW_PD)
#define MODE_INPUT_PULLUP       (PIN_MODE_INPUT | PIN_STATE_HIGH_PU)
#define MODE_OUTPUT_LOW         (PIN_MODE_OUTPUT_PP | PIN_STATE_LOW_PD)
#define MODE_OUTPUT_HIGH        (PIN_MODE_OUTPUT_PP | PIN_STATE_HIGH_PU)
#define MODE_OUTPUT_OD_LOW      (PIN_MODE_OUTPUT_OD | PIN_STATE_LOW_PD)

#define I2C_PORT            GPIOB
#define I2C_SCL_PIN         GPIO_Pin_6  // PB6
#define I2C_SDA_PIN         GPIO_Pin_7  // PB7
#define I2C_RCC_PORT        RCC_APB2Periph_GPIOB // Clock do barramento da porta

// Endereços (7 bits) dos 6 componentes PCF8574
// --- A FAZER: Defina os endereços corretos de acordo com a pinagem A0-A2 da sua placa ---
#define ADDR_LEDS_V_HI      0x20  // PCF para Leds Verdes 15-8 (Byte alto)
#define ADDR_LEDS_V_LO      0x21  // PCF para Leds Verdes 7-0 (Byte baixo)
#define ADDR_LEDS_R_HI      0x23  // PCF para Leds Vermelhos 15-8
#define ADDR_LEDS_R_LO      0x24  // PCF para Leds Vermelhos 7-0
#define ADDR_SENS_HI        0x25  // PCF para Sensores 15-8
#define ADDR_SENS_LO        0x26  // PCF para Sensores 7-0

// Macros para facilitar a manipulação dos pinos
#define SCL_H()             GPIO_SetBits(I2C_PORT, I2C_SCL_PIN)
#define SCL_L()             GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN)
#define SDA_H()             GPIO_SetBits(I2C_PORT, I2C_SDA_PIN)
#define SDA_L()             GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN)
#define SDA_READ()          GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN)

#define ZIF_PORT_LO         GPIOC // Pinos 1-8 do ZIF (PC0-PC7)
#define ZIF_PORT_HI         GPIOB // Pinos 9-16 do ZIF (PB8-PB15)
#define ZIF_RCC_PORT_LO     RCC_APB2Periph_GPIOC
#define ZIF_RCC_PORT_HI     RCC_APB2Periph_GPIOB

// --- Pinos SPI e Controle ADXL345  ---
#define ADXL_SPI            SPI1
#define ADXL_SPI_RCC        RCC_APB2Periph_SPI1
#define ADXL_GPIO_RCC       RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC // Habilita GPIOA e GPIOC

#define ADXL_SCK_PORT       GPIOA
#define ADXL_SCK_PIN        GPIO_Pin_5
#define ADXL_MISO_PORT      GPIOA
#define ADXL_MISO_PIN       GPIO_Pin_6
#define ADXL_MOSI_PORT      GPIOA
#define ADXL_MOSI_PIN       GPIO_Pin_7
#define ADXL_CS_PORT        GPIOA
#define ADXL_CS_PIN         GPIO_Pin_4
#define ADXL_INT_PORT       GPIOC
#define ADXL_INT_PIN        GPIO_Pin_4

// === Variables globales ===
uint8_t MaeRcp = 0;
uint8_t entete = 0;
uint8_t dataIndex = 0;
bool flagMajCasiers = false;
bool flagMajZIF16 = false;

// === Consignes 16 bits ===
uint16_t consigne_leds_vertes = 0;
uint16_t consigne_leds_rouges = 0;
uint16_t consigne_dir = 0;
uint16_t consigne_etat = 0;
uint16_t etat_casiers =0;
uint16_t etat_ZIF16 = 0;

uint8_t fifo_tx[32];
uint8_t pw_tx = 0;
uint8_t pr_tx = 0;
volatile int place_libre_tx = 32;

bool fifo_push(uint8_t byte);
bool fifo_pop(uint8_t* p_byte);

// --- Funções de Envio de Trames ---
void envoyer_trame_casiers(void);
void envoyer_trame_ZIF16(void);
void envoyer_trame_accelero(void);

typedef union access_sensor {//16 bit

        struct {
            uint16_t x; 
            uint16_t y; 
					  uint16_t z; 
        } c;
        unsigned char b[6];
    } struct_xyz_t;

struct_xyz_t capteurs;


#define B7_MASK 0x80
#define B6_MASK 0x40

void Configure_Pin_Generic(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t mode) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    // Decodifica o modo principal
    if (mode & PIN_MODE_INPUT) {
        if (mode & PIN_STATE_HIGH_PU) GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; // Input Pull-Up
        else GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD; // Input Pull-Down
    }
    else if (mode & PIN_MODE_OUTPUT_PP) {
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    }
    else if (mode & PIN_MODE_OUTPUT_OD) {
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    }

    GPIO_Init(GPIOx, &GPIO_InitStruct);

    // Define o nível de saída APÓS a inicialização do modo
    if ((mode & PIN_MODE_OUTPUT_PP) || (mode & PIN_MODE_OUTPUT_OD)) {
        if (mode & PIN_STATE_HIGH_PU) GPIO_SetBits(GPIOx, GPIO_Pin);   // Nível Alto
        else GPIO_ResetBits(GPIOx, GPIO_Pin); // Nível Baixo
    }
}
		
void init_ZIF16(void)
{		uint8_t boucle;
    uint8_t dir_bit;        // Armazena o bit de direção (1=Input, 0=Output) para o pino atual
    uint8_t etat_bit;       // Armazena o bit de estado (1=High/PullUp, 0=Low/PullDown)
    uint8_t mode_final;     // Armazena o modo combinado para a função genérica
    
    // Habilita o clock para as duas portas GPIO que controlam o ZIF16
    RCC_APB2PeriphClockCmd(ZIF_RCC_PORT_LO | ZIF_RCC_PORT_HI, ENABLE);

    // Primeiro loop: para os pinos 1 a 8 do ZIF, que estão em GPIOC (PC0 a PC7)
    for(boucle = 0; boucle < 8; boucle++)
    {
        // 1. Extrai o bit de configuração para o pino 'boucle'
        dir_bit  = (consigne_dir >> boucle) & 1;
        etat_bit = (consigne_etat >> boucle) & 1;
        
        // 2. Combina os dois bits para formar o parâmetro 'mode'
        //    (conforme a lógica do def_mode_gpio.h)
        if (dir_bit == 1 && etat_bit == 1)      mode_final = MODE_INPUT_PULLUP;
        else if (dir_bit == 1 && etat_bit == 0) mode_final = MODE_INPUT_PULLDOWN;
        else if (dir_bit == 0 && etat_bit == 1) mode_final = MODE_OUTPUT_HIGH;
        else /* dir=0, etat=0 */                mode_final = MODE_OUTPUT_LOW;

        // 3. Chama a função genérica para configurar o pino PC[boucle]
        Configure_Pin_Generic(ZIF_PORT_LO, (1 << boucle), mode_final);
    }
    
    // Segundo loop: para os pinos 9 a 16 do ZIF, que estão em GPIOB (PB8 a PB15)
    for(boucle = 8; boucle < 16; boucle++)
    {
        // 1. Extrai o bit de configuração para o pino 'boucle'
        dir_bit  = (consigne_dir >> boucle) & 1;
        etat_bit = (consigne_etat >> boucle) & 1;

        // 2. Combina os dois bits para formar o parâmetro 'mode'
        if (dir_bit == 1 && etat_bit == 1)      mode_final = MODE_INPUT_PULLUP;
        else if (dir_bit == 1 && etat_bit == 0) mode_final = MODE_INPUT_PULLDOWN;
        else if (dir_bit == 0 && etat_bit == 1) mode_final = MODE_OUTPUT_HIGH;
        else /* dir=0, etat=0 */                mode_final = MODE_OUTPUT_LOW;

        // 3. Chama a função genérica para configurar o pino PB[boucle]
        //    Note que o pino 9 corresponde ao bit 8 (1<<8), o pino 10 ao bit 9 (1<<9), etc.
        Configure_Pin_Generic(ZIF_PORT_HI, (1 << boucle), mode_final);
    }
}

uint16_t lire_etat_ZIF16(void)
{
    uint16_t etat_actuel = 0;
    for (int i = 0; i < 16; i++) {
        if (i < 8) {
            if (GPIO_ReadInputDataBit(ZIF_PORT_LO, (1 << i))) {
                etat_actuel |= (1 << i);
            }
        } else {
            if (GPIO_ReadInputDataBit(ZIF_PORT_HI, (1 << i))) {
                etat_actuel |= (1 << i);
            }
        }
    }
    return etat_actuel;
}

void  init_I2C_BITBANGING(void){
    // 1. Ativar o clock para a porta GPIO escolhida
    RCC_APB2PeriphClockCmd(I2C_RCC_PORT, ENABLE);

    // 2. Configurar os pinos SCL e SDA
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD; // Saída Open-Drain, essencial para I2C
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);

    // 3. Garantir que o barramento comece em estado de repouso (Stop)
    SCL_H();
    SDA_H();     

}	

void init_serial2(void)
{
    // 1. Activer les horloges pour GPIOA et USART2
    // O GPIOA está no barramento APB2, enquanto o USART2 está no barramento APB1.
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // Estrutura para configurar os pinos GPIO
    GPIO_InitTypeDef GPIO_InitStruct;

    // 2. Configurer PA2 (TX)
    // O pino de transmissão (TX) deve ser configurado como uma saída de função alternativa.
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-Pull
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. Configurer PA3 (RX)
    // O pino de recepção (RX) deve ser configurado como uma entrada.
    // O modo "Input Floating" é o padrão para RX.
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 4. Configurer USART2 : 9600 bauds, 8N1
    // A biblioteca padrão calcula o valor do registrador BRR automaticamente.
    // Com PCLK1 = 36MHz, para 9600 bauds, o valor escrito em USART2->BRR será 0xEA6.
    // (Cálculo: 36,000,000 / (16 * 9600) = 234.375 => Mantissa=234, Fração=0.375*16=6)
    
    // Estrutura para configurar o periférico USART
    USART_InitTypeDef USART_InitStruct;

    USART_InitStruct.USART_BaudRate = 9600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;        // 8 bits de dados
    USART_InitStruct.USART_StopBits = USART_StopBits_1;             // 1 bit de parada
    USART_InitStruct.USART_Parity = USART_Parity_No;                // Sem paridade
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    // Ativar recepção e transmissão
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Sem controle de fluxo
    
    // Aplica a configuração ao USART2
    USART_Init(USART2, &USART_InitStruct);

    // 5. Ativar o periférico USART2
    USART_Cmd(USART2, ENABLE);
}

uint8_t SPI_transceiver(uint8_t data) {
    // Espera o buffer de transmissão ficar vazio
    while (SPI_I2S_GetFlagStatus(ADXL_SPI, SPI_I2S_FLAG_TXE) == RESET);
    // Envia o byte
    SPI_I2S_SendData(ADXL_SPI, data);
    // Espera o buffer de recepção conter um dado
    while (SPI_I2S_GetFlagStatus(ADXL_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    // Retorna o byte recebido
    return SPI_I2S_ReceiveData(ADXL_SPI);
}


void config_regADXL(uint8_t reg, uint8_t data) {
    CS_ADXL_SELECT();       // Baixa o Chip Select para iniciar a comunicação
    SPI_transceiver(reg);   // Envia o endereço do registrador (bit R/W=0 implícito)
    SPI_transceiver(data);  // Envia o dado a ser escrito
    CS_ADXL_DESELECT();     // Levanta o Chip Select para terminar
}


uint8_t lire_regADXL(uint8_t reg) {
    uint8_t data;
    CS_ADXL_SELECT();
    // Para ler, o bit 7 (R/W) do primeiro byte deve ser 1
    SPI_transceiver(reg | 0x80);
    // Envia um byte "dummy" (0x00) para gerar o clock e receber o dado
    data = SPI_transceiver(0x00);
    CS_ADXL_DESELECT();
    return data;
}

void lire_multiple_regADXL(uint8_t start_reg, uint8_t count, uint8_t* p_buffer) {
    CS_ADXL_SELECT();
    // Para ler múltiplos bytes, os bits 7 (R/W) e 6 (MB) devem ser 1
    SPI_transceiver(start_reg | 0x80 | 0x40);
    for (uint8_t i = 0; i < count; i++) {
        p_buffer[i] = SPI_transceiver(0x00);
    }
    CS_ADXL_DESELECT();
}

void envoyer_trame_accelero(void) {
    if (place_libre_tx < 7) return; // Precisa de 7 bytes livres

    uint8_t ent = 0xF0;
    // Dados em 13 bits, enviados em 2 bytes com b7=0
    // capteurs.b[0] = X_LSB, capteurs.b[1] = X_MSB, etc.
    uint8_t x_msb = capteurs.b[1] & 0x3F; // Pega 6 bits
    uint8_t x_lsb = capteurs.b[0];       // Pega 7 bits (b7 já é 0)
    uint8_t y_msb = capteurs.b[3] & 0x3F;
    uint8_t y_lsb = capteurs.b[2];
    uint8_t z_msb = capteurs.b[5] & 0x3F;
    uint8_t z_lsb = capteurs.b[4];

    fifo_push(ent);
    fifo_push(x_msb);
    fifo_push(x_lsb);
    fifo_push(y_msb);
    fifo_push(y_lsb);
    fifo_push(z_msb);
    fifo_push(z_lsb);
}


void init_SPI1_ADXL(void)
{
    // 1. Activer les horloges pour GPIOA et SPI1
    RCC_APB2PeriphClockCmd(ADXL_SPI_RCC | ADXL_GPIO_RCC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    // 2. Configurar pinos SPI (SCK, MISO, MOSI) como Alternate Function
    GPIO_InitStruct.GPIO_Pin = ADXL_SCK_PIN | ADXL_MOSI_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADXL_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = ADXL_MISO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; // MISO como entrada com pull-up
    GPIO_Init(ADXL_MISO_PORT, &GPIO_InitStruct);
    
    // 3. Configurar pino CS como GPIO Saída
    Configure_Pin_Generic(ADXL_CS_PORT, ADXL_CS_PIN, MODE_OUTPUT_HIGH);
    
    // 4. Configurar pino INT como GPIO Entrada
    Configure_Pin_Generic(ADXL_INT_PORT, ADXL_INT_PIN, MODE_INPUT_PULLUP);

    // 5. Configurar o periférico SPI1
    SPI_InitTypeDef SPI_InitStruct;
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High; // CPOL = 1
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge; // CPHA = 1 -> Modo 3
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;    // Controle manual do CS
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // 72MHz / 16 = 4.5MHz
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(ADXL_SPI, &SPI_InitStruct);

    // 6. Ativar o SPI1
    SPI_Cmd(ADXL_SPI, ENABLE);
}

  
     
     
void init_adxl_345(void){
    // Garante que o Chip Select comece em nível alto (não selecionado)
    CS_ADXL_DESELECT();
    
    // --- Configuração de Energia (POWER_CTL, reg 0x2D) ---
    // Sequência recomendada: Standby -> Measure
    config_regADXL(ADXL345_POWER_CTL, 0x00); // Coloca em modo Standby para alterar configurações
    config_regADXL(ADXL345_POWER_CTL, 0x08); // Ativa o modo de Medição

    // --- Formato dos Dados (DATA_FORMAT, reg 0x31) ---
    // Conforme os comentários: SPI 4-fios, INT ativo alto, Full Res, justificado à direita, range de 16G.
    // O valor 0x0B (0b00001011) configura Full Res (B3=1) e 16g (B1B0=11).
    // Para INT ativo alto (B5=0) e SPI 4-fios (B6=0), o valor 0x0B está correto.
    config_regADXL(ADXL345_DATA_FORMAT, 0x0B);

    // --- Taxa de Dados e Energia (BW_RATE, reg 0x2C) ---
    // O comentário no final da função indica 100 Hz.
    // 100 Hz corresponde ao valor 0x0A.
    config_regADXL(ADXL345_BW_RATE, 0x0A);
    
    // --- Controle de Atividade/Inatividade (ACT_INACT_CTL, etc.) ---
    // Ativa a detecção de atividade e inatividade em todos os eixos (X, Y, Z) em modo AC (relativo).
    config_regADXL(ADXL345_ACT_INACT_CTL, 0xFF);
    config_regADXL(ADXL345_THRESH_ACT, 16);    // Limiar de atividade: 16 * 62.5mg = 1g
    config_regADXL(ADXL345_THRESH_INACT, 8);   // Limiar de inatividade: 8 * 62.5mg = 0.5g
    config_regADXL(ADXL345_TIME_INACT, 2);     // Tempo em inatividade para gerar interrupção (2 segundos)
    
    // --- Detecção de Choc (TAP) ---
    // Os comentários indicam duas escritas conflitantes em TAP_AXES. Vamos usar a primeira.
    config_regADXL(ADXL345_DUR, 16);           // Duração mínima do choque para ser detectado (~10ms)
    config_regADXL(ADXL345_THRESH_TAP, 160);   // Limiar do choque: 160 * 62.5mg = 10g. 0xA0 = 160.
    config_regADXL(ADXL345_TAP_AXES, 0x07);    // Ativa a detecção de TAP nos eixos X, Y e Z.
    
    // --- Mapeamento e Ativação das Interrupções ---
    // Objetivo: Mapear todas as interrupções para o pino INT1, exceto DATA_READY.
    // No seu código original, INT_MAP estava 0x7F, que mapeia DATA_READY para INT2 e o resto para INT1.
    // INT_ENABLE estava 0xDF, que habilita todas as interrupções, exceto DOUBLE_TAP.
    // Esta configuração parece complexa e talvez conflitante. Uma configuração mais simples seria:
    config_regADXL(ADXL345_INT_MAP, 0x00);      // Mapeia TODAS as interrupções para o pino INT1
    config_regADXL(ADXL345_INT_ENABLE, 0x98);   // Habilita APENAS Data Ready(B7), Activity(B4) e Inactivity(B3)

    // --- Controle da FIFO ---
    // Modo Stream: a FIFO armazena as 32 amostras mais recentes, descartando as antigas.
    config_regADXL(ADXL345_FIFO_CTL, 0x80); // 0b10000000 -> Modo Stream
} 

bool fifo_push(uint8_t byte) {
    if (place_libre_tx == 0) return false;
    fifo_tx[pw_tx] = byte;
    pw_tx = (pw_tx + 1) % 32;
    place_libre_tx--;
    return true;
}

bool fifo_pop(uint8_t* p_byte) {
    if (place_libre_tx == 32) return false;
    *p_byte = fifo_tx[pr_tx];
    pr_tx = (pr_tx + 1) % 32;
    place_libre_tx++;
    return true;
}

void envoyer_trame_casiers(void) {
    if (place_libre_tx < 3) return;
    uint8_t header = 0xC0;
    uint8_t octet1 = (etat_casiers >> 8) & 0xFF;
    uint8_t octet2 = etat_casiers & 0xFF;
    
    header |= ((octet1 & 0x80) >> 7) << 0; // b7 do byte alto no bit 0
    header |= ((octet2 & 0x80) >> 7) << 1; // b7 do byte baixo no bit 1
    
    fifo_push(header);
    fifo_push(octet1 & 0x7F);
    fifo_push(octet2 & 0x7F);
}

void envoyer_trame_ZIF16(void) {
    if (place_libre_tx < 3) return;
    uint8_t header = 0x80;
    uint8_t octet1 = (etat_ZIF16 >> 8) & 0xFF;
    uint8_t octet2 = etat_ZIF16 & 0xFF;
    
    header |= ((octet1 & 0x80) >> 7) << 0;
    header |= ((octet2 & 0x80) >> 7) << 1;
    
    fifo_push(header);
    fifo_push(octet1 & 0x7F);
    fifo_push(octet2 & 0x7F);
}


void gere_serial2(void) {
	uint8_t status = USART2->SR;
	uint8_t ch_recu = USART2->DR;
	if(status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {MaeRcp=0;return;}
    if (ch_recu & B7_MASK) {
        // === ENT?TE ===
        entete = ch_recu;
        dataIndex = 0;

        if (ch_recu & B6_MASK) {
            // Message pour les LEDs (casiers)
            MaeRcp = 1;
        } else {
            // Message pour ZIF16
            MaeRcp = 5;
        }
    } else {
        // === DONN?ES ===
        switch (MaeRcp) {
            case 0:
                // Attente d'ent?te
                break;

            // === CASIERS : LEDs vertes/rouges ===
            case 1: {
                if (entete & (1 << 0)) ch_recu |= B7_MASK;
                consigne_leds_vertes = (consigne_leds_vertes & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 2;
                break;
            }
            case 2: {
                if (entete & (1 << 1)) ch_recu |= B7_MASK;
                consigne_leds_vertes = (consigne_leds_vertes & 0xFF00) | ch_recu;
                MaeRcp = 3;
                break;
            }
            case 3: {
                if (entete & (1 << 2)) ch_recu |= B7_MASK;
                consigne_leds_rouges = (consigne_leds_rouges & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 4;
                break;
            }
            case 4: {
                if (entete & (1 << 3)) ch_recu |= B7_MASK;
                consigne_leds_rouges = (consigne_leds_rouges & 0xFF00) | ch_recu;
                flagMajCasiers = true;
                MaeRcp = 0;
                break;
            }

            // === ZIF16 : direction et ?tat ===
            case 5: {
                if (entete & (1 << 0)) ch_recu |= B7_MASK;
                consigne_dir = (consigne_dir & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 6;
                break;
            }
            case 6: {
                if (entete & (1 << 1)) ch_recu |= B7_MASK;
                consigne_dir = (consigne_dir & 0xFF00) | ch_recu;
                MaeRcp = 7;
                break;
            }
            case 7: {
                if (entete & (1 << 2)) ch_recu |= B7_MASK;
                consigne_etat = (consigne_etat & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 8;
                break;
            }
            case 8: {
                if (entete & (1 << 3)) ch_recu |= B7_MASK;
                consigne_etat = (consigne_etat & 0xFF00) | ch_recu;
                flagMajZIF16 = true;
                MaeRcp = 0;
                break;
            }

            default:
                MaeRcp = 0;
                break;
        }
    }
}
// Função auxiliar, n ssei se vai entrar nao 
static void SDA_set_output(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

// Função auxiliar para mudar a direção do pino SDA para Entrada
static void SDA_set_input(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; // Entrada com Pull-up interno
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

void I2C_Delay(void) {volatile int i;
    for ( i = 0; i < 10; i++);
}

// Start condition
void I2C_Start(void) {
    SDA_set_output();
    SDA_H();
    SCL_H();
    I2C_Delay();
    SDA_L();
    I2C_Delay();
    SCL_L();
}

// Stop condition
void I2C_Stop(void) {
    SDA_set_output(); 
    SDA_L();
    SCL_L();
    I2C_Delay();
    SCL_H();
    I2C_Delay();
    SDA_H();
    I2C_Delay();
}

// Send 1 bit
void I2C_WriteBit(uint8_t bit) {
    SDA_set_output();
    if (bit) {
        SDA_H();
    } else {
        SDA_L();
    }
    I2C_Delay();
    SCL_H(); 
    I2C_Delay();
    SCL_L();  
}

// Read 1 bit
uint8_t I2C_ReadBit(void) {
    uint8_t bit;
    SDA_set_input(); 
    I2C_Delay();
    SCL_H(); 
    I2C_Delay();
    bit = SDA_READ();
    SCL_L();
    return bit;
}

// Send 1 byte (returns ACK=0 / NACK=1)
uint8_t I2C_WriteByte(uint8_t data) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        I2C_WriteBit(data & 0x80); 
        data <<= 1; // ???verificar dps
    }
    return I2C_ReadBit(); // Read ACK/NACK
}

// Read 1 byte and send ACK/NACK
uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t i;
    uint8_t byte = 0; // uatizap? nao era pra dar read nos bits?
    for (i = 0; i < 8; i++) {
        byte <<= 1;
        if (I2C_ReadBit()) {
            byte |= 1;
        }
    }
    // Envia ACK (0) se quiser continuar lendo, ou NACK (1) se for o último byte
    I2C_WriteBit(ack);
    return byte;
}
void I2C_write_PCF8574(uint8_t adresse, uint8_t data){
    I2C_Start();
    I2C_WriteByte(adresse << 1); // Endereço de 7 bits + bit de escrita (0)
    I2C_WriteByte(data);
    I2C_Stop();     
}

uint8_t I2C_read_PCF8574(uint8_t adresse){
    uint8_t data;
    I2C_Start();
    I2C_WriteByte((adresse << 1) | 1); // Endereço de 7 bits + bit de leitura (1)
    data = I2C_ReadByte(1); // Lê 1 byte e envia NACK (fim da leitura)
    I2C_Stop();
    return data;	
}
void maj_leds_casiers(void){
    uint8_t leds_v_hi = (consigne_leds_vertes >> 8) & 0xFF;
    uint8_t leds_v_lo = consigne_leds_vertes & 0xFF;
    uint8_t leds_r_hi = (consigne_leds_rouges >> 8) & 0xFF;
    uint8_t leds_r_lo = consigne_leds_rouges & 0xFF;

    // Os LEDs são ativos em nível baixo.
    // Uma consigne '1' significa LED aceso, então precisamos enviar '0' para o pino do PCF.
    // Portanto, invertemos todos os bits.
    I2C_write_PCF8574(ADDR_LEDS_V_HI, ~leds_v_hi);
    I2C_write_PCF8574(ADDR_LEDS_V_LO, ~leds_v_lo);
    I2C_write_PCF8574(ADDR_LEDS_R_HI, ~leds_r_hi);
    I2C_write_PCF8574(ADDR_LEDS_R_LO, ~leds_r_lo);
}

void lire_etat_casiers(void){
    uint8_t sens_hi, sens_lo;

    // Para usar os pinos do PCF como entrada, primeiro escrevemos '1' em todos eles.
    // Isso ativa as resistências de pull-up internas fracas do PCF.
    I2C_write_PCF8574(ADDR_SENS_HI, 0xFF);
    I2C_write_PCF8574(ADDR_SENS_LO, 0xFF);

    // Agora, lemos o estado dos pinos.
    sens_hi = I2C_read_PCF8574(ADDR_SENS_HI);
    sens_lo = I2C_read_PCF8574(ADDR_SENS_LO);

    // Combina os dois bytes em uma variável global de 16 bits.
    // Um interruptor fechado conecta o pino ao GND, lendo '0'.
    // Invertemos para que um casier fechado seja representado por um bit '1'.
    etat_casiers = ~( (uint16_t)sens_hi << 8 | sens_lo );
}


void init_proc(void)
{
 consigne_dir = 0xFFFF; // 1 = Input para todos os 16 pinos
 consigne_etat = 0x0000; // 0 = Pull-down para todos os 16 pinos
	
 init_ZIF16();
 init_I2C_BITBANGING();
 init_serial2();
 init_SPI1_ADXL();
 init_adxl_345();
}

void long_delay(void) {
    // Este valor de loop é um exemplo, ajuste conforme a velocidade do seu microcontrolador
    // para ter um intervalo visível entre as tramas no osciloscópio.
    for(volatile int i = 0; i < 50000; i++);
}
int main(void)
{ int rep_ack;
	uint8_t bits_entete;
	uint16_t etat_ZIF16;
	init_proc();
	
	while(1)
		{
        // 1. Processa recepção serial (TP1)
        if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET) {
            gere_serial2();
        }

        // 2. Executa ações baseadas nos flags (TP1)
        if (flagMajCasiers) {
            flagMajCasiers = false;
            maj_leds_casiers();
            lire_etat_casiers();
            envoyer_trame_casiers();
        }
        if (flagMajZIF16) {
            flagMajZIF16 = false;
            init_ZIF16();
            etat_ZIF16 = lire_etat_ZIF16();
            envoyer_trame_ZIF16();
        }

        // 3. Verifica o acelerômetro (TP2)
        // O roteiro diz "on scruptera la patte", então faremos polling.
        if (GPIO_ReadInputDataBit(ADXL_INT_PORT, ADXL_INT_PIN) == SET) {
            // A flag de "Data Ready" está ativa, então lemos os 6 bytes de dados
            lire_multiple_regADXL(ADXL345_DATAX0, 6, capteurs.b);
            envoyer_trame_accelero();
        }

        // 4. Processa transmissão serial (TP1 e TP2)
        if ((USART_GetFlagStatus(USART2, USART_FLAG_TXE) == SET) && (place_libre_tx < 32)) {
            uint8_t byte_a_enviar;
            if (fifo_pop(&byte_a_enviar)) {
                USART_SendData(USART2, byte_a_enviar);
            }
        }																
 } // fin while(1)
}//fin main
