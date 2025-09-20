#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>




#define I2C_PORT            GPIOB
#define I2C_SCL_PIN         GPIO_Pin_6  // PB6
#define I2C_SDA_PIN         GPIO_Pin_7  // PB7
#define I2C_RCC_PORT        RCC_APB2Periph_GPIOB // Clock do barramento da porta

// Endereços (7 bits) dos 6 componentes PCF8574
// --- A FAZER: Defina os endereços corretos de acordo com a pinagem A0-A2 da sua placa ---
#define ADDR_LEDS_V_HI      0x41  // PCF para Leds Verdes 15-8 (Byte alto)
#define ADDR_LEDS_V_LO      0x43  // PCF para Leds Verdes 7-0 (Byte baixo)
#define ADDR_LEDS_R_HI      0x45  // PCF para Leds Vermelhos 15-8
#define ADDR_LEDS_R_LO      0x47  // PCF para Leds Vermelhos 7-0
#define ADDR_SENS_HI        0x49  // PCF para Sensores 15-8
#define ADDR_SENS_LO        0x4B  // PCF para Sensores 7-0

// Macros para facilitar a manipulação dos pinos
#define SCL_H()             GPIO_SetBits(I2C_PORT, I2C_SCL_PIN)
#define SCL_L()             GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN)
#define SDA_H()             GPIO_SetBits(I2C_PORT, I2C_SDA_PIN)
#define SDA_L()             GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN)
#define SDA_READ()          GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN)

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


uint8_t fifo[256];
uint8_t pw =0;
uint8_t pr =0;
int place_libre = 256;

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


/*void init_ZIF16(void)
{uint8_t boucle;
 uint32_t choix_INOUT;
 uint8_t	choix_etat;	
for(boucle = 0; boucle<8; boucle++)
	{ choix_INOUT = 	;
		choix_etat = 		 ;
	  
	}
for(boucle = 8; boucle<16; boucle++)
	{ choix_INOUT =  ;
	  choix_etat =   ;
	 
	}
}

uint16_t lire_etat_ZIF16(void)
{
	return ;
}*/

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

void init_SPI1_ADXL(void)
{
    // 1. Activer les horloges pour GPIOA et SPI1
 

    // 2. Configurer les broches SPI
    // PA5 - SCK : 
    // PA6 - MISO : 
    // PA7 - MOSI : 
    // PA4 - nSS : GPIO Output Push-Pull (manuelle)
   

    // 3. Configurer SPI1
 
}


void config_regADXL(unsigned char reg, unsigned char data) 
 { 
 

	 }   
     
     
     
/*void init_adxl_345(void){
    CS_ADXL(RELEASE); 
    //initialiser power : sequence en plusieurs ?tapes conseill?e ?????? supprim? ?tape autosleep...
    config_regADXL(ADXL345_POWER_CTL, 0);// Wakeup  
   // config_regADXL(ADXL345_POWER_CTL, 16);// Auto_Sleep sera ?cras? par mode Measure
    config_regADXL(ADXL345_POWER_CTL, 8);// Measure
 //*******************************************DATA FORMAT ******************************************************************   
    // FORMAT DES DONNES  :  passer en 16G justifi? droit 
    // B7 : self test : garder ? 0
    // B6 : mode SPI 3 ou SPI 4 fils : mettre 0 pour selectionner le mode 4 FILS
    // B5 : niveau logique des ITs   : mettre 1 = /int         mettre 0   =  int (actif haut) : mettre 1
    // B4 : laisser ? 0
    // B3 : Full_ RES : mettre 1 pour disposer de plus de bits possibles
    // B2 : JUSTIFICATION DES DATAs  :  1 left justify, 0 right justify avec extension de signe : mettre 0 
    // B1 B0 : sensibilit? : mettre 00 pour 2G , 01 pour 4G, 10 pour 8G, 11 pour 16G :   11
    // choix  0b00101011 soit 0x2B   
    config_regADXL(ADXL345_DATA_FORMAT, 0x2B);
 
 //***************************************inactivit?/ choc niveau 1****************************************************************
    //D7 : activit? DC ou AC   (absolu ou relatif aux pr?c?dentes mesures) :mettre 1 pour choisir AC
    //D6 : activit? sur X
    //D5 : activit? sur Y 
    //D4 : activit? sur Z  
    //D3 : inactivit? DC ou AC   (absolu ou relatif aux pr?c?dentes mesures) :mettre 1 pour choisir AC
    //D2 : inactivit? sur X
    //D1 : inactivit? sur Y 
    //D0 : inactivit? sur Z  
    // on va dire qu'il y a activit? si on d?tecte une activit? sur X Y ou Z
    // on va dire qu'il y a inactivit? si on ne d?tecte rien ni sur X ni sur Y ni sur Z en mode AC
    config_regADXL(ADXL345_ACT_INACT_CTL, 0xFF);
    
    // la valeur est par pas de 62.5mG  ainsi  256 correspondrait ? 16G : mettre 
    config_regADXL(ADXL345_THRESH_ACT,16 );
    config_regADXL(ADXL345_THRESH_INACT,8 );
    config_regADXL(ADXL345_TIME_INACT, 1 );//sec 1-255
    
 //*******************************************choc**********************************************************   
    //D7 ? D4 non definis : 0
    //D3 : suppress double tap si acceleration reste elev?e entre les TAP avec valeur 1
    //D2 : tap X enable si 1
    //D1 : tap Y enable si 1
    //D0 : tap Z enable si 1
    
   config_regADXL(ADXL345_TAP_AXES, 0x0F); // detection choc de tous les cot?s
   config_regADXL(ADXL345_THRESH_TAP, 0xA0); // detection choc r?gl?e ? 10G
   config_regADXL(ADXL345_DUR, 16); // duree minimale du choc 625us increment ici 10ms
   config_regADXL(ADXL345_LATENT, 0x00); // ecart minimum entre tap pas 1.25ms : 0 desactive
   config_regADXL(ADXL345_TAP_AXES, 0x00); // fenetre seconde frappe pas 1.25ms : 0 desactive
 
 //*****************************************************************************************************************
   
   config_regADXL(ADXL345_THRESH_FF, 0x09); // seuil detection FREE FALL  pas 62.5mG 0.6G
   config_regADXL(ADXL345_TIME_FF, 10); // duree minimale de chute pas 5ms  : 100 ms =20
  
 //***************************************************************************************************************
 //interruptions  activer les ITs : ADXL345_INT_ENABLE  bit ? 0 = INT1 , bit ? 1 = INT2
 // selectionner patte INT1 ou INT2: ADXL345_INT_MAP
 // en cas de selection multiple, la lecture de ADXL345_INT_SOURCE
   // pour les deux registres, meme emplacement :
   // B7 = DATA_READY
   // B6 = SINGLE_TAP
   // B5 = DOUBLE_TAP
   // B4 = ACTIVITY
   // B3 = INACTIVITY
   // B2 = FREE_FALL
   // B1 = WATER_MARK  (niveau remplissage FIFO)
   // B0 = OVERRUN     (pas lu assez frequemment)
    config_regADXL(ADXL345_INT_ENABLE, 0x00);
    config_regADXL(ADXL345_INT_MAP, 0x7F); //0x7F; 0x00 tout le monde en INT 2 sauf DATA_READY
    config_regADXL(ADXL345_INT_ENABLE, 0xDF); //0xDF  11011111
   
  //***********************************************************************************************************
  //gestion par FIFO pour stocker sans danger  
   //ADXL345_FIFO_CTL : les 4 bits de poids faible choisissent le d?bit (voir doc))
    //F : 3200Hz, E:1600, D:800, C:400, B:200, A:100, 9:50, 8:25, 7:12.5, 6:6.25, 5:3.125, 
   // ADXL345_FIFO_CTL : 
    // bits B7 B6   : 
    // 00 bypass (no fifo)
    // 01 FIFO (blocage plein) 
    // 10 STREAM (ecrasement) 
    // 11 Trigger (photo evenement)
    // bit B5  Trigger sur INT1 (0) ou INT2 (1)
    // bit B4 ? B0 : niveau remplissage pour watermark
    
    config_regADXL(ADXL345_BW_RATE, 0x0A);  // Fonctionnement ? 100 HZ
    config_regADXL(ADXL345_FIFO_CTL, 0x90); // stream, trig int1, avertissement sur mi remplissage (16)

} */




/*void gere_serial2(void) {
	uint8_t status = USART2->SR;
	uint8_t ch_recu = USART2->DR;
	if(status & ??????) {MaeRcp=0;return;}
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
}*/
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
 //init_ZIF16();
 init_I2C_BITBANGING();
 init_serial2();
 init_SPI1_ADXL();
 //init_adxl_345();// fonction donn?e ? condition d'?crire 	config_regADXL(unsigned char reg, unsigned char data)  
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
	// init_proc();
	init_I2C_BITBANGING();
	
	while(1)
		{
           // Acende o LED 0 (bit 0 = 1)
            consigne_leds_vertes = 0x0001;
            maj_leds_casiers();
            long_delay();

            // Apaga todos os LEDs (todos os bits = 0)
            consigne_leds_vertes = 0x0000;
            maj_leds_casiers();
            long_delay();	
		/*if (flagMajCasiers) {flagMajCasiers = false;
				// Appliquer consigne_leds_vertes et consigne_leds_rouges
				//en envoyant les bonnes trames I2C, chaque PCF sera configur? avec une adresse diff?rente
			                  maj_leds_casiers(); // par ecriture i2C
			                  lire_etat_casiers(); // par lecture i2c
			                  envoyer_trame_casiers();
		}

		if (flagMajZIF16) {flagMajZIF16 = false;
				// Appliquer consigne_dir et consigne_etat
			                 init_ZIF16();//pour mettre ? jour les pattes
			                 etat_ZIF16 = lire_etat_ZIF16();
			                 envoyer_trame_ZIF16();
											}	
	if(USART2->SR & ?????? )
	  {// gerer la r?ception  serie
	  } //
	if(USART2->SR & ?????? )	
	   { //d?piler la fifo()
	   }
	if(INT_ADXL)
	{read_ADXL_sensors(&capteurs.b[0]);
	 envoi_trame_accelero();
    } */
     																			
 } // fin while(1)
}//fin main
