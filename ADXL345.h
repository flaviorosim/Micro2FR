#ifndef ADXL345_H
#define ADXL345_H

// --- Endereços dos Registradores do ADXL345 ---
// (Conforme datasheet e o uso na sua função)
#define ADXL345_POWER_CTL       0x2D
#define ADXL345_DATA_FORMAT     0x31
#define ADXL345_BW_RATE         0x2C
#define ADXL345_FIFO_CTL        0x38

#define ADXL345_INT_ENABLE      0x2E
#define ADXL345_INT_MAP         0x2F
#define ADXL345_INT_SOURCE      0x30

#define ADXL345_THRESH_ACT      0x24
#define ADXL345_THRESH_INACT    0x25
#define ADXL345_TIME_INACT      0x26
#define ADXL345_ACT_INACT_CTL   0x27

#define ADXL345_THRESH_TAP      0x1D
#define ADXL345_DUR             0x21
#define ADXL345_LATENT          0x22
#define ADXL345_WINDOW          0x23 // Registrador para a "janela" do segundo tap
#define ADXL345_TAP_AXES        0x2A

#define ADXL345_THRESH_FF       0x28
#define ADXL345_TIME_FF         0x29

#define ADXL345_DATAX0          0x32 // Primeiro byte de dados (X LSB)

// --- Macros para o pino CS (Chip Select) ---
#define CS_ADXL_SELECT()        GPIO_ResetBits(ADXL_CS_PORT, ADXL_CS_PIN)   // CS para 0 (seleciona)
#define CS_ADXL_DESELECT()      GPIO_SetBits(ADXL_CS_PORT, ADXL_CS_PIN)     // CS para 1 (libera)

#endif // ADXL345_H