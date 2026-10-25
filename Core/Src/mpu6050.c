#include "mpu6050.h"
#include "board_c10b.h"
#include "car_config.h"

static void DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
}

static void SDA(bool high)
{
    HAL_GPIO_WritePin(BOARD_MPU_PORT, BOARD_MPU_SDA_PIN,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool SCL(bool high)
{
    uint32_t start = DWT->CYCCNT;
    HAL_GPIO_WritePin(BOARD_MPU_PORT, BOARD_MPU_SCL_PIN,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    while (high && HAL_GPIO_ReadPin(BOARD_MPU_PORT, BOARD_MPU_SCL_PIN) == GPIO_PIN_RESET)
    {
        if ((uint32_t)(DWT->CYCCNT - start) >= SystemCoreClock / 10000U)
        {
            return false; /* 100 us bounded clock-stretch timeout */
        }
    }
    DelayUs(5U);
    return true;
}

static bool Start(void)
{
    SDA(true);
    if (!SCL(true) || HAL_GPIO_ReadPin(BOARD_MPU_PORT, BOARD_MPU_SDA_PIN) == GPIO_PIN_RESET)
    {
        return false;
    }
    SDA(false);
    DelayUs(5U);
    return SCL(false);
}

static void Stop(void)
{
    (void)SCL(false);
    SDA(false);
    DelayUs(5U);
    (void)SCL(true);
    SDA(true);
    DelayUs(5U);
}

static bool Write(uint8_t byte)
{
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
        SDA((byte & 0x80U) != 0U);
        DelayUs(5U);
        if (!SCL(true)) { return false; }
        (void)SCL(false);
        byte <<= 1U;
    }
    SDA(true);
    DelayUs(5U);
    if (!SCL(true)) { return false; }
    bool ack = HAL_GPIO_ReadPin(BOARD_MPU_PORT, BOARD_MPU_SDA_PIN) == GPIO_PIN_RESET;
    (void)SCL(false);
    return ack;
}

static bool Read(uint8_t *byte, bool final)
{
    uint8_t value = 0U;
    SDA(true);
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
        if (!SCL(true)) { return false; }
        value = (uint8_t)((value << 1U) |
                (HAL_GPIO_ReadPin(BOARD_MPU_PORT, BOARD_MPU_SDA_PIN) == GPIO_PIN_SET));
        (void)SCL(false);
    }
    /* ACK intermediate bytes, NACK the last byte. */
    SDA(final);
    if (!SCL(true)) { return false; }
    (void)SCL(false);
    *byte = value;
    return true;
}

bool MPU6050_CheckIdentity(uint8_t *identity)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t value = 0U;
    bool ok;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HAL_GPIO_WritePin(BOARD_MPU_PORT, BOARD_MPU_SCL_PIN | BOARD_MPU_SDA_PIN, GPIO_PIN_SET);
    gpio.Pin = BOARD_MPU_SCL_PIN | BOARD_MPU_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL; /* external pull-ups to 3V3 on the module */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BOARD_MPU_PORT, &gpio);
    ok = Start() && Write(0xD0U) && Write(0x75U) &&
         Start() && Write(0xD1U) && Read(&value, true);
    Stop();
    *identity = value;
    return ok && value == 0x68U;
}

static bool WriteRegister(uint8_t reg, uint8_t value)
{
    bool ok=Start() && Write(0xD0U) && Write(reg) && Write(value);
    Stop(); return ok;
}
static bool ReadRegisters(uint8_t reg, uint8_t *bytes, unsigned count)
{
    bool ok=Start() && Write(0xD0U) && Write(reg) && Start() && Write(0xD1U);
    for (unsigned i=0;ok && i<count;++i) ok=Read(&bytes[i],i+1==count);
    Stop(); return ok;
}
bool MPU6050_Init(uint8_t *identity)
{
    if (!MPU6050_CheckIdentity(identity)) return false;
    /* PLL X gyro, awake; DLPF 3 (42 Hz gyro), 1 kHz sampling (read at 200 Hz),
     * gyro +/-500 deg/s, acceleration +/-2g. */
    if (!WriteRegister(0x6B,0x01) || !WriteRegister(0x6C,0x00) ||
        !WriteRegister(0x1A,0x03) || !WriteRegister(0x19,0x00) ||
        !WriteRegister(0x1B,0x08) || !WriteRegister(0x1C,0x00) ||
        /* Poll DATA_RDY; clear status only when INT_STATUS is read. */
        !WriteRegister(0x37,0x00) || !WriteRegister(0x38,0x01)) return false;
    uint8_t value=0;
    return ReadRegisters(0x1B,&value,1) && value==0x08;
}
bool MPU6050_ReadGyroZ(float *z_dps)
{
    uint8_t bytes[2], status=0;
    /* Require a newly generated sensor sample, not just an ACK from I2C. */
    if (!ReadRegisters(0x3A,&status,1) || !(status&0x01U) ||
        !ReadRegisters(0x47,bytes,2)) return false;
    uint16_t raw=(uint16_t)(((uint16_t)bytes[0]<<8)|bytes[1]);
    int32_t signed_raw=raw<=32767U ? (int32_t)raw : (int32_t)raw-65536;
    *z_dps=(float)(signed_raw*CAR_GYRO_Z_SIGN)/65.5f;
    return true;
}
