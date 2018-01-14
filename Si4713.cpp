/*****************************************************************************
 * This lib provides API for SiLab Si4732 FM/RDS Trsnmitter based on AN332
 * document.
 *
 * Files uploaded to: https://github.com/sarace77/Si4713lib
 *
 * LICENSE:
 *
 * BSD 2-Clause License
 *
 * Copyright (c) 2018, Antonio Scopelliti
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

#include "Si4713.h"

//#define SI4713_VERBOSE_ENABLE

#ifdef SI4713_VERBOSE_ENABLE
#define PRINTRESULT Serial.print(__FUNCTION__); Serial.print("() - Cmd Result: "); Serial.println(m_i2c_buffer[0], HEX);
#endif //SI4713_VERBOSE_ENABLE

#define MASK_ASQ_OVERMOD        0x04
#define MASK_ASQ_IA_HL_OVERMOD  0x02
#define MASK_ASQ_IA_LL_OVERMOD  0x01

Si4713::Si4713() :
    m_i2c_addr(SI4713_I2C_ADDR1),
    m_rst_pin(D3)
{
    clearI2cBuffer();
}

bool Si4713::begin(uint8_t a_scl_pin, uint8_t a_sda_pin, uint8_t a_i2c_addr, uint8_t a_rst_pin)
{
    Wire.begin(a_sda_pin, a_scl_pin);
    m_i2c_addr  = a_i2c_addr;
    m_rst_pin   = a_rst_pin;
    reset();
    Cmd_Power_up(MASK_POWER_UP_X_OSC_ENABLE|MASK_POWER_UP_TRANSMIT, POWER_UP_OP_MODE_ANALOG);
    if (13 != Cmd_Get_rev())
    {
        Serial.println("Si4713 not found");
        return false;
    }
    return true;
}

void Si4713::clearI2cBuffer()
{
    memset(m_i2c_buffer, 0, SI4713_I2C_BUFF_MAX_SIZE);
}

void Si4713::getResult(uint8_t a_len)
{
    clearI2cBuffer();
    Wire.requestFrom((uint8_t)m_i2c_addr, a_len);
    for(uint8_t i = 0; i < a_len; i++)
    {
        m_i2c_buffer[i] = Wire.read();
    }
}

void Si4713::reset()
{
    if (m_rst_pin > 0)
    {
      pinMode(m_rst_pin, OUTPUT);
      digitalWrite(m_rst_pin, HIGH);
      delay(10);
      digitalWrite(m_rst_pin, LOW);
      delay(10);
      digitalWrite(m_rst_pin, HIGH);
    }

}

bool Si4713::sendI2cCommand(uint8_t a_len)
{
    if (0 == a_len)
    {
        return false;
    }
    uint8_t l_status    = 0;
    uint8_t l_count     = 0;

    Wire.beginTransmission(m_i2c_addr);
#ifdef SI4713_VERY_VERBOSE_ENABLE
            Serial.print(__FUNCTION__);Serial.print("() -  Sending:");
#endif //SI4713_VERBOSE_ENABLE
    for (uint8_t i = 0; i < a_len; i++)
    {
#ifdef SI4713_VERY_VERBOSE_ENABLE
            Serial.print(" 0x"); Serial.print(m_i2c_buffer[i],HEX);
#endif //SI4713_VERBOSE_ENABLE
        Wire.write(m_i2c_buffer[i]);
    }
    Wire.endTransmission();
    Serial.println();
    while (0 == (l_status & MASK_I2C_CTS))
    {
        Wire.requestFrom((uint8_t)m_i2c_addr, (uint8_t) 1);
        while ((Wire.available() < 1) && (SI4713_I2C_MAX_RETRY_COUNT > l_count++))
        {
#ifdef SI4713_VERY_VERBOSE_ENABLE
            Serial.print(".");
#endif //SI4713_VERBOSE_ENABLE
            delay(1);
        }
        if (SI4713_I2C_MAX_RETRY_COUNT == l_count)
        {
#ifdef SI4713_VERY_VERBOSE_ENABLE
            Serial.println(" Failed!");
#endif //SI4713_VERBOSE_ENABLE
        return false;
        }
        l_status = Wire.read();
    }
    return true;
}


bool Si4713::Cmd_Power_up(uint8_t a_mask, uint8_t a_op_mode)
{
    clearI2cBuffer();
    if(POWER_UP_OP_MODE_ANALOG != a_op_mode && POWER_UP_OP_MODE_DIGITAL != a_op_mode)
    {
        return false;
    }
    m_i2c_buffer[0] = SI4713_CMD_POWER_UP;
    m_i2c_buffer[1] = a_mask;
    m_i2c_buffer[2] = a_op_mode;
    sendI2cCommand(SIZE_POWER_UP);
    getResult(SIZE_POWER_UP_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
#endif //SI4713_VERBOSE_ENABLE
    return (m_i2c_buffer[0] & MASK_I2C_CTS);
}

uint8_t Si4713::Cmd_Get_rev()
{
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_GET_REV;
    sendI2cCommand(SIZE_GET_REV);
    getResult(SIZE_GET_REV_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
    Serial.print(__FUNCTION__);Serial.print("() -  Si47"); Serial.println(m_i2c_buffer[1]);
    Serial.print(__FUNCTION__);Serial.print("() -  FW: "); Serial.print(m_i2c_buffer[2]); Serial.print(".");Serial.print(m_i2c_buffer[3]);
    Serial.print("-patch_"); Serial.print(m_i2c_buffer[4]);Serial.println(m_i2c_buffer[5]);
    Serial.print(__FUNCTION__);Serial.print("() -  Comp: "); Serial.print(m_i2c_buffer[6]); Serial.print(".");Serial.print(m_i2c_buffer[7]);
    Serial.print("-rev"); Serial.println(m_i2c_buffer[8]);
#endif //SI4713_VERBOSE_ENABLE
    return m_i2c_buffer[1];
}

bool Si4713::Cmd_Power_down()
{
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_POWER_DOWN;
    sendI2cCommand(SIZE_POWER_DOWN);
    getResult(SIZE_POWER_DOWN_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
#endif //SI4713_VERBOSE_ENABLE
    return (m_i2c_buffer[0] & MASK_I2C_CTS);
}

bool Si4713::Cmd_Set_Property(uint16_t a_property, uint16_t a_value)
{
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_SET_PROPERTY;
    m_i2c_buffer[2] = HI_BYTE(a_property);
    m_i2c_buffer[3] = LO_BYTE(a_property);
    m_i2c_buffer[4] = HI_BYTE(a_value);
    m_i2c_buffer[5] = LO_BYTE(a_value);
    sendI2cCommand(SIZE_SET_PROPERTY);
    getResult(SIZE_SET_PROPERTY_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
    Serial.print(__FUNCTION__);Serial.print("() - property: 0x"); Serial.print(a_property, HEX); Serial.print(" = 0x"); Serial.println(a_value, HEX);
#endif //SI4713_VERBOSE_ENABLE
    return (m_i2c_buffer[0] & MASK_I2C_CTS);
}

uint16_t Si4713::Cmd_Get_Property(uint16_t a_property)
{
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_GET_PROPERTY;
    m_i2c_buffer[2] = HI_BYTE(a_property);
    m_i2c_buffer[3] = LO_BYTE(a_property);
    sendI2cCommand(SIZE_GET_PROPERTY);
    getResult(SIZE_GET_PROPERTY_RESULT);
//    uint16_t l_value    = m_i2c_buffer[2];
//    l_value             <<= 8;
//    l_value             |= m_i2c_buffer[3];
    uint16_t l_value    = PACK16BITS(m_i2c_buffer[2], m_i2c_buffer[3]);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
    Serial.print(__FUNCTION__);Serial.print("() - property: 0x"); Serial.print(a_property, HEX); Serial.print(" = 0x"); Serial.println(l_value, HEX);
#endif //SI4713_VERBOSE_ENABLE
    return l_value;
}

uint8_t Si4713::Cmd_Get_Int_Status()
{
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_GET_INT_STATUS;
    sendI2cCommand(SIZE_GET_INT_STATUS);
    getResult(SIZE_GET_INT_STATUS_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
#endif //SI4713_VERBOSE_ENABLE
    return m_i2c_buffer[0];
}

bool Si4713::Cmd_Tx_Tune_Freq(uint16_t a_freq)
{
    if(FM_BAND_LIMIT_LO > a_freq || FM_BAND_LIMIT_HI < a_freq)
    {
        return false;
    }
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_TX_TUNE_FREQ;
    m_i2c_buffer[2] = HI_BYTE(a_freq);
    m_i2c_buffer[3] = LO_BYTE(a_freq);
    sendI2cCommand(SIZE_TX_TUNE_FREQ);
    getResult(SIZE_TX_TUNE_FREQ_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
    Serial.print(__FUNCTION__);Serial.print("() - New Frequency: "); Serial.print(a_freq); Serial.println("MHz");
#endif //SI4713_VERBOSE_ENABLE
    return (m_i2c_buffer[0] & MASK_I2C_CTS);
}

bool Si4713::Cmd_Tx_Tune_Power(uint8_t a_level, uint8_t a_ant_cap)
{
    if((0 != a_level && TX_POWER_LIMIT_LO > a_level) || a_level > TX_POWER_LIMIT_HI || a_ant_cap > ANT_CAPACITOR_LIMIT_HI)
    {
        return false;
    }
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_TX_TUNE_POWER;
    m_i2c_buffer[3] = a_level;
    m_i2c_buffer[4] = a_ant_cap;
    sendI2cCommand(SIZE_TX_TUNE_POWER);
    getResult(SIZE_TX_TUNE_POWER_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
    Serial.print(__FUNCTION__);Serial.print("() - New Output Power Level: "); Serial.print(a_level); Serial.println("dBuV");
    Serial.print(__FUNCTION__);Serial.print("() - New Antenna Capacitor value: "); Serial.print(((float)a_level)/4); Serial.println("pF");
#endif //SI4713_VERBOSE_ENABLE
    return (m_i2c_buffer[0] & MASK_I2C_CTS);
}

bool Si4713::Cmd_Tx_Tune_Measure(uint16_t a_freq, uint8_t a_ant_cap)
{
    if(FM_BAND_LIMIT_LO > a_freq || FM_BAND_LIMIT_HI < a_freq || a_ant_cap > ANT_CAPACITOR_LIMIT_HI)
    {
        return false;
    }
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_TX_TUNE_MEASURE;
    m_i2c_buffer[2] = HI_BYTE(a_freq);
    m_i2c_buffer[3] = LO_BYTE(a_freq);
    m_i2c_buffer[4] = a_ant_cap;
    sendI2cCommand(SIZE_TX_TUNE_MEASURE);
    getResult(SIZE_TX_TUNE_MEASURE_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
    Serial.print(__FUNCTION__);Serial.print("() - Scanned Frequency: "); Serial.print(a_freq); Serial.println("MHz");
#endif //SI4713_VERBOSE_ENABLE
    return (m_i2c_buffer[0] & MASK_I2C_CTS);
}

TuneSts_t Si4713::Cmd_Tx_Tune_Status(bool a_clear_irq)
{
    TuneSts_t   l_status;
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_TX_TUNE_STATUS;
    m_i2c_buffer[1] = a_clear_irq ? 0x1: 0x0;
    sendI2cCommand(SIZE_TX_TUNE_STATUS);
    getResult(SIZE_TX_TUNE_STATUS_RESULT);
    l_status.frequency      = m_i2c_buffer[2];
    l_status.frequency      <<= 8;
    l_status.frequency      |= m_i2c_buffer[3];
    l_status.outputLevel    = m_i2c_buffer[5];
    l_status.capacitor      = m_i2c_buffer[6];
    l_status.capacitor      *= 0.25;
    l_status.noiseLevel     = m_i2c_buffer[7];
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
    Serial.print(__FUNCTION__);Serial.print("() - Frequency: "); Serial.print(l_status.frequency); Serial.println("MHz");
    Serial.print(__FUNCTION__);Serial.print("() - Output Level: "); Serial.print(l_status.outputLevel); Serial.println("dBuV");
    Serial.print(__FUNCTION__);Serial.print("() - Capacitor: "); Serial.print((float)l_status.capacitor); Serial.println("pF");
    Serial.print(__FUNCTION__);Serial.print("() - Noise Level: "); Serial.println(l_status.frequency); Serial.println("dBuV");
#endif //SI4713_VERBOSE_ENABLE
    return l_status;
}

AsqSts_t Si4713::Cmd_Tx_ASQ_Status(bool a_clear_irq)
{
    AsqSts_t    l_status;
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_TX_ASQ_STATUS;
    m_i2c_buffer[1] = a_clear_irq ? 0x1: 0x0;
    sendI2cCommand(SIZE_TX_ASQ_STATUS);
    getResult(SIZE_TX_ASQ_STATUS_RESULT);
    l_status.overmod    = m_i2c_buffer[1] & MASK_ASQ_OVERMOD;
    l_status.ia_hl      = m_i2c_buffer[1] & MASK_ASQ_IA_HL_OVERMOD;
    l_status.ia_ll      = m_i2c_buffer[1] & MASK_ASQ_IA_LL_OVERMOD;
    l_status.inputLevel = m_i2c_buffer[4];
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
    if(l_status.overmod)Serial.print(__FUNCTION__);Serial.println("() - Modulation Overloaded");
    if(l_status.ia_hl)Serial.print(__FUNCTION__);Serial.println("() - Input Audio Level exceeded High Threshold");
    if(l_status.ia_ll)Serial.print(__FUNCTION__);Serial.println("() - Input Audio Level exceeded Low Threshold");
    Serial.print(__FUNCTION__);Serial.print("Input Level: "); Serial.print(l_status.inputLevel); Serial.println("dBuV");
#endif //SI4713_VERBOSE_ENABLE
    return l_status;
}

bool Si4713::Cmd_Tx_RDS_Buff(uint8_t a_mode, uint16_t a_block_b, uint16_t a_block_c, uint16_t a_block_d)
{
    if(a_mode & 0x78) //bits 6:3 must be 0
    {
        return false;
    }
    bool l_status   = false;
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_TX_RDS_BUFF;
    m_i2c_buffer[1] = a_mode;
    m_i2c_buffer[2] = HI_BYTE(a_block_b);
    m_i2c_buffer[3] = LO_BYTE(a_block_b);
    m_i2c_buffer[4] = HI_BYTE(a_block_c);
    m_i2c_buffer[5] = LO_BYTE(a_block_c);
    m_i2c_buffer[6] = HI_BYTE(a_block_d);
    m_i2c_buffer[7] = LO_BYTE(a_block_d);
    sendI2cCommand(SIZE_TX_RDS_BUFFER);
    getResult(SIZE_TX_RDS_BUFFER_RESULT);
    l_status = (0 != (MASK_TX_RDS_BUFFER_CBUFXMIT & m_i2c_buffer[1])) || (0 != (MASK_TX_RDS_BUFFER_FIFOXMIT & m_i2c_buffer[1]));
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
    if(!l_status)Serial.print(__FUNCTION__); Serial.print("() - RDS emit failed!");
#endif //SI4713_VERBOSE_ENABLE
    return l_status;
}

bool Si4713::Cmd_Tx_RDS_PS(uint8_t a_ps_id, char *p_ps_chars)
{
    if(a_ps_id & 0xE0) // bits 7:5 must be 0
    {
        return false;
    }
    clearI2cBuffer();
    memset(m_i2c_buffer, ' ', SIZE_TX_RDS_PS);
    m_i2c_buffer[0] = SI4713_CMD_TX_RDS_PS;
    m_i2c_buffer[1] = a_ps_id;
    memcpy(m_i2c_buffer + 2, p_ps_chars, min(4, strlen(p_ps_chars)));
    sendI2cCommand(SIZE_TX_RDS_PS);
    getResult(SIZE_TX_RDS_PS_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
#endif //SI4713_VERBOSE_ENABLE
    return (m_i2c_buffer[0] & MASK_I2C_CTS);
}

bool Si4713::Cmd_GPIO_Ctl(uint8_t a_mask)
{
    if (a_mask & 0x0E) // only bits 3:1 must be masked
    {
        return false;
    }
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_GPIO_CTL;
    m_i2c_buffer[1] = a_mask;
    sendI2cCommand(SIZE_GPIO_CTL);
    getResult(SIZE_GPIO_CTL_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
#endif //SI4713_VERBOSE_ENABLE
    return (m_i2c_buffer[0] & MASK_I2C_CTS);
}

bool Si4713::Cmd_GPIO_Set(uint8_t a_value)
{
    if (a_value & 0x0E) // only bits 3:1 must be set
    {
        return false;
    }
    clearI2cBuffer();
    m_i2c_buffer[0] = SI4713_CMD_GPIO_SET;
    m_i2c_buffer[1] = a_value;
    sendI2cCommand(SIZE_GPIO_SET);
    getResult(SIZE_GPIO_SET_RESULT);
#ifdef SI4713_VERBOSE_ENABLE
    PRINTRESULT;
#endif //SI4713_VERBOSE_ENABLE
    return (m_i2c_buffer[0] & MASK_I2C_CTS);
}

bool Si4713::waitOn(uint8_t a_mask)
{
    Cmd_Get_Int_Status();
    uint8_t l_counter = 0;
    while(0 == (m_i2c_buffer[0] & a_mask) && SI4713_I2C_MAX_RETRY_COUNT > l_counter++)
    {
        delay(10);
        getResult(SIZE_STATUS);
    }
    return (0 != (m_i2c_buffer[0] & a_mask));
}

