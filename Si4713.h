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

#ifndef SI4713_H
#define SI4713_H

#include "Arduino.h"
#include <Wire.h>


//Si4713 Commands from AN332.pdf
#define SI4713_CMD_POWER_UP         0x01
#define SI4713_CMD_GET_REV          0x10
#define SI4713_CMD_POWER_DOWN       0x11
#define SI4713_CMD_SET_PROPERTY     0x12
#define SI4713_CMD_GET_PROPERTY     0x13
#define SI4713_CMD_GET_INT_STATUS   0x14
#define SI4713_CMD_TX_TUNE_FREQ     0x30
#define SI4713_CMD_TX_TUNE_POWER    0x31
#define SI4713_CMD_TX_TUNE_MEASURE  0x32
#define SI4713_CMD_TX_TUNE_STATUS   0x33
#define SI4713_CMD_TX_ASQ_STATUS    0x34
#define SI4713_CMD_TX_RDS_BUFF      0x35
#define SI4713_CMD_TX_RDS_PS        0x36
#define SI4713_CMD_GPIO_CTL         0x80
#define SI4713_CMD_GPIO_SET         0x81

//Si4713 Properties from AN332.pdf
#define SI4713_PROP_GPO_IEN                     0x0001
#define SI4713_PROP_DIGITAL_INPUT_FORMAT        0x0101
#define SI4713_PROP_DIGITAL_INPUT_SAMPLE_RATE   0x0103
#define SI4713_PROP_REFCLK_FREQ                 0x0201
#define SI4713_PROP_REFCLK_PRESCALE             0x0202
#define SI4713_PROP_TX_COMPONENT_ENABLE         0x2100
#define SI4713_PROP_TX_AUDIO_DEVIATION          0x2101
#define SI4713_PROP_TX_PILOT_DEVIATION          0x2102
#define SI4713_PROP_TX_RDS_DEVIATION            0x2103
#define SI4713_PROP_TX_LINE_LEVEL_INPUT_LEVEL   0x2104
#define SI4713_PROP_TX_LINE_INPUT_MUTE          0x2105
#define SI4713_PROP_TX_PREEMPHASIS              0x2106
#define SI4713_PROP_TX_PILOT_FREQUENCY          0x2107
#define SI4713_PROP_TX_ACOMP_ENABLE             0x2200
#define SI4713_PROP_TX_ACOMP_THRESHOLD          0x2201
#define SI4713_PROP_TX_ATTACK_TIME              0x2202
#define SI4713_PROP_TX_RELEASE_TIME             0x2203
#define SI4713_PROP_TX_ACOMP_GAIN               0x2204
#define SI4713_PROP_TX_LIMITER_RELEASE_TIME     0x2205
#define SI4713_PROP_TX_ASQ_INTERRUPT_SOURCE     0x2300
#define SI4713_PROP_TX_ASQ_LEVEL_LOW            0x2301
#define SI4713_PROP_TX_ASQ_DURATION_LOW         0x2302
#define SI4713_PROP_TX_AQS_LEVEL_HIGH           0x2303
#define SI4713_PROP_TX_AQS_DURATION_HIGH        0x2304
#define SI4713_PROP_TX_RDS_INTERRUPT_SOURCE     0x2C00
#define SI4713_PROP_TX_RDS_PI                   0x2C01
#define SI4713_PROP_TX_RDS_PS_MIX               0x2C02
#define SI4713_PROP_TX_RDS_PS_MISC              0x2C03
#define SI4713_PROP_TX_RDS_PS_REPEAT_COUNT      0x2C04
#define SI4713_PROP_TX_RDS_MESSAGE_COUNT        0x2C05
#define SI4713_PROP_TX_RDS_PS_AF                0x2C06
#define SI4713_PROP_TX_RDS_FIFO_SIZE            0x2C07

#define MASK_I2C_CTS                            0x80 // Clear To Send
#define MASK_I2C_ERR                            0x40
#define MASK_IRQ_RDS                            0x04
#define MASK_IRQ_ASQ                            0x02
#define MASK_IRQ_STC                            0x01

#define SIZE_CMD                                1
#define SIZE_STATUS                             1

#define SIZE_POWER_UP                           SIZE_CMD + 2
#define SIZE_POWER_UP_RESULT                    SIZE_STATUS
#define MASK_POWER_UP_CTS_IRQ_ENABLE            0x80
#define MASK_POWER_UP_GPIO2_OUTPUT_ENABLE       0x40
#define MASK_POWER_UP_PATCH                     0x20
#define MASK_POWER_UP_X_OSC_ENABLE              0x10
#define MASK_POWER_UP_TRANSMIT                  0x02
#define POWER_UP_OP_MODE_ANALOG                 0x50
#define POWER_UP_OP_MODE_DIGITAL                0x0F

#define SIZE_GET_REV                            SIZE_CMD
#define SIZE_GET_REV_RESULT                     SIZE_STATUS + 8

#define SIZE_POWER_DOWN                         SIZE_CMD
#define SIZE_POWER_DOWN_RESULT                  SIZE_STATUS

#define SIZE_SET_PROPERTY                       SIZE_CMD + 5
#define SIZE_SET_PROPERTY_RESULT                SIZE_STATUS

#define SIZE_GET_PROPERTY                       SIZE_CMD + 3
#define SIZE_GET_PROPERTY_RESULT                SIZE_STATUS + 3

#define SIZE_GET_INT_STATUS                     SIZE_CMD
#define SIZE_GET_INT_STATUS_RESULT              SIZE_STATUS

#define SIZE_TX_TUNE_FREQ                       SIZE_CMD + 3
#define SIZE_TX_TUNE_FREQ_RESULT                SIZE_STATUS

#define SIZE_TX_TUNE_POWER                      SIZE_CMD + 4
#define SIZE_TX_TUNE_POWER_RESULT               SIZE_STATUS

#define SIZE_TX_TUNE_MEASURE                    SIZE_CMD + 4
#define SIZE_TX_TUNE_MEASURE_RESULT             SIZE_STATUS

#define SIZE_TX_TUNE_STATUS                     SIZE_CMD + 1
#define SIZE_TX_TUNE_STATUS_RESULT              SIZE_STATUS + 7

#define SIZE_TX_ASQ_STATUS                      SIZE_CMD + 1
#define SIZE_TX_ASQ_STATUS_RESULT               SIZE_STATUS + 4

#define SIZE_TX_RDS_BUFFER                      SIZE_CMD + 7
#define SIZE_TX_RDS_BUFFER_RESULT               SIZE_STATUS + 5
#define MASK_TX_RDS_BUFFER_FIFO_ENABLE          0x80
#define MASK_TX_RDS_BUFFER_LOAD_BUFFER          0x04
#define MASK_TX_RDS_BUFFER_CLEAR_BUFFER         0x02
#define MASK_TX_RDS_BUFFER_CLEAR_RDS_IRQ        0x01
#define MASK_TX_RDS_BUFFER_RDSPSXMIT            0x10
#define MASK_TX_RDS_BUFFER_CBUFXMIT             0x08
#define MASK_TX_RDS_BUFFER_FIFOXMIT             0x04
#define MASK_TX_RDS_BUFFER_CBUFWRAP             0x02
#define MASK_TX_RDS_BUFFER_EMPTY_FIFO           0x01

#define SIZE_TX_RDS_PS                          SIZE_CMD + 5
#define SIZE_TX_RDS_PS_RESULT                   SIZE_STATUS

#define SIZE_GPIO_CTL                           SIZE_CMD + 1
#define SIZE_GPIO_CTL_RESULT                    SIZE_STATUS

#define SIZE_GPIO_SET                           SIZE_CMD + 1
#define SIZE_GPIO_SET_RESULT                    SIZE_STATUS

#define MASK_GPIO3_OUTPUT_ENABLE                0x08
#define MASK_GPIO2_OUTPUT_ENABLE                0x04
#define MASK_GPIO1_OUTPUT_ENABLE                0x02

#define SI4713_I2C_ADDR0                        0x11  // CS LOW
#define SI4713_I2C_ADDR1                        0x63  // CS HIGH (default)

#define SI4713_I2C_BUFF_MAX_SIZE                SIZE_GET_REV_RESULT
#define SI4713_I2C_MAX_RETRY_COUNT              100

#define FM_BAND_LIMIT_LO                        8750
#define FM_BAND_LIMIT_HI                        10800

#define TX_POWER_LIMIT_LO                       1
#define TX_POWER_LIMIT_HI                       121
#define ANT_CAPACITOR_LIMIT_HI                  191

#define MASK_TX_COMPONENT_ENABLE_RDS            0x04
#define MASK_TX_COMPONENT_ENABLE_STEREO         0x02
#define MASK_TX_COMPONENT_ENABLE_PILOT_TONE     0x01

#define MASK_TX_RDS_MISC_DYN_PTY_CODE           0x8000
#define MASK_TX_RDS_MISC_COMPRESSED_CODE        0x4000
#define MASK_TX_RDS_MISC_ARTIFICIAL_HEAD_CODE   0x2000
#define MASK_TX_RDS_MISC_STEREO                 0x1000
#define MASK_TX_RDS_MISC_FORCE_TP_PTY_BLOCK_B   0x0800
#define MASK_TX_RDS_MISC_TP                     0x0400
#define MASK_TX_RDS_MISC_PTY                    0x03E0
#define MASK_TX_RDS_MISC_TA                     0x0010
#define MASK_TX_RDS_MISC_RDSMS                  0x0008 //Spech or Music

#define TX_RDS_PS_AF_OFF                        0xE0E0
#define TX_RDS_PS_AF_MIN                        0xE0E1  //87.6 MHz
#define TX_RDS_PS_AF_MAX                        0xE1CC  //107.9 MHz

#define TX_PILOT_FREQUENCY                      19000

///MACROS
#define HI_BYTE(X)      (uint8_t) ((X & 0xFF00) >> 8)
#define LO_BYTE(X)      (uint8_t) (X & 0x00FF)
#define PACK16BITS(X,Y) (((uint16_t) X) << 8 | ((uint16_t) Y))
#define STRINGMHz(X)    String(((float) X)/100) + "MHz "
#define COMPL_MASK(X)   (0xFFFF -((uint16_t) X))

typedef struct TuneSts_t
{
    uint16_t    frequency;
    uint8_t     outputLevel;
    float       capacitor;
    uint8_t     noiseLevel;
} TuneSts_t;

typedef struct AsqSts_t
{
    bool        overmod;
    bool        ia_hl;
    bool        ia_ll;
    uint8_t     inputLevel;
} AsqSts_t;

class Si4713
{
    uint8_t m_i2c_buffer[SI4713_I2C_BUFF_MAX_SIZE];
    uint8_t m_rst_pin;

protected:
    uint8_t m_i2c_addr;

public:
    Si4713();
    bool begin(uint8_t a_scl_pin  = D2, uint8_t a_sda_pin = D1, uint8_t a_i2c_addr = SI4713_I2C_ADDR1, uint8_t a_rst_pin = D3);

protected:
    bool        Cmd_Power_up(uint8_t a_mask, uint8_t a_op_mode);
    uint8_t     Cmd_Get_rev();
    bool        Cmd_Power_down();
    bool        Cmd_Set_Property(uint16_t a_property, uint16_t a_value);
    uint16_t    Cmd_Get_Property(uint16_t a_property);
    uint8_t     Cmd_Get_Int_Status();
    bool        Cmd_Tx_Tune_Freq(uint16_t a_freq);
    bool        Cmd_Tx_Tune_Power(uint8_t a_level, uint8_t a_ant_cap = 0);
    bool        Cmd_Tx_Tune_Measure(uint16_t a_freq, uint8_t a_ant_cap = 0);
    TuneSts_t   Cmd_Tx_Tune_Status(bool a_clear_irq = true);
    AsqSts_t    Cmd_Tx_ASQ_Status(bool a_clear_irq = true);
    bool        Cmd_Tx_RDS_Buff(uint8_t a_mode, uint16_t a_block_b, uint16_t a_block_c, uint16_t a_block_d);
    bool        Cmd_Tx_RDS_PS(uint8_t a_ps_id, char *p_ps_chars);
    bool        Cmd_GPIO_Ctl(uint8_t a_mask);
    bool        Cmd_GPIO_Set(uint8_t a_value);
    bool        waitOn(uint8_t a_mask);

private:
    void        clearI2cBuffer();
    void        getResult(uint8_t a_len);
    void        reset();
    bool        sendI2cCommand(uint8_t a_len);
};

#endif // SI4713_H
