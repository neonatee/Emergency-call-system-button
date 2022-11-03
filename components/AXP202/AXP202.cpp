/////////////////////////////////////////////////////////////////
/*
           __   _______ ___   ___ ___
     /\    \ \ / /  __ \__ \ / _ \__ \
    /  \    \ V /| |__) | ) | | | | ) |
   / /\ \    > < |  ___/ / /| | | |/ /
  / ____ \  / . \| |    / /_| |_| / /_
 /_/    \_\/_/ \_\_|   |____|\___/____|


MIT License

Copyright (c) 2019 lewis he

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

axp20x.cpp - Arduino library for X-Power AXP202 chip.
*/
/////////////////////////////////////////////////////////////////

#include "AXP202.h"
#include <math.h>

#define ISCONNECETD(ret) \
    do                   \
    {                    \
        if (!_init)      \
            return ret;  \
    } while (0)

const uint8_t AXP202::startupParams[] = {
    0b00000000,
    0b01000000,
    0b10000000,
    0b11000000};

const uint8_t AXP202::longPressParams[] = {
    0b00000000,
    0b00010000,
    0b00100000,
    0b00110000};

const uint8_t AXP202::shutdownParams[] = {
    0b00000000,
    0b00000001,
    0b00000010,
    0b00000011};

const uint8_t AXP202::targetVolParams[] = {
    0b00000000,
    0b00100000,
    0b01000000,
    0b01100000};

// Power Output Control register
uint8_t AXP202::_outputReg;

int AXP202::_axp_probe(void)
{
    uint8_t data;

    _readByte(AXP202_IC_TYPE, 1, &_chip_id);
    AXP_DEBUG("chip id detect 0x%x\n", _chip_id);
    // Serial.println("_chip_id");
    // Serial.println(_chip_id);
    if (_chip_id)
    {
        _readByte(AXP202_LDO234_DC23_CTL, 1, &_outputReg);
        AXP_DEBUG("OUTPUT Register 0x%x\n", _outputReg);
        _init = true;
        return AXP_PASS;
    }
    return AXP_FAIL;
}

#ifdef ARDUINO
int AXP202::init(TwoWire &port, uint8_t addr, bool isAxp173)
{
    _i2cPort = &port; // Grab which port the user wants us to use
    _address = addr;
    _isAxp173 = isAxp173;

    return _axp_probe();
}
#endif

int AXP202::init(axp_com_fptr_t read_cb, axp_com_fptr_t write_cb, uint8_t addr, bool isAxp173)
{
    if (read_cb == nullptr || write_cb == nullptr)
        return AXP_FAIL;
    _read_cb = read_cb;
    _write_cb = write_cb;
    _address = addr;
    _isAxp173 = isAxp173;
    return _axp_probe();
}

bool AXP202::isExtenEnable(void)
{
    return IS_OPEN(_outputReg, AXP202_EXTEN);
}

bool AXP202::isLDO2Enable(void)
{
    return IS_OPEN(_outputReg, AXP202_LDO2);
}

bool AXP202::isLDO3Enable(void)
{
    return IS_OPEN(_outputReg, AXP202_LDO3);
}

bool AXP202::isLDO4Enable(void)
{
    return IS_OPEN(_outputReg, AXP202_LDO4);
}

bool AXP202::isDCDC2Enable(void)
{
    return IS_OPEN(_outputReg, AXP202_DCDC2);
}

bool AXP202::isDCDC3Enable(void)
{
    return IS_OPEN(_outputReg, AXP202_DCDC3);
}

int AXP202::setPowerOutPut(uint8_t ch, bool en)
{
    uint8_t data;
    uint8_t val = 0;
    if (!_init)
        return AXP_NOT_INIT;

    _readByte(AXP202_LDO234_DC23_CTL, 1, &data);
    if (en)
    {
        data |= (1 << ch);
    }
    else
    {
        data &= (~(1 << ch));
    }

    if (ch == AXP202_DCDC3)
    {
        FORCED_OPEN_DCDC3(data); //! Must be forced open in T-Watch
    }

    _writeByte(AXP202_LDO234_DC23_CTL, 1, &data);

#ifdef ARDUINO
    delay(1);
#endif
    _readByte(AXP202_LDO234_DC23_CTL, 1, &val);
    if (data == val)
    {
        _outputReg = val;
        return AXP_PASS;
    }
    return AXP_FAIL;
}

bool AXP202::isChargeing(void)
{
    uint8_t reg;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_MODE_CHGSTATUS, 1, &reg);
    return IS_OPEN(reg, 6);
}

bool AXP202::isBatteryConnect(void)
{
    uint8_t reg;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_MODE_CHGSTATUS, 1, &reg);
    return IS_OPEN(reg, 5);
}

float AXP202::getAcinVoltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_ACIN_VOL_H8, AXP202_ACIN_VOL_L4) * AXP202_ACIN_VOLTAGE_STEP;
}

float AXP202::getAcinCurrent(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_ACIN_CUR_H8, AXP202_ACIN_CUR_L4) * AXP202_ACIN_CUR_STEP;
}

float AXP202::getVbusVoltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_VBUS_VOL_H8, AXP202_VBUS_VOL_L4) * AXP202_VBUS_VOLTAGE_STEP;
}

float AXP202::getVbusCurrent(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_VBUS_CUR_H8, AXP202_VBUS_CUR_L4) * AXP202_VBUS_CUR_STEP;
}

float AXP202::getTemp(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    // Internal temperature
    // 000H => -144.7℃
    // STEP => 0.1℃
    // FFFH => 264.8℃
    return _getRegistResult(AXP202_INTERNAL_TEMP_H8, AXP202_INTERNAL_TEMP_L4) * AXP202_INTERNAL_TEMP_STEP - 144.7;
}

float AXP202::getTSTemp(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_TS_IN_H8, AXP202_TS_IN_L4) * AXP202_TS_PIN_OUT_STEP;
}

float AXP202::getGPIO0Voltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_GPIO0_VOL_ADC_H8, AXP202_GPIO0_VOL_ADC_L4) * AXP202_GPIO0_STEP;
}

float AXP202::getGPIO1Voltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_GPIO1_VOL_ADC_H8, AXP202_GPIO1_VOL_ADC_L4) * AXP202_GPIO1_STEP;
}

/*
Note: the battery power formula:
Pbat =2* register value * Voltage LSB * Current LSB / 1000.
(Voltage LSB is 1.1mV; Current LSB is 0.5mA, and unit of calculation result is mW.)
*/
float AXP202::getBattInpower(void)
{
    float rslt;
    uint8_t hv, mv, lv;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_BAT_POWERH8, 1, &hv);
    _readByte(AXP202_BAT_POWERM8, 1, &mv);
    _readByte(AXP202_BAT_POWERL8, 1, &lv);
    rslt = (hv << 16) | (mv << 8) | lv;
    rslt = 2 * rslt * 1.1 * 0.5 / 1000;
    return rslt;
}

float AXP202::getBattVoltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_BAT_AVERVOL_H8, AXP202_BAT_AVERVOL_L4) * AXP202_BATT_VOLTAGE_STEP;
}

float AXP202::getBattChargeCurrent(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    else
        return _getRegistResult(AXP202_BAT_AVERCHGCUR_H8, AXP202_BAT_AVERCHGCUR_L4) * AXP202_BATT_CHARGE_CUR_STEP;
}

float AXP202::getBattDischargeCurrent(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistH8L5(AXP202_BAT_AVERDISCHGCUR_H8, AXP202_BAT_AVERDISCHGCUR_L5) * AXP202_BATT_DISCHARGE_CUR_STEP;
}

float AXP202::getSysIPSOUTVoltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_APS_AVERVOL_H8, AXP202_APS_AVERVOL_L4);
}

/*
Coulomb calculation formula:
C= 65536 * current LSB *（charge coulomb counter value - discharge coulomb counter value） /
3600 / ADC sample rate. Refer to REG84H setting for ADC sample rate；the current LSB is
0.5mA；unit of the calculation result is mAh. ）
*/
uint32_t AXP202::getBattChargeCoulomb(void)
{
    uint8_t buffer[4];
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(0xB0, 4, buffer);
    return (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
}

uint32_t AXP202::getBattDischargeCoulomb(void)
{
    uint8_t buffer[4];
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(0xB4, 4, buffer);
    return (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
}

float AXP202::getCoulombData(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint32_t charge = getBattChargeCoulomb(), discharge = getBattDischargeCoulomb();
    uint8_t rate = getAdcSamplingRate();
    float result = 65536.0 * 0.5 * ((float)charge - (float)discharge) / 3600.0 / rate;
    return result;
}

//-------------------------------------------------------
// New Coulomb functions  by MrFlexi
//-------------------------------------------------------

uint8_t AXP202::getCoulombRegister(void)
{
    uint8_t buffer;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_COULOMB_CTL, 1, &buffer);
    return buffer;
}

int AXP202::setCoulombRegister(uint8_t val)
{
    if (!_init)
        return AXP_NOT_INIT;
    _writeByte(AXP202_COULOMB_CTL, 1, &val);
    return AXP_PASS;
}

int AXP202::EnableCoulombcounter(void)
{

    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val = 0x80;
    _writeByte(AXP202_COULOMB_CTL, 1, &val);
    return AXP_PASS;
}

int AXP202::DisableCoulombcounter(void)
{

    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val = 0x00;
    _writeByte(AXP202_COULOMB_CTL, 1, &val);
    return AXP_PASS;
}

int AXP202::StopCoulombcounter(void)
{

    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val = 0xB8;
    _writeByte(AXP202_COULOMB_CTL, 1, &val);
    return AXP_PASS;
}

int AXP202::ClearCoulombcounter(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val = 0xA0;
    _writeByte(AXP202_COULOMB_CTL, 1, &val);
    return AXP_PASS;
}

//-------------------------------------------------------
// END
//-------------------------------------------------------

uint8_t AXP202::getAdcSamplingRate(void)
{
    // axp192 same axp202 aregister address 0x84
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val;
    _readByte(AXP202_ADC_SPEED, 1, &val);
    return 25 * (int)pow(2, (val & 0xC0) >> 6);
}

int AXP202::setAdcSamplingRate(axp_adc_sampling_rate_t rate)
{
    // axp192 same axp202 aregister address 0x84
    if (!_init)
        return AXP_NOT_INIT;
    if (rate > AXP_ADC_SAMPLING_RATE_200HZ)
        return AXP_FAIL;
    uint8_t val;
    _readByte(AXP202_ADC_SPEED, 1, &val);
    uint8_t rw = rate;
    val &= 0x3F;
    val |= (rw << 6);
    _writeByte(AXP202_ADC_SPEED, 1, &val);
    return AXP_PASS;
}

int AXP202::setTSfunction(axp_ts_pin_function_t func)
{
    // axp192 same axp202 aregister address 0x84
    if (!_init)
        return AXP_NOT_INIT;
    if (func > AXP_TS_PIN_FUNCTION_ADC)
        return AXP_FAIL;
    uint8_t val;
    _readByte(AXP202_ADC_SPEED, 1, &val);
    uint8_t rw = func;
    val &= 0xFA;
    val |= (rw << 2);
    _writeByte(AXP202_ADC_SPEED, 1, &val);
    return AXP_PASS;
}

int AXP202::setTScurrent(axp_ts_pin_current_t current)
{
    // axp192 same axp202 aregister address 0x84
    if (!_init)
        return AXP_NOT_INIT;
    if (current > AXP_TS_PIN_CURRENT_80UA)
        return AXP_FAIL;
    uint8_t val;
    _readByte(AXP202_ADC_SPEED, 1, &val);
    uint8_t rw = current;
    val &= 0xCF;
    val |= (rw << 4);
    _writeByte(AXP202_ADC_SPEED, 1, &val);
    return AXP_PASS;
}

int AXP202::setTSmode(axp_ts_pin_mode_t mode)
{
    // axp192 same axp202 aregister address 0x84
    if (!_init)
        return AXP_NOT_INIT;
    if (mode > AXP_TS_PIN_MODE_ENABLE)
        return AXP_FAIL;
    uint8_t val;
    _readByte(AXP202_ADC_SPEED, 1, &val);
    uint8_t rw = mode;
    val &= 0xFC;
    val |= rw;
    _writeByte(AXP202_ADC_SPEED, 1, &val);

    // TS pin ADC function enable/disable
    if (mode == AXP_TS_PIN_MODE_DISABLE)
        adc1Enable(AXP202_TS_PIN_ADC1, false);
    else
        adc1Enable(AXP202_TS_PIN_ADC1, true);
    return AXP_PASS;
}

int AXP202::adc1Enable(uint16_t params, bool en)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val;
    _readByte(AXP202_ADC_EN1, 1, &val);
    if (en)
        val |= params;
    else
        val &= ~(params);
    _writeByte(AXP202_ADC_EN1, 1, &val);
    return AXP_PASS;
}

int AXP202::adc2Enable(uint16_t params, bool en)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val;
    _readByte(AXP202_ADC_EN2, 1, &val);
    if (en)
        val |= params;
    else
        val &= ~(params);
    _writeByte(AXP202_ADC_EN2, 1, &val);
    return AXP_PASS;
}

int AXP202::enableIRQ(uint64_t params, bool en)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val, val1;
    if (params & 0xFFUL)
    {
        val1 = params & 0xFF;
        _readByte(AXP202_INTEN1, 1, &val);
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN1, val);
        _writeByte(AXP202_INTEN1, 1, &val);
    }
    if (params & 0xFF00UL)
    {
        val1 = params >> 8;
        _readByte(AXP202_INTEN2, 1, &val);
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN2, val);
        _writeByte(AXP202_INTEN2, 1, &val);
    }

    if (params & 0xFF0000UL)
    {
        val1 = params >> 16;
        _readByte(AXP202_INTEN3, 1, &val);
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN3, val);
        _writeByte(AXP202_INTEN3, 1, &val);
    }

    if (params & 0xFF000000UL)
    {
        val1 = params >> 24;
        _readByte(AXP202_INTEN4, 1, &val);
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN4, val);
        _writeByte(AXP202_INTEN4, 1, &val);
    }

    if (params & 0xFF00000000ULL)
    {
        val1 = params >> 32;
        uint8_t reg = AXP202_INTEN5;
        _readByte(reg, 1, &val);
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", reg, val);
        _writeByte(reg, 1, &val);
    }
    return AXP_PASS;
}

int AXP202::readIRQ(void)
{
    if (!_init)
        return AXP_NOT_INIT;

    for (int i = 0; i < 5; ++i)
    {
        _readByte(AXP202_INTSTS1 + i, 1, &_irq[i]);
    }
    return AXP_PASS;
}

void AXP202::clearIRQ(void)
{
    uint8_t val = 0xFF;

    for (int i = 0; i < 5; i++)
    {
        _writeByte(AXP202_INTSTS1 + i, 1, &val);
    }

    memset(_irq, 0, sizeof(_irq));
}

bool AXP202::isVBUSPlug(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t reg;
    _readByte(AXP202_STATUS, 1, &reg);
    return IS_OPEN(reg, 5);
}

// IRQ1 REGISTER : AXP202:0x40H AXP192:0X44H
bool AXP202::isAcinOverVoltageIRQ(void)
{
    return (bool)(_irq[0] & _BV(7));
}

bool AXP202::isAcinPlugInIRQ(void)
{
    return (bool)(_irq[0] & _BV(6));
}

bool AXP202::isAcinRemoveIRQ(void)
{
    return (bool)(_irq[0] & _BV(5));
}

bool AXP202::isVbusOverVoltageIRQ(void)
{
    return (bool)(_irq[0] & _BV(4));
}

bool AXP202::isVbusPlugInIRQ(void)
{
    return (bool)(_irq[0] & _BV(3));
}

bool AXP202::isVbusRemoveIRQ(void)
{
    return (bool)(_irq[0] & _BV(2));
}

bool AXP202::isVbusLowVHOLDIRQ(void)
{
    return (bool)(_irq[0] & _BV(1));
}

// IRQ2 REGISTER : AXP202:0x41H AXP192:0X45H
bool AXP202::isBattPlugInIRQ(void)
{
    return (bool)(_irq[1] & _BV(7));
}

bool AXP202::isBattRemoveIRQ(void)
{
    return (bool)(_irq[1] & _BV(6));
}

bool AXP202::isBattEnterActivateIRQ(void)
{
    return (bool)(_irq[1] & _BV(5));
}

bool AXP202::isBattExitActivateIRQ(void)
{
    return (bool)(_irq[1] & _BV(4));
}

bool AXP202::isChargingIRQ(void)
{
    return (bool)(_irq[1] & _BV(3));
}

bool AXP202::isChargingDoneIRQ(void)
{
    return (bool)(_irq[1] & _BV(2));
}

bool AXP202::isBattTempHighIRQ(void)
{
    return (bool)(_irq[1] & _BV(1));
}

bool AXP202::isBattTempLowIRQ(void)
{
    return (bool)(_irq[1] & _BV(0));
}

// IRQ3 REGISTER : AXP202:0x42H AXP192:0X46H
bool AXP202::isChipOvertemperatureIRQ(void)
{
    return (bool)(_irq[2] & _BV(7));
}

bool AXP202::isChargingCurrentLessIRQ(void)
{
    return (bool)(_irq[2] & _BV(6));
}

// retention bit5

bool AXP202::isDC2VoltageLessIRQ(void)
{
    return (bool)(_irq[2] & _BV(4));
}

bool AXP202::isDC3VoltageLessIRQ(void)
{
    return (bool)(_irq[2] & _BV(3));
}

bool AXP202::isLDO3VoltageLessIRQ(void)
{
    return (bool)(_irq[2] & _BV(2));
}

bool AXP202::isPEKShortPressIRQ(void)
{
    return (bool)(_irq[2] & _BV(1));
}

bool AXP202::isPEKLongtPressIRQ(void)
{
    return (bool)(_irq[2] & _BV(0));
}

// IRQ4 REGISTER : AXP202:0x43H AXP192:0X47H
bool AXP202::isNOEPowerOnIRQ(void)
{
    return (bool)(_irq[3] & _BV(7));
}

bool AXP202::isNOEPowerDownIRQ(void)
{
    return (bool)(_irq[3] & _BV(6));
}

bool AXP202::isVBUSEffectiveIRQ(void)
{
    return (bool)(_irq[3] & _BV(5));
}

bool AXP202::isVBUSInvalidIRQ(void)
{
    return (bool)(_irq[3] & _BV(4));
}

bool AXP202::isVUBSSessionIRQ(void)
{
    return (bool)(_irq[3] & _BV(3));
}

bool AXP202::isVUBSSessionEndIRQ(void)
{
    return (bool)(_irq[3] & _BV(2));
}

bool AXP202::isLowVoltageLevel1IRQ(void)
{
    return (bool)(_irq[3] & _BV(1));
}

bool AXP202::isLowVoltageLevel2IRQ(void)
{
    return (bool)(_irq[3] & _BV(0));
}

// IRQ5 REGISTER : AXP202:0x44H AXP192:0X4DH
bool AXP202::isTimerTimeoutIRQ(void)
{
    return (bool)(_irq[4] & _BV(7));
}

bool AXP202::isPEKRisingEdgeIRQ(void)
{
    return (bool)(_irq[4] & _BV(6));
}

bool AXP202::isPEKFallingEdgeIRQ(void)
{
    return (bool)(_irq[4] & _BV(5));
}

// retention bit4

bool AXP202::isGPIO3InputEdgeTriggerIRQ(void)
{
    return (bool)(_irq[4] & _BV(3));
}

bool AXP202::isGPIO2InputEdgeTriggerIRQ(void)
{
    return (bool)(_irq[4] & _BV(2));
}

bool AXP202::isGPIO1InputEdgeTriggerIRQ(void)
{
    return (bool)(_irq[4] & _BV(1));
}

bool AXP202::isGPIO0InputEdgeTriggerIRQ(void)
{
    return (bool)(_irq[4] & _BV(0));
}

int AXP202::setDCDC2Voltage(uint16_t mv)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (mv < 700)
    {
        AXP_DEBUG("DCDC2:Below settable voltage:700mV~2275mV");
        mv = 700;
    }
    if (mv > 2275)
    {
        AXP_DEBUG("DCDC2:Above settable voltage:700mV~2275mV");
        mv = 2275;
    }
    uint8_t val = (mv - 700) / 25;
    //! axp173/192/202 same register
    _writeByte(AXP202_DC2OUT_VOL, 1, &val);
    return AXP_PASS;
}

uint16_t AXP202::getDCDC2Voltage(void)
{
    uint8_t val = 0;
    //! axp173/192/202 same register
    _readByte(AXP202_DC2OUT_VOL, 1, &val);
    return val * 25 + 700;
}

uint16_t AXP202::getDCDC3Voltage(void)
{
    if (!_init)
        return 0;
    uint8_t val = 0;
    _readByte(AXP202_DC3OUT_VOL, 1, &val);
    return val * 25 + 700;
}

int AXP202::setDCDC3Voltage(uint16_t mv)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (mv < 700)
    {
        AXP_DEBUG("DCDC3:Below settable voltage:700mV~3500mV");
        mv = 700;
    }
    if (mv > 3500)
    {
        AXP_DEBUG("DCDC3:Above settable voltage:700mV~3500mV");
        mv = 3500;
    }
    uint8_t val = (mv - 700) / 25;
    _writeByte(AXP202_DC3OUT_VOL, 1, &val);
    return AXP_PASS;
}

int AXP202::setLDO2Voltage(uint16_t mv)
{
    uint8_t rVal, wVal;
    if (!_init)
        return AXP_NOT_INIT;
    if (mv < 1800)
    {
        AXP_DEBUG("LDO2:Below settable voltage:1800mV~3300mV");
        mv = 1800;
    }
    if (mv > 3300)
    {
        AXP_DEBUG("LDO2:Above settable voltage:1800mV~3300mV");
        mv = 3300;
    }
    wVal = (mv - 1800) / 100;

    _readByte(AXP202_LDO24OUT_VOL, 1, &rVal);
    rVal &= 0x0F;
    rVal |= (wVal << 4);
    _writeByte(AXP202_LDO24OUT_VOL, 1, &rVal);
    return AXP_PASS;
}

uint16_t AXP202::getLDO2Voltage(void)
{
    uint8_t rVal;

    _readByte(AXP202_LDO24OUT_VOL, 1, &rVal);
    rVal &= 0xF0;
    rVal >>= 4;
    return rVal * 100 + 1800;
}

int AXP202::setLDO3Voltage(uint16_t mv)
{
    uint8_t rVal;
    if (!_init)
        return AXP_NOT_INIT;
    if (mv < 700)
    {
        AXP_DEBUG("LDO3:Below settable voltage:700mV~3500mV");
        mv = 700;
    }

    if (mv > 3500)
    {
        AXP_DEBUG("LDO3:Above settable voltage:700mV~3500mV");
        mv = 3500;
    }

    _readByte(AXP202_LDO3OUT_VOL, 1, &rVal);
    rVal &= 0x80;
    rVal |= ((mv - 700) / 25);
    _writeByte(AXP202_LDO3OUT_VOL, 1, &rVal);
    return AXP_PASS;
}

uint16_t AXP202::getLDO3Voltage(void)
{
    uint8_t rVal;
    if (!_init)
        return AXP_NOT_INIT;

    _readByte(AXP202_LDO3OUT_VOL, 1, &rVal);
    if (rVal & 0x80)
    {
        //! According to the hardware N_VBUSEN Pin selection
        return getVbusVoltage() * 1000;
    }
    else
    {
        return (rVal & 0x7F) * 25 + 700;
    }
}

uint16_t AXP202::getLDO4Voltage(void)
{
    const uint16_t ldo4_table[] = {1250, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2500, 2700, 2800, 3000, 3100, 3200, 3300};
    if (!_init)
        return 0;
    uint8_t val = 0;

    _readByte(AXP202_LDO24OUT_VOL, 1, &val);
    val &= 0xF;
    return ldo4_table[val];
}

//! Only axp202 support
int AXP202::setLDO4Voltage(axp_ldo4_table_t param)
{
    if (!_init)
        return AXP_NOT_INIT;

    if (param >= AXP202_LDO4_MAX)
        return AXP_INVALID;
    uint8_t val;
    _readByte(AXP202_LDO24OUT_VOL, 1, &val);
    val &= 0xF0;
    val |= param;
    _writeByte(AXP202_LDO24OUT_VOL, 1, &val);
    return AXP_PASS;
}

//! Only AXP202 support
int AXP202::setLDO3Mode(axp202_ldo3_mode_t mode)
{
    uint8_t val;

    _readByte(AXP202_LDO3OUT_VOL, 1, &val);
    if (mode)
    {
        val |= _BV(7);
    }
    else
    {
        val &= (~_BV(7));
    }
    _writeByte(AXP202_LDO3OUT_VOL, 1, &val);
    return AXP_PASS;
}

int AXP202::setStartupTime(uint8_t param)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    if (param > sizeof(startupParams) / sizeof(startupParams[0]))
        return AXP_INVALID;
    _readByte(AXP202_POK_SET, 1, &val);
    val &= (~0b11000000);
    val |= startupParams[param];
    _writeByte(AXP202_POK_SET, 1, &val);
    return AXP_PASS;
}

int AXP202::setlongPressTime(uint8_t param)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    if (param > sizeof(longPressParams) / sizeof(longPressParams[0]))
        return AXP_INVALID;
    _readByte(AXP202_POK_SET, 1, &val);
    val &= (~0b00110000);
    val |= longPressParams[param];
    _writeByte(AXP202_POK_SET, 1, &val);
    return AXP_PASS;
}

int AXP202::setShutdownTime(uint8_t param)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    if (param > sizeof(shutdownParams) / sizeof(shutdownParams[0]))
        return AXP_INVALID;
    _readByte(AXP202_POK_SET, 1, &val);
    val &= (~0b00000011);
    val |= shutdownParams[param];
    _writeByte(AXP202_POK_SET, 1, &val);
    return AXP_PASS;
}

int AXP202::setTimeOutShutdown(bool en)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_POK_SET, 1, &val);
    if (en)
        val |= (1 << 3);
    else
        val &= (~(1 << 3));
    _writeByte(AXP202_POK_SET, 1, &val);
    return AXP_PASS;
}

int AXP202::shutdown(void)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_OFF_CTL, 1, &val);
    val |= (1 << 7);
    _writeByte(AXP202_OFF_CTL, 1, &val);
    return AXP_PASS;
}

float AXP202::getSettingChargeCurrent(void)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_CHARGE1, 1, &val);
    val &= 0b00000111;
    float cur = 300.0 + val * 100.0;
    AXP_DEBUG("Setting Charge current : %.2f mA\n", cur);
    return cur;
}

bool AXP202::isChargeingEnable(void)
{
    uint8_t val;
    if (!_init)
        return false;
    _readByte(AXP202_CHARGE1, 1, &val);
    if (val & (1 << 7))
    {
        AXP_DEBUG("Charging enable is enable\n");
        val = true;
    }
    else
    {
        AXP_DEBUG("Charging enable is disable\n");
        val = false;
    }
    return val;
}

int AXP202::enableChargeing(bool en)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_CHARGE1, 1, &val);
    val = en ? (val | _BV(7)) : val & (~_BV(7));
    _writeByte(AXP202_CHARGE1, 1, &val);
    return AXP_PASS;
}

int AXP202::setChargingTargetVoltage(axp_chargeing_vol_t param)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    if (param > sizeof(targetVolParams) / sizeof(targetVolParams[0]))
        return AXP_INVALID;
    _readByte(AXP202_CHARGE1, 1, &val);
    val &= ~(0b01100000);
    val |= targetVolParams[param];
    _writeByte(AXP202_CHARGE1, 1, &val);
    return AXP_PASS;
}

int AXP202::getBattPercentage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val;
    if (!isBatteryConnect())
        return 0;
    _readByte(AXP202_BATT_PERCENTAGE, 1, &val);
    if (!(val & _BV(7)))
    {
        // Serial.println(val);
        return val & (~_BV(7));
    }
    return 0;
}

int AXP202::setMeteringSystem(bool en)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id != AXP202_CHIP_ID)
        return AXP_NOT_SUPPORT;
    uint8_t val = 0;
    _readByte(AXP202_BATT_PERCENTAGE, 1, &val);
    en ? (val |= _BV(7)) : (val &= (~_BV(7)));
    return AXP_PASS;
}

int AXP202::setChgLEDMode(axp_chgled_mode_t mode)
{
    uint8_t val;
    _readByte(AXP202_OFF_CTL, 1, &val);
    val &= 0b11001111;
    val |= _BV(3);
    switch (mode)
    {
    case AXP20X_LED_OFF:
        _writeByte(AXP202_OFF_CTL, 1, &val);
        break;
    case AXP20X_LED_BLINK_1HZ:
        val |= 0b00010000;
        _writeByte(AXP202_OFF_CTL, 1, &val);
        break;
    case AXP20X_LED_BLINK_4HZ:
        val |= 0b00100000;
        _writeByte(AXP202_OFF_CTL, 1, &val);
        break;
    case AXP20X_LED_LOW_LEVEL:
        val |= 0b00110000;
        _writeByte(AXP202_OFF_CTL, 1, &val);
        break;
    default:
        return AXP_FAIL;
    }
    return AXP_PASS;
}

int AXP202::debugCharging(void)
{
    uint8_t val;
    _readByte(AXP202_CHARGE1, 1, &val);
    AXP_DEBUG("SRC REG:0x%x\n", val);
    if (val & (1 << 7))
    {
        AXP_DEBUG("Charging enable is enable\n");
    }
    else
    {
        AXP_DEBUG("Charging enable is disable\n");
    }
    AXP_DEBUG("Charging target-voltage : 0x%x\n", ((val & 0b01100000) >> 5) & 0b11);
    if (val & (1 << 4))
    {
        AXP_DEBUG("end when the charge current is lower than 15%% of the set value\n");
    }
    else
    {
        AXP_DEBUG(" end when the charge current is lower than 10%% of the set value\n");
    }
    val &= 0b00000111;
    AXP_DEBUG("Charge current : %.2f mA\n", 300.0 + val * 100.0);
    return AXP_PASS;
}

int AXP202::debugStatus(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val, val1, val2;
    _readByte(AXP202_STATUS, 1, &val);
    _readByte(AXP202_MODE_CHGSTATUS, 1, &val1);
    _readByte(AXP202_IPS_SET, 1, &val2);
    AXP_DEBUG("AXP202_STATUS:   AXP202_MODE_CHGSTATUS   AXP202_IPS_SET\n");
    AXP_DEBUG("0x%x\t\t\t 0x%x\t\t\t 0x%x\n", val, val1, val2);
    return AXP_PASS;
}

int AXP202::limitingOff(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val;
    _readByte(AXP202_IPS_SET, 1, &val);
    if (_chip_id == AXP202_CHIP_ID)
    {
        val |= 0x03;
    }
    else
    {
        val &= ~(1 << 1);
    }
    _writeByte(AXP202_IPS_SET, 1, &val);
    return AXP_PASS;
}

/***********************************************
 *              !!! TIMER FUNCTION !!!
 * *********************************************/

int AXP202::setTimer(uint8_t minutes)
{
    if (!_init)
        return AXP_NOT_INIT;

    if (minutes > 63)
    {
        return AXP_ARG_INVALID;
    }
    minutes |= 0x80; // Clear timer flag
    _writeByte(AXP202_TIMER_CTL, 1, &minutes);
    return AXP_PASS;
}

int AXP202::offTimer(void)
{
    if (!_init)
        return AXP_NOT_INIT;

    uint8_t minutes = 0x80;
    _writeByte(AXP202_TIMER_CTL, 1, &minutes);
    return AXP_PASS;
}

int AXP202::clearTimerStatus(void)
{
    if (!_init)
        return AXP_NOT_INIT;

    uint8_t val;
    _readByte(AXP202_TIMER_CTL, 1, &val);
    val |= 0x80;
    _writeByte(AXP202_TIMER_CTL, 1, &val);
    return AXP_PASS;
}

bool AXP202::getTimerStatus(void)
{
    if (!_init)
        return AXP_NOT_INIT;

    uint8_t val;
    _readByte(AXP202_TIMER_CTL, 1, &val);
    return (val & 0x80) >> 7;
}

/***********************************************
 *              !!! GPIO FUNCTION !!!
 * *********************************************/

int AXP202::_axp202_gpio_0_select(axp_gpio_mode_t mode)
{
    switch (mode)
    {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0;
    case AXP_IO_OUTPUT_HIGH_MODE:
        return 1;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_LDO_MODE:
        return 3;
    case AXP_IO_ADC_MODE:
        return 4;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP202::_axp202_gpio_1_select(axp_gpio_mode_t mode)
{
    switch (mode)
    {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0;
    case AXP_IO_OUTPUT_HIGH_MODE:
        return 1;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_ADC_MODE:
        return 4;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP202::_axp202_gpio_2_select(axp_gpio_mode_t mode)
{
    switch (mode)
    {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_FLOATING_MODE:
        return 1;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP202::_axp202_gpio_3_select(axp_gpio_mode_t mode)
{
    switch (mode)
    {
    case AXP_IO_INPUT_MODE:
        return 1;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 0;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP202::_axp202_gpio_set(axp_gpio_t gpio, axp_gpio_mode_t mode)
{
    uint8_t val;
    int rslt;
    switch (gpio)
    {
    case AXP_GPIO_0:
    {
        rslt = _axp202_gpio_0_select(mode);
        if (rslt < 0)
            return rslt;
        _readByte(AXP202_GPIO0_CTL, 1, &val);
        val &= 0b11111000;
        val |= (uint8_t)rslt;
        _writeByte(AXP202_GPIO0_CTL, 1, &val);
        return AXP_PASS;
    }
    case AXP_GPIO_1:
    {
        rslt = _axp202_gpio_1_select(mode);
        if (rslt < 0)
            return rslt;
        _readByte(AXP202_GPIO1_CTL, 1, &val);
        val &= 0b11111000;
        val |= (uint8_t)rslt;
        _writeByte(AXP202_GPIO1_CTL, 1, &val);
        return AXP_PASS;
    }
    case AXP_GPIO_2:
    {
        rslt = _axp202_gpio_2_select(mode);
        if (rslt < 0)
            return rslt;
        _readByte(AXP202_GPIO2_CTL, 1, &val);
        val &= 0b11111000;
        val |= (uint8_t)rslt;
        _writeByte(AXP202_GPIO2_CTL, 1, &val);
        return AXP_PASS;
    }
    case AXP_GPIO_3:
    {
        rslt = _axp202_gpio_3_select(mode);
        if (rslt < 0)
            return rslt;
        _readByte(AXP202_GPIO3_CTL, 1, &val);
        val = rslt ? (val | _BV(2)) : (val & (~_BV(2)));
        _writeByte(AXP202_GPIO3_CTL, 1, &val);
        return AXP_PASS;
    }
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP202::setGPIOMode(axp_gpio_t gpio, axp_gpio_mode_t mode)
{
    if (!_init)
        return AXP_NOT_INIT;

    return _axp202_gpio_set(gpio, mode);
}

int AXP202::_axp_irq_mask(axp_gpio_irq_t irq)
{
    switch (irq)
    {
    case AXP_IRQ_NONE:
        return 0;
    case AXP_IRQ_RISING:
        return _BV(7);
    case AXP_IRQ_FALLING:
        return _BV(6);
    case AXP_IRQ_DOUBLE_EDGE:
        return 0b1100000;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP202::_axp202_gpio_irq_set(axp_gpio_t gpio, axp_gpio_irq_t irq)
{
    uint8_t reg;
    uint8_t val;
    int mask;
    mask = _axp_irq_mask(irq);

    if (mask < 0)
        return mask;
    switch (gpio)
    {
    case AXP_GPIO_0:
        reg = AXP202_GPIO0_CTL;
        break;
    case AXP_GPIO_1:
        reg = AXP202_GPIO1_CTL;
        break;
    case AXP_GPIO_2:
        reg = AXP202_GPIO2_CTL;
        break;
    case AXP_GPIO_3:
        reg = AXP202_GPIO3_CTL;
        break;
    default:
        return AXP_NOT_SUPPORT;
    }
    _readByte(reg, 1, &val);
    val = mask == 0 ? (val & 0b00111111) : (val | mask);
    _writeByte(reg, 1, &val);
    return AXP_PASS;
}

int AXP202::setGPIOIrq(axp_gpio_t gpio, axp_gpio_irq_t irq)
{
    if (!_init)
        return AXP_NOT_INIT;

    return _axp202_gpio_irq_set(gpio, irq);
}

int AXP202::setLDO5Voltage(axp_ldo5_table_t vol)
{
    const uint8_t params[] = {
        0b11111000, // 1.8V
        0b11111001, // 2.5V
        0b11111010, // 2.8V
        0b11111011, // 3.0V
        0b11111100, // 3.1V
        0b11111101, // 3.3V
        0b11111110, // 3.4V
        0b11111111, // 3.5V
    };
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id != AXP202_CHIP_ID)
        return AXP_NOT_SUPPORT;
    if (vol > sizeof(params) / sizeof(params[0]))
        return AXP_ARG_INVALID;
    uint8_t val = 0;
    _readByte(AXP202_GPIO0_VOL, 1, &val);
    val &= 0b11111000;
    val |= params[vol];
    _writeByte(AXP202_GPIO0_VOL, 1, &val);
    return AXP_PASS;
}

int AXP202::_axp202_gpio_write(axp_gpio_t gpio, uint8_t val)
{
    uint8_t reg;
    uint8_t wVal = 0;
    switch (gpio)
    {
    case AXP_GPIO_0:
        reg = AXP202_GPIO0_CTL;
        break;
    case AXP_GPIO_1:
        reg = AXP202_GPIO1_CTL;
        break;
    case AXP_GPIO_2:
        reg = AXP202_GPIO2_CTL;
        if (val)
        {
            return AXP_NOT_SUPPORT;
        }
        break;
    case AXP_GPIO_3:
        if (val)
        {
            return AXP_NOT_SUPPORT;
        }
        _readByte(AXP202_GPIO3_CTL, 1, &wVal);
        wVal &= 0b11111101;
        _writeByte(AXP202_GPIO3_CTL, 1, &wVal);
        return AXP_PASS;
    default:
        return AXP_NOT_SUPPORT;
    }
    _readByte(reg, 1, &wVal);
    wVal = val ? (wVal | 1) : (wVal & 0b11111000);
    _writeByte(reg, 1, &wVal);
    return AXP_PASS;
}

int AXP202::_axp202_gpio_read(axp_gpio_t gpio)
{
    uint8_t val;
    uint8_t reg = AXP202_GPIO012_SIGNAL;
    uint8_t offset;
    switch (gpio)
    {
    case AXP_GPIO_0:
        offset = 4;
        break;
    case AXP_GPIO_1:
        offset = 5;
        break;
    case AXP_GPIO_2:
        offset = 6;
        break;
    case AXP_GPIO_3:
        reg = AXP202_GPIO3_CTL;
        offset = 0;
        break;
    default:
        return AXP_NOT_SUPPORT;
    }
    _readByte(reg, 1, &val);
    return val & _BV(offset) ? 1 : 0;
}

int AXP202::gpioWrite(axp_gpio_t gpio, uint8_t val)
{
    if (!_init)
        return AXP_NOT_INIT;

    return _axp202_gpio_write(gpio, val);
}

int AXP202::gpioRead(axp_gpio_t gpio)
{
    if (!_init)
        return AXP_NOT_INIT;

    return _axp202_gpio_read(gpio);
}

int AXP202::getChargeControlCur(void)
{
    int cur;
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;

    _readByte(AXP202_CHARGE1, 1, &val);
    val &= 0x0F;
    cur = val * 100 + 300;
    if (cur > 1800 || cur < 300)
        return 0;
    return cur;
}

int AXP202::setChargeControlCur(uint16_t mA)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;

    _readByte(AXP202_CHARGE1, 1, &val);
    val &= 0b11110000;
    mA -= 300;
    val |= (mA / 100);
    _writeByte(AXP202_CHARGE1, 1, &val);
    return AXP_PASS;
}

int AXP202::setSleep()
{
    int ret;
    uint8_t val = 0;
    ret = _readByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)
        return AXP_FAIL;
    val |= _BV(3);
    ret = _writeByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)
        return AXP_FAIL;
    ret = _readByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)
        return AXP_FAIL;
    return (val & _BV(3)) ? AXP_PASS : AXP_FAIL;
}

// VOFF =[2.6+(Bit2-0)*0.1]V
int AXP202::setPowerDownVoltage(uint16_t mv)
{
    int ret;
    uint8_t val = 0;
    ret = _readByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)
        return AXP_FAIL;
    val &= AXP202_VOFF_MASK;
    val |= ((mv - 2600) / 100);
    ret = _writeByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)
        return AXP_FAIL;
    return AXP_PASS;
}

uint16_t AXP202::getPowerDownVoltage(void)
{
    int ret = 0;
    uint8_t val = 0;
    ret = _readByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)
        return 0;
    val &= ~(AXP202_VOFF_MASK);
    uint16_t voff = val * 100 + 2600;
    return voff;
}

int AXP202::setCurrentLimitControl(axp202_limit_setting_t opt)
{
    uint8_t val = 0;
    if (!_init)
        return AXP_NOT_INIT;

    _readByte(AXP202_IPS_SET, 1, &val);
    val &= (~AXP202_LIMIT_MASK);
    val |= opt;
    _writeByte(AXP202_IPS_SET, 1, &val);
    return AXP_PASS;
}

int AXP202::setVWarningLevel1(uint16_t mv)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = (mv - 2867) / 5.6;
    AXP_DEBUG("setVWarningLevel1:0x%x\n", val);
    _writeByte(AXP202_APS_WARNING1, 1, &val);
    return AXP_PASS;
}

int AXP202::setVWarningLevel2(uint16_t mv)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = (mv - 2867) / 5.6;
    AXP_DEBUG("setVWarningLevel2:0x%x\n", val);
    _writeByte(AXP202_APS_WARNING2, 1, &val);
    return AXP_PASS;
}

uint16_t AXP202::getVWarningLevel1(void)
{
    ISCONNECETD(0);
    uint8_t val = 0;
    _readByte(AXP202_APS_WARNING1, 1, &val);
    AXP_DEBUG("TarageVoltage:%.2f HEX:0x%x\n", 2.8672 + 0.0014 * val * 4.0, val);
    return (2.8672 + 0.0014 * val * 4.0) * 1000;
}

uint16_t AXP202::getVWarningLevel2(void)
{
    ISCONNECETD(0);
    uint8_t val = 0;
    _readByte(AXP202_APS_WARNING2, 1, &val);
    AXP_DEBUG("TarageVoltage:%.2f HEX:0x%x\n", 2.8672 + 0.0014 * val * 4.0, val);
    return (2.8672 + 0.0014 * val * 4.0) * 1000;
}

int AXP202::setDCDCMode(axp202_dc_mode_t opt)
{
    uint8_t val = 0;
    _readByte(AXP202_DCDC_MODESET, 1, &val);
    val &= 0xF9;
    val |= opt;
    _writeByte(AXP202_DCDC_MODESET, 1, &val);
    return AXP_PASS;
}

axp202_dc_mode_t AXP202::getDCDCMode(void)
{
    uint8_t val = 0;
    _readByte(AXP202_DCDC_MODESET, 1, &val);
    val &= 0x6;
    return val ? AXP202_DCDC_AUTO_MODE : AXP202_DCDC_PWM_MODE;
}

int AXP202::enableLDO3VRC(bool en)
{
    ISCONNECETD(AXP_NOT_INIT);
    if (_chip_id != AXP202_CHIP_ID)
    {
        return AXP_NOT_SUPPORT;
    }
    uint8_t val = 0;
    _readByte(AXP202_LDO3_DC2_DVM, 1, &val);
    val &= (~_BV(3));
    val |= en;
    _writeByte(AXP202_LDO3_DC2_DVM, 1, &val);
    return AXP_PASS;
}

int AXP202::enableDC2VRC(bool en)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_LDO3_DC2_DVM, 1, &val);
    val &= (~_BV(2));
    val |= en;
    _writeByte(AXP202_LDO3_DC2_DVM, 1, &val);
    return AXP_PASS;
}

int AXP202::setLDO3VRC(axp202_vrc_control_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    if (_chip_id != AXP202_CHIP_ID)
    {
        return AXP_NOT_SUPPORT;
    }
    uint8_t val = 0;
    _readByte(AXP202_LDO3_DC2_DVM, 1, &val);
    val &= (~_BV(1));
    val |= opt;
    _writeByte(AXP202_LDO3_DC2_DVM, 1, &val);
    return AXP_PASS;
}

int AXP202::setDC2VRC(axp202_vrc_control_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_LDO3_DC2_DVM, 1, &val);
    val &= (~_BV(0));
    val |= opt;
    _writeByte(AXP202_LDO3_DC2_DVM, 1, &val);
    return AXP_PASS;
}

int AXP202::setBackupChargeControl(bool en)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_BACKUP_CHG, 1, &val);
    val &= (~_BV(7));
    val |= en;
    _writeByte(AXP202_BACKUP_CHG, 1, &val);
    return AXP_PASS;
}

int AXP202::setBackupChargeVoltage(axp202_backup_voltage_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_BACKUP_CHG, 1, &val);
    val &= 0x9F;
    val |= opt;
    _writeByte(AXP202_BACKUP_CHG, 1, &val);
    return AXP_PASS;
}

int AXP202::setBackupChargeCurrent(axp202_backup_current_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_BACKUP_CHG, 1, &val);
    val &= 0xFC;
    val |= opt;
    _writeByte(AXP202_BACKUP_CHG, 1, &val);
    return AXP_PASS;
}

int AXP202::setPrechargeTimeout(axp202_precharge_timeout_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_CHARGE2, 1, &val);
    val &= 0x3F;
    val |= opt;
    _writeByte(AXP202_CHARGE2, 1, &val);
    return AXP_PASS;
}

int AXP202::setConstantCurrentTimeout(axp202_constant_current_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_CHARGE2, 1, &val);
    val &= 0xFC;
    val |= opt;
    _writeByte(AXP202_CHARGE2, 1, &val);
    return AXP_PASS;
}

// Low-level I2C communication
uint16_t AXP202::_getRegistH8L5(uint8_t regh8, uint8_t regl5)
{
    uint8_t hv, lv;
    _readByte(regh8, 1, &hv);
    _readByte(regl5, 1, &lv);
    return (hv << 5) | (lv & 0x1F);
}

uint16_t AXP202::_getRegistResult(uint8_t regh8, uint8_t regl4)
{
    uint8_t hv, lv;
    _readByte(regh8, 1, &hv);
    _readByte(regl4, 1, &lv);
    return (hv << 4) | (lv & 0x0F);
}

int AXP202::_readByte(uint8_t reg, uint8_t nbytes, uint8_t *data)
{
    if (_read_cb != nullptr)
    {
        return _read_cb(_address, reg, data, nbytes);
    }
#ifdef ARDUINO
    if (nbytes == 0 || !data)
        return -1;
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(reg);
    if (_i2cPort->endTransmission() != 0)
    {
        return -1;
    }
    _i2cPort->requestFrom(_address, nbytes);
    uint8_t index = 0;
    while (_i2cPort->available())
        data[index++] = _i2cPort->read();
#endif
    return 0;
}

int AXP202::_writeByte(uint8_t reg, uint8_t nbytes, uint8_t *data)
{
    if (_write_cb != nullptr)
    {
        return _write_cb(_address, reg, data, nbytes);
    }
#ifdef ARDUINO
    if (nbytes == 0 || !data)
        return -1;
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(reg);
    for (uint8_t i = 0; i < nbytes; i++)
    {
        _i2cPort->write(data[i]);
    }
    return _i2cPort->endTransmission();
#endif
    return 0;
}
