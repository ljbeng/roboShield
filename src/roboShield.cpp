/*  Arduino Library for the PCA9685 16-Channel PWM Driver Module.
    Copyright (c) 2016 NachtRaveVL      <nachtravevl@gmail.com>
    Copyright (C) 2012 Kasper Skårhøj   <kasperskaarhoj@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Created by Kasper Skårhøj, August 3rd, 2012.
    Forked by Vitska, June 18th, 2016.
    Forked by NachtRaveVL, July 29th, 2016.

    PCA9685-Arduino - Version 1.2.10
*/

#include "roboShield.h"

#define PCA9685_I2C_BASE_ADDRESS    (byte)0x40

// Register addresses from data sheet
#define PCA9685_MODE1_REG           (byte)0x00
#define PCA9685_MODE2_REG           (byte)0x01
#define PCA9685_SUBADR1_REG         (byte)0x02
#define PCA9685_SUBADR2_REG         (byte)0x03
#define PCA9685_SUBADR3_REG         (byte)0x04
#define PCA9685_ALLCALL_REG         (byte)0x05
#define PCA9685_LED0_REG            (byte)0x06 // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_PRESCALE_REG        (byte)0xFE
#define PCA9685_ALLLED_REG          (byte)0xFA

// Mode1 register pin layout
#define PCA9685_MODE_RESTART        (byte)0x80
#define PCA9685_MODE_EXTCLK         (byte)0x40
#define PCA9685_MODE_AUTOINC        (byte)0x20
#define PCA9685_MODE_SLEEP          (byte)0x10
#define PCA9685_MODE_SUBADR1        (byte)0x08
#define PCA9685_MODE_SUBADR2        (byte)0x04
#define PCA9685_MODE_SUBADR3        (byte)0x02
#define PCA9685_MODE_ALLCALL        (byte)0x01

#define PCA9685_SW_RESET            (byte)0x06 // Sent to address 0x00 to reset all devices on Wire line
#define PCA9685_PWM_FULL            (uint16_t)0x01000 // Special value for full on/full off LEDx modes


// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 13
#define AIN2 14
#define BIN1 11
#define BIN2 10
#define PWMA 15
#define PWMB 9
#define STBY 12

// To balance the load out in a weaved fashion, we use this offset table to distribute
// the load on the outputs in a more interleaving fashion than just a simple 16 offset
// per channel. We can set the off cycle value to be lower than the on cycle, which will
// put the high edge across the 0-4095 phase cycle range, which is supported by device.
static uint16_t phaseDistTable[16] = { 0, 2048, 1024, 3072, 512, 3584, 1536, 2560, 256, 3840, 1280, 2304, 3328, 768, 2816, 1792 };

#ifndef PCA9685_ENABLE_SOFTWARE_I2C
roboShield::roboShield(TwoWire& i2cWire, PCA9685_PhaseBalancer phaseBalancer, int enMotor) {
    _i2cWire = &i2cWire;
#else
roboShield::roboShield(PCA9685_PhaseBalancer phaseBalancer, int In1pin, int enMotor) {
#endif
  _i2cAddress = 0;
  _phaseBalancer = phaseBalancer;
  _isProxyAddresser = false;
  _lastI2CError = 0;

  if (enMotor == 1){//enables motor PWM and Digital Pins
	  In1 = AIN1;
	  In2 = AIN2; 
	  PWM = PWMA;
	  Offset = 1.0;
	  Standby = STBY;
  }
  if (enMotor == 2){
	  In1 = BIN1;
	  In2 = BIN2; 
	  PWM = PWMB;
	  Offset = 1.0;
	  Standby = STBY;
  }
  
}


void roboShield::drive(int speed)
{

	writeChannelBegin(Standby);
	writeChannelPWM(PCA9685_PWM_FULL, 0);  // time_on = FULL; time_off = 0;
	writeChannelEnd(); 
	delayMicroseconds(500);	
	speed = speed * Offset;
	if (speed>=0) fwd(speed);
	else rev(-speed);
  
}
void roboShield::drive(int speed, int duration)//changed so motor starts, runs for duration and brakes
{
  drive(speed);
  delay(duration);
  brake();
}

void roboShield::fwd(int speed)
{

    //set motor direction
    writeChannelBegin(In1);
    writeChannelPWM(PCA9685_PWM_FULL, 0);  // time_on = FULL; time_off = 0;
    writeChannelEnd();  
	delayMicroseconds(500);
    writeChannelBegin(In2);
    writeChannelPWM(0, PCA9685_PWM_FULL);  // time_on = FULL; time_off = 0;
    writeChannelEnd();  
	delayMicroseconds(500);
	//set speed
	writeChannelBegin(PWM);
    int16_t phaseBegin, phaseEnd;
    getPhaseCycle(PWM, speed, &phaseBegin, &phaseEnd);
    writeChannelPWM(phaseBegin, phaseEnd);
    writeChannelEnd();
	delayMicroseconds(500);

}

void roboShield::rev(int speed)
{
    //set motor direction
    writeChannelBegin(In1);
    writeChannelPWM(0, PCA9685_PWM_FULL);  // time_on = 0; time_off = FULL;
    writeChannelEnd();  
	delayMicroseconds(500);
    writeChannelBegin(In2);
    writeChannelPWM(PCA9685_PWM_FULL, 0);  // time_on = FULL; time_off = 0;
    writeChannelEnd();  
	delayMicroseconds(500);
	//set speed
	writeChannelBegin(PWM);
    int16_t phaseBegin, phaseEnd;
    getPhaseCycle(PWM, speed, &phaseBegin, &phaseEnd);
    writeChannelPWM(phaseBegin, phaseEnd);
    writeChannelEnd();
	delayMicroseconds(500);
}

void roboShield::brake()
{
    //set In1 and In2 high to brake and speed to 0
    writeChannelBegin(In1);
    writeChannelPWM(PCA9685_PWM_FULL, 0);  // 
    writeChannelEnd();  
	delayMicroseconds(500);
    writeChannelBegin(In2);
    writeChannelPWM(PCA9685_PWM_FULL, 0);  // 
    writeChannelEnd();  
	delayMicroseconds(500);
	//set speed
	writeChannelBegin(PWM);
    int16_t phaseBegin, phaseEnd;
    getPhaseCycle(PWM, 0, &phaseBegin, &phaseEnd);
    writeChannelPWM(phaseBegin, phaseEnd);
    writeChannelEnd();
	delayMicroseconds(500);   
}

void roboShield::standby()
{
   digitalWrite(Standby, LOW);
}

void forward(roboShield motor1, roboShield motor2, int speed)
{
	motor1.drive(speed);
	motor2.drive(speed);
}
void forward(roboShield motor1, roboShield motor2)
{
	motor1.drive(DEFAULTSPEED);
	motor2.drive(DEFAULTSPEED);
}


void back(roboShield motor1, roboShield motor2, int speed)
{
	int temp = abs(speed);
	motor1.drive(-temp);
	motor2.drive(-temp);
}
void back(roboShield motor1, roboShield motor2)
{
	motor1.drive(-DEFAULTSPEED);
	motor2.drive(-DEFAULTSPEED);
}
void left(roboShield left, roboShield right, int speed)
{
	int temp = abs(speed)/2;
	left.drive(-temp);
	right.drive(temp);
	
}


void right(roboShield left, roboShield right, int speed)
{
	int temp = abs(speed)/2;
	left.drive(temp);
	right.drive(-temp);
	
}
void brake(roboShield motor1, roboShield motor2)
{
	motor1.brake();
	motor2.brake();
}





void roboShield::resetDevices() {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("roboShield::resetDevices");
#endif

    i2cWire_beginTransmission(0x00);
    i2cWire_write(PCA9685_SW_RESET);
    i2cWire_endTransmission();

    delayMicroseconds(10);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void roboShield::init(byte i2cAddress, byte mode) {
    if (_isProxyAddresser) return;

    // I2C 7-bit address is B 1 A5 A4 A3 A2 A1 A0
    // RW bit added by Arduino core TWI library
    _i2cAddress = PCA9685_I2C_BASE_ADDRESS | (i2cAddress & 0x3F);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("roboShield::init i2cAddress: 0x");
    Serial.println(_i2cAddress, HEX);
#endif

    writeRegister(PCA9685_MODE1_REG, PCA9685_MODE_RESTART | PCA9685_MODE_AUTOINC);
    writeRegister(PCA9685_MODE2_REG, mode);
}

#ifndef PCA9685_EXCLUDE_EXT_FUNC

void roboShield::initAsProxyAddresser(byte i2cAddress) {
    _i2cAddress = i2cAddress & 0xFE;
    _isProxyAddresser = true;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("roboShield::initAsProxyAddresser i2cAddress: 0x");
    Serial.println(_i2cAddress, HEX);
#endif
}

#endif

byte roboShield::getI2CAddress() {
    return _i2cAddress;
}

PCA9685_PhaseBalancer roboShield::getPhaseBalancer() {
    return _phaseBalancer;
}

void roboShield::setPWMFrequency(float pwmFrequency) {
    if (pwmFrequency < 0 || _isProxyAddresser) return;

    // This equation comes from section 7.3.5 of the datasheet, but the rounding has been
    // removed because it isn't needed. Lowest freq is 23.84, highest is 1525.88.
    int preScalerVal = (25000000 / (4096 * pwmFrequency)) - 1;
    if (preScalerVal > 255) preScalerVal = 255;
    if (preScalerVal < 3) preScalerVal = 3;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("roboShield::setPWMFrequency pwmFrequency: ");
    Serial.print(pwmFrequency);
    Serial.print(", preScalerVal: 0x");
    Serial.println(preScalerVal, HEX);
#endif

    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE_RESTART) | PCA9685_MODE_SLEEP));
    writeRegister(PCA9685_PRESCALE_REG, (byte)preScalerVal);

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE_SLEEP) | PCA9685_MODE_RESTART));
    delayMicroseconds(500);
}

void roboShield::setChannelOn(int channel) {
    if (channel < 0 || channel > 8) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("roboShield::setChannelOn");
#endif

    writeChannelBegin(channel);
    writeChannelPWM(PCA9685_PWM_FULL, 0);  // time_on = FULL; time_off = 0;
    writeChannelEnd();
}

void roboShield::setChannelOff(int channel) {
    if (channel < 0 || channel > 8) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("roboShield::setChannelOff");
#endif

    writeChannelBegin(channel);
    writeChannelPWM(0, PCA9685_PWM_FULL);  // time_on = 0; time_off = FULL;
    writeChannelEnd();
}

void roboShield::setChannelPWM(int channel, uint16_t pwmAmount) {
    if (channel < 0 || channel > 8) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("roboShield::setChannelPWM");
#endif

    writeChannelBegin(channel);

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(channel, pwmAmount, &phaseBegin, &phaseEnd);

    writeChannelPWM(phaseBegin, phaseEnd);

    writeChannelEnd();
}

void roboShield::setChannelsPWM(int begChannel, int numChannels, const uint16_t *pwmAmounts) {
    if (begChannel < 0 || begChannel > 8 || numChannels < 0) return;
    if (begChannel + numChannels > 9) numChannels -= (begChannel + numChannels) - 9;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("roboShield::setChannelsPWM numChannels: ");
    Serial.println(numChannels);
#endif

    // In avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many channels can be written at once. Therefore, we loop around until all
    // channels have been written out into their registers.

    while (numChannels > 0) {
        writeChannelBegin(begChannel);

        int maxChannels = min(numChannels, (BUFFER_LENGTH - 1) / 4);
        while (maxChannels-- > 0) {
            uint16_t phaseBegin, phaseEnd;
            getPhaseCycle(begChannel++, *pwmAmounts++, &phaseBegin, &phaseEnd);

            writeChannelPWM(phaseBegin, phaseEnd);
            --numChannels;
        }

        writeChannelEnd();
        if (_lastI2CError) return;
    }
}

#ifndef PCA9685_EXCLUDE_EXT_FUNC

void roboShield::setAllChannelsPWM(uint16_t pwmAmount) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("roboShield::setAllChannelsPWM");
#endif

    writeChannelBegin(-1); // Special value for ALLLED registers

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(-1, pwmAmount, &phaseBegin, &phaseEnd);

    writeChannelPWM(phaseBegin, phaseEnd);

    writeChannelEnd();
}

uint16_t roboShield::getChannelPWM(int channel) {
    if (channel < 0 || channel > 8 || _isProxyAddresser) return 0;

    byte regAddress = PCA9685_LED0_REG + (channel << 2);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("roboShield::getChannelPWM channel: ");
    Serial.print(channel);
    Serial.print(", regAddress: 0x");
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
    if (i2cWire_endTransmission()) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    int bytesRead = i2cWire_requestFrom((uint8_t)_i2cAddress, (uint8_t)4);
    if (bytesRead != 4) {
        while (bytesRead-- > 0)
            i2cWire_read();
#ifdef PCA9685_ENABLE_SOFTWARE_I2C
        i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif
        _lastI2CError = 4;
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    uint16_t phaseBegin = (uint16_t)i2cWire_read();
    phaseBegin |= (uint16_t)i2cWire_read() << 8;
    uint16_t phaseEnd = (uint16_t)i2cWire_read();
    phaseEnd |= (uint16_t)i2cWire_read() << 8;

#ifdef PCA9685_ENABLE_SOFTWARE_I2C
    i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  roboShield::getChannelPWM phaseBegin: ");
    Serial.print(phaseBegin);
    Serial.print(", phaseEnd: ");
    Serial.println(phaseEnd);
#endif

    // See datasheet section 7.3.3
    uint16_t retVal;
    if (phaseEnd >= PCA9685_PWM_FULL)
        // Full OFF
        // Figure 11 Example 4: full OFF takes precedence over full ON
        // See also remark after Table 7
        retVal = 0;
    else if (phaseBegin >= PCA9685_PWM_FULL)
        // Full ON
        // Figure 9 Example 3
        retVal = PCA9685_PWM_FULL;
    else if (phaseBegin <= phaseEnd)
        // start and finish in same cycle
        // Section 7.3.3 example 1
        retVal = phaseEnd - phaseBegin;
    else
        // span cycles
        // Section 7.3.3 example 2
        retVal = (phaseEnd + PCA9685_PWM_FULL) - phaseBegin;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  roboShield::getChannelPWM retVal: ");
    Serial.println(retVal);
#endif

    return retVal;
}

void roboShield::enableAllCallAddress(byte i2cAddress) {
    if (_isProxyAddresser) return;

    i2cAddress &= 0xFE;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("roboShield::enableAllCallAddress i2cAddress: 0x");
    Serial.println(i2cAddress, HEX);
#endif

    writeRegister(PCA9685_ALLCALL_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE_ALLCALL));
}

void roboShield::enableSub1Address(byte i2cAddress) {
    if (_isProxyAddresser) return;

    i2cAddress &= 0xFE;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("roboShield::enableSub1Address i2cAddress: 0x");
    Serial.println(i2cAddress, HEX);
#endif

    writeRegister(PCA9685_SUBADR1_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE_SUBADR1));
}

void roboShield::enableSub2Address(byte i2cAddress) {
    if (_isProxyAddresser) return;

    i2cAddress &= 0xFE;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("roboShield::enableSub2Address i2cAddress: 0x");
    Serial.println(i2cAddress, HEX);
#endif

    writeRegister(PCA9685_SUBADR2_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE_SUBADR2));
}

void roboShield::enableSub3Address(byte i2cAddress) {
    if (_isProxyAddresser) return;

    i2cAddress &= 0xFE;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("roboShield::enableSub3Address i2cAddress: 0x");
    Serial.println(i2cAddress, HEX);
#endif

    writeRegister(PCA9685_SUBADR3_REG, i2cAddress);

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE_SUBADR3));
}

void roboShield::disableAllCallAddress() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("roboShield::disableAllCallAddress");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE_ALLCALL));
}

void roboShield::disableSub1Address() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("roboShield::disableSub1Address");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE_SUBADR1));
}

void roboShield::disableSub2Address() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("roboShield::disableSub2Address");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE_SUBADR2));
}

void roboShield::disableSub3Address() {
    if (_isProxyAddresser) return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("roboShield::disableSub3Address");
#endif

    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE_SUBADR3));
}

void roboShield::enableExtClockLine() {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("roboShield::enableExtClockLine");
#endif

    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE_RESTART) | PCA9685_MODE_SLEEP));
    writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE_EXTCLK));

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE_SLEEP) | PCA9685_MODE_RESTART));
    delayMicroseconds(500);
}

#endif

byte roboShield::getLastI2CError() {
    return _lastI2CError;
}

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT

static const char *textForI2CError(byte errorCode) {
    switch (errorCode) {
    case 0:
        return "Success";
    case 1:
        return "Data too long to fit in transmit buffer";
    case 2:
        return "Received NACK on transmit of address";
    case 3:
        return "Received NACK on transmit of data";
    default:
        return "Other error";
    }
}

void roboShield::checkForErrors() {
    if (_lastI2CError) {
        Serial.print("  roboShield::checkErrors lastI2CError: ");
        Serial.print(_lastI2CError);
        Serial.print(": ");
        Serial.println(textForI2CError(getLastI2CError()));
    }
}

#endif

void roboShield::getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd) {
    // Set delay
    if (channel < 0) {
        // All channels
        *phaseBegin = 0;
    }
    else if (_phaseBalancer == PCA9685_PhaseBalancer_Linear) {
        // Distribute high phase area over entire phase range to balance load.
        *phaseBegin = channel * (4096 / 16);
    }
    else if (_phaseBalancer == PCA9685_PhaseBalancer_Weaved) {
        // Distribute high phase area over entire phase range to balance load.
        *phaseBegin = phaseDistTable[channel];
    }
    else {
        *phaseBegin = 0;
    }
    
    // See datasheet section 7.3.3
    if (pwmAmount == 0) {
        // Full OFF => time_off[12] = 1;
        *phaseEnd = PCA9685_PWM_FULL;
    }
    else if (pwmAmount >= PCA9685_PWM_FULL) {
        // Full ON => time_on[12] = 1; time_off = ignored;
        *phaseBegin |= PCA9685_PWM_FULL;
        *phaseEnd = 0;
    }
    else {
        *phaseEnd = *phaseBegin + pwmAmount;
        if (*phaseEnd >= PCA9685_PWM_FULL)
            *phaseEnd -= PCA9685_PWM_FULL;
    }
}

void roboShield::writeChannelBegin(int channel) {
    byte regAddress;

    if (channel != -1)
        regAddress = PCA9685_LED0_REG + (channel * 0x04);
    else
        regAddress = PCA9685_ALLLED_REG;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  roboShield::writeChannelBegin channel: ");
    Serial.print(channel);
    Serial.print(", regAddress: 0x");
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
}

void roboShield::writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  roboShield::writeChannelPWM phaseBegin: ");
    Serial.print(phaseBegin);
    Serial.print(", phaseEnd: ");
    Serial.println(phaseEnd);
#endif

    i2cWire_write(lowByte(phaseBegin));
    i2cWire_write(highByte(phaseBegin));
    i2cWire_write(lowByte(phaseEnd));
    i2cWire_write(highByte(phaseEnd));
}

void roboShield::writeChannelEnd() {
    i2cWire_endTransmission();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void roboShield::writeRegister(byte regAddress, byte value) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  roboShield::writeRegister regAddress: 0x");
    Serial.print(regAddress, HEX);
    Serial.print(", value: 0x");
    Serial.println(value, HEX);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
    i2cWire_write(value);
    i2cWire_endTransmission();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

byte roboShield::readRegister(byte regAddress) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  roboShield::readRegister regAddress: 0x");
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
    if (i2cWire_endTransmission()) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    int bytesRead = i2cWire_requestFrom((uint8_t)_i2cAddress, (uint8_t)1);
    if (bytesRead != 1) {
        while (bytesRead-- > 0)
            i2cWire_read();
#ifdef PCA9685_ENABLE_SOFTWARE_I2C
        i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif
        _lastI2CError = 4;
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    byte retVal = i2cWire_read();

#ifdef PCA9685_ENABLE_SOFTWARE_I2C
    i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("    roboShield::readRegister retVal: 0x");
    Serial.println(retVal, HEX);
#endif

    return retVal;
}

#ifdef PCA9685_ENABLE_SOFTWARE_I2C
bool __attribute__((noinline)) i2c_start(uint8_t addr);
void __attribute__((noinline)) i2c_stop(void) asm("ass_i2c_stop");
bool __attribute__((noinline)) i2c_write(uint8_t value) asm("ass_i2c_write");
uint8_t __attribute__((noinline)) i2c_read(bool last);
#endif

void roboShield::i2cWire_beginTransmission(uint8_t addr) {
    _lastI2CError = 0;
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
    _i2cWire->beginTransmission(addr);
#else
    i2c_start(addr);
#endif
}

uint8_t roboShield::i2cWire_endTransmission(void) {
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
    return (_lastI2CError = _i2cWire->endTransmission());
#else
    i2c_stop();
    return (_lastI2CError = 0);
#endif
}

uint8_t roboShield::i2cWire_requestFrom(uint8_t addr, uint8_t len) {
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
    return _i2cWire->requestFrom(addr, len);
#else
    i2c_start(addr | 0x01);
    return (_readBytes = len);
#endif
}

size_t roboShield::i2cWire_write(uint8_t data) {
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
    return _i2cWire->write(data);
#else
    return (size_t)i2c_write(data);
#endif
}

uint8_t roboShield::i2cWire_read(void) {
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
    return (uint8_t)(_i2cWire->read() & 0xFF);
#else
    if (_readBytes > 1) {
        _readBytes -= 1;
        return (uint8_t)(i2c_read(false) & 0xFF);
    }
    else {
        _readBytes = 0;
        return (uint8_t)(i2c_read(true) & 0xFF);
    }
#endif
}

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT

void roboShield::printModuleInfo() {
    Serial.println(""); Serial.println(" ~~~ PCA9685 Module Info ~~~");

    Serial.println(""); Serial.println("i2c Address:");
    Serial.print("0x");
    Serial.println(_i2cAddress, HEX);

    Serial.println(""); Serial.println("Phase Balancer:");
    switch (_phaseBalancer) {
        case PCA9685_PhaseBalancer_None:
            Serial.println("PCA9685_PhaseBalancer_None"); break;
        case PCA9685_PhaseBalancer_Linear:
            Serial.println("PCA9685_PhaseBalancer_Linear"); break;
        case PCA9685_PhaseBalancer_Weaved:
            Serial.println("PCA9685_PhaseBalancer_Weaved"); break;
        default:
            Serial.println(""); break;
    }

    if (!_isProxyAddresser) {
        Serial.println(""); Serial.println("Proxy Addresser:");
        Serial.println("false");

        Serial.println(""); Serial.println("Mode1 Register:");
        byte mode1Reg = readRegister(PCA9685_MODE1_REG);
        Serial.print("0x");
        Serial.print(mode1Reg, HEX);
        Serial.print(", Bitset:");
        if (mode1Reg & PCA9685_MODE_RESTART)
            Serial.print(" PCA9685_MODE_RESTART");
        if (mode1Reg & PCA9685_MODE_EXTCLK)
            Serial.print(" PCA9685_MODE_EXTCLK");
        if (mode1Reg & PCA9685_MODE_AUTOINC)
            Serial.print(" PCA9685_MODE_AUTOINC");
        if (mode1Reg & PCA9685_MODE_SLEEP)
            Serial.print(" PCA9685_MODE_SLEEP");
        if (mode1Reg & PCA9685_MODE_SUBADR1)
            Serial.print(" PCA9685_MODE_SUBADR1");
        if (mode1Reg & PCA9685_MODE_SUBADR2)
            Serial.print(" PCA9685_MODE_SUBADR2");
        if (mode1Reg & PCA9685_MODE_SUBADR3)
            Serial.print(" PCA9685_MODE_SUBADR3");
        if (mode1Reg & PCA9685_MODE_ALLCALL)
            Serial.print(" PCA9685_MODE_ALLCALL");
        Serial.println("");

        Serial.println(""); Serial.println("Mode2 Register:");
        byte mode2Reg = readRegister(PCA9685_MODE2_REG);
        Serial.print("0x");
        Serial.print(mode2Reg, HEX);
        Serial.print(", Bitset:");
        if (mode2Reg & PCA9685_MODE_INVRT)
            Serial.print(" PCA9685_MODE_INVRT");
        if (mode2Reg & PCA9685_MODE_OUTPUT_ONACK)
            Serial.print(" PCA9685_MODE_OUTPUT_ONACK");
        if (mode2Reg & PCA9685_MODE_OUTPUT_TPOLE)
            Serial.print(" PCA9685_MODE_OUTPUT_TPOLE");
        if (mode2Reg & PCA9685_MODE_OUTNE_HIGHZ)
            Serial.print(" PCA9685_MODE_OUTNE_HIGHZ");
        if (mode2Reg & PCA9685_MODE_OUTNE_LOW)
            Serial.print(" PCA9685_MODE_OUTNE_LOW");
        Serial.println("");

        Serial.println(""); Serial.println("SubAddress1 Register:");
        byte subAdr1Reg = readRegister(PCA9685_SUBADR1_REG);
        Serial.print("0x");
        Serial.println(subAdr1Reg, HEX);

        Serial.println(""); Serial.println("SubAddress2 Register:");
        byte subAdr2Reg = readRegister(PCA9685_SUBADR2_REG);
        Serial.print("0x");
        Serial.println(subAdr2Reg, HEX);

        Serial.println(""); Serial.println("SubAddress3 Register:");
        byte subAdr3Reg = readRegister(PCA9685_SUBADR3_REG);
        Serial.print("0x");
        Serial.println(subAdr3Reg, HEX);

        Serial.println(""); Serial.println("AllCall Register:");
        byte allCallReg = readRegister(PCA9685_ALLCALL_REG);
        Serial.print("0x");
        Serial.println(allCallReg, HEX);
    }
    else {
        Serial.println(""); Serial.println("Proxy Addresser:");
        Serial.println("true");
    }
}

#endif


#ifndef PCA9685_EXCLUDE_SERVO_EVAL

roboShield_ServoEvaluator::roboShield_ServoEvaluator(uint16_t n90PWMAmount, uint16_t p90PWMAmount) {
    n90PWMAmount = constrain(n90PWMAmount, 0, PCA9685_PWM_FULL);
    p90PWMAmount = constrain(p90PWMAmount, n90PWMAmount, PCA9685_PWM_FULL);

    _coeff = new float[2];
    _isCSpline = false;

    _coeff[0] = n90PWMAmount;
    _coeff[1] = (p90PWMAmount - n90PWMAmount) / 180.0f;
}

roboShield_ServoEvaluator::roboShield_ServoEvaluator(uint16_t n90PWMAmount, uint16_t zeroPWMAmount, uint16_t p90PWMAmount) {
    n90PWMAmount = constrain(n90PWMAmount, 0, PCA9685_PWM_FULL);
    zeroPWMAmount = constrain(zeroPWMAmount, n90PWMAmount, PCA9685_PWM_FULL);
    p90PWMAmount = constrain(p90PWMAmount, zeroPWMAmount, PCA9685_PWM_FULL);

    if (p90PWMAmount - zeroPWMAmount != zeroPWMAmount - n90PWMAmount) {
        _coeff = new float[8];
        _isCSpline = true;

        // Cubic spline code adapted from: https://shiftedbits.org/2011/01/30/cubic-spline-interpolation/
        /* "THE BEER-WARE LICENSE" (Revision 42): Devin Lane wrote this [part]. As long as you retain
        * this notice you can do whatever you want with this stuff. If we meet some day, and you
        * think this stuff is worth it, you can buy me a beer in return. */

        float x[3] = { 0, 90, 180 };
        float y[3] = { (float)n90PWMAmount, (float)zeroPWMAmount, (float)p90PWMAmount };
        float c[3], b[2], d[2], h[2], l[1], u[2], a[1], z[2]; // n = 3

        h[0] = x[1] - x[0];
        u[0] = z[0] = 0;
        c[2] = 0;

        for (int i = 1; i < 2; ++i) {
            h[i] = x[i + 1] - x[i];
            l[i - 1] = (2 * (x[i + 1] - x[i - 1])) - h[i - 1] * u[i - 1];
            u[i] = h[i] / l[i - 1];
            a[i - 1] = (3 / h[i]) * (y[i + 1] - y[i]) - (3 / h[i - 1]) * (y[i] - y[i - 1]);
            z[i] = (a[i - 1] - h[i - 1] * z[i - 1]) / l[i - 1];
        }

        for (int i = 1; i >= 0; --i) {
            c[i] = z[i] - u[i] * c[i + 1];
            b[i] = (y[i + 1] - y[i]) / h[i] - (h[i] * (c[i + 1] + 2 * c[i])) / 3;
            d[i] = (c[i + 1] - c[i]) / (3 * h[i]);

            _coeff[4 * i + 0] = y[i]; // a
            _coeff[4 * i + 1] = b[i]; // b
            _coeff[4 * i + 2] = c[i]; // c
            _coeff[4 * i + 3] = d[i]; // d
        }
    }
    else {
        _coeff = new float[2];
        _isCSpline = false;

        _coeff[0] = n90PWMAmount;
        _coeff[1] = (p90PWMAmount - n90PWMAmount) / 180.0f;
    }
}

roboShield_ServoEvaluator::~roboShield_ServoEvaluator() {
    if (_coeff) delete[] _coeff;
}

uint16_t roboShield_ServoEvaluator::pwmForAngle(float angle) {
    float retVal;
    angle = constrain(angle + 90, 0, 180);

    if (!_isCSpline) {
        retVal = _coeff[0] + (_coeff[1] * angle);
    }
    else {
        if (angle <= 90) {
            retVal = _coeff[0] + (_coeff[1] * angle) + (_coeff[2] * angle * angle) + (_coeff[3] * angle * angle * angle);
        }
        else {
            angle -= 90;
            retVal = _coeff[4] + (_coeff[5] * angle) + (_coeff[6] * angle * angle) + (_coeff[7] * angle * angle * angle);
        }
    }
    
    return (uint16_t)constrain((int)roundf(retVal), 0, PCA9685_PWM_FULL);
};





#endif
