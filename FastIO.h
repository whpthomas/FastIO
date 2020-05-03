/* Excerpt from Arduino DigitalIO Library
 * Copyright (C) 2013 by William Greiman
 *
 * This file is part of the Arduino DigitalIO Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino DigitalIO Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
 
#pragma once

/** GpioPinMap type */
struct GpioPinMap_t {
  volatile uint8_t* pin;   /**< address of PIN for this pin */
  volatile uint8_t* ddr;   /**< address of DDR for this pin */
  volatile uint8_t* port;  /**< address of PORT for this pin */
  uint8_t mask;            /**< bit mask for this pin */
};

/** Initializer macro. */
#define GPIO_PIN(reg, bit) {&PIN##reg, &DDR##reg, &PORT##reg, 1 << bit}

#if defined(__AVR_ATmega328P__)

static const GpioPinMap_t GpioPinMap[] = {
  GPIO_PIN(D, 0),  // D0
  GPIO_PIN(D, 1),  // D1
  GPIO_PIN(D, 2),  // D2
  GPIO_PIN(D, 3),  // D3
  GPIO_PIN(D, 4),  // D4
  GPIO_PIN(D, 5),  // D5
  GPIO_PIN(D, 6),  // D6
  GPIO_PIN(D, 7),  // D7
  GPIO_PIN(B, 0),  // D8
  GPIO_PIN(B, 1),  // D9
  GPIO_PIN(B, 2),  // D10
  GPIO_PIN(B, 3),  // D11
  GPIO_PIN(B, 4),  // D12
  GPIO_PIN(B, 5),  // D13
  GPIO_PIN(C, 0),  // D14
  GPIO_PIN(C, 1),  // D15
  GPIO_PIN(C, 2),  // D16
  GPIO_PIN(C, 3),  // D17
  GPIO_PIN(C, 4),  // D18
  GPIO_PIN(C, 5)   // D19
};

#elif defined(__AVR_ATmega1284P__)\
|| defined(__AVR_ATmega1284__)\
|| defined(__AVR_ATmega644P__)\
|| defined(__AVR_ATmega644__)\
|| defined(__AVR_ATmega64__)

static const GpioPinMap_t GpioPinMap[] = {
  GPIO_PIN(B, 0),  // D0
  GPIO_PIN(B, 1),  // D1
  GPIO_PIN(B, 2),  // D2
  GPIO_PIN(B, 3),  // D3
  GPIO_PIN(B, 4),  // D4
  GPIO_PIN(B, 5),  // D5
  GPIO_PIN(B, 6),  // D6
  GPIO_PIN(B, 7),  // D7
  GPIO_PIN(D, 0),  // D8
  GPIO_PIN(D, 1),  // D9
  GPIO_PIN(D, 2),  // D10
  GPIO_PIN(D, 3),  // D11
  GPIO_PIN(D, 4),  // D12
  GPIO_PIN(D, 5),  // D13
  GPIO_PIN(D, 6),  // D14
  GPIO_PIN(D, 7),  // D15
  GPIO_PIN(C, 0),  // D16
  GPIO_PIN(C, 1),  // D17
  GPIO_PIN(C, 2),  // D18
  GPIO_PIN(C, 3),  // D19
  GPIO_PIN(C, 4),  // D20
  GPIO_PIN(C, 5),  // D21
  GPIO_PIN(C, 6),  // D22
  GPIO_PIN(C, 7),  // D23
  GPIO_PIN(A, 0),  // D24
  GPIO_PIN(A, 1),  // D25
  GPIO_PIN(A, 2),  // D26
  GPIO_PIN(A, 3),  // D27
  GPIO_PIN(A, 4),  // D28
  GPIO_PIN(A, 5),  // D29
  GPIO_PIN(A, 6),  // D30
  GPIO_PIN(A, 7)   // D31
};

#else
#error Unsupported board type.
#endif

/** generate bad pin number error */
void badPinNumber(void)
  __attribute__((error("Pin number is too large or not a constant")));

/** Check for valid pin number
 * @param[in] pin Number of pin to be checked.
 */
static inline __attribute__((always_inline))
void badPinCheck(uint8_t pin) {
  if (!__builtin_constant_p(pin) || pin >= NUM_DIGITAL_PINS) {
     badPinNumber();
  }
}

/** DDR register address
 * @param[in] pin Arduino pin number
 * @return register address
 */
static inline __attribute__((always_inline))
volatile uint8_t* ddrReg(uint8_t pin) {
  badPinCheck(pin);
  return GpioPinMap[pin].ddr;
}

/** Bit mask for pin
 * @param[in] pin Arduino pin number
 * @return mask
 */
static inline __attribute__((always_inline))
uint8_t pinMask(uint8_t pin) {
  badPinCheck(pin);
  return GpioPinMap[pin].mask;
}

/** PIN register address
 * @param[in] pin Arduino pin number
 * @return register address
 */
static inline __attribute__((always_inline))
volatile uint8_t* pinReg(uint8_t pin) {
  badPinCheck(pin);
  return GpioPinMap[pin].pin;
}

/** PORT register address
 * @param[in] pin Arduino pin number
 * @return register address
 */
static inline __attribute__((always_inline))
volatile uint8_t* portReg(uint8_t pin) {
  badPinCheck(pin);
  return GpioPinMap[pin].port;
}

/** Fast write helper.
 * @param[in] address I/O register address
 * @param[in] mask bit mask for pin
 * @param[in] level value for bit
 */
static inline __attribute__((always_inline))
void fastBitWriteSafe(volatile uint8_t* address, uint8_t mask, bool level) {
  uint8_t s;
  if (address > reinterpret_cast<uint8_t*>(0X3F)) {
    s = SREG;
    cli();
  }
  if (level) {
    *address |= mask;
  } else {
    *address &= ~mask;
  }
  if (address > reinterpret_cast<uint8_t*>(0X3F)) {
    SREG = s;
  }
}

/** Read pin value.
 * @param[in] pin Arduino pin number
 * @return value read
 */
static inline __attribute__((always_inline))
bool fastDigitalRead(uint8_t pin) {
  return *pinReg(pin) & pinMask(pin);
}

/** Toggle a pin.
 * @param[in] pin Arduino pin number
 *
 * If the pin is in output mode toggle the pin level.
 * If the pin is in input mode toggle the state of the 20K pullup.
 */
static inline __attribute__((always_inline))
void fastDigitalToggle(uint8_t pin) {
    if (pinReg(pin) > reinterpret_cast<uint8_t*>(0X3F)) {
      // must write bit to high address port
      *pinReg(pin) = pinMask(pin);
    } else {
      // will compile to sbi and PIN register will not be read.
      *pinReg(pin) |= pinMask(pin);
    }
}

/** Set pin value.
 * @param[in] pin Arduino pin number
 * @param[in] level value to write
 */
static inline __attribute__((always_inline))
void fastDigitalWrite(uint8_t pin, bool level) {
  fastBitWriteSafe(portReg(pin), pinMask(pin), level);
}

/** Write the DDR register.
 * @param[in] pin Arduino pin number
 * @param[in] level value to write
 */
static inline __attribute__((always_inline))
void fastDdrWrite(uint8_t pin, bool level) {
  fastBitWriteSafe(ddrReg(pin), pinMask(pin), level);
}

/** Set pin mode.
 * @param[in] pin Arduino pin number
 * @param[in] mode INPUT, OUTPUT, or INPUT_PULLUP.
 *
 * The internal pullup resistors will be enabled if mode is INPUT_PULLUP
 * and disabled if the mode is INPUT.
 */
static inline __attribute__((always_inline))
void fastPinMode(uint8_t pin, uint8_t mode) {
  fastDdrWrite(pin, mode == OUTPUT);
  if (mode != OUTPUT) {
    fastDigitalWrite(pin, mode == INPUT_PULLUP);
  }
}

/** set pin configuration
 * @param[in] pin Arduino pin number
 * @param[in] mode mode INPUT or OUTPUT.
 * @param[in] level If mode is output, set level high/low.
 *                  If mode is input, enable or disable the pin's 20K pullup.
 */
#define fastPinConfig(pin, mode, level)\
  {fastPinMode(pin, mode); fastDigitalWrite(pin, level);}
