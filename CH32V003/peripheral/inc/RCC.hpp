/* ***************************************************************************
MIT License

Copyright (c) 2025 k-off pacovali@student.42berlin.de

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
*************************************************************************** */

#pragma once

#include <ch32v00x.h>
#include <Register.hpp>

inline constexpr uintptr_t RCC_CTLR_ADDR       = RCC_BASE + offsetof(RCC_TypeDef, CTLR);        // CTLR         register address
inline constexpr uintptr_t RCC_CFGR0_ADDR      = RCC_BASE + offsetof(RCC_TypeDef, CFGR0);       // CFGR0        register address
inline constexpr uintptr_t RCC_INTR_ADDR       = RCC_BASE + offsetof(RCC_TypeDef, INTR);        // INTR         register address
inline constexpr uintptr_t RCC_APB2PRSTR_ADDR  = RCC_BASE + offsetof(RCC_TypeDef, APB2PRSTR);   // APB2PRSTR    register address
inline constexpr uintptr_t RCC_APB1PRSTR_ADDR  = RCC_BASE + offsetof(RCC_TypeDef, APB1PRSTR);   // APB1PRSTR    register address
inline constexpr uintptr_t RCC_AHBPCENR_ADDR   = RCC_BASE + offsetof(RCC_TypeDef, AHBPCENR);    // AHBPCENR     register address
inline constexpr uintptr_t RCC_APB2PCENR_ADDR  = RCC_BASE + offsetof(RCC_TypeDef, APB2PCENR);   // APB2PCENR    register address
inline constexpr uintptr_t RCC_APB1PCENR_ADDR  = RCC_BASE + offsetof(RCC_TypeDef, APB1PCENR);   // APB1PCENR    register address
inline constexpr uintptr_t RCC_RESERVED0_ADDR  = RCC_BASE + offsetof(RCC_TypeDef, RESERVED0);   // RESERVED0    register address
inline constexpr uintptr_t RCC_RSTSCKR_ADDR    = RCC_BASE + offsetof(RCC_TypeDef, RSTSCKR);     // RSTSCKR      register address

// Reset and Clock Control
class Rcc {
public:
    // System Clock Frequency values
    struct Speed {
        enum  : uint32_t {
            HSI_8  = 8000000,   // HSI  8MHz
            HSI_24 = 24000000,  // HSI 24MHz
            HSI_48 = 48000000,  // HSI 48MHz
            HSE_8  = -HSI_8,    // HSE  8MHz
            HSE_24 = -HSI_24,   // HSE 24MHz
            HSE_48 = -HSI_48    // HSE 48MHz
        };
    };

    // Clock Control Register
    struct Ctlr : public Register<uint32_t, RCC_CTLR_ADDR> {
        // predefined values
        enum : uint32_t {
            CLEAR    = 0b11111110111100101111111110000111u, // default CTLR register value

            // sXXX hi()/lo() method values, state change (switch)

            sHSI      = (1u << 0),        // set HSI state bit
            sHSE      = (1u << 16),       // set HSE state bit
            sHSE_BYP  = (1u << 18),       // set HSE bypass state bit
            sCSS      = (1u << 19),       // set Clock Security System state bit
            sPLL      = (1u << 24),       // set PLL state bit

            //rXXX read() method values

            rHSICAL  =  (0b11111111u <<  8), // read HSI calibration value
            rHSI     =  (0b00000001u <<  1), // read HSI is ready bit
            rHSE     =  (0b00000001u << 17), // read HSE is ready bit
            rPLL     =  (0b00000001u << 25), // read PLL is ready bit

            // wXXX write() method values, wmXXX write() method masks

            wmHSITRIM = ~(0b00011111u <<  3), // HSI trim write mask
            wHSITRIM_0  = ( 0u << 3),     // hsi calibration adjustment write value
            wHSITRIM_1  = ( 1u << 3),     // hsi calibration adjustment write value
            wHSITRIM_2  = ( 2u << 3),     // hsi calibration adjustment write value
            wHSITRIM_3  = ( 3u << 3),     // hsi calibration adjustment write value
            wHSITRIM_4  = ( 4u << 3),     // hsi calibration adjustment write value
            wHSITRIM_5  = ( 5u << 3),     // hsi calibration adjustment write value
            wHSITRIM_6  = ( 6u << 3),     // hsi calibration adjustment write value
            wHSITRIM_7  = ( 7u << 3),     // hsi calibration adjustment write value
            wHSITRIM_8  = ( 8u << 3),     // hsi calibration adjustment write value
            wHSITRIM_9  = ( 9u << 3),     // hsi calibration adjustment write value
            wHSITRIM_10 = (10u << 3),     // hsi calibration adjustment write value
            wHSITRIM_11 = (11u << 3),     // hsi calibration adjustment write value
            wHSITRIM_12 = (12u << 3),     // hsi calibration adjustment write value
            wHSITRIM_13 = (13u << 3),     // hsi calibration adjustment write value
            wHSITRIM_14 = (14u << 3),     // hsi calibration adjustment write value
            wHSITRIM_15 = (15u << 3),     // hsi calibration adjustment write value
            wHSITRIM_16 = (16u << 3),     // hsi calibration adjustment write value
            wHSITRIM_17 = (17u << 3),     // hsi calibration adjustment write value
            wHSITRIM_18 = (18u << 3),     // hsi calibration adjustment write value
            wHSITRIM_19 = (19u << 3),     // hsi calibration adjustment write value
            wHSITRIM_20 = (20u << 3),     // hsi calibration adjustment write value
            wHSITRIM_21 = (21u << 3),     // hsi calibration adjustment write value
            wHSITRIM_22 = (22u << 3),     // hsi calibration adjustment write value
            wHSITRIM_23 = (23u << 3),     // hsi calibration adjustment write value
            wHSITRIM_24 = (24u << 3),     // hsi calibration adjustment write value
            wHSITRIM_25 = (25u << 3),     // hsi calibration adjustment write value
            wHSITRIM_26 = (26u << 3),     // hsi calibration adjustment write value
            wHSITRIM_27 = (27u << 3),     // hsi calibration adjustment write value
            wHSITRIM_28 = (28u << 3),     // hsi calibration adjustment write value
            wHSITRIM_29 = (29u << 3),     // hsi calibration adjustment write value
            wHSITRIM_30 = (30u << 3),     // hsi calibration adjustment write value
            wHSITRIM_31 = (31u << 3),     // hsi calibration adjustment write value
        };
    };

    // Clock Configuration Register
    struct Cfgr0 : public Register<uint32_t, RCC_CFGR0_ADDR> {
        // predefined values
        enum  : uint32_t {
            CLEAR = 0b11111000111111100000000000101100, // default register value, use with &= to reset

            wmSW      = ~3u,  // clock source set mask
            wSW_HSI   =  0u,  // set HSI as system clock
            wSW_HSE   =  1u,  // set HSE as system clock
            wSW_PLL   =  2u,  // set PLL as system clock

            wmHPRE     = ~(15u<< 4), // sysclock prescaler write mask
            wHPRE_0    =  (0u << 4), // set sysclock prescaler off
            wHPRE_2    =  (1u << 4), // set sysclock divided by 2
            wHPRE_3    =  (2u << 4), // set sysclock divided by 3
            wHPRE_4    =  (3u << 4), // set sysclock divided by 4
            wHPRE_5    =  (4u << 4), // set sysclock divided by 5
            wHPRE_6    =  (5u << 4), // set sysclock divided by 6
            wHPRE_7    =  (6u << 4), // set sysclock divided by 7
            wHPRE_8    =  (7u << 4), // set sysclock divided by 8
            wHPRE_16   = (11u << 4), // set sysclock divided by 16
            wHPRE_32   = (12u << 4), // set sysclock divided by 32
            wHPRE_64   = (13u << 4), // set sysclock divided by 64
            wHPRE_128  = (14u << 4), // set sysclock divided by 128
            wHPRE_256  = (15u << 4), // set sysclock divided by 256

            wmADCPRE     = ~(0b11111u << 11), // adc prescaler mask
            wADCPRE_2    =  (0b00000u << 11), // set adc prescaler divides by 2
            wADCPRE_4    =  (0b00100u << 11), // set adc prescaler divides by 4
            wADCPRE_8    =  (0b00101u << 11), // set adc prescaler divides by 8
            wADCPRE_16   =  (0b11100u << 11), // set adc prescaler divides by 16
            wADCPRE_32   =  (0b11101u << 11), // set adc prescaler divides by 32
            wADCPRE_48   =  (0b10110u << 11), // set adc prescaler divides by 48
            wADCPRE_64   =  (0b11110u << 11), // set adc prescaler divides by 64
            wADCPRE_96   =  (0b10111u << 11), // set adc prescaler divides by 96
            wADCPRE_128  =  (0b11111u << 11), // set adc prescaler divides by 128

            wmPLL = ~(1u << 16),       // pll mask
            wPLL_HSE  =  (1u << 16),   // set hse as source of pll
            wPLL_HSI  = ~(wPLL_HSE),   // set hsi as source of pll

            wmMCO   = ~(7u << 26),        // mco mask
            wMCO_NO      = (0u << 26),    // set no mco pin output
            wMCO_SYSCLK  = (4u << 26),    // set sysclk to mco pin output
            wMCO_RC      = (5u << 26),    // set internal rc to  mco pin output
            wMCO_HSE     = (6u << 26),    // set hse to mco pin output
            wMCO_PLL     = (7u << 26),    // set pll to mco pin output

            //rXXX read() method values

            rSWS_HSI    = (0u << 3),     // read hsi is system clock
            rSWS_HSE    = (1u << 3),     // read hse is system clock
            rSWS_PLL    = (2u << 3),     // read pll is system clock
            rSWS_ANY    = (3u << 3),     // read any pll or hse is system clock
        };
    };

    // Clock Interrupt Register
    struct Intr : public Register<uint32_t, RCC_INTR_ADDR> {
        // predefined values
        enum : uint32_t {
            CLEAR = 0b11111111100111111110001011111111, // default register value, use with &= to reset

            //rXXX read() method values

            rLSIRDY = (1u << 0), // read LSI clock-ready interrupt status (flag)
            rHSIRDY = (1u << 2), // read HSI clock-ready interrupt status (flag)
            rHSERDY = (1u << 3), // read HSE clock-ready interrupt status (flag)
            rPLLRDY = (1u << 4), // read PLL clock-ready interrupt status (flag)
            rCSS    = (1u << 7), // read Clock Security System interrupt status (flag)

            // sXXX hi()/lo() method values, state change (switch)

            sLSIRDY = (1u << 8),        // set LSI clock-ready interrupt bit
            sHSIRDY = (1u << 10),       // set HSI clock-ready interrupt bit
            sHSERDY = (1u << 11),       // set HSE clock-ready interrupt bit
            sPLLRDY = (1u << 12),       // set PLL clock-ready interrupt bit
            sALL    = (0b11101 << 8),   // set all interupt bits

            sLSIRDY_CLR = (1u << 16), // clear LSI clock-ready interrupt status (flag)
            sHSIRDY_CLR = (1u << 18), // clear HSI clock-ready interrupt status (flag)
            sHSERDY_CLR = (1u << 19), // clear HSE clock-ready interrupt status (flag)
            sPLLRDY_CLR = (1u << 20), // clear PLL clock-ready interrupt status (flag)
            sCSS_CLR    = (1u << 23), // clear Clock Security System interrupt status (flag)
            sALL_CLR    = (0b10011111 << 16), // clear all interrupt status (flags)
        };
    };

    // PB2 Peripheral Reset Register values
    struct Apb2PRst : public Register<uint32_t, RCC_APB2PRSTR_ADDR> {
        // predefined values
        enum : uint32_t {

            // sXXX hi()/lo() method values, state change (switch)

            sAFIO   = (1u <<  0), // reset AFIO peripheral
            sIOPA   = (1u <<  2), // reset IOPA peripheral
            sIOPC   = (1u <<  4), // reset IOPC peripheral
            sIOPD   = (1u <<  5), // reset IOPD peripheral
            sADC1   = (1u <<  9), // reset ADC1 peripheral
            sTIM1   = (1u << 11), // reset TIM1 peripheral
            sSPI1   = (1u << 12), // reset SPI1 peripheral
            sUSART1 = (1u << 14), // reset USART1 peripheral
            sALL = (0b101101000110101u << 0), // reset all peripherals
        };
    };

    // PB1 Peripheral Reset Register values
    struct Apb1PRst : public Register<uint32_t, RCC_APB1PRSTR_ADDR> {
        // predefined values
        enum : uint32_t {

            // sXXX hi()/lo() method values, state change (switch)

            sTIM2 = (1u <<  0), // reset TIM2 module
            sWWDG = (1u << 11), // reset WWDG
            sI2C1 = (1u << 21), // reset I2C1 interface
            sPWR  = (1u << 28), // reset PWR interface module
            sALL = (0b10000001000000000100000000001u << 0), // reset all peripherals
        };
    };

    // HB peripheral Clock Enable register values
    struct AhbPCEn : public Register<uint32_t, RCC_AHBPCENR_ADDR> {
        // predefined values
        enum : uint32_t {

            // sXXX hi()/lo() method values, state change (switch)

            sCLEAR   = ~0b00101u, // default register value, use with &= to reset
            sDMA1 = (1u << 0), // DMA1 state during sleep mode bit
            sSRAM = (1u << 2), // SRAM state during sleep mode bit
        };
    };

    // PB2 Peripheral Clock Enable register values
    struct Apb2PCEn : public Register<uint32_t, RCC_APB2PCENR_ADDR> {
        // predefined values
        enum : uint32_t {

            // sXXX hi()/lo() method values, state change (switch)

            sAFIO   = (1u <<  0), //   AFIO peripheral bit
            sIOPA   = (1u <<  2), //   IOPA peripheral bit
            sIOPC   = (1u <<  4), //   IOPC peripheral bit
            sIOPD   = (1u <<  5), //   IOPD peripheral bit
            sADC1   = (1u <<  9), //   ADC1 peripheral bit
            sTIM1   = (1u << 11), //   TIM1 peripheral bit
            sSPI1   = (1u << 12), //   SPI1 peripheral bit
            sUSART1 = (1u << 14), // USART1 peripheral bit
            sALL    = (0b101101000110101u << 0), // all peripheral bits
        };
    };

    // PB1 Peripheral Clock Enable register values
    struct Apb1PCEn : public Register<uint32_t, RCC_APB1PCENR_ADDR> {
        enum : uint32_t {

            // sXXX hi()/lo() method values, state change (switch)

            sTIM2  = (1u <<  0), // reset TIM2 module
            sWWDG  = (1u << 11), // reset WWDG
            sI2C1  = (1u << 21), // reset I2C1 interface
            sPWR   = (1u << 28), // reset PWR interface module
            sALL   = (0b10000001000000000100000000001u << 0), // reset all peripherals
        };
    };

    struct RstSck : public Register<uint32_t, RCC_RSTSCKR_ADDR> {
        enum : uint32_t {

            // sXXX hi()/lo() method values, state change (switch)

            sLSI      = (1u <<  0), // LSI clock enable/disable bit
            sALL_RST  = (1u << 24), // clear all flags

            //rXXX read() method values

            rLSI_FLG  = (1u <<  1), // check whether LSI clock is ready
            rPIN_RST  = (1u << 26), // check NRST pin reset flag
            rPWR_RST  = (1u << 27), // check power-up/power-down reset flag
            rSFT_RST  = (1u << 28), // check software reset flag
            rIWDG_RST = (1u << 29), // check independent watchdog reset flag
            rWWDG_RST = (1u << 30), // check window watchdog reset flag
            rLPWR_RST = (1u << 31), // check power-up/power-down reset flag
        };
    };

private:
    Speed speed;

public:
    Rcc(Speed s) : speed(s) {
        (void)speed;
    }

    // reset rcc settings to default
    static inline void DeInit() {
        Ctlr::reset();
        Cfgr0::reset();
        Intr::reset();                                    // unset all interrupts
        Intr::hi(Intr::sALL_CLR);                         // clear all interrupt flags
        RCC_AdjustHSICalibrationValue(0x10);
    }

    static inline void Init() {
        Ctlr::hi(Ctlr::sHSI);
        while (!Ctlr::read(Ctlr::rHSI)) {  // wait while HSI is not ready
            ;
        }
        Cfgr0::reset();                                   // clear all settings                                  
        Ctlr::lo(Ctlr::sPLL | Ctlr::sHSE | Ctlr::sCSS);   // disable PLL, CSS, HSE
        Ctlr::lo(Ctlr::sHSE_BYP);                         // disable HSE_BYPASS (must be done after hse is disabled)
        Cfgr0::write(Cfgr0::wPLL_HSI, Cfgr0::wmPLL);      // set hsi as pll source
        Intr::reset();                                    // unset all interrupts
        Intr::hi(Intr::sALL_CLR);                         // clear all interrupt flags
    }
};
