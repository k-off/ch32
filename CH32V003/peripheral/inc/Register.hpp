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

/**
 * @brief Basic register functionality: reset register, set bits high or low, write and read values
 * @tparam T    register type (size), ie uint8_t (8-bit), uint16_t (16-bit), uint32_t (32-bit) etc.
 * @tparam addr address of the register, uintptr_t 
 */
template<class T, uintptr_t addr>
class Register {
public:

    /**
     * @brief Resets register to the default value
     */
    static inline void reset() {
        reg() = CLEAR;
    }

    /**
     * @brief Ensures that bits are set high: passing parameter bits=0b0101 will set bits #0 and #3 to logical 1
     * @param bits set of bits to be set high
     */
    static inline void hi(uint32_t bits)  {
        reg() |= bits;
    }

    /**
     * @brief Ensures that bits are set low: passing parameter bits=0b0101 will set bits #0 and #3 to logical 0.
     * @param bits set of bits to be set low
     */
    static inline void lo(uint32_t bits) {
        reg() &= (~bits);
    }

    /**
     * @brief Writes specific register bits: using mask=~0b0110 and value=0b0100 will
     *        clear bits #1 and #2 and then set bit #2 high
     * @param value set of bits to be written to the register
     * @param mask  optional mask to protect bits that are not supposed to be overwritten
     */
    static inline void write(uint32_t value, uint32_t mask=WRITE_ALL) {
        reg() &= mask;
        reg() |= value;
    }

    /**
     * @brief Reads specific register bits: using mask=0b1111 will return 4 least-
     *          significant bits of the register
     * @param mask user defines wich bit(s) to read, default - READ_ALL
     * @return uint32_t bits under the mask
     */
    static inline uint32_t read(uint32_t mask=READ_ALL) {
        return (reg() & mask);
    }

private:

    /**
     * @brief Gets non-const volatile reference to the register
     * @return volatile& non-const reference to the register
     */
    inline static volatile T& reg(void) {
        return *reinterpret_cast<volatile T*>(addr);
    }

    /**
     * @brief Predefined set of register values. Shall be expanded and/or overriden
     *          by child classes.
     */
    enum : T {
        CLEAR     =     0u,
        FILL      = ~CLEAR,
        WRITE_ALL =  CLEAR,
        READ_ALL  =   FILL,
    };
};
