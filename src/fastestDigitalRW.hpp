// https://qiita.com/HypnagogicHallucinations/items/ab03cc9de3218d0e6c0b
#ifndef _HPP_FASTEST_DIGITAL_RW_HPP_
#define _HPP_FASTEST_DIGITAL_RW_HPP_
#include <Arduino.h>

#define _INLINE_ inline __attribute__((always_inline)) 
_INLINE_ void fastestDigitalWrite(uint8_t pin, uint8_t val)
{
#if defined(__AVR_ATmega328P__)
    switch(val) {
        case HIGH:
            switch(pin) {
                // PORTD
                case  0: asm volatile("sbi 0x0b, 0x00"); break;
                case  1: asm volatile("sbi 0x0b, 0x01"); break;
                case  2: asm volatile("sbi 0x0b, 0x02"); break;
                case  3: asm volatile("sbi 0x0b, 0x03"); break;
                case  4: asm volatile("sbi 0x0b, 0x04"); break;
                case  5: asm volatile("sbi 0x0b, 0x05"); break;
                case  6: asm volatile("sbi 0x0b, 0x06"); break;
                case  7: asm volatile("sbi 0x0b, 0x07"); break;
                // PORTB
                case  8: asm volatile("sbi 0x05, 0x00"); break;
                case  9: asm volatile("sbi 0x05, 0x01"); break;
                case 10: asm volatile("sbi 0x05, 0x02"); break;
                case 11: asm volatile("sbi 0x05, 0x03"); break;
                case 12: asm volatile("sbi 0x05, 0x04"); break;
                case 13: asm volatile("sbi 0x05, 0x05"); break;
                // PORTC
                case 14: asm volatile("sbi 0x08, 0x00"); break;
                case 15: asm volatile("sbi 0x08, 0x01"); break;
                case 16: asm volatile("sbi 0x08, 0x02"); break;
                case 17: asm volatile("sbi 0x08, 0x03"); break;
                case 18: asm volatile("sbi 0x08, 0x04"); break;
                case 19: asm volatile("sbi 0x08, 0x05"); break;
            }
        break;
        case LOW:
            switch(pin) {
                // PORTD
                case  0: asm volatile("cbi 0x0b, 0x00"); break;
                case  1: asm volatile("cbi 0x0b, 0x01"); break;
                case  2: asm volatile("cbi 0x0b, 0x02"); break;
                case  3: asm volatile("cbi 0x0b, 0x03"); break;
                case  4: asm volatile("cbi 0x0b, 0x04"); break;
                case  5: asm volatile("cbi 0x0b, 0x05"); break;
                case  6: asm volatile("cbi 0x0b, 0x06"); break;
                case  7: asm volatile("cbi 0x0b, 0x07"); break;
                // PORTB
                case  8: asm volatile("cbi 0x05, 0x00"); break;
                case  9: asm volatile("cbi 0x05, 0x01"); break;
                case 10: asm volatile("cbi 0x05, 0x02"); break;
                case 11: asm volatile("cbi 0x05, 0x03"); break;
                case 12: asm volatile("cbi 0x05, 0x04"); break;
                case 13: asm volatile("cbi 0x05, 0x05"); break;
                // PORTC
                case 14: asm volatile("cbi 0x08, 0x00"); break;
                case 15: asm volatile("cbi 0x08, 0x01"); break;
                case 16: asm volatile("cbi 0x08, 0x02"); break;
                case 17: asm volatile("cbi 0x08, 0x03"); break;
                case 18: asm volatile("cbi 0x08, 0x04"); break;
                case 19: asm volatile("cbi 0x08, 0x05"); break;
            }
        break;
        default:
        break;
    }
#elif defined(ARDUINO_MINIMA) || defined(ARDUINO_UNOR4_MINIMA)
    switch(val) {
        case HIGH:
            switch(pin) {
                // PORTD
                case  0: R_PORT3->PODR_b.PODR1 = 1; break;
                case  1: R_PORT3->PODR_b.PODR2 = 1; break;
                case  2: R_PORT1->PODR_b.PODR5 = 1; break;
                case  3: R_PORT1->PODR_b.PODR4 = 1; break;
                case  4: R_PORT1->PODR_b.PODR3 = 1; break;
                case  5: R_PORT1->PODR_b.PODR2 = 1; break;
                case  6: R_PORT1->PODR_b.PODR6 = 1; break;
                case  7: R_PORT1->PODR_b.PODR7 = 1; break;
                // PORTB
                case  8: R_PORT3->PODR_b.PODR4 = 1; break;
                case  9: R_PORT3->PODR_b.PODR3 = 1; break;
                case 10: R_PORT1->PODR_b.PODR12 = 1; break;
                case 11: R_PORT1->PODR_b.PODR9 = 1; break;
                case 12: R_PORT1->PODR_b.PODR10 = 1; break;
                case 13: R_PORT1->PODR_b.PODR11 = 1; break;
                // PORTC
                case 14: R_PORT0->PODR_b.PODR14 = 1; break;
                case 15: R_PORT0->PODR_b.PODR0 = 1; break;
                case 16: R_PORT0->PODR_b.PODR1 = 1; break;
                case 17: R_PORT0->PODR_b.PODR2 = 1; break;
                case 18: R_PORT1->PODR_b.PODR1 = 1; break;
                case 19: R_PORT1->PODR_b.PODR0 = 1; break;
            }
        break;
        case LOW:
            switch(pin) {
                // PORTD
                case  0: R_PORT3->PODR_b.PODR1 = 0; break;
                case  1: R_PORT3->PODR_b.PODR2 = 0; break;
                case  2: R_PORT1->PODR_b.PODR5 = 0; break;
                case  3: R_PORT1->PODR_b.PODR4 = 0; break;
                case  4: R_PORT1->PODR_b.PODR3 = 0; break;
                case  5: R_PORT1->PODR_b.PODR2 = 0; break;
                case  6: R_PORT1->PODR_b.PODR6 = 0; break;
                case  7: R_PORT1->PODR_b.PODR7 = 0; break;
                // PORTB
                case  8: R_PORT3->PODR_b.PODR4 = 0; break;
                case  9: R_PORT3->PODR_b.PODR3 = 0; break;
                case 10: R_PORT1->PODR_b.PODR12 = 0; break;
                case 11: R_PORT1->PODR_b.PODR9 = 0; break;
                case 12: R_PORT1->PODR_b.PODR10 = 0; break;
                case 13: R_PORT1->PODR_b.PODR11 = 0; break;
                // PORTC
                case 14: R_PORT0->PODR_b.PODR14 = 0; break;
                case 15: R_PORT0->PODR_b.PODR0 = 0; break;
                case 16: R_PORT0->PODR_b.PODR1 = 0; break;
                case 17: R_PORT0->PODR_b.PODR2 = 0; break;
                case 18: R_PORT1->PODR_b.PODR1 = 0; break;
                case 19: R_PORT1->PODR_b.PODR0 = 0; break;
            }
        break;
        default:
        break;
    }
#else
    digitalWrite(pin, val);
#endif
}

_INLINE_ int fastestDigitalRead(uint8_t pin)
{
#if defined(__AVR_ATmega328P__)
    switch(pin) {
        // PIND
        case  0: return (PIND >> 0) & 1;
        case  1: return (PIND >> 1) & 1;
        case  2: return (PIND >> 2) & 1;
        case  3: return (PIND >> 3) & 1;
        case  4: return (PIND >> 4) & 1;
        case  5: return (PIND >> 5) & 1;
        case  6: return (PIND >> 6) & 1;
        case  7: return (PIND >> 7) & 1;
        // PINB
        case  8: return (PINB >> 0) & 1;
        case  9: return (PINB >> 1) & 1;
        case 10: return (PINB >> 2) & 1;
        case 11: return (PINB >> 3) & 1;
        case 12: return (PINB >> 4) & 1;
        case 13: return (PINB >> 5) & 1;
        // PINC
        case 14: return (PINC >> 0) & 1;
        case 15: return (PINC >> 1) & 1;
        case 16: return (PINC >> 2) & 1;
        case 17: return (PINC >> 3) & 1;
        case 18: return (PINC >> 4) & 1;
        case 19: return (PINC >> 5) & 1;
        default:
        return 0;
    }
#elif defined(ARDUINO_MINIMA) || defined(ARDUINO_UNOR4_MINIMA)
    switch(pin) {
        // PIND
        case  0: return R_PORT3->PIDR_b.PIDR1;
        case  1: return R_PORT3->PIDR_b.PIDR2;
        case  2: return R_PORT1->PIDR_b.PIDR5;
        case  3: return R_PORT1->PIDR_b.PIDR4;
        case  4: return R_PORT1->PIDR_b.PIDR3;
        case  5: return R_PORT1->PIDR_b.PIDR2;
        case  6: return R_PORT1->PIDR_b.PIDR6;
        case  7: return R_PORT1->PIDR_b.PIDR7;
        // PINB
        case  8: return R_PORT3->PIDR_b.PIDR4;
        case  9: return R_PORT3->PIDR_b.PIDR3;
        case 10: return R_PORT1->PIDR_b.PIDR12;
        case 11: return R_PORT1->PIDR_b.PIDR9;
        case 12: return R_PORT1->PIDR_b.PIDR10;
        case 13: return R_PORT1->PIDR_b.PIDR11;
        // PINC
        case 14: return R_PORT0->PIDR_b.PIDR14;
        case 15: return R_PORT0->PIDR_b.PIDR0;
        case 16: return R_PORT0->PIDR_b.PIDR1;
        case 17: return R_PORT0->PIDR_b.PIDR2;
        case 18: return R_PORT1->PIDR_b.PIDR1;
        case 19: return R_PORT1->PIDR_b.PIDR0;
        default:
        return 0;
    }
#else
    return digitalRead(pin);
#endif
}

#endif  /* _HPP_FASTEST_DIGITAL_RW_HPP_ */