/*
 * Copyright (c) 2015, David R. Chase.
 * This work is licensed under the Creative Commons Attribution 4.0 International License.
 * To view a copy of this license, visit
 * http://creativecommons.org/licenses/by/4.0/
 * or send a letter to
 * Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
 */

#include <stdint.h>
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000UL        // Sets up the default speed for delay.h
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/pgmspace.h>

// #include "/Users/dr2chase/simavr/simavr/sim/avr/avr_mcu_section.h"

/* Clock at 8MHz */
#define X8 1

#define TEMP_ENABLED 0 // Goofed in the early board and wired AREF to AVCC.
// this prevents use of 1.1v for ADC reference, necessary for temperature measurement.

#define ACCELERATION_FLASH 1
#define DOUBLER_TEST 0

// Critical voltages.  32 should never be exceeded.
#define V_MAX 35 // established by voltage divider
#define VOLTAGE(x) (((x)*1024 + 17)/V_MAX)
#define FUDGE_MAX 20
const uint16_t V_SHUNT    = VOLTAGE(31); // Dump power to shunt
const uint16_t V_TOO_HI   = VOLTAGE(30); // Dump power to lights
const uint16_t V_DBLR_OFF = VOLTAGE(27); // Disable doubler from doubled state
const uint16_t V_DBLR_ON  = VOLTAGE(13); // Enable doubler from undoubled state
const uint16_t V_SEEMS_ON = VOLTAGE(12); // For whatever reason, there is power
const uint16_t V_FUDGE_1  = VOLTAGE(11); // Decrease current if below,
                                         // from 50=200mA at 11 interpolated down to
                                         // 30 = 120mA @ 9 volts.
const uint16_t V_TOO_LO   = VOLTAGE(9);  // Enable battery

static inline uint8_t fudge_amount(uint16_t v) {
    if (v >= V_FUDGE_1) return 0;
    int16_t diff = v - V_TOO_LO; //
    if (diff <= 0) return FUDGE_MAX;
    // 0 < diff < 2 * 1024/35 = 58.51
    // full scale 20 for fudge amount
    // i.e., fudge = 20 * diff * 35 / 2048
    // fudge = (diff * 700) >> 11
    // with rounding: fudge = (diff * 700 + 1024) >> 11
    // NOTE Hardcoded dependence on V_FUDGE_1 - V_TOO_LO = 2
    return (uint8_t)(((uint16_t) diff * FUDGE_MAX * V_MAX + 1024) >> 11);
}

#define SEC_TICKS 4000
#define MIN (60*SEC_TICKS)

#define COUNTER_TOP 250
#define uS_PER_TICK COUNTER_TOP

// State bits
#define OFF 0
#define STOPPED 1
#define ROLLING 2
#define DOUBLER_OFF 4
#define BATTERY_ON 8
#define ACCEL_PLUS 16
#define ACCEL_MINUS 32
#define OVERVOLTAGE 64

volatile uint8_t state;
static inline void set_state_int(uint8_t new_state) {
    state = state | new_state;
}
static void set_state(uint8_t new_state) {
    cli();
    set_state_int(new_state);
    sei();
}
static inline void reset_state_int(uint8_t new_state) {
    state = state & ~new_state;
}

static void reset_state(uint8_t new_state) {
    cli();
    reset_state_int(new_state);
    sei();
}

static void replace_state(uint8_t old_state, uint8_t new_state) {
    cli();
    state = (state & ~old_state) | new_state;
    sei();
}

#define STATE_IS(x) ((state & (x)) != 0)
#define STATE_IS_NOT(x) ((state & (x)) == 0)

// Events from scheduling queue
#define EVENT_TEST1 1
#define EVENT_TEST2 2
#define EVENT_TEST3 3
#define EVENT_TEST4 4

uint8_t event_test4 = 1;

#define EVENT_ZERO     7
#define EVENT_ADC_DONE 8
#define EVENT_STATE_CHANGE 9

#define EVENT_DOUBLER_TEST_ON 10
#define EVENT_DOUBLER_TEST_OFF 11

#define EVENT_DISPLAY 12

#ifdef OLD_PINS
#define BATTERY_ON_PIN _BV(PD3)
#define DOUBLER_OFF_PIN _BV(PD2)
// JAM_OFF is normally Tri-state
#define D_OUTPINS BATTERY_ON_PIN|DOUBLER_OFF_PIN|SHUNT_ENABLE
#define B_OUTPINS _BV(PB2)|_BV(PB1)

#else
#define NEW_PINS
#define BATTERY_ON_PIN _BV(PD0)  // Connects battery when hi.
#define DOUBLER_OFF_PIN _BV(PD1) // Disables doubler when hi.
#define JAM_OFF _BV(PD2)         // Jams LED current to zero when hi.
#define SHUNT_ENABLE _BV(PD3)    // Enables power dumping shunt when hi.
// JAM_OFF is normally Tri-state
#define D_OUTPINS BATTERY_ON_PIN|DOUBLER_OFF_PIN|SHUNT_ENABLE|_BV(PD4)|_BV(PD5)
#define B_OUTPINS _BV(PB2)|_BV(PB0)|_BV(PB1)|_BV(PB6)|_BV(PB7)
#endif

/* Encoding of 32-bit unsigned numbers into 16-bit floating point. */

uint32_t expand(uint16_t x) {
    uint8_t exp = x >> 12;
    if (exp <= 1)
        return x;
    {
        uint32_t mantissa = (x & 4095) + (exp == 0 ? 0 : 4096);
        return mantissa << (exp - 1);
    }
}

uint16_t contract(uint32_t x) {
    if (x <= 8192)
        return x;
    {
        // x is <= 8191 << 14
        uint32_t y = x;
        uint8_t exp = 0;
        uint16_t frac;
        uint16_t mask;
        while (y > 8191) {
            y = y >> 1;
            exp++;
        }
        /* rounding; nearest, zero if tie.
         examine the "fraction" bits below
         the mantissa.
         */
        mask = (1 << exp) - 1;
        frac = x & mask;
        frac += mask >> 1;      // frac += .5 - ulp
        frac += (x >> exp) & 1; // add low bit of mantissa; rounds up if odd
        frac = frac >> exp;     // lose the fraction bits, keep the potential 1
        y = y + frac;
        /* Hack for exponent correct.
         Implicit 1 bit is added to exponent.
         If rounding overflowed y to 8192,
         that adds two to the exponent,
         which is what we want.
         */
        return (exp << 12) + y;
    }
}

/*
 * Log base 2 plus 1
 * 0 -> 0
 * 1 -> 1
 * 2,3 -> 2
 * etc
 */
uint8_t l2p1(uint8_t x) {
    uint8_t t = 0;
    uint8_t y = x >> 4;
    if (y != 0) {
        t += 4;
        x = y;
    }
    y = x >> 2;
    if (y != 0) {
        t += 2;
        x = y;
    }
    y = x >> 1;
    if (y != 0) {
        t += 1;
        x = y;
    }
    if (x != 0)
        t++;

    return t;
}

#define N_IMMEDIATE 16
uint16_t do_now[N_IMMEDIATE];
uint16_t n_immediate;

#define N_EVENTS 64
struct heap {
    uint32_t next_time;
    uint16_t times[N_EVENTS];
    uint8_t events[N_EVENTS];
    uint8_t last;

} h;

void adjust_child(uint8_t child_i, uint32_t adjust) {
    h.times[child_i] = contract(adjust + expand(h.times[child_i]));
}

void adjust_children(uint8_t node, uint32_t adjust) {
    node <<= 1;
    adjust_child(node, adjust);
    adjust_child(node + 1, adjust);
}

/*
 * There's no point in wasting time/space to log interrupt
 * events into the heap; register them for immediate attention,
 * which can include processing into other events.
 */
void immediate_insert(uint16_t what) {
    if (n_immediate < N_IMMEDIATE) {
        do_now[n_immediate++] = what;
        return;
    }
}

/*
 * Schedule "what" to occur "at" ticks in the future.
 * A tick is (currently) 250 microseconds, or 1/4000 of a second.
 * For an 8Mhz clock, this corresponds to 2000 cycles.
 */
void heap_insert(uint32_t at, uint16_t what) {
    int l = h.last;
    if (l == 1) {
        h.next_time = at;
        h.last = 2;
        h.events[1] = what;
        h.times[1] = 0; // special case
    } else {
        int s = l2p1(l);
        // Make at, what be after first item in heap.
        // Swap if necessary.
        uint32_t delta = h.next_time;
        s--;
        uint8_t i = l >> s;
        while (1) {
            if (at < delta) {
                uint16_t s = h.events[i];
                h.events[i] = what;
                what = s;

                if (i == 1)
                    h.next_time = at;
                else
                    h.times[i] = contract(at);

                at = delta - at;
                adjust_children(i, at);
            } else {
                at -= delta;
            }
            if (s <= 1)
                break;
            s--;
            i = l >> s;
            delta = expand(h.times[i]);
        }

        h.events[l] = what;
        h.times[l] = contract(at);
        h.last = l + 1;
    }
}

uint16_t heap_remove(void) {
    uint8_t l = h.last - 1;
    if (l < 2) {
        if (l != 1) {
            return 0;
        } else {
            h.last = 1;
            h.next_time = 0;
            return h.events[1];
        }
    }
    uint16_t result = h.events[1];

    uint8_t s = l2p1(l);

    // Reconstitute scheduled time
    uint32_t at = 0;
    while (s > 0) {
        s--;
        at += expand(h.times[l >> s]);
    }

    uint16_t what = h.events[l];

    h.last = l;
    uint8_t i = 1;

    uint8_t ii0 = i << 1;

    while (ii0 < l) {
        uint8_t ii1 = ii0 + 1;
        // Note stores to ii1 are legal, since ii0 is less than l.

        uint32_t t0 = expand(h.times[ii0]);
        uint32_t t1 = ii1 < l ? expand(h.times[ii1]) : UINT32_MAX;

        if (at <= t0 && at <= t1) {
            /* at <= c1, c2 */
            t0 -= at;
            t1 -= at;
            h.times[ii0] = contract(t0);
            h.times[ii1] = contract(t1);
            break;
        }

        /* Smaller bubbles up */
        if (t0 > t1) {
            // Swap roles to make t0 smaller
            uint32_t tt = t0;
            t0 = t1;
            t1 = tt;

            uint8_t iit = ii0;
            ii0 = ii1;
            ii1 = iit;
        }

        h.times[i] = h.times[ii0];
        h.events[i] = h.events[ii0];
        t1 -= t0;
        h.times[ii1] = contract(t1);
        at -= t0;
        i = ii0;
        ii0 = i << 1;
    }

    if (i == 1) {
        h.next_time = at;
    } else {
        h.times[i] = contract(at);
        h.next_time = expand(h.times[1]);
        h.times[1] = 0;
    }
    h.events[i] = what;

    return result;
}

/*
 * Returns "is the queue of immediate-service events empty?"
 */
uint16_t immed_empty(void) {
    return (n_immediate == 0);
}

/**
 * Returns non-zero if there is any immediate or scheduled work to do,
 * giving priority to immediate work.
 */
uint16_t heap_cond_remove(void) {
    // Check for any fresh interrupt work.
    cli();
    if (n_immediate > 0) {
        uint16_t x = do_now[--n_immediate];
        sei();
        return x;
    }
    sei();

    // The heap proper is not touched from interrupts;
    // minimize interrupt blocking times.
    if (h.next_time == 0)
        return heap_remove();
    return 0;
}

void heap_init(void) {
    h.last = 1;
    h.next_time = 0;
}

/*
 * This computes x cubed, right-shifted by 36.
 *
 * ups = 1000000 // microseconds per second
 * C = 2.1       // circumference in meters
 * N = 28        // zeroes per revolution
 * m_per_z = C/N // meters traveled per zero
 *
 * observed_delta_at_mps(x, a) = ups * a * m_per_z^2 / (x ^ 3 + x * a * m_per_z)
 * observed_u_at_mps(x)        = m_per_z * ups / x
 * observed_u_at_mps(x)^3      = m_per_z^3 * ups^3 / x^3
 * observed_u^3 / observed_delta = a * m_per_z * ups^2 (roughly)
 *                               = 1 * 0.075 * 1,000,000,000,000
 *                               = 2^36.126
 *
 * Doubling the accumulation time doubles observed_u,
 * but quadruples observed delta
 * with the result that cube>>36 divided by  delta
 * increases by a factor of 2 for the same acceleration.
 * Hence, for sample window 2^i, the comparison to make is
 * cube>>36 vs observed_delta << i
 *
 * 0.447 mps = 1 mph
 * mps_from_mph(x) = 0.447 * x
 */
#define OVERFLOW 0xffffffffL
uint32_t cube_rshift36(uint32_t x) {
    char big = 0;
    while (x >= 0x10000) {
        big++;
        x = x >> 4;
    }

    if (big == 3)
        return OVERFLOW;

    uint8_t a = x >> 8;
    uint8_t b = x;

// To compute a full cube from 8-bit:
//    (aaH, aaL) = a * a = (24, 16)
//    (bbH, bbL) = b * b = (8, 0)
//    (aaHaH, aaHaL) = aaH * a = (40, 32)
//    (aaLaH, aaLaL) = aaL * a = (32, 24)
//    (aaHbH, aaHbL) = aaH * b = (32, 24)

//    (aaLbH, aaLbL) = aaL * b = (24, 16)
//    (bbHaH, bbHaL) = bbH * a = (24, 16)

//    (bbLaH, bbLaL) = bbL * a = (16, 8)
//    (bbHbH, bbHnL) = bbH * b = (16, 8)
//    (bbLbH, bbLbL) = bbL * b = (8, 0)
//
// Then do *3 as necessary.
// Error is less than one part in 1024.
//
// If big = 0, then result is bits 47-36
// If big = 1, then result is bits 47-24 (note 24 bits!)
// If big = 2, then result is bits 47-12 (note 36 bits -- too many)
// Return 0xffffffff for overflow.
    uint16_t aaHaaL = (uint16_t) a * a;
    uint16_t bbHbbL = (uint16_t) b * b;
    uint8_t aaH = aaHaaL >> 8;
    uint8_t aaL = aaHaaL;
    uint8_t bbH = bbHbbL >> 8;
    uint8_t bbL = bbHbbL;

    uint32_t aaHaHaaHaL = (uint16_t) aaH * a; // 47-32
    uint32_t aaLaHaaLaL = (uint16_t) aaL * a; // 39-24
    uint32_t aaHbHaaHbL = (uint16_t) aaH * b; // 39-24

    if (big == 0) { // 47-36
        uint8_t bit = (uint8_t) (((aaLaHaaLaL & 4095) + 3 * (aaHbHaaHbL & 4095)
                + ((aaHaHaaHaL & 15) << 8)) >> 12);
        return (aaHaHaaHaL >> 4) + (aaLaHaaLaL >> 12) + 3 * (aaHbHaaHbL >> 12)
                + bit;
    }

    uint32_t aaLbHaaLbL = (uint16_t) aaL * b; // 31-16
    uint32_t bbHaHbbHaL = (uint16_t) bbH * a; // 31-16

    uint32_t bbLaHbbLaL = (uint16_t) bbL * a; // 23-8
    uint32_t bbHbHbbHbL = (uint16_t) bbH * b; // 23-8

    uint32_t bbLbHbbLbL = (uint16_t) bbL * b; // 15-0

    if (big == 1) { // 47-24 (24 bits of shift remaining)
        uint32_t partial = (3 * bbLaHbbLaL + bbHbHbbHbL + (bbLbHbbLbL >> 8))
                >> 8;
        partial = (partial + 3 * (aaLbHaaLbL + bbHaHbbHaL)) >> 8;
        partial += (aaHaHaaHaL << 8) + aaLaHaaLaL + 3 * aaHbHaaHbL;
        return partial;
    }

    if (big == 2) { // 47-12 -- 36 bits, can overflow.
                    // 12 bits of shift remaining.

        uint32_t partial = (aaHaHaaHaL << 8) + (aaLaHaaLaL) + 3 * (aaHbHaaHbL);

        if (partial >> 19 != 0)
            return OVERFLOW;

        partial = partial << 12;

        return partial + 3 * ((aaLbHaaLbL << 4) + (bbHaHbbHaL << 4))
                + (bbLbHbbLbL >> 12);
    }

    return OVERFLOW;
}

volatile int16_t inc_C = 0;  // to add to C at next timer interrupt.
volatile int32_t C = 0;      // microseconds currently accumulated
volatile int32_t tz = 0;    // microseconds between last zero crossings
volatile uint16_t crossings; // number of crossings recently observed
volatile uint32_t ticks_since_crossing; // (250 uS) ticks since last zero crossing.

ISR(ANALOG_COMP_vect) {
    uint8_t c = TCNT1L; // Capture this as cleanly as possible.
    crossings++;
    if (TIFR1 & _BV(ICF1)) {
        // Timer interrupt is pending, therefore the increments have
        // not yet occurred.  Do them here instead and prebias C to be negative.
        tz = uS_PER_TICK + C + c;
        C = -uS_PER_TICK; // + 250 - c + 250 = 250 - c
    } else {
        // No pending timer interrupt, therefore what we see is what we get.
        // Microseconds is simply accumulated plus how many elapsed since last
        // timer interrupt.
        tz = C + c;
        C = 0;    // + 250 - c = 250 - c
    }
    // the next zero crossing also needs this count.
    inc_C = uS_PER_TICK - c;
    immediate_insert(EVENT_ZERO);
    ticks_since_crossing = 0;
    // At the zero crossing be sure to set the doubler into the
    // proper state.  Doing it here avoids high currents.
    if (STATE_IS(DOUBLER_OFF)) {
        PORTD |= DOUBLER_OFF_PIN;
    } else {
        PORTD &= ~DOUBLER_OFF_PIN;
    }
}

volatile uint16_t ticks;
ISR(TIMER1_OVF_vect) {
    ticks++;
    ticks_since_crossing++;
    C += inc_C;
    inc_C = COUNTER_TOP;
}

void pwm_init(void) {
    /*
     * No clock scaling (1MHz) (CS12:10 = 001b)
     * Fast PWM, top = ICR1 (WGM13:10 = 1110b)
     * Top = 250.
     */
    /* Note option of scaling by 8 (CS11) and
     * counting up to 32 (or 31), and setting
     * power levels to 6 and 12.
     */
    ICR1 = COUNTER_TOP;
    OCR1B = COUNTER_TOP/5; // 20% of 250.
    TCCR1A = _BV(COM1B1) | _BV(WGM11);
#ifdef X8
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // CS11 for /8
#else
            TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // CS11 for /8
#endif
    TIFR1 = _BV(TOV1);
    TIMSK1 = _BV(TOIE1); // Enable overflow interrupts @ 4kHz
}

#define ADMUX_REF_AREF  _BV(REFS0)
#define ADMUX_REF_1_1  (_BV(REFS0) | _BV(REFS1))

#define ADMUX_5_times_I  (0 | ADMUX_REF_AREF) // ADC0

#if OLD_PINS
#define  ADMUX_v_over_7  (5 | ADMUX_REF_AREF) // ADC5
#else
#define  ADMUX_v_over_7  (1 | ADMUX_REF_AREF) // ADC1
#endif

#define  ADMUX_temp      (8 | ADMUX_REF_1_1)  // Temperature, if TEMP_ENABLED

/* Analog configuration -- 1M/16 (= 62.5kHz), Vcc ref, from PB2
 * adps2:0 = log adc clock scale.
 * 10 bits requires 50kHz-200kHz
 * 8M/64 => 125kHz, requires PS2 and PS1
 *
 */
#ifdef X8
const uint8_t adcsra_enable = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2)
        | (1 << ADPS1);
const uint8_t adcsra_start = (1 << ADSC) | (1 << ADEN) | (1 << ADIE)
        | (1 << ADPS2) | (1 << ADPS1);
#else
const uint8_t adcsra_enable = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2);
const uint8_t adcsra_start = (1<<ADSC)|(1<<ADEN)|(1<<ADIE)|(1<<ADPS2);
#endif
const uint8_t adcsrb = 0;

#define ADC_SRC_5I 0
#define ADC_SRC_Vo7 1
#if TEMP_ENABLED
#define ADC_SRC_temp 2
#define N_ADC_SOURCES 3
#else
#define N_ADC_SOURCES 2
#endif

uint8_t adc_sources[N_ADC_SOURCES];
uint16_t adc_results[N_ADC_SOURCES];
uint8_t which_adc = 0;

void start_adc(void) {
    ADMUX = adc_sources[which_adc];
    ADCSRA = adcsra_start;
}

void adc_init(void) {
    adc_sources[ADC_SRC_5I] = ADMUX_5_times_I;
    adc_sources[ADC_SRC_Vo7] = ADMUX_v_over_7;
#if TEMP_ENABLED
    adc_sources[ADC_SRC_temp] = ADMUX_temp;
#endif

    ADCSRA = adcsra_enable;
    ADCSRB = adcsrb;
    DIDR0 = _BV(ADC5D) | _BV(ADC0D); /* disable digital input on B0,5 */

    start_adc();
}

/* Completed analog to digital conversion, move to next measurement
 * or signal done if last one was completed.
 */
ISR(ADC_vect) {
    uint8_t adcl = ADCL;
    uint8_t adch = ADCH;
    uint16_t result = ((uint16_t) adch << 8) + adcl;
    adc_results[which_adc++] = result;
    if (which_adc < N_ADC_SOURCES)
        start_adc();
    else {
        which_adc = 0;
        immediate_insert(EVENT_ADC_DONE);
    }
}

#define SCREEN_SIZE 80
char screen[SCREEN_SIZE];

struct screen_data {
    char * data;
    uint8_t size;
};

char start_string[2] = {0xFE, 0x41};
char stop_string[2] = {0xFE, 0x42};
char contrast50_string[3] = {0xFE, 0x52, 25}; // 1-50
char brightness50_string[3] = {0xFE, 0x53, 4}; // 1-8

struct screen_data normal_display = {screen, sizeof(screen)};
struct screen_data start_display = {start_string, sizeof(start_string)};

struct screen_data * screen_data_ptr;
int8_t cursor;
int8_t screen_state;

#define SCREEN_STOPPED 0
#define SCREEN_STARTING 1
#define SCREEN_ADDRESS 2
#define SCREEN_DATA 3
#define SCREEN_STOPPING 4

void twi_step(uint8_t twie) {
    switch (screen_state) {
    // screen state is where we expect to be when interrupt arrives.
    case SCREEN_STARTING:
        // TODO check status
        // starting; send address
        TWDR = 0x50 << 1; // Newhaven display
        TWCR = _BV(TWINT)|_BV(TWEN)|twie;
        screen_state = SCREEN_ADDRESS;
        break;
    case SCREEN_ADDRESS:
        // TODO check status from address
        cursor = 0;
        screen_state = SCREEN_DATA;
        goto data_tail;
    case SCREEN_DATA:
        // TODO check status from data
        data_tail:
        if (cursor >= screen_data_ptr->size) {
            screen_state = SCREEN_STOPPING;
            TWCR = _BV(TWSTO)|_BV(TWINT)|_BV(TWEN)|twie;
        } else {
            TWDR = screen_data_ptr->data[cursor++];
            TWCR = _BV(TWINT)|_BV(TWEN)|twie;
        }
        break;
    case SCREEN_STOPPING:
        // TODO check status from stop command
        screen_state = SCREEN_STOPPED;
        break;
    case SCREEN_STOPPED: // do nothing
        break;
    }
}

void display_start(struct screen_data * data, int use_interrupt) {
    cursor = -1;
    screen_state = SCREEN_STARTING;
    screen_data_ptr = data;

    if (use_interrupt) {
        TWCR = _BV(TWSTA)|_BV(TWINT)|_BV(TWEN)|_BV(TWIE);
        return;
    } else {
        TWCR = _BV(TWSTA)|_BV(TWINT)|_BV(TWEN);
        while (screen_state != SCREEN_STOPPED) {
            while(!(TWCR & _BV(TWINT))) ;
            twi_step(0);
        }
    }
}

void display_init(void) {
    int i;

    DDRC |= _BV(PC4)|_BV(PC5)|_BV(PC2)|_BV(PC3);
    PORTC = _BV(PC4)|_BV(PC5);

    TWBR = 32; // 100kHz
    TWSR &= ~3; // TWPS = 0
    TWCR = _BV(TWEN);

    // write a test pattern
    char c = ' ';
    for (i = 0; i < SCREEN_SIZE; i++) {
        screen[i] = c; c++;
    }
    display_start(&start_display, 0);
    display_start(&normal_display, 0);
}

ISR(TWI_vect) {
    twi_step(_BV(TWIE));
}

// ectr is a rapidly overflowing index of events used to help
// accumulate averaged samples.
#define HIST_LIM 8
struct history {
    uint16_t ectr;
    uint32_t Asum[HIST_LIM]; // most recent sum of 2**i events.
    uint32_t Bsum[HIST_LIM]; // previous sum of 2**i events.
    uint32_t Amin[HIST_LIM]; // most recent min of 2**i events.
    uint32_t Bmin[HIST_LIM]; // previous min of 2**i events.
    uint32_t Amax[HIST_LIM]; // most recent min of 2**i events.
    uint32_t Bmax[HIST_LIM]; // previous min of 2**i events.
};

#define H1 0
#define H2 1
#define H4 2
#define H8 3
#define H16 4
#define H32 5
#define H64 6
#define H128 7

static struct history zero_hist;
static struct history voltage_hist;

inline uint32_t min(uint32_t x, uint32_t y) {
    return x > y ? y : x;
}

inline uint32_t max(uint32_t x, uint32_t y) {
    return x < y ? y : x;
}

// incorporate new data into a history; returns the index of highest
// order change in the history.
uint8_t update_history(struct history * hist, uint32_t new_data) {
     uint16_t ectrp = hist -> ectr + 1;
     uint16_t flips = hist -> ectr ^ ectrp;
     hist -> ectr = ectrp;
     hist -> Bsum[0] = hist -> Asum[0];
     hist -> Bmin[0] = hist -> Amin[0];
     hist -> Bmax[0] = hist -> Amax[0];
     hist -> Asum[0] = new_data;
     hist -> Amin[0] = new_data;
     hist -> Amax[0] = new_data;

     flips &= (1 << HIST_LIM)-1;
     // suppose HIST_LIM = 1, flips &=1, downshift 1 = 0, okay.
     flips >>= 1;
     int i = 0;
     while (flips != 0) {
         int ip1 = i + 1;
         hist -> Bsum[ip1] = hist -> Asum[ip1];
         hist -> Asum[ip1] = hist -> Asum[i] + hist -> Bsum[i];
         hist -> Bmin[ip1] = hist -> Amin[ip1];
         hist -> Amin[ip1] = min(hist -> Amin[i], hist -> Bmin[i]);
         hist -> Bmax[ip1] = hist -> Amax[ip1];
         hist -> Amax[ip1] = max(hist -> Amax[i], hist -> Bmax[i]);
         flips >>= 1;
         i = ip1;
     }
    return i;
}

// Record the ticks at battery-on,
// reduce that slightly and use that
// as the trigger to turn the battery off.
uint32_t battery_off_again = 33000; // about 5mph

// Sample index for battery off -- log2 of # of zeroes
#define TBZ_SAMPLING_LOG 4

// If this gets too large, we decide that we are stopped.
int8_t check_stopped = 0; //Increment every CHECK_STOPPED_INTERVAL
#define CHECKS_TO_DECLARE_STOPPED 8
#define CHECK_STOPPED_INTERVAL (SEC_TICKS>>4) // 1/16 second

// Require 4 zero crossings in one second to unstop.
// 4 x 0.075m = about a foot.
#define ZEROES_TO_UNSTOP 4
// When this is zero or negative, we are "rolling".
int8_t check_rolling = 0;

#define STANDLIGHT_SECONDS 60
int16_t standlight_countdown = 0;

#define N_STEPS 25
const uint8_t wv_sine_delays[N_STEPS] PROGMEM =
      { 13, 12, 13, 13, 13,
        13, 13, 14, 13, 14,
        14, 14, 15, 15, 16,
        16, 17, 18, 19, 20,
        22, 25, 30, 38, 90 };

const uint8_t wv_square_sine_delays[N_STEPS] PROGMEM =
      { 64, 27, 22, 18, 17,
        15, 14, 14, 14, 13,
        13, 13, 12, 13, 13,
        13, 14, 14, 14, 15,
        17, 18, 22, 27, 64 };

const uint8_t wv_cube_sine_delays[N_STEPS]  PROGMEM =
     { 111, 31, 22, 19, 16,
        14, 14, 13, 12, 12,
        11, 11, 11, 11, 11,
        12, 11, 12, 12, 13,
        14, 15, 17, 23, 52 };

const uint8_t wv_fifth_sine_delays[N_STEPS]  PROGMEM =
     { 176, 30, 21, 17, 14,
        13, 11, 11, 10, 10,
        10,  9,  9,  9,  9,
         9,  9,  9,  9, 11,
        10, 12, 14, 17, 41 };

const uint8_t wv_fastest_ramp[N_STEPS]  PROGMEM = {
        1,1,1,1,1,
        1,1,1,1,1,
        1,1,1,1,1,
        1,1,1,1,1,
        1,1,1,1,1
};

const uint8_t wv_fast_ramp[N_STEPS+N_STEPS]  PROGMEM = {
        // .1 second
        16,16,16,16,16,
        16,16,16,16,16,
        16,16,16,16,16,
        16,16,16,16,16,
        16,16,16,16,16,

        16,16,16,16,16,
        16,16,16,16,16,
        16,16,16,16,16,
        16,16,16,16,16,
        16,16,16,16,16
};

const uint8_t wv_medium_ramp[N_STEPS]  PROGMEM = {
        // .5 second.
        80, 80, 80, 80, 80,
        80, 80, 80, 80, 80,
        80, 80, 80, 80, 80,
        80, 80, 80, 80, 80,
        80, 80, 80, 80, 80
};

const uint8_t wv_slow_ramp[N_STEPS] PROGMEM =   {
        // 1 second.
        160,160,160,160,160,
        160,160,160,160,160,
        160,160,160,160,160,
        160,160,160,160,160,
        160,160,160,160,160
};

const uint8_t wv_slower_ramp[N_STEPS] PROGMEM =   {
        // 1.5 second.
        240,240,240,240,240,
        240,240,240,240,240,
        240,240,240,240,240,
        240,240,240,240,240,
        240,240,240,240,240
};


struct flash_curve {
    uint8_t n_steps[4];
    int8_t delta[4];
    const uint8_t *(curve[4]);
};

struct flash_curve wv_running =
{{25, 25, 25, 25},
 {1, -1, -1, 1},
 {wv_cube_sine_delays, wv_cube_sine_delays,
  wv_cube_sine_delays, wv_cube_sine_delays}};

struct flash_curve wv_stopped =
{{50, 25, 50, 25},
 {-1, 0, 1, 0},
 {wv_fast_ramp, wv_slower_ramp, wv_fast_ramp, wv_slower_ramp}};

struct flash_curve * wv_curve = & wv_running;

uint8_t wv_next_phase(uint8_t p) {
    return 3 & (p - 1);
}

uint16_t record;

int8_t not_stopped_conditions(void) {
    // We've been stopped, see if we are still stopped.
    int8_t not_stopped =
            (check_rolling <= 0) ||
            STATE_IS(ROLLING) ||
            (voltage_hist.Asum[H128] > V_SEEMS_ON);
    return not_stopped;
}

void accel_change(void) {
    // acceleration just changed.
}

#define LO 50
#define HI 100
// Current light level (250=1A, 50 = 200mA, 100 = 400mA)
uint8_t fudge; // adjusted according to voltage

uint8_t wv_phase; // phase of curve (quadrant of a circle).
uint8_t wv_index; // position within phase
int16_t wv_offset = 0; // curve offset from goal
uint8_t wv_log_step = 0; // 0 for low current, 1 for high current
uint8_t wv_goal = 51;  // neutral level (changes w/ stop, double, single)

void doubler_off(uint8_t increase_current) {
    // turn doubler off, increase goal current
    if (increase_current) {
        wv_goal = HI;
        wv_log_step = 1;
    }
    set_state(DOUBLER_OFF); // will take effect at next zero crossing
}

void doubler_on(void) {
    // turn doubler on, decrease goal current
    wv_goal = LO;
    wv_log_step = 0;
    reset_state(DOUBLER_OFF); // will take effect at next zero crossing
}

void process(uint16_t event) {
    switch (event) {

    case EVENT_TEST1:
        break;

    case EVENT_TEST2:
        break;

    case EVENT_TEST3: {
        // Sanity -- all curves reset to start at goal.
        if (wv_phase == 0 && wv_index == 0)
            wv_offset = 0;

        // Clip all computed curve values to a sensible range.
        // Note that (with current hardware) values below 20 (about 80 mA)
        // don't get any dimmer.
        int16_t candidate =
                (int16_t) wv_goal + (wv_offset << wv_log_step) - fudge;
        if (candidate <= 0) candidate = 0;
        else if (candidate >= COUNTER_TOP) candidate = COUNTER_TOP;

        OCR1B = candidate;

        uint8_t delay = 0;
        const uint8_t *wv_delays = wv_curve->curve[wv_phase];
        int8_t delta = wv_curve->delta[wv_phase];
        uint8_t n_steps = wv_curve->n_steps[wv_phase];

        if (wv_phase & 1) {
            // counting down
            delay = pgm_read_byte(&(wv_delays[--wv_index]));
            if (wv_index == 0) {
                wv_phase = (wv_phase + 1) & 3;
            }

        } else {
            // counting up
            delay = pgm_read_byte(&(wv_delays[wv_index++]));
            if (wv_index == n_steps) {
                wv_phase = (wv_phase + 1) & 3;
            }
        }
        wv_offset += delta;
        heap_insert(delay, EVENT_TEST3);
    }
        break;

    case EVENT_TEST4: {
        PORTB ^= _BV(PB7)|_BV(PB1);
        heap_insert(SEC_TICKS, EVENT_TEST4);
    } break;

    case EVENT_ADC_DONE: {
        record |= 1<<EVENT_ADC_DONE;
        uint16_t voltage = adc_results[ADC_SRC_Vo7]; // 1024 = 35V
        uint16_t current = adc_results[ADC_SRC_5I];  // 1024 = 1A
#if TEMP_ENABLED
        uint16_t temp = adc_results[ADC_SRC_temp];  // 1024 = ??
#endif
        start_adc();

        update_history(&voltage_hist, voltage);

        // fudge adjusts lights/current draw according to voltage
        fudge = fudge_amount(voltage);

        if (voltage > V_TOO_HI) {
            cli();
            if (voltage > V_SHUNT) {
                PORTD |= SHUNT_ENABLE;
                DDRD |= SHUNT_ENABLE;
            } else {
                PORTD &= ~SHUNT_ENABLE;
                DDRD &= ~SHUNT_ENABLE;
            }
            if (STATE_IS_NOT(OVERVOLTAGE)) {
                set_state_int(OVERVOLTAGE | DOUBLER_OFF);
            }
            sei();
            // dump power, overvoltage
            wv_goal = COUNTER_TOP;
            OCR1B = COUNTER_TOP;
        } else if (STATE_IS(ROLLING)) {
            if (STATE_IS(OVERVOLTAGE)) {
                // quit dumping power.
                cli();
                reset_state_int(OVERVOLTAGE);
                PORTD &= ~SHUNT_ENABLE;
                DDRD &= ~SHUNT_ENABLE;
                sei();
            }

            if (voltage < V_TOO_LO) {
                // power too low, battery on
                // record ticks between zeroes at 16-zero sampling, times 7/8
                // battery_off_again = (A[TBZ_SAMPLING_LOG] * 7) >> 3;
                cli();
                PORTD |= BATTERY_ON_PIN;
                set_state_int(BATTERY_ON);
                sei();
            }
            if (voltage > V_DBLR_OFF) {
                doubler_off(1);
            } else if (voltage < V_DBLR_ON) {
                doubler_on();
            }
        }
    }
    break;

    case EVENT_ZERO: {
        int32_t local_tz = tz; // uS since last zero.

        // If recently transitioned to off there can be noise, usually HF
        // 250 * (SEC>>6) is 1/64 of a second (about 10mph), a quick wheel spin,
        // so it is probably not happening (and if it is, the voltage will go
        // up, and we will see that instead).
        if ((state == OFF || STATE_IS(STOPPED)) &&
                                       local_tz < uS_PER_TICK * (SEC_TICKS>>6))
            break;

        record |= 1 << EVENT_ZERO;

        PORTC ^= _BV(PC3);

        check_stopped = 0;
        if (check_rolling > 0) {
            check_rolling--;
        }

        // Update running averages
        // Asum[i] = sum of most recent 2**i times between zeroes.
        // Bsum[i] = sum of previous 2**i times between zeroes.
        // distances correspond to multiples of 7.5cm (fat 26" tire or thin 700c)
        // e.g., 0 -> 7.5, 1 -> 15, 2 -> 30, 3 -> 60, 4 -> 120
        // and 7 -> 960cm = 9.6m = about 31 feet.

        int count = 1;

        // Simulate 'count' iterated zeroes
        int i = 0;
        while (count -- > 0) {
          uint8_t ii = update_history(&zero_hist, local_tz);
          if (ii > i) i = ii;
        }

        if ((state & STOPPED) || state == OFF) {
            // If stopped, leave the lights alone.
            // Enough zero-crossing will lead to an unstopping.
            if (check_rolling <= 0) {
                // TODO this is probably a good place to look for weird
                // power-down artifacts.  Also think about how we would
                // figure times if many zero-crossings occurred in a hurry.
                state = (state & ~STOPPED) | ROLLING;
                check_rolling = 0;
            }
        } else {
            // i = max flipped bit.
            // See if speed has increased past battery disabling level.
            // Note possible interaction with overvoltage; if we check for
            // persistent overvoltage, we might enable the battery (it will
            // sink the overvoltage).
            if (i >= TBZ_SAMPLING_LOG &&
               (state & BATTERY_ON) &&
                zero_hist.Asum[TBZ_SAMPLING_LOG] < battery_off_again) {
                // turn battery off.
                cli();
                reset_state_int(BATTERY_ON);
                PORTD &= ~BATTERY_ON_PIN;
                sei();
            }
            /*
             * Abs(Asum[i] - Bsum[i]) gives diff of 2^i zeroes at 2^(i+1) distance
             * Speed in Asum gives clue as to i to probe.
             *
             * difference (Asum-Bsum) quadruples, time (Asum) doubles but is cubed.
             * cube>>36 the time, shift << i the diff, compare.
             * If diff is larger, then we have acceleration of about .1g.
             * Do not bother to try this until a time is found that is between
             * 2^15 and 2^16.
             */
            int j = 0;
            uint32_t aj = 0;
            for (j = 0; j < i; j++) {
                // Looking for 32-64 ms of sample.
                aj = zero_hist.Asum[j];
                if (aj >= 0x8000)
                    break;
            }
            if (STATE_IS(ROLLING) && aj >= 0x8000) {
                uint32_t cubeshift = cube_rshift36(aj>>j);
                // positive diff means increased speed
                int32_t diff = (int)(zero_hist.Bsum[j] - aj)>>2*j;
                uint8_t accel_state = ACCEL_PLUS;
                if (diff < 0) {
                    diff = -diff;
                    accel_state = ACCEL_MINUS;
                }
                if (ACCELERATION_FLASH && diff > cubeshift) {
                    // more than about .1G acceleration detected.
                    if (0 == (accel_state & state)) {
                        // new state
                        replace_state(ACCEL_PLUS | ACCEL_MINUS, accel_state);
                        accel_change();
                    }
                } else {
                    // no acceleration
                    uint8_t old_accel = state & (ACCEL_PLUS | ACCEL_MINUS);
                    if (old_accel != 0) {
                        reset_state(ACCEL_PLUS | ACCEL_MINUS);
                        // Call the new state running, even though it might
                        // soon be "check_rolling".
                        accel_change();
                    }
                }
            }
        }
    }
    break;

    case EVENT_STATE_CHANGE:
        heap_insert(CHECK_STOPPED_INTERVAL, EVENT_STATE_CHANGE);

        if (check_rolling <= 0) {
            replace_state(STOPPED, ROLLING);
        }

        check_stopped++;
        check_rolling = ZEROES_TO_UNSTOP;

        if (check_stopped >= CHECKS_TO_DECLARE_STOPPED) {
            check_stopped = CHECKS_TO_DECLARE_STOPPED; // avoid oflo.
            if (STATE_IS(STOPPED) || state == OFF) {
                PORTC ^= _BV(PC2);
                if (--standlight_countdown <= 0) {
                    standlight_countdown = 0; // avoid oflo
                    state = OFF;
                    PORTD &= ~BATTERY_ON_PIN;
// TODO low light level
                    DDRD ^= JAM_OFF;
                    PORTD ^= JAM_OFF;
                }
            } else {
                replace_state(ACCEL_PLUS|ACCEL_MINUS|ROLLING, STOPPED);
// TODO low light level
                standlight_countdown = STANDLIGHT_SECONDS *
                        (SEC_TICKS / CHECK_STOPPED_INTERVAL);
            }
        }
        break;


    case EVENT_DOUBLER_TEST_ON:
        doubler_on();
        heap_insert(SEC_TICKS/2, EVENT_DOUBLER_TEST_OFF);
        break;

    case EVENT_DOUBLER_TEST_OFF:
        doubler_off(0); // Do not change goal current
        heap_insert(SEC_TICKS/2, EVENT_DOUBLER_TEST_ON);
        break;

    case EVENT_DISPLAY:
        display_start(&normal_display, 1);
        heap_insert(SEC_TICKS, EVENT_DISPLAY);
        break;
    }
}

/*
 * Set clock prescaler to 2-to-the-log_scale,
 * 0 <= log_scale <= 16.
 * Assumes interrupts are disabled.
 */
void set_prescalar(unsigned char log_scale) {
    log_scale &= 15;
    CLKPR = _BV(CLKPCE);
    CLKPR = log_scale;
}

int main(void) {
    /*
     * Begin with no interrupts.
     */
    cli();
#ifdef X8
    set_prescalar(0);
#endif
    /* Initialize control outputs.
     * PD1 - power override.
     *       hi = on, lo = off-ish, hi-impedance normally.
     * PD2 - hi = doubler off
     * PD3 - hi = battery on
     * PB2/OC1B - PWM setting goal current (2V/4V)
     *            should be 20% duty cycle for 2V, 40% for 4V
     */

    /* Default is no override (tristate), no doubler on, no battery.
     * Do this ASAP.
     */
    PORTD = 0;
    DDRD = D_OUTPINS;
    PORTB = 0;
    DDRB = B_OUTPINS;
    PORTC = 0;
    DDRC = _BV(PC4)|_BV(PC5)|_BV(PC2)|_BV(PC3);

    /* Shutdown unneeded stuff. */
    PRR = _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRSPI) | _BV(PRUSART0);

    /*
     * Prepare event heap for other initializations, which can trigger
     * events.
     */
    heap_init();

    /* Initialize analog comparator input. */
    DIDR1 = _BV(AIN0D) | _BV(AIN1D);
    ACSR = _BV(ACIE); /* Enable analog compare interrupts */

    /* Initialize analog to digital. */
    adc_init();

    /* Initialize PWM/Timer. */
    pwm_init();

    // heap_insert(SEC / 16, EVENT_TEST1);
    // heap_insert((uint32_t)SEC*10L, EVENT_TEST3);
    check_rolling = 1;
    state = STOPPED;
    level = 50;
    current_flash = &low;
    flash_i = 0;

    heap_insert(SEC_TICKS / 2 - SEC_TICKS/16, EVENT_TEST3);
    heap_insert(SEC_TICKS / 2 + SEC_TICKS/16, EVENT_TEST4);

    #if DOUBLER_TEST
    heap_insert(SEC_TICKS/2 + 3 * SEC_TICKS/16, EVENT_DOUBLER_TEST_ON);
#endif

    // display_init();
    // heap_insert(SEC_TICKS * 3 / 2, EVENT_DISPLAY);

    while (1) {
        cli();
        if (ticks == 0 && immed_empty()) {
            // Sleep only if no work to do.
            sleep_enable()
            ;
            sei();
            // enable interrupts
            sleep_cpu()
            ;
            sleep_disable()
            ;
            // could wake up, interrupts could occur here.
            cli();
        }
        // interrupts blocked, read + update ticks
        uint16_t ticks_local = ticks;
        ticks = 0;
        sei();

        // First probe should only return "immediate" events from interrupts.
        // A timer tick produces no event; that work is open-coded here.
        uint16_t event = heap_cond_remove();
        while (ticks_local > 0 || event > 0) {
            // As long as there is an event,
            // or there is elapsed time to record,
            // there is (possible) work to do.
            // Processing existing events takes
            // priority over advancing the queue.
            if (event > 0) {
                // Process event;
                process(event);
                // Next event if any.
            } else {
                if (ticks_local >= h.next_time) {
                    ticks_local -= h.next_time;
                    h.next_time = 0;
                } else {
                    h.next_time -= ticks_local;
                    ticks_local = 0;
                }
            }
            event = heap_cond_remove();
        }
    }
}

