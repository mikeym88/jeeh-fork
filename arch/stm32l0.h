// see [1] https://jeelabs.org/ref/STM32L0x2-RM0376.pdf

struct Periph {
    constexpr static uint32_t iwdg  = 0x40003000;
    constexpr static uint32_t pwr   = 0x40007000;
    constexpr static uint32_t rcc   = 0x40021000;
    constexpr static uint32_t flash = 0x40022000;
    constexpr static uint32_t gpio  = 0x50000000;
};

// interrupt vector table in ram

struct VTable {
    typedef void (*Handler)();

    uint32_t* initial_sp_value;
    Handler
        reset, nmi, hard_fault, memory_manage_fault, bus_fault, usage_fault,
        dummy_x001c[4], sv_call, debug_monitor, dummy_x0034, pend_sv, systick;
    Handler
        wwdg, pvd, rtc, flash, rcc, exti0_1, exti2_3, exti4_15, tsc,
        dma1_channel1, dma1_channel2_3, dma1_channel4_5, adc_comp, lptim1,
        usart4_5, tim2, tim3, tim6_dac, tim7, reserved4, tim21, i2c3, tim22,
        i2c1, i2c2, spi1, spi2, usart1, usart2, lpuart1_aes_rng, lcd, usb;
};

// systick and delays

constexpr static int defaultHz = 2097000;
extern void enableSysTick (uint32_t divider =defaultHz/1000);

// gpio

enum class Pinmode {
    // mode (2), typer (1), pupdr (2)
    in_analog        = 0b11000,
    in_float         = 0b00000,
    in_pulldown      = 0b00010,
    in_pullup        = 0b00001,

    out              = 0b01000,
    out_od           = 0b01100,
    alt_out          = 0b10000,
    alt_out_od       = 0b10100,
};

template<char port>
struct Port {
    constexpr static uint32_t base    = Periph::gpio + 0x400 * (port-'A');
    constexpr static uint32_t moder   = base + 0x00;
    constexpr static uint32_t typer   = base + 0x04;
    constexpr static uint32_t ospeedr = base + 0x08;
    constexpr static uint32_t pupdr   = base + 0x0C;
    constexpr static uint32_t idr     = base + 0x10;
    constexpr static uint32_t odr     = base + 0x14;
    constexpr static uint32_t bsrr    = base + 0x18;
    constexpr static uint32_t afrl    = base + 0x20;
    constexpr static uint32_t afrh    = base + 0x24;
    constexpr static uint32_t brr     = base + 0x28;

    static void mode (int pin, Pinmode m, int alt =0) {
        // enable GPIOx clock
        MMIO32(Periph::rcc + 0x2C) |= 1 << (port-'A');

        auto mval = static_cast<int>(m);
        MMIO32(moder) = (MMIO32(moder) & ~(3 << 2*pin))
                      | ((mval >> 3) << 2*pin);
        MMIO32(typer) = (MMIO32(typer) & ~(1 << pin))
                      | (((mval >> 2) & 1) << pin);
        MMIO32(pupdr) = (MMIO32(pupdr) & ~(3 << 2*pin))
                      | ((mval & 3) << 2*pin);
        MMIO32(ospeedr) = (MMIO32(ospeedr) & ~(3 << 2*pin)) | (0b11 << 2*pin);

        uint32_t afr = pin & 8 ? afrh : afrl;
        int shift = 4 * (pin & 7);
        MMIO32(afr) = (MMIO32(afr) & ~(0xF << shift)) | (alt << shift);
    }

    static void modeMap (uint16_t pins, Pinmode m, int alt =0) {
        for (int i = 0; i < 16; ++i) {
            if (pins & 1)
                mode(i, m, alt);
            pins >>= 1;
        }
    }
};

template<char port,int pin>
struct Pin {
    typedef Port<port> gpio;
    constexpr static uint16_t mask = 1U << pin;
    constexpr static int id = 16 * (port-'A') + pin;

    static void mode (Pinmode m, int alt =0) {
        gpio::mode(pin, m, alt);
    }

    static int read () {
        return mask & MMIO32(gpio::idr) ? 1 : 0;
    }

    static void write (int v) {
        // MMIO32(v ? gpio::bsrr : gpio::brr) = mask;
        // this is slightly faster when v is not known at compile time:
        MMIO32(gpio::bsrr) = v ? mask : mask << 16;
    }

    // shorthand
    operator int () const { return read(); }
    void operator= (int v) const { write(v); }

    static void toggle () {
        // both versions below are non-atomic, they access and set in two steps
        // this is smaller and faster (1.6 vs 1.2 MHz on F103 @ 72 MHz):
        // MMIO32(gpio::odr) ^= mask;
        // but this code is safer, because it can't interfere with nearby pins:
        MMIO32(gpio::bsrr) = mask & MMIO32(gpio::odr) ? mask << 16 : mask;
    }
};

// u(s)art

template< typename TX, typename RX >
struct UartDev {
    // TODO does not recognise alternate TX pins
    constexpr static int uidx = TX::id ==  9 ? 0 :  // PA9, USART1
                                TX::id ==  2 ? 1 :  // PA2, USART2
                                               0;   // else USART1
    constexpr static uint32_t base = uidx == 0 ? 0x40013800 :
                                                 0x40004000 + 0x400 * uidx;
    constexpr static uint32_t cr1 = base + 0x00;
    //constexpr static uint32_t cr2 = base + 0x04;
    //constexpr static uint32_t cr3 = base + 0x08;
    constexpr static uint32_t brr = base + 0x0C;
    constexpr static uint32_t isr = base + 0x1C;
    constexpr static uint32_t icr = base + 0x20;
    constexpr static uint32_t rdr = base + 0x24;
    constexpr static uint32_t tdr = base + 0x28;

    static void init () {
        tx.mode(Pinmode::alt_out, 4);
        rx.mode(Pinmode::alt_out, 4);

        if (uidx == 0)
            MMIO32(Periph::rcc + 0x34) |= 1 << 14; // enable USART1 clock
        else
            MMIO32(Periph::rcc + 0x38) |= 1 << (16+uidx); // USART 2..5

        MMIO32(brr) = defaultHz / 115200;  // 115200 baud @ 2.1 MHz
        MMIO32(cr1) = 1<<3 | 1<<2 | 1<<0;  // TE, RE, UE
    }

    static void baud (uint32_t baud, uint32_t hz =defaultHz) {
        MMIO32(cr1) &= ~(1<<0);              // disable
        MMIO32(brr) = (hz + baud/2) / baud;  // change while disabled
        MMIO32(cr1) |= 1<<0;                 // enable
    }

    static bool writable () {
        return (MMIO32(isr) & 0x80) != 0;  // TXE
    }

    static void putc (int c) {
        while (!writable())
            ;
        MMIO32(tdr) = (uint8_t) c;
    }

    static bool readable () {
        return (MMIO32(isr) & 0x24) != 0;  // RXNE or ORE
    }

    static int getc () {
        while (!readable())
            ;
        int c = MMIO32(rdr);
        MMIO32(icr) = 0xA; // clear ORE and FE, reading RDR is not enough
        return c;
    }

    static TX tx;
    static RX rx;
};

template< typename TX, typename RX >
TX UartDev<TX,RX>::tx;

template< typename TX, typename RX >
RX UartDev<TX,RX>::rx;

// interrupt-enabled uart, sits of top of polled uart

template< typename TX, typename RX, int N =50 >
struct UartBufDev : UartDev<TX,RX> {
    typedef UartDev<TX,RX> base;

    static void init () {
        auto handler = []() {
            if (base::readable()) {
                int c = base::getc();
                if (recv.free())
                    recv.put(c);
                // else discard the input
            }
            if (base::writable()) {
                if (xmit.avail() > 0)
                    base::putc(xmit.get());
                else
                    MMIO32(base::cr1) &= ~(1<<7);  // disable TXEIE
            }
        };

        switch (base::uidx) {
            case 0: VTableRam().usart1 = handler; break;
            case 1: VTableRam().usart2 = handler; break;
        }

        // nvic interrupt numbers are 27 and 28, respectively
        constexpr uint32_t nvic_en0r = 0xE000E100;
        constexpr int irq = 27 + base::uidx;
        MMIO32(nvic_en0r) = 1 << irq;  // enable USART interrupt

        MMIO32(base::cr1) |= (1<<5);  // enable RXNEIE
    }

    static bool writable () {
        return xmit.free();
    }

    static void putc (int c) {
        while (!writable())
            ;
        xmit.put(c);
        MMIO32(base::cr1) |= (1<<7);  // enable TXEIE
    }

    static bool readable () {
        return recv.avail() > 0;
    }

    static int getc () {
        while (!readable())
            ;
        return recv.get();
    }

    static RingBuffer<N> recv;
    static RingBuffer<N> xmit;
};

template< typename TX, typename RX, int N >
RingBuffer<N> UartBufDev<TX,RX,N>::recv;

template< typename TX, typename RX, int N >
RingBuffer<N> UartBufDev<TX,RX,N>::xmit;

// system clock

static void enableClkAt32mhz () {  // [1] p.49
    constexpr uint32_t rcc_cr   = Periph::rcc + 0x00;
    constexpr uint32_t rcc_cfgr = Periph::rcc + 0x0C;

    // switch to HSI 16 and turn everything else off
    MMIO32(rcc_cr) |= (1<<0); // turn hsi16 on
    MMIO32(rcc_cfgr) = 0x01;  // revert to hsi16, no PLL, no prescalers
    MMIO32(rcc_cr) = 0x01;    // turn off MSI, HSE, and PLL
    while ((MMIO32(rcc_cr) & (1<<25)) != 0) ; // wait for PPLRDY to clear

    MMIO32(Periph::flash + 0x00) = 0x03; // flash acr, 1 wait, enable prefetch
    MMIO32(rcc_cfgr) |= 1<<18 | 1<<22; // set PLL src HSI16, PLL x4, PLL div 2
    MMIO32(rcc_cr) |= 1<<24; // turn PLL on
    while ((MMIO32(rcc_cr) & (1<<25)) == 0) ; // wait for PPLRDY
    MMIO32(rcc_cfgr) |= 0x3; // set system clk to PLL
}

static int fullSpeedClock () {
    constexpr uint32_t hz = 32000000;
    enableClkAt32mhz();
    enableSysTick(hz/1000);             // systick once every 1 ms
    MMIO32(0x4001380C) = hz/115200;     // usart1: 115200 baud @ 32 MHz
    return hz;
}

// independent watchdog

struct Iwdg {  // [1] pp.495
    constexpr static uint32_t kr  = Periph::iwdg + 0x00;
    constexpr static uint32_t pr  = Periph::iwdg + 0x04;
    constexpr static uint32_t rlr = Periph::iwdg + 0x08;
    constexpr static uint32_t sr  = Periph::iwdg + 0x0C;

    Iwdg (int rate =7) {
        while (MMIO32(sr) & (1<<0)) {}  // wait until !PVU
        MMIO32(kr) = 0x5555;   // unlock PR
        MMIO32(pr) = rate;     // max timeout, 0 = 400ms, 7 = 26s
        MMIO32(kr) = 0xCCCC;   // start watchdog
    }

    static void kick () {
        MMIO32(kr) = 0xAAAA;  // reset the watchdog timout
    }

    static void reload (int n) {
        while (MMIO32(sr) & (1<<1)) {}  // wait until !RVU
        MMIO32(kr) = 0x5555;   // unlock PR
        MMIO32(rlr) = n;
        kick();
    }
};

// low-power modes

static void powerDown () {
    MMIO32(Periph::rcc+0x38) |= (1<<28); // PWREN
    MMIO32(Periph::pwr) |= (1<<10) | (1<<9) | (1<<1); // FWU, ULP, PDDS

    constexpr uint32_t scr = 0xE000ED10;
    MMIO32(scr) |= (1<<2);  // set SLEEPDEEP

    __asm("cpsid i");
    __asm("wfi");
}
