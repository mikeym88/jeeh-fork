// Hardware access for STM32F103 family microcontrollers
// see [1] https://jeelabs.org/ref/STM32F4-RM0090.pdf

struct Periph {
    constexpr static uint32_t fsmc  = 0x40000000;
    constexpr static uint32_t rtc   = 0x40002800;
    constexpr static uint32_t pwr   = 0x40007000;
    constexpr static uint32_t gpio  = 0x40020000;
    constexpr static uint32_t rcc   = 0x40023800;
    constexpr static uint32_t flash = 0x40023C00;
};

// interrupt vector table in ram

struct VTable {
    typedef void (*Handler)();

    uint32_t* initial_sp_value;
    Handler
        reset, nmi, hard_fault, memory_manage_fault, bus_fault, usage_fault,
        dummy_x001c[4], sv_call, debug_monitor, dummy_x0034, pend_sv, systick;
    Handler
	wwdg, pvd, tamp_stamp, rtc_wkup, flash, rcc, exti0, exti1, exti2,
        exti3, exti4, dma1_stream0, dma1_stream1, dma1_stream2, dma1_stream3,
	dma1_stream4, dma1_stream5, dma1_stream6, adc, can1_tx, can1_rx0,
	can1_rx1, can1_sce, exti9_5, tim1_brk_tim9, tim1_up_tim10,
	tim1_trg_com_tim11, tim1_cc, tim2, tim3, tim4, i2c1_ev, i2c1_er,
	i2c2_ev, i2c2_er, spi1, spi2, usart1, usart2, usart3, exti15_10,
	rtc_alarm, usb_fs_wkup, tim8_brk_tim12, tim8_up_tim13,
	tim8_trg_com_tim14, tim8_cc, dma1_stream7, fsmc, sdio, tim5, spi3,
	uart4, uart5, tim6_dac, tim7, dma2_stream0, dma2_stream1, dma2_stream2,
	dma2_stream3, dma2_stream4, eth, eth_wkup, can2_tx, can2_rx0, can2_rx1,
	can2_sce, otg_fs, dma2_stream5, dma2_stream6, dma2_stream7, usart6,
	i2c3_ev, i2c3_er, otg_hs_ep1_out, otg_hs_ep1_in, otg_hs_wkup, otg_hs,
	dcmi, cryp, hash_rng, fpu, uart7, uart8, spi4, spi5, spi6, sai1,
	lcd_tft, lcd_tft_err, dma2d;
};

// systick and delays

constexpr static int defaultHz = 16000000;
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

    static void mode (int pin, Pinmode m, int alt =0) {
        // enable GPIOx clock
        MMIO32(Periph::rcc + 0x30) |= 1 << (port-'A');

        auto mval = static_cast<int>(m);
        MMIO32(moder) = (MMIO32(moder) & ~(3<<(2*pin))) | ((mval>>3) << 2*pin);
        MMIO32(typer) = (MMIO32(typer) & ~(1<<pin)) | (((mval>>2) & 1) << pin);
        MMIO32(pupdr) = (MMIO32(pupdr) & ~(3<<(2*pin))) | ((mval & 3) << 2*pin);
        MMIO32(ospeedr) = (MMIO32(ospeedr) & ~(3<<(2*pin))) | (0b01 << 2*pin);

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
    constexpr static int uidx = TX::id ==  2 ? 1 :  // PA2, USART2
								TX::id ==  9 ? 0 :  // PA9, USART1
                                TX::id == 22 ? 0 :  // PB6, USART1
                                TX::id == 26 ? 2 :  // PB10, USART3
                                TX::id == 42 ? 2 :  // PC10, USART3
                                TX::id == 53 ? 1 :  // PD5, USART2
                                TX::id == 56 ? 2 :  // PD8, USART3
                                // TODO more possible, using alt mode 8 iso 7
                                               0;   // else USART1
    constexpr static uint32_t base = uidx == 0 ? 0x40011000 : // USART1
								     uidx == 5 ? 0x40011400 : // USART6
                                                 0x40004000 + 0x400 * uidx;
    constexpr static uint32_t sr  = base + 0x00;
    constexpr static uint32_t dr  = base + 0x04;
    constexpr static uint32_t brr = base + 0x08;
    constexpr static uint32_t cr1 = base + 0x0C;

    static void init () {
        tx.mode(Pinmode::alt_out, 7);
        rx.mode(Pinmode::alt_out, 7);

        if (uidx == 0)
            MMIO32(Periph::rcc + 0x44) |= 1 << 4; // enable USART1 clock
        else
            MMIO32(Periph::rcc + 0x40) |= 1 << (16+uidx); // U(S)ART 2..5

        baud(115200);
        MMIO32(cr1) = (1<<13) | (1<<3) | (1<<2);  // UE, TE, RE
    }

    static void baud (uint32_t baud, uint32_t hz =defaultHz) {
        MMIO32(brr) = (hz + baud/2) / baud;
    }

    static bool writable () {
        return (MMIO32(sr) & (1<<7)) != 0;  // TXE
    }

    static void putc (int c) {
        while (!writable())
            ;
        MMIO32(dr) = (uint8_t) c;
    }

    static bool readable () {
        return (MMIO32(sr) & ((1<<5) | (1<<3))) != 0;  // RXNE or ORE
    }

    static int getc () {
        while (!readable())
            ;
        return MMIO32(dr);
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

    UartBufDev () {
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
            case 2: VTableRam().usart3 = handler; break;
            case 3: VTableRam().uart4  = handler; break;
            case 4: VTableRam().uart5  = handler; break;
        }

        // nvic interrupt numbers are 37, 38, 39, 52, and 53, respectively
        constexpr uint32_t nvic_en1r = 0xE000E104;
        constexpr int irq = (base::uidx < 3 ? 37 : 49) + base::uidx;
        MMIO32(nvic_en1r) = 1 << (irq-32);  // enable USART interrupt

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

static void enableClkAt168MHz () {
    MMIO32(Periph::flash + 0x00) = 0x705; // flash acr, 5 wait states
    MMIO32(Periph::rcc + 0x00) = (1<<16); // HSEON
    while ((MMIO32(Periph::rcc + 0x00) & (1<<17)) == 0) ; // wait for HSERDY
    MMIO32(Periph::rcc + 0x08) = (4<<13) | (5<<10) | (1<<0); // prescaler w/ HSE
    MMIO32(Periph::rcc + 0x04) = (7<<24) | (1<<22) | (0<<16) | (168<<6) | (4<<0);
    MMIO32(Periph::rcc + 0x00) |= (1<<24); // PLLON
    while ((MMIO32(Periph::rcc + 0x00) & (1<<25)) == 0) ; // wait for PLLRDY
    MMIO32(Periph::rcc + 0x08) = (4<<13) | (5<<10) | (2<<0);
}

static int fullSpeedClock () {
    constexpr uint32_t hz = 168000000;
    enableClkAt168MHz();                 // using external 8 MHz crystal
    enableSysTick(hz/1000);              // systick once every 1 ms
    MMIO32(0x40011008) = (hz/2)/115200;  // usart1: 115200 baud @ 84 MHz
    return hz;
}

// can bus(es)

template< int N >
struct CanDev {
    constexpr static uint32_t base = N == 0 ? 0x40006400 : 0x40006800;

    constexpr static uint32_t mcr  = base + 0x000;
    constexpr static uint32_t msr  = base + 0x004;
    constexpr static uint32_t tsr  = base + 0x008;
    constexpr static uint32_t rfr  = base + 0x00C;
    constexpr static uint32_t btr  = base + 0x01C;
    constexpr static uint32_t tir  = base + 0x180;
    constexpr static uint32_t tdtr = base + 0x184;
    constexpr static uint32_t tdlr = base + 0x188;
    constexpr static uint32_t tdhr = base + 0x18C;
    constexpr static uint32_t rir  = base + 0x1B0;
    constexpr static uint32_t rdtr = base + 0x1B4;
    constexpr static uint32_t rdlr = base + 0x1B8;
    constexpr static uint32_t rdhr = base + 0x1BC;
    constexpr static uint32_t fmr  = base + 0x200;
    constexpr static uint32_t fsr  = base + 0x20C;
    constexpr static uint32_t far  = base + 0x21C;
    constexpr static uint32_t fr1  = base + 0x240;
    constexpr static uint32_t fr2  = base + 0x244;

    static void init () {
        if (N == 0) {
            // alt mode CAN1:    5432109876543210
            Port<'B'>::modeMap(0b0000001100000000, Pinmode::alt_out, 9);
            MMIO32(Periph::rcc + 0x40) |= (1<<25);  // enable CAN1
        } else {
            // alt mode CAN2:    5432109876543210
            Port<'B'>::modeMap(0b0000000001100000, Pinmode::alt_out, 9);
            MMIO32(Periph::rcc + 0x40) |= (1<<26);  // enable CAN2
        }

        MMIO32(mcr) &= ~(1<<1); // exit sleep
        MMIO32(mcr) |= (1<<6) | (1<<0); // set ABOM, init req
        while ((MMIO32(msr) & (1<<0)) == 0) {}
        MMIO32(btr) = (7<<20) | (5<<16) | (2<<0); // 1 MBps
        MMIO32(mcr) &= ~(1<<0); // init leave
        while (MMIO32(msr) & (1<<0)) {}
        MMIO32(fmr) &= ~(1<<0); // ~FINIT
    }

    static void filterInit (int num, int id =0, int mask =0) {
        MMIO32(far) &= ~(1<<num); // ~FACT
        MMIO32(fsr) |= (1<<num); // FSC 32b
        MMIO32(fr1 + 8 * num) = id;
        MMIO32(fr2 + 8 * num) = mask;
        MMIO32(far) |= (1<<num); // FACT
    }

    static void transmit (int id, const void* ptr, int len) {
        if (MMIO32(tsr) & (1<<26)) { // TME0
            MMIO32(tir) = (id<<21);
            MMIO32(tdtr) = (len<<0);
            // this assumes that misaligned word access works
            MMIO32(tdlr) = ((const uint32_t*) ptr)[0];
            MMIO32(tdhr) = ((const uint32_t*) ptr)[1];

            MMIO32(tir) |= (1<<0); // TXRQ
        }
    }

    static int receive (int* id, void* ptr) {
        int len = -1;
        if (MMIO32(rfr) & (3<<0)) { // FMP
            *id = MMIO32(rir) >> 21;
            len = MMIO32(rdtr) & 0x0F;
            ((uint32_t*) ptr)[0] = MMIO32(rdlr);
            ((uint32_t*) ptr)[1] = MMIO32(rdhr);
            MMIO32(rfr) |= (1<<5); // RFOM
        }
        return len;
    }
};

// real-time clock

struct RTC {  // [1] pp.486
    constexpr static uint32_t bdcr = Periph::rcc + 0x70;
    constexpr static uint32_t tr   = Periph::rtc + 0x00;
    constexpr static uint32_t dr   = Periph::rtc + 0x04;
    constexpr static uint32_t cr   = Periph::rtc + 0x08;
    constexpr static uint32_t isr  = Periph::rtc + 0x0C;
    constexpr static uint32_t wpr  = Periph::rtc + 0x24;
    constexpr static uint32_t bkpr = Periph::rtc + 0x50;

    struct DateTime {
        uint32_t yr :6;
        uint32_t mo :4;
        uint32_t dy :5;
        uint32_t hh :5;
        uint32_t mm :6;
        uint32_t ss :6;
    };

    RTC () {
        MMIO32(Periph::rcc + 0x40) |= (1<<28);  // enable PWREN
        MMIO32(Periph::pwr) |= (1<<8);  // set DBP [1] p.481
        MMIO32(wpr) = 0xCA;  // disable write protection, [1] p.803
        MMIO32(wpr) = 0x53;
    }

    void init () {
        MMIO32(bdcr) |= (1<<0);                 // LSEON backup domain
        while ((MMIO32(bdcr) & (1<<1)) == 0) {} // wait for LSERDY
        MMIO32(bdcr) |= (1<<8);                 // RTSEL = LSE
        MMIO32(bdcr) |= (1<<15);                // RTCEN
        MMIO32(cr) |= (1<<5);                   // BYPSHAD
    }

    DateTime get () {
        while (true) {
            uint32_t tod = MMIO32(tr);
            uint32_t doy = MMIO32(dr);
            if (tod == MMIO32(tr)) {
                DateTime dt;
                dt.ss = (tod & 0xF) + 10 * ((tod>>4) & 0x7);
                dt.mm = ((tod>>8) & 0xF) + 10 * ((tod>>12) & 0x7);
                dt.hh = ((tod>>16) & 0xF) + 10 * ((tod>>20) & 0x3);
                dt.dy = (doy & 0xF) + 10 * ((doy>>4) & 0x3);
                dt.mo = ((doy>>8) & 0xF) + 10 * ((doy>>12) & 0x1);
                // works until end 2063, will fail (i.e. roll over) in 2064 !
                dt.yr = ((doy>>16) & 0xF) + 10 * ((doy>>20) & 0xF);
                return dt;
            }
            // if time of day changed, try again
        }
    }

    void set (DateTime dt) {
        MMIO32(isr) |= (1<<7);                  // set INIT
        while ((MMIO32(isr) & (1<<6)) == 0) {}  // wait for INITF
        MMIO32(tr) = (dt.ss + 6 * (dt.ss/10)) |
                    ((dt.mm + 6 * (dt.mm/10)) << 8) |
                    ((dt.hh + 6 * (dt.hh/10)) << 16);
        MMIO32(dr) = (dt.dy + 6 * (dt.dy/10)) |
                    ((dt.mo + 6 * (dt.mo/10)) << 8) |
                    ((dt.yr + 6 * (dt.yr/10)) << 16);
        MMIO32(isr) &= ~(1<<7);                 // clear INIT
    }

    // access to the backup registers

    uint32_t getData (int reg) {
        return MMIO32(bkpr + 4 * reg);  // regs 0..18
    }

    void setData (int reg, uint32_t val) {
        MMIO32(bkpr + 4 * reg) = val;  // regs 0..18
    }
};

// flash memory writing and erasing

struct Flash {
    constexpr static uint32_t keyr = Periph::flash + 0x04;
    constexpr static uint32_t sr   = Periph::flash + 0x0C;
    constexpr static uint32_t cr   = Periph::flash + 0x10;

    static void write32 (void const* addr, uint32_t val) {
        if (*(uint32_t const*) addr != 0xFFFFFFFF)
            return;
        unlock();
        MMIO32(cr) = (2<<8) | (1<<0); // PSIZE, PG
        MMIO32((uint32_t) addr | 0x08000000) = val;
        finish();
    }

    static void erasePage (void const* addr) {
        // sectors are 16/16/16/16/64/128... KB
        int sector = (uint32_t) addr < 0x10000 ? ((int) addr >> 14) :
                     (uint32_t) addr < 0x20000 ? ((int) addr >> 16) + 3 :
                                                 ((int) addr >> 17) + 4;
        unlock();
        MMIO32(cr) = (1<<16) | (2<<8) | (sector<<3) | (1<<1); // STRT PG SNB SER
        finish();
    }

    static void unlock () {
        MMIO32(keyr) = 0x45670123;
        MMIO32(keyr) = 0xCDEF89AB;
    }

    static void finish () {
        while (MMIO32(sr) & (1<<16)) {}
        MMIO32(cr) = (1<<31); // LOCK
    }
};
