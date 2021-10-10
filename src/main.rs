#![no_std]
#![no_main]

//use panic_halt as _;
use cortex_m::interrupt::CriticalSection;
use panic_semihosting as _;
use rp2040_hal::pac::UART0;
use rp2040_hal::uart;

mod audio_buffer;
mod i2s_master;
mod usb;
mod usb_audio;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

/*pub fn uart<'cs>(cs: &'cs CriticalSection) -> &'cs mut uart::UartPeripheral<uart::Enabled, UART0> {
    use panic_write::PanicHandler;

    unsafe {
        let panic_handler = app::PANIC_HANDLER.borrow(cs).get();
        let panic_handler_ref = core::mem::transmute::<
            *mut Option<core::pin::Pin<PanicHandler<uart::UartPeripheral<uart::Enabled, UART0>>>>,
            &mut Option<core::pin::Pin<PanicHandler<uart::UartPeripheral<uart::Enabled, UART0>>>>,
        >(panic_handler);
        panic_handler_ref.as_mut().unwrap()
    }
}*/

#[rtic::app(device = rp2040_hal::pac, peripherals = true, dispatchers = [RTC_IRQ, XIP_IRQ])]
mod app {
    use embedded_hal::digital::v2::OutputPin;
    use embedded_time::duration::Milliseconds;
    use embedded_time::rate::{Extensions, Megahertz};
    use rp2040_hal::clocks::ClocksManager;
    use rp2040_hal::gpio;
    //use rp2040_hal::pio::{PIOBuilder, PIOExt};
    use crate::audio_buffer::AudioBuffer;
    use crate::i2s_master::I2sMaster;
    use crate::usb::Usb;
    //use core::cell::UnsafeCell;
    use core::fmt::Write;
    use cortex_m::interrupt::{self, Mutex};
    //use panic_write::PanicHandler;
    use rp2040_hal::dma::{DMAExt, SingleChannel, CH0};
    use rp2040_hal::gpio::bank0::{Gpio0, Gpio1, Gpio3, Gpio6};
    use rp2040_hal::gpio::{Function, FunctionUart, Pin, Pio0};
    use rp2040_hal::pac::UART0;
    use rp2040_hal::pio::{PIOExt, PIO0SM0};
    use rp2040_hal::pll::common_configs::PLL_USB_48MHZ;
    use rp2040_hal::pll::{setup_pll_blocking, PLLConfig};
    use rp2040_hal::sio::Sio;
    use rp2040_hal::timer::Timer;
    //use rp2040_hal::uart;
    use rp2040_hal::usb::UsbBus;
    use rp2040_hal::watchdog::Watchdog;
    use rp2040_hal::xosc::setup_xosc_blocking;
    use systick_monotonic::Systick;
    use usb_device::class_prelude::*;

    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
    pub const PLL_SYS_86MHZ: PLLConfig<Megahertz> = PLLConfig {
        vco_freq: Megahertz(1032),
        refdiv: 1,
        post_div1: 6,
        post_div2: 2,
    };

    static AUDIO_BUFFER: AudioBuffer<4> = AudioBuffer::new();
    /*pub static UART: Mutex<
        MaybeUninit<Option<&'static mut uart::UartPeripheral<uart::Enabled, UART0>>>,
    > = Mutex::new(UnsafeCell::new(None));*/
    /*pub static mut PANIC_HANDLER: Mutex<
        UnsafeCell<
            Option<core::pin::Pin<PanicHandler<uart::UartPeripheral<uart::Enabled, UART0>>>>,
        >,
    > = Mutex::new(UnsafeCell::new(None));*/

    #[monotonic(binds = SysTick, default = true)]
    type SystickMono = Systick<100>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::Pin<gpio::pin::bank0::Gpio25, gpio::PushPullOutput>,
        i2s: I2sMaster<
            PIO0SM0,
            CH0,
            Pin<Gpio0, Function<Pio0>>,
            Pin<Gpio1, Function<Pio0>>,
            Pin<Gpio6, Function<Pio0>>,
            Pin<Gpio3, Function<Pio0>>,
        >,
        usb: Usb,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;

        // The timer needs a 1MHz input from the watchdog.
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        watchdog.enable_tick_generation((XOSC_CRYSTAL_FREQ / 1000000) as u8);

        // We let an LED blink to show that the system is still alive.
        let sio = Sio::new(c.device.SIO);
        let pins = gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // We select a 86 MHz system clock as that lets us generate a clean and precise 12.288 MHz
        // master clock by dividing the system clock by 7.
        // TODO: Select 123 MHz instead (0.1% deviation, whereas 86 MHz results in 0.02%
        // deviation), or look for an even better frequency that is not a multiple of 1MHz.
        let mut clocks = ClocksManager::new(c.device.CLOCKS);
        let xosc = setup_xosc_blocking(c.device.XOSC, XOSC_CRYSTAL_FREQ.Hz())
            .ok()
            .unwrap();
        let pll_sys = setup_pll_blocking(
            c.device.PLL_SYS,
            xosc.operating_frequency().into(),
            PLL_SYS_86MHZ,
            &mut clocks,
            &mut resets,
        )
        .ok()
        .unwrap();
        let pll_usb = setup_pll_blocking(
            c.device.PLL_USB,
            xosc.operating_frequency().into(),
            PLL_USB_48MHZ,
            &mut clocks,
            &mut resets,
        )
        .ok()
        .unwrap();
        let _clocks = clocks.init_default(&xosc, &pll_sys, &pll_usb).ok().unwrap();

        let mut led = pins.gpio25.into_push_pull_output();
        led.set_low().ok().unwrap();

        /*// We need a UART to debug panics.
        let mut uart = uart::UartPeripheral::<_, _>::enable(
            c.device.UART0,
            &mut resets,
            uart::common_configs::_115200_8_N_1,
            clocks.peripheral_clock.into(),
        )
        .unwrap();
        let _tx_pin = pins.gpio16.into_mode::<FunctionUart>();
        let _rx_pin = pins.gpio17.into_mode::<FunctionUart>();
        writeln!(&mut uart, "Initializing...\r").unwrap();
        interrupt::free(|cs| unsafe {
            let panic_handler = PANIC_HANDLER.borrow(cs);
            *panic_handler.get() = Some(PanicHandler::new(uart));
            writeln!(super::uart(cs), "Panic handler installed.\r").unwrap();
        });*/

        // We need a timer so that we can determine the USB SOF frequency for asynchronous USB
        // audio.
        let timer = Timer::new(c.device.TIMER, &mut resets);

        // We initialize the output audio buffers with some data. As the I2S input buffers are
        // filled at the same rate as the I2S output buffers, this initial state ensures that the
        // output buffers never run empty (note that the USB audio interface performs its own
        // adaptive synchronization).
        interrupt::free(|cs| {
            let write = AUDIO_BUFFER.write();
            for _ in 0..2 {
                let mut packet = write.borrow_write(cs).unwrap();
                let data = packet.as_mut();
                for i in 0..data.len() {
                    //data[i] = 0x80000000; // "0"
                    data[i] = 0xa5a5aa55;
                }
                write.write_finished(packet, cs);
            }
        });

        // Configure USB audio.
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
        unsafe {
            USB_BUS = Some(UsbBusAllocator::new(UsbBus::new(
                c.device.USBCTRL_REGS,
                c.device.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut resets,
            )))
        };

        let usb = Usb::init(
            unsafe { USB_BUS.as_ref().unwrap() },
            AUDIO_BUFFER.write(),
            timer,
        );

        // Configure the PIO unit and start I2S.
        let mut dma = c.device.DMA.split(&mut resets);
        let (mut pio, sm0, _sm1, _sm2, _sm3) = c.device.PIO0.split(&mut resets);
        dma.ch0.listen_irq0();
        let i2s = I2sMaster::init(
            &mut pio,
            sm0,
            dma.ch0,
            AUDIO_BUFFER.read(),
            pins.gpio0.into_mode(),
            pins.gpio1.into_mode(),
            pins.gpio6.into_mode(),
            pins.gpio3.into_mode(),
        );

        // A speaker amplifier (TAS5760) is connected to I2S - we need to configure it via I2C.
        // TODO
        // TODO: I2C stuff should happen in a low-priority task as we do not want to make data
        // streaming to I2S wait.

        let systick = c.core.SYST;
        let mono = Systick::new(systick, 86_000_000);
        blink::spawn().ok().unwrap();
        (Shared {}, Local { led, i2s, usb }, init::Monotonics(mono))
    }

    #[task(shared = [], local = [led, state: bool = false])]
    fn blink(ctx: blink::Context) {
        blink::spawn_after(Milliseconds(500_u32)).ok().unwrap();
        *ctx.local.state = !*ctx.local.state;
        if *ctx.local.state {
            ctx.local.led.set_high().ok().unwrap();
        } else {
            ctx.local.led.set_low().ok().unwrap();
        }
    }

    /*/// DMA interrupt handler.
    ///
    /// DMA interrupts need to have highest priority as we need to quickly restart the DMA transfer
    /// to ensure that the PIO FIFO always contains data.
    #[task(binds = DMA_IRQ_0, priority = 3, local = [i2s])]
    fn dma(ctx: dma::Context) {
        ctx.local.i2s.next_packet();
        process_buffers::spawn().unwrap();
    }

    #[task]
    fn process_buffers(_: process_buffers::Context) {
        interrupt::free(|cs| {
            // If we have run out of data from USB, insert a packet filled with zeros.
            // TODO

            let write = AUDIO_BUFFER.write();
            let mut packet = write.borrow_write(cs).unwrap();
            let data = packet.as_mut();
            for i in 0..data.len() {
                //data[i] = 0x80000000; // "0"
                data[i] = 0xa5a5aa55;
            }
            write.write_finished(packet, cs);
        });
    }*/

    /// USB interrupt handler.
    ///
    /// USB interrupts have a high priority (albeit not as high as DMA) so that we can precisely
    /// measure the SOF frequency relative to our system clock. The frequency ratio is required for
    /// asynchronous USB audio.
    #[task(binds = USBCTRL_IRQ, priority = 2, local = [usb])]
    fn usb(ctx: usb::Context) {
        ctx.local.usb.poll();
    }
}
