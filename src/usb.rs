use crate::audio_buffer::{WriteBuffer, WritePacket};
use crate::usb_audio::UsbAudioClass;
use rp2040_hal::pac::USBCTRL_REGS;
use rp2040_hal::timer::Timer;
use rp2040_hal::usb::UsbBus;
use usb_device::{class_prelude::*, prelude::*};

pub struct Usb {
    usb_audio: UsbAudioClass<'static, UsbBus, WritePacket<4>>,
    usb_dev: UsbDevice<'static, UsbBus>,
    in_buffer: WriteBuffer<4>,

    timer: Timer,
}

impl Usb {
    pub fn init(
        bus: &'static UsbBusAllocator<UsbBus>,
        in_buffer: WriteBuffer<4>,
        timer: Timer,
    ) -> Usb {
        let usb_audio = UsbAudioClass::new(bus);

        // This PID/VID combination is selected from the pid.codes PID space and only intended for
        // software development. It is not universally unique and should not be used outside of
        // test environments!
        let usb_dev = UsbDeviceBuilder::new(bus, UsbVidPid(0x1209, 0x000d))
            .manufacturer("TEST")
            .product("USB audio test")
            .serial_number("TEST")
            .build();

        // TODO
        Usb {
            usb_audio,
            usb_dev,
            in_buffer,
            timer,
        }
    }

    pub fn poll(&mut self) {
        // Safety: The read access does not have any side effect, and this function is the only one
        // using the USB peripheral at this time.
        let sof = unsafe { (&*USBCTRL_REGS::ptr()).intr.read().dev_sof().bit_is_set() };
        if sof {
            // Clear the SOF IRQ.
            unsafe { (&*USBCTRL_REGS::ptr()).sof_rd.read() };

            // Calculate the frequency difference (frames versus 1kHz derived from the system
            // clock) and update the synchronization data for the USB audio class accordingly.
            // Also, add a correction factor if the buffers are running low or high.
            // TODO

            /*let mut buffer = [0u8; 64];
            for i in 0..64 {
                buffer[i] = i as u8 * 4;
            }
            let _err = self.usb_audio.write_packet(&buffer);*/
        }
        if self.usb_dev.poll(&mut [&mut self.usb_audio]) {
            // If we received audio data, move it into the buffer.
            // TODO
        }
    }
}
