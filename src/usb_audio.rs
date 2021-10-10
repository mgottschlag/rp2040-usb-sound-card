//! USB audio class for asynchronous, bidirectional (microphone + speakers) 48kHz audio.
//!
//! See "Universal Serial Bus Device Class Definition for Audio Devices", release 1.0 (March 18,
//! 1998) for a specification of the USB audio class.
use cortex_m::interrupt;
use usb_device::class_prelude::*;
use usb_device::endpoint::{IsochronousSynchronizationType, IsochronousUsageType};
use usb_device::Result;

const DEVICE_CLASS_AUDIO: u8 = 0x01;
const AUDIO_SUBCLASS_CONTROL: u8 = 0x01;
const AUDIO_SUBCLASS_STREAMING: u8 = 0x02;
const AUDIO_PROTOCOL_NONE: u8 = 0x00;

const AUDIO_INTERFACE_DESC_TYPE: u8 = 0x24;
const AUDIO_ENDPOINT_DESC_TYPE: u8 = 0x25;

const AUDIO_CONTROL_HEADER: u8 = 0x01;
const AUDIO_CONTROL_INPUT_TERMINAL: u8 = 0x02;
const AUDIO_CONTROL_OUTPUT_TERMINAL: u8 = 0x03;
const AUDIO_CONTROL_FEATURE_UNIT: u8 = 0x06;

const AUDIO_STREAMING_GENERAL: u8 = 0x01;
const AUDIO_STREAMING_FORMAT_TYPE: u8 = 0x02;

const AUDIO_ENDPOINT_GENERAL: u8 = 0x01;

const SET_CUR: u8 = 0x01;
const GET_CUR: u8 = 0x81;
/*const SET_MIN: u8 = 0x02;*/
const GET_MIN: u8 = 0x82;
/*const SET_MAX: u8 = 0x03;*/
const GET_MAX: u8 = 0x83;
/*const SET_RES: u8 = 0x04;*/
const GET_RES: u8 = 0x84;
/*const SET_MEM: u8 = 0x05;
const GET_MEM: u8 = 0x85;
const GET_STAT: u8 = 0xFF;*/

const MUTE_CONTROL: u8 = 0x01;
const VOLUME_CONTROL: u8 = 0x02;

// TODO: Update bDelay.

pub struct UsbAudioClass<'a, B: UsbBus, OUTBUF: AsRef<[u32]>> {
    audio_control: InterfaceNumber,
    //audio_streaming_inactive: InterfaceNumber,
    audio_streaming: InterfaceNumber,
    audio_out: EndpointOut<'a, B>,
    //audio_out_sync: EndpointIn<'a, B>,
    //audio_in: EndpointIn<'a, B>,
    samples_per_frame: [u8; 3],

    audio_out_buf: Option<OUTBUF>,

    volume: [u8; 2],
    mute: u8,
}

impl<'a, B: UsbBus, OUTBUF: AsRef<[u32]>> UsbAudioClass<'a, B, OUTBUF> {
    pub fn new(bus_alloc: &'a UsbBusAllocator<B>) -> Self {
        UsbAudioClass {
            audio_control: bus_alloc.interface(),
            //audio_streaming_inactive: bus_alloc.interface(),
            audio_streaming: bus_alloc.interface(),
            // 48kHz * 2 * 16bit = 192B per packet. We allocate a bit more in case the device clock
            // is faster than the host clock.
            /*audio_out: bus_alloc.isochronous(
                IsochronousSynchronizationType::Asynchronous,
                IsochronousUsageType::Data,
                256,
                1,
            ),*/
            audio_out: bus_alloc.isochronous(
                IsochronousSynchronizationType::Synchronous,
                IsochronousUsageType::Data,
                256,
                1,
            ),
            /*audio_out_sync: bus_alloc.isochronous(
                IsochronousSynchronizationType::Asynchronous,
                IsochronousUsageType::Feedback,
                3,
                1,
            ),*/
            /*audio_in: bus_alloc.isochronous(
                IsochronousSynchronizationType::Asynchronous,
                IsochronousUsageType::Data,
                256,
                1,
            ),*/
            samples_per_frame: [48 >> 2, 48 << 6, 0], // 48 kHz
            audio_out_buf: None,

            volume: [1, 0],
            mute: 0,
        }
    }

    // TODO: Functions to read and write data?
}

impl<B: UsbBus, OUTBUF: AsRef<[u32]>> UsbClass<B> for UsbAudioClass<'_, B, OUTBUF> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        /*interrupt::free(|cs| {
            writeln!(crate::uart(cs), "A\r").unwrap();
        });*/
        writer.iad(
            self.audio_control,
            2,   // Two interfaces (control + streaming).
            0x0, // Each interface specifies its own class.
            0x0, // Each interface specifies its own subclass.
            0x0, // No class-specific protocols for this device.
        )?;

        // Audio control interface.
        writer.interface(
            self.audio_control,
            DEVICE_CLASS_AUDIO,
            AUDIO_SUBCLASS_CONTROL,
            AUDIO_PROTOCOL_NONE,
        )?;

        writer.write(
            AUDIO_INTERFACE_DESC_TYPE,
            &[
                AUDIO_CONTROL_HEADER,        // bDescriptorSubtype
                0x00,                        // bcdADC
                0x01,                        //
                37,                          // wTotalLength
                0,                           //
                0x01,                        // bInCollection
                self.audio_streaming.into(), // baInterfaceNr
            ],
        )?;

        // Input terminal (USB streaming).
        writer.write(
            AUDIO_INTERFACE_DESC_TYPE,
            &[
                AUDIO_CONTROL_INPUT_TERMINAL, // bDescriptorSubtype
                0x01,                         // bTerminalID
                0x01,                         // wTerminalType
                0x01,                         //
                0x00,                         // bAssocTerminal
                0x02,                         // bNrChannels
                0x03,                         // wChannelConfig
                0x00,                         //
                0x00,                         // iChannelNames
                0x00,                         // iTerminal
            ],
        )?;

        // Feature unit (volume and mute).
        writer.write(
            AUDIO_INTERFACE_DESC_TYPE,
            &[
                AUDIO_CONTROL_FEATURE_UNIT, // bDescriptorSubtype
                0x02,                       // bUnitID
                0x01,                       // bSourceID
                0x01,                       // bControlSize
                0x03,                       // bmaControls(0)
                0x00,                       // iFeature
            ],
        )?;

        // Output terminal (speaker).
        writer.write(
            AUDIO_INTERFACE_DESC_TYPE,
            &[
                AUDIO_CONTROL_OUTPUT_TERMINAL, // bDescriptorSubtype
                0x03,                          // bTerminalID
                0x01,                          // wTerminalType
                0x03,                          //
                0x00,                          // bAssocTerminal
                0x02,                          // bSourceID
                0x00,                          // iTerminal
            ],
        )?;

        // Audio streaming interface (zero-bandwidth).
        writer.interface(
            self.audio_streaming,
            DEVICE_CLASS_AUDIO,
            AUDIO_SUBCLASS_STREAMING,
            AUDIO_PROTOCOL_NONE,
        )?;

        // Audio streaming interface (operational).
        writer.interface_alt(
            self.audio_streaming,
            1,
            DEVICE_CLASS_AUDIO,
            AUDIO_SUBCLASS_STREAMING,
            AUDIO_PROTOCOL_NONE,
            None,
        )?;

        writer.write(
            AUDIO_INTERFACE_DESC_TYPE,
            &[
                AUDIO_STREAMING_GENERAL, // bDescriptorSubtype
                0x01,                    // bTerminalLink
                0x03,                    // bDelay
                0x01,                    // wFormatTag
                0x00,                    //
            ],
        )?;

        writer.write(
            AUDIO_INTERFACE_DESC_TYPE,
            &[
                AUDIO_STREAMING_FORMAT_TYPE, // bDescriptorSubtype
                0x01,                        // bFormatType
                0x02,                        // bNrChannels
                0x02,                        // bSubframeSize
                0x10,                        // bBitResolution
                0x01,                        // bSamFreqType
                0x80,                        // tSamFreq[1]
                0xbb,                        //
                0x00,                        //
            ],
        )?;

        /*writer.endpoint_ex(&self.audio_out, |data| {
            // TODO: Faster refresh
            data[0] = 0x09; // bRefresh
            data[1] = self.audio_out_sync.address().into(); // bSynchAddress
            Ok(2)
        })?;*/
        writer.endpoint(&self.audio_out)?;

        writer.write(
            AUDIO_ENDPOINT_DESC_TYPE,
            &[
                AUDIO_ENDPOINT_GENERAL, // bDescriptorSubtype
                0x00,                   // bmAttributes
                0x01,                   // bLockDelayUnits
                0x00,                   // wLockDelay
                0x00,
            ],
        )?;

        /*writer.endpoint_ex(&self.audio_out_sync, |data| {
            data[0] = 0x00; // bRefresh
            data[1] = 0x00; // bSynchAddress
            Ok(2)
        })?;*/

        /*interrupt::free(|cs| {
            writeln!(crate::uart(cs), "B\r").unwrap();
        });*/

        Ok(())
    }

    fn get_string(&self, _index: StringIndex, _lang_id: u16) -> Option<&str> {
        // TODO
        None
    }

    fn reset(&mut self) {
        // Start sending synchronization data.
        //self.endpoint_in_complete(self.audio_out_sync.address());
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();

        // TODO: Alternate interfaces?
        if req.request_type == control::RequestType::Standard
            && req.recipient == control::Recipient::Interface
            && req.request == control::Request::SET_INTERFACE
        {
            // TODO
            xfer.accept().ok();
            return;
        }

        // We only support interface requests.
        if req.request_type != control::RequestType::Class
            || req.recipient != control::Recipient::Interface
        {
            return;
        }

        let interface = req.index & 0xff;
        let entity = req.index >> 8;

        if interface != u8::from(self.audio_control) as u16 {
            return;
        }

        if entity != 0x2 {
            xfer.reject().ok();
            return;
        }

        /*if !(req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && req.index == u8::from(self.audio_control) as u16)
        {
            return;
        }*/

        match req.request {
            SET_CUR => {
                let control_selector = (req.value >> 8) as u8;
                let channel_number = req.value & 0xff;

                match control_selector {
                    MUTE_CONTROL if xfer.data().len() == 1 => {
                        // TODO: Channel?
                        self.mute = xfer.data()[0];
                        xfer.accept().ok();
                    }
                    VOLUME_CONTROL if xfer.data().len() == 2 => {
                        // TODO: Channel?
                        self.volume[0] = xfer.data()[0];
                        self.volume[1] = xfer.data()[1];
                        xfer.accept().ok();
                    }
                    _ => {
                        xfer.reject().ok();
                    }
                }
            }
            _ => {
                xfer.reject().ok();
            }
        };
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();

        // We only support interface requests.
        if req.request_type != control::RequestType::Class
            || req.recipient != control::Recipient::Interface
        {
            return;
        }

        let interface = req.index & 0xff;
        let entity = req.index >> 8;

        if interface != u8::from(self.audio_control) as u16 {
            return;
        }

        if entity != 0x2 {
            xfer.reject().ok();
            return;
        }

        /*if !(req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && req.index == u8::from(self.audio_control) as u16)
        {
            return;
        }*/

        match req.request {
            GET_MIN => {
                let control_selector = (req.value >> 8) as u8;
                let channel_number = req.value & 0xff;

                match control_selector {
                    MUTE_CONTROL => {
                        if channel_number == 0xff && req.length == 2 {
                            // TODO
                            xfer.accept_with(&[0, 0]).ok();
                        } else if channel_number != 0xff && req.length == 1 {
                            // TODO
                            xfer.accept_with(&[0]).ok();
                        } else {
                            xfer.reject().ok();
                        }
                    }
                    VOLUME_CONTROL => {
                        if channel_number == 0xff && req.length == 4 {
                            // TODO
                            xfer.accept_with(&[0x00, 0x00, 0x00, 0x00]).ok();
                        } else if channel_number != 0xff && req.length == 2 {
                            // TODO
                            xfer.accept_with(&[0x00, 0x00]).ok();
                        } else {
                            xfer.reject().ok();
                        }
                    }
                    _ => {
                        xfer.reject().ok();
                    }
                }
            }
            GET_MAX => {
                let control_selector = (req.value >> 8) as u8;
                let channel_number = req.value & 0xff;

                match control_selector {
                    MUTE_CONTROL => {
                        if channel_number == 0xff && req.length == 2 {
                            // TODO
                            xfer.accept_with(&[1, 1]).ok();
                        } else if channel_number != 0xff && req.length == 1 {
                            // TODO
                            xfer.accept_with(&[1]).ok();
                        } else {
                            xfer.reject().ok();
                        }
                    }
                    VOLUME_CONTROL => {
                        if channel_number == 0xff && req.length == 4 {
                            // TODO
                            xfer.accept_with(&[0x00, 0x01, 0x00, 0x01]).ok();
                        } else if channel_number != 0xff && req.length == 2 {
                            // TODO
                            xfer.accept_with(&[0x00, 0x01]).ok();
                        } else {
                            xfer.reject().ok();
                        }
                    }
                    _ => {
                        xfer.reject().ok();
                    }
                }
            }
            GET_RES => {
                let control_selector = (req.value >> 8) as u8;
                let channel_number = req.value & 0xff;

                match control_selector {
                    VOLUME_CONTROL => {
                        if channel_number == 0xff && req.length == 4 {
                            // TODO
                            xfer.accept_with(&[0x01, 0x00, 0x01, 0x00]).ok();
                        } else if channel_number != 0xff && req.length == 2 {
                            // TODO
                            xfer.accept_with(&[0x01, 0x00]).ok();
                        } else {
                            xfer.reject().ok();
                        }
                    }
                    _ => {
                        xfer.reject().ok();
                    }
                }
            }
            GET_CUR => {
                let control_selector = (req.value >> 8) as u8;
                let channel_number = req.value & 0xff;

                match control_selector {
                    MUTE_CONTROL => {
                        if channel_number == 0xff && req.length == 2 {
                            // TODO
                            xfer.accept_with(&[self.mute, self.mute]).ok();
                        } else if channel_number != 0xff && req.length == 1 {
                            // TODO
                            xfer.accept_with(&[self.mute]).ok();
                        } else {
                            xfer.reject().ok();
                        }
                    }
                    VOLUME_CONTROL => {
                        if channel_number == 0xff && req.length == 4 {
                            // TODO
                            xfer.accept_with(&[
                                self.volume[0],
                                self.volume[1],
                                self.volume[0],
                                self.volume[1],
                            ])
                            .ok();
                        } else if channel_number != 0xff && req.length == 2 {
                            // TODO
                            xfer.accept_with(&self.volume).ok();
                        } else {
                            xfer.reject().ok();
                        }
                    }
                    _ => {
                        xfer.reject().ok();
                    }
                }
            }
            // TODO
            _ => {
                xfer.reject().ok();
            }
        };
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr == self.audio_out.address() {
            if self.audio_out_buf.is_some() {
                // TODO: Write data into buffer, move buffer somewhere else.
            } else {
                // TODO
            }
            // TODO: Process incoming audio data.
            let mut buffer = [0u8; 256];
            self.audio_out.read(&mut buffer).ok();
        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        /*if addr == self.audio_out_sync.address() {
            // Immediately write the next sync value.
            //self.audio_out_sync.write(&self.samples_per_frame).unwrap();
        }*/
    }
}
