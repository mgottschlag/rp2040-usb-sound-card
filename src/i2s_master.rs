use crate::audio_buffer::{ReadBuffer, ReadPacket};
use core::marker::PhantomData;
use cortex_m::interrupt;
use rp2040_hal::dma::{Channel, ChannelIndex, SingleBuffering, SingleBufferingConfig};
use rp2040_hal::gpio::bank0::{Gpio0, Gpio1, Gpio3, Gpio6};
use rp2040_hal::gpio::{Function, Pin, Pio0};
use rp2040_hal::pac::PIO0;
use rp2040_hal::pio::{
    PIOBuilder, PIOExt, StateMachineIndex, Tx, UninitStateMachine, ValidStateMachine, PIO,
};

pub trait PinConfig {
    type PIO;

    const OUT_MASK: u32;
}

impl PinConfig
    for (
        Pin<Gpio0, Function<Pio0>>,
        Pin<Gpio1, Function<Pio0>>,
        Pin<Gpio6, Function<Pio0>>,
        Pin<Gpio3, Function<Pio0>>,
    )
{
    type PIO = PIO0;

    const OUT_MASK: u32 = 0x4b;
}

pub struct I2sMaster<SM: ValidStateMachine, DMACH: ChannelIndex, LRCLK, BCLK, SDO, SDI> {
    tx: Option<SingleBuffering<Channel<DMACH>, ReadPacket<4>, Tx<SM>>>,
    out_buffer: ReadBuffer<4>,
    _phantom: PhantomData<(SM, LRCLK, BCLK, SDO, SDI)>,
}

impl<P: PIOExt, SM: StateMachineIndex, DMACH: ChannelIndex, LRCLK, BCLK, SDO, SDI>
    I2sMaster<(P, SM), DMACH, LRCLK, BCLK, SDO, SDI>
where
    (LRCLK, BCLK, SDO, SDI): PinConfig,
{
    pub fn init(
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        dma: Channel<DMACH>,
        out_buffer: ReadBuffer<4>,
        _lrclk: LRCLK,
        _bclk: BCLK,
        _sdo: SDO,
        _sdi: SDI,
    ) -> Self {
        let program = pio_proc::pio!(
            32,
            "
        .side_set 2

        .wrap_target
            pull noblock      side 0b11 [0]
            set x, 14         side 0b11 [0]

        leftloop:
            out pins, 1       side 0b00 [6]
            in pins, 1        side 0b01 [5]
            jmp x-- leftloop  side 0b01 [0]

            out pins, 1       side 0b00 [6]
            in pins, 1        side 0b01 [5]
            set x, 14         side 0b01 [0]
        rightloop:
            out pins, 1       side 0b10 [6]
            in pins, 1        side 0b11 [5]
            jmp x-- rightloop side 0b11 [0]

            out pins, 1       side 0b10 [6]
            in pins, 1        side 0b11 [3]
            push noblock      side 0b11 [0]
        .wrap
                    "
        );

        let installed = pio.install(&program.program).unwrap();
        let (mut sm, _rx, tx) = PIOBuilder::from_program(installed)
            .side_set_pin_base(0)
            //.set_pins(6, 1)
            .out_pins(6, 1)
            .in_pin_base(3)
            .clock_divisor(4.0)
            .build(sm);
        sm.set_pindirs_with_mask(
            <(LRCLK, BCLK, SDO, SDI)>::OUT_MASK,
            <(LRCLK, BCLK, SDO, SDI)>::OUT_MASK,
        );

        // Start the first DMA transfer.
        let packet = interrupt::free(|cs| out_buffer.borrow_read(cs).unwrap());
        let tx = SingleBufferingConfig::new(dma, packet, tx).start();

        // Start I2S.
        // TODO: Synchronized start of multiple I2S instances?
        sm.start();

        I2sMaster {
            tx: Some(tx),
            out_buffer,
            _phantom: PhantomData,
        }
    }

    pub fn next_packet(&mut self) {
        if !self.tx.as_mut().unwrap().check_irq0() {
            return;
        }

        let (dma, prev_packet, tx) = self.tx.take().unwrap().wait();
        let packet = interrupt::free(|cs| {
            self.out_buffer.read_finished(prev_packet, cs);
            self.out_buffer.borrow_read(cs).unwrap()
        });
        self.tx = Some(SingleBufferingConfig::new(dma, packet, tx).start());
    }
}
