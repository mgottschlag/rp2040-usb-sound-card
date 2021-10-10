//! Audio buffer type to transfer audio data within different software/hardware components.
#![warn(missing_docs)]

use core::cell::{Cell, UnsafeCell};
use core::ops::Deref;

use cortex_m::interrupt::{CriticalSection, Mutex};
use stable_deref_trait::StableDeref;

/// Each packet contains 1 millisecond of audio data (48 samples at 48000kHz).
///
/// This translates two twice the number of values, as each sample contains two channels.
pub const SAMPLES_PER_PACKET: usize = 48;
/// Number of channels for all audio data.
pub const CHANNELS: usize = 2;

/// Buffer for a single packet with audio data.
pub type Packet = [u32; SAMPLES_PER_PACKET * CHANNELS];

/// Type for audio packet buffering.
///
/// The audio packets always contain two channels with 32-bit samples. The type contains a number
/// of buffers each containing the same fixed number of samples.
pub struct AudioBuffer<const PACKETS: usize> {
    mutex: Mutex<AudioBufferInner<PACKETS>>,
}

struct AudioBufferInner<const PACKETS: usize> {
    /// Packet contents.
    packets: [UnsafeCell<Packet>; PACKETS],
    /// Bitmask of packets which contain valid data and are not currently being read.
    valid: Cell<u32>,
    /// Bitmask of packets which do not contain valid data and which are not currently being
    /// written.
    invalid: Cell<u32>,
    /// Bitmask of all packets which are currently borrowed via `borrow_read()`.
    reading: Cell<u32>,
    /// Bitmask of all packets which are currently borrowed via `borrow_write()`.
    writing: Cell<u32>,
}

impl AudioBuffer<4> {
    /// Creates a new audio buffer.
    pub const fn new() -> Self {
        const PACKETS: usize = 4;
        // Initially, all packet contents are invalid and no packets are currently being read or
        // written.
        AudioBuffer {
            mutex: Mutex::new(AudioBufferInner {
                packets: [
                    UnsafeCell::new([0; SAMPLES_PER_PACKET * CHANNELS]),
                    UnsafeCell::new([0; SAMPLES_PER_PACKET * CHANNELS]),
                    UnsafeCell::new([0; SAMPLES_PER_PACKET * CHANNELS]),
                    UnsafeCell::new([0; SAMPLES_PER_PACKET * CHANNELS]),
                ],
                valid: Cell::new(0),
                invalid: Cell::new(((1 << PACKETS) - 1) as u32),
                reading: Cell::new(0),
                writing: Cell::new(0),
            }),
        }
    }
}

impl<const PACKETS: usize> AudioBuffer<PACKETS> {
    /// Returns an interface to read packets from this buffer.
    pub fn read(&'static self) -> ReadBuffer<PACKETS> {
        ReadBuffer { buffer: self }
    }

    /// Returns an interface to write packets to this buffer.
    pub fn write(&'static self) -> WriteBuffer<PACKETS> {
        WriteBuffer { buffer: self }
    }
}

/// Interface to read packets from a buffer.
pub struct ReadBuffer<const PACKETS: usize> {
    buffer: &'static AudioBuffer<PACKETS>,
}

impl<const PACKETS: usize> ReadBuffer<PACKETS> {
    /// Returns whether there are any buffers with valid data ready for reading.
    pub fn can_read(&self, cs: &CriticalSection) -> bool {
        let inner = self.buffer.mutex.borrow(cs);
        inner.valid.get() != 0
    }

    /// Borrows a single buffer for reading, returns `None` if no buffer with valid data is
    /// available.
    ///
    /// The caller needs to return the buffer via `read_finished()` once the data has been
    /// processed.
    pub fn borrow_read(&self, cs: &CriticalSection) -> Option<ReadPacket<PACKETS>> {
        let inner = self.buffer.mutex.borrow(cs);
        let valid = inner.valid.get();
        if valid == 0 {
            return None;
        }
        let index = valid.trailing_zeros();
        inner.valid.set(valid & !(1 << index));
        inner.reading.set(inner.reading.get() | (1 << index));

        let index = index as usize;
        Some(ReadPacket {
            buffer: self.buffer,
            data: unsafe { &*inner.packets[index].get() },
            index,
        })
    }

    /// Returns a buffer that was allocated via `borrow_read()` and marks it as empty.
    pub fn read_finished(&self, packet: ReadPacket<PACKETS>, cs: &CriticalSection) {
        assert!((self.buffer as *const _) == (packet.buffer as *const _));
        let inner = self.buffer.mutex.borrow(cs);
        let index = packet.index;
        inner.reading.set(inner.reading.get() & !(1 << index));
        inner.invalid.set(inner.invalid.get() | (1 << index));
    }
}

/// Interface to write packets to a buffer.
pub struct WriteBuffer<const PACKETS: usize> {
    buffer: &'static AudioBuffer<PACKETS>,
}

impl<const PACKETS: usize> WriteBuffer<PACKETS> {
    /// Returns whether there are any empty buffers ready for writing.
    pub fn can_write(&self, cs: &CriticalSection) -> bool {
        let inner = self.buffer.mutex.borrow(cs);
        inner.invalid.get() != 0
    }

    /// Borrows a single buffer for writing, returns `None` if no empty buffer is available.
    ///
    /// The caller needs to return the buffer via `write_finished()` once it was filled with data.
    pub fn borrow_write(&self, cs: &CriticalSection) -> Option<WritePacket<PACKETS>> {
        let inner = self.buffer.mutex.borrow(cs);
        let invalid = inner.invalid.get();
        if invalid == 0 {
            return None;
        }
        let index = invalid.trailing_zeros();
        inner.invalid.set(invalid & !(1 << index));
        inner.writing.set(inner.writing.get() | (1 << index));

        let index = index as usize;
        Some(WritePacket {
            buffer: self.buffer,
            data: unsafe { &mut *inner.packets[index].get() },
            index,
        })
    }

    /// Returns a buffer that was allocated via `borrow_write()` and marks it as ready for reading.
    pub fn write_finished(&self, packet: WritePacket<PACKETS>, cs: &CriticalSection) {
        assert!((self.buffer as *const _) == (packet.buffer as *const _));
        let inner = self.buffer.mutex.borrow(cs);
        let index = packet.index;
        inner.writing.set(inner.writing.get() & !(1 << index));
        inner.valid.set(inner.valid.get() | (1 << index));
    }
}

/// Packet buffer that has been borrowed for reading (i.e., the buffer contains valid data).
pub struct ReadPacket<const PACKETS: usize> {
    buffer: &'static AudioBuffer<PACKETS>,
    data: &'static Packet,
    index: usize,
}

impl<const PACKETS: usize> AsRef<[u32]> for ReadPacket<PACKETS> {
    fn as_ref(&self) -> &[u32] {
        self.data
    }
}

impl<const PACKETS: usize> Deref for ReadPacket<PACKETS> {
    type Target = Packet;

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

unsafe impl<const PACKETS: usize> StableDeref for ReadPacket<PACKETS> {}

/// Packet buffer that has been borrowed for writing (i.e., the buffer was previously empty).
pub struct WritePacket<const PACKETS: usize> {
    buffer: &'static AudioBuffer<PACKETS>,
    data: &'static mut Packet,
    index: usize,
}

impl<const PACKETS: usize> AsRef<[u32]> for WritePacket<PACKETS> {
    fn as_ref(&self) -> &[u32] {
        self.data
    }
}

impl<const PACKETS: usize> AsMut<[u32]> for WritePacket<PACKETS> {
    fn as_mut(&mut self) -> &mut [u32] {
        self.data
    }
}
