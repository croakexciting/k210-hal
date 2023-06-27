//! (TODO) Serial Peripheral Interface (SPI)

use crate::clock::Clocks;
use crate::pac::SPI0;
use crate::sysctl::{self, APB2};
use crate::time::Hertz;
use core::convert::Infallible;
use crate::pac::spi0::{ctrlr0::TMOD_A, spi_ctrlr0::AITM_A};
pub use embedded_hal::spi::{Mode, Phase, Polarity};
use k210_pac::SYSCTL;

///
pub struct Spi<SPI> {
    spi: SPI,
    cs_id: u8,
    clk: Hertz,
}

impl Spi<SPI0> {
    pub fn spi0(
        spi: SPI0,
        cs_id: u8,
        mode: Mode,
        frame_format: FrameFormat,
        data_bit_length: u8,
        endian: Endian,
        clock: &Clocks,
        apb2: &mut APB2,
    ) -> Self {
        let work_mode = hal_mode_to_pac(mode);
        let frame_format = frame_format_to_pac(frame_format);
        let tmod = crate::pac::spi0::ctrlr0::TMOD_A::TRANS; // todo other modes
        let endian = endian as u32;
        let _ = clock; // todo
        unsafe {
            // no interrupts for now
            spi.imr.write(|w| w.bits(0x00));
            // no dma for now
            spi.dmacr.write(|w| w.bits(0x00));
            spi.dmatdlr.write(|w| w.bits(0x10));
            spi.dmardlr.write(|w| w.bits(0x00));
            // no slave access for now
            spi.ser.write(|w| w.bits(0x00));
            spi.ssienr.write(|w| w.bits(0x00));
            // set control registers
            spi.ctrlr0.write(|w| {
                w.work_mode()
                    .variant(work_mode)
                    .tmod()
                    .variant(tmod)
                    .frame_format()
                    .variant(frame_format)
                    .data_length()
                    .bits(data_bit_length - 1)
            });
            spi.spi_ctrlr0.reset(); // standard
            spi.endian.write(|w| w.bits(endian));
        }
        // enable APB2 bus
        apb2.enable();
        // enable peripheral via sysctl
        unsafe {
            let ptr = SYSCTL::ptr();
            (*ptr).misc.write(|w| {
                w.spi_dvp_data_enable().set_bit()
            });
            (*ptr).clk_th1.write(|w| w.spi0_clk().bits(0));
        }
        sysctl::clk_en_peri().modify(|_r, w| w.spi0_clk_en().set_bit());
        Spi { spi, cs_id, clk: clock.apb2() }
    }

    pub fn set_non_standard_mode(
        &mut self, instruction_len: u32, addr_len: u32,
        wait_cycles: u8, aitm: AITM_A
    ) {
        let instl_r = match instruction_len {
            0 => 0,
            4 => 1,
            8 => 2,
            16 => 3,
            _ => todo!(),
        };

        let addrl_r = (addr_len / 4) as u8;

        unsafe {
            self.spi.spi_ctrlr0.write(|w| {
                w.aitm().variant(aitm).
                wait_cycles().bits(wait_cycles).
                inst_length().bits(instl_r).
                addr_length().bits(addrl_r)
            });
        }
    }

    pub fn set_baudrate(&mut self, freq: u32) {
        let mut baud = self.clk.0 / freq;
        if baud < 2 {
            baud = 2;
        } else if baud > 65534 {
            baud = 65534;
        }

        unsafe{
            self.spi.baudr.write(|w| w.bits(baud));
        }
    }

    pub fn release(self) -> SPI0 {
        // power off
        sysctl::clk_en_peri().modify(|_r, w| w.spi0_clk_en().clear_bit());
        self.spi
    }
}

impl embedded_hal::spi::FullDuplex<u8> for Spi<SPI0> {
    /// An enumeration of SPI errors
    type Error = Infallible;

    /// Reads the word stored in the shift register
    ///
    /// **NOTE** A word must be sent to the slave before attempting to call this
    /// method.
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.spi.ctrlr0.modify(|_, w| w.tmod().variant(TMOD_A::RECV));
        unsafe {
            self.spi.ctrlr1.write(|w| w.bits(0x0));
            self.spi.ssienr.write(|w| w.bits(0x1));
            self.spi.ser.write(|w| w.bits(0x1 << self.cs_id));
            self.spi.dr[0].write(|w| w.bits(0xffffffff));
        }

        let recv_byte = self.spi.rxflr.read().bits();

        let r = if recv_byte == 0 {
            Err(nb::Error::WouldBlock)
        } else {
            Ok(self.spi.dr[0].read().bits() as u8)
        };

        self.spi.ser.reset();
        self.spi.ssienr.reset();

        r
    }

    /// Sends a word to the slave
    fn send(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.spi.ctrlr0.modify(|_, w| w.tmod().variant(TMOD_A::TRANS));
        unsafe {
            self.spi.ssienr.write(|w| w.bits(0x1));
            self.spi.ser.write(|w| w.bits(0x1 << self.cs_id));
        }
        const MAX_FIFO_SIZE: u32 = 32;
        let room = MAX_FIFO_SIZE - self.spi.txflr.read().bits();
        if room > 0 {
            unsafe {
                self.spi.dr[0].write(|w| w.bits(word as u32));
            }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FrameFormat {
    Standard,
    Dual,
    Quad,
    Octal,
}
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Endian {
    Little = 0,
    Big = 1,
}

#[inline]
fn hal_mode_to_pac(mode: Mode) -> crate::pac::spi0::ctrlr0::WORK_MODE_A {
    use crate::pac::spi0::ctrlr0::WORK_MODE_A;
    use {Phase::*, Polarity::*};
    match (mode.polarity, mode.phase) {
        (IdleLow, CaptureOnFirstTransition) => WORK_MODE_A::MODE0,
        (IdleLow, CaptureOnSecondTransition) => WORK_MODE_A::MODE1,
        (IdleHigh, CaptureOnFirstTransition) => WORK_MODE_A::MODE2,
        (IdleHigh, CaptureOnSecondTransition) => WORK_MODE_A::MODE3,
    }
}

#[inline]
fn frame_format_to_pac(frame_format: FrameFormat) -> crate::pac::spi0::ctrlr0::FRAME_FORMAT_A {
    use crate::pac::spi0::ctrlr0::FRAME_FORMAT_A;
    match frame_format {
        FrameFormat::Standard => FRAME_FORMAT_A::STANDARD,
        FrameFormat::Dual => FRAME_FORMAT_A::DUAL,
        FrameFormat::Quad => FRAME_FORMAT_A::QUAD,
        FrameFormat::Octal => FRAME_FORMAT_A::OCTAL,
    }
}
