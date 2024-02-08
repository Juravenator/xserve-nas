use embassy_stm32::gpio::{Level, Output, Pin, Speed};

pub struct DMAShiftRegister {
    sck: u8, // pin number for SCK
    sin: u8, // pin number for SIN
    lat: u8, // pin number for LAT
}

#[inline]
pub fn set_bit(buf: &mut u16, pin: u8) {
    *buf |= 1 << pin
}
#[inline]
pub fn unset_bit(buf: &mut u16, pin: u8) {
    *buf &= u16::MAX ^ 1 << pin
}

/// Controls the serial pins of a DMA'd shift register.
impl DMAShiftRegister {
    pub const CLOCKS_PER_BIT: usize = 3;

    #[cfg(test)]
    pub fn testnew(sck: u8, sin: u8, lat: u8) -> DMAShiftRegister {
        DMAShiftRegister { sck, sin, lat }
    }

    pub fn init_dma_pin(pin: impl Pin) -> u8 {
        let pin_nr = pin._pin();
        // We need to make sure the Output<> destructors are not called after init_dma_pin()
        // (or the parent function) finishes, since we'll be DMAing them manually in the ODR reg.
        core::mem::forget(Output::new(pin, Level::Low, Speed::VeryHigh));
        pin_nr
    }

    pub fn new(sck: u8, sin: impl Pin, lat: impl Pin) -> DMAShiftRegister {
        // We assume these pins are all on the same ODR register
        // assert_eq!(sck, sin._port());
        assert_eq!(sin._port(), lat._port());

        let r = DMAShiftRegister {
            sck,
            sin: sin._pin(),
            lat: lat._pin(),
        };

        // We need to make sure the Output<> destructors are not called after new() finishes.
        Self::init_dma_pin(sin);
        Self::init_dma_pin(lat);

        r
    }

    pub fn init_sck(&self, slice: &mut [u16; Self::CLOCKS_PER_BIT]) {
        // for i in 0..Self::CLOCKS_PER_BIT {
        //   if let Some(sck) = self.sck {
        //     match i % 2 {
        //       0 => unset_bit(&mut slice[i], sck),
        //       _ => set_bit(&mut slice[i], sck),
        //     }
        //   }
        // }
        set_bit(&mut slice[1], self.sck)
    }

    pub fn set_sin(&self, slice: &mut [u16; Self::CLOCKS_PER_BIT], level: Level) {
        for b in slice {
            match level {
                Level::Low => unset_bit(b, self.sin),
                Level::High => set_bit(b, self.sin),
            }
        }
    }

    pub fn latch(&self, buf: &mut u16) {
        set_bit(buf, self.lat);
    }
}
