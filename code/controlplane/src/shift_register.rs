use embassy_stm32::gpio::{Pin, Output, Level, Speed};

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

impl DMAShiftRegister {
  pub const CLOCKS_PER_BIT: usize = 3;

  pub fn new(sck: impl Pin, sin: impl Pin, lat: impl Pin) -> DMAShiftRegister {
    assert_eq!(sck._port(), sin._port());
    assert_eq!(sck._port(), lat._port());

    let r = DMAShiftRegister { sck: sck._pin(), sin: sin._pin(), lat: lat._pin() };

    // We need to make sure the Output<> destructors are not called after new() finishes.
    core::mem::forget(Output::new(sck, Level::Low, Speed::VeryHigh));
    core::mem::forget(Output::new(sin, Level::Low, Speed::VeryHigh));
    core::mem::forget(Output::new(lat, Level::Low, Speed::VeryHigh));

    r
  }

  // pub fn send_bits<const N: usize>(&mut self, data: [Level; N], buf: &mut [u16; N*3]) {
  pub fn encode_bits<const N: usize>(&mut self, data: [Level; N], buf: &mut [u16]) -> usize {
    assert!(buf.len() >= N*3);

    for (level_nr, l) in data.iter().enumerate() {
      let buf_i = level_nr*Self::CLOCKS_PER_BIT;

      // Reset SCK and LAT pins.
      // Set SIN pin for whole CLOCKS_PER_BIT duration.
      for i in buf_i..buf_i+Self::CLOCKS_PER_BIT {
        unset_bit(&mut buf[i], self.sck);
        match l {
          Level::High => set_bit(&mut buf[i], self.sin),
          Level::Low => unset_bit(&mut buf[i], self.sin),
        }
      }

      // Clock pulse in the middle.
      // This ensures a nice SIN signal around the clock signal edge.
      set_bit(&mut buf[buf_i+1], self.sck);
      // unset_bit(&mut buf[i+2], sck);
    }

    N*3
  }

  pub fn latch(&mut self, buf: &mut u16) {
    set_bit(buf, self.lat);
  }
}
