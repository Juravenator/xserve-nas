use embassy_stm32::gpio::{Pin, Output, Level};
use embassy_time::{Timer, Duration, block_for};


pub struct ShiftRegister<SCK: Pin, SIN: Pin, LATCH: Pin> {
  sck: Output<'static, SCK>,
  sin: Output<'static, SIN>,
  lat: Output<'static, LATCH>,
}

impl<SCK: Pin, SIN: Pin, LATCH: Pin> ShiftRegister<SCK, SIN, LATCH> {
  pub fn new(
    sck: Output<'static, SCK>,
    sin: Output<'static, SIN>,
    lat: Output<'static, LATCH>,
  ) -> ShiftRegister<SCK, SIN, LATCH> {
    ShiftRegister { sck, sin, lat }
  }

  pub async fn send_bits<const N: usize>(&mut self, data: [Level; N]) {
    for l in data {
      self.sin.set_level(l);
      self.sck.set_high();
      // block_for(Duration::from_ticks(1));
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after_millis(40).await;
      self.sck.set_low();
      // block_for(Duration::from_ticks(1));
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after_millis(40).await;
    }
  }

  pub async fn latch(&mut self) {
    self.lat.set_high();
    // Timer::after(Duration::from_micros_floor(1)).await;
    // Timer::after_millis(40).await;
    self.lat.set_low();
  }
}

// impl ShiftRegister {
//   pub fn new(
//     sck: Output<'static, AnyPin>,
//     sin: Output<'static, AnyPin>,
//     lat: Output<'static, AnyPin>,
//   ) -> ShiftRegister { ShiftRegister{sck, sin, lat} }
// }