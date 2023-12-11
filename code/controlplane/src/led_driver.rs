use embassy_stm32::gpio::{Output, AnyPin, Pin, Level};
use embassy_time::{Timer, Duration, Delay, block_for};

use crate::{shift_register::ShiftRegister, led::{Led, LedIntensity}};

// Maximum PWM before flashing gets noticeable, in microseconds of off-duty
// static MAX_LED_PWM: u64 = 20_000;
static MAX_LED_PWM: u64 = 5_000;
// static MAX_LED_PWM: u64 = 10_000;

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct LedChipSequence<const CHIPS: usize, const LEDS_PER_CHIP: usize> where [Led; CHIPS * LEDS_PER_CHIP]: Sized {
  leds: [Led; CHIPS * LEDS_PER_CHIP],
}

impl<const CHIPS: usize, const LEDS_PER_CHIP: usize> Default for LedChipSequence<CHIPS, LEDS_PER_CHIP> where [Led; CHIPS * LEDS_PER_CHIP]: Sized {
  fn default() -> Self {
    Self { leds: [Led::default(); CHIPS * LEDS_PER_CHIP] }
  }
}

impl<const CHIPS: usize, const LEDS_PER_CHIP: usize> LedChipSequence<CHIPS, LEDS_PER_CHIP> where [Led; CHIPS * LEDS_PER_CHIP]: Sized {
  pub fn to_levels(&self, iteration: u8) -> [[Level; 16]; CHIPS] {
    let mut result = [[Level::Low; 16]; CHIPS];
    for i_chip in 0..CHIPS {
      for i_led in 0..LEDS_PER_CHIP {
        let chip_levels = &mut result[i_chip];
        let led = &self.leds[i_chip*LEDS_PER_CHIP+i_led];
        let levels_index = i_led*3;
        chip_levels[levels_index..levels_index+3].copy_from_slice(&led.to_levels(iteration))
      }
    }
    result
  }
}

pub struct LedBankSet<const BANKS: usize, const CHIPS: usize, const LEDS_PER_CHIP: usize> where [Led; CHIPS * LEDS_PER_CHIP]: Sized {
  bank_enable_pins: [Output<'static, AnyPin>; BANKS],
  led_sequence: [LedChipSequence<CHIPS, LEDS_PER_CHIP>; BANKS],
}

impl<const BANKS: usize, const CHIPS: usize, const LEDS_PER_CHIP: usize> LedBankSet<BANKS, CHIPS, LEDS_PER_CHIP> where [Led; CHIPS * LEDS_PER_CHIP]: Sized {
  pub fn enable(&mut self, bank: usize) {
    self.bank_enable_pins[bank].set_low();
  }
  pub fn disable(&mut self, bank: usize) {
    self.bank_enable_pins[bank].set_high();
  }
  pub fn disable_all(&mut self) {
    for bank in 0..BANKS {
      self.disable(bank);
    }
  }
}

// pub struct LedChipSequence<const CHIPS: usize, const LEDS_PER_CHIP: usize>
// where [Led; CHIPS * LEDS_PER_CHIP]: Sized {

// }


pub struct LedDriver<
  SCK: Pin, SIN: Pin, LATCH: Pin,
  const BANKS: usize = 1,
  const CHIPS: usize = 1,
  const LEDS_PER_CHIP: usize = 5,
> where [Led; CHIPS * LEDS_PER_CHIP]: Sized {
  reg: ShiftRegister<SCK, SIN, LATCH>,
  banks: LedBankSet<BANKS, CHIPS, LEDS_PER_CHIP>,
}

impl<
  SCK: Pin, SIN: Pin, LATCH: Pin,
  const CHIPS: usize,
  const BANKS: usize,
  const LEDS_PER_CHIP: usize>
    LedDriver<SCK, SIN, LATCH, BANKS, CHIPS, LEDS_PER_CHIP> where [Led; CHIPS * LEDS_PER_CHIP]: Sized
{
  pub fn new(
    sck: Output<'static, SCK>,
    sin: Output<'static, SIN>,
    lat: Output<'static, LATCH>,
    bank_enable_pins: [Output<'static, AnyPin>; BANKS],
  ) -> LedDriver<SCK, SIN, LATCH, BANKS, CHIPS, LEDS_PER_CHIP> {
    let reg = ShiftRegister::new(sck, sin, lat);
    let banks = LedBankSet{
      bank_enable_pins,
      led_sequence: [Default::default(); BANKS],
    };
    LedDriver{reg, banks}
  }

  pub fn get_bank_leds(&mut self, bank: usize) -> &mut [Led; CHIPS * LEDS_PER_CHIP] {
    &mut self.banks.led_sequence[bank].leds
  }

  pub async fn display_cycle_test(&mut self) {
      for iteration in 0..255 {
        let l = iteration < 1;

        self.reg.send_bits([l.into(); 32]).await;
        self.reg.latch().await;
        // block_for(Duration::from_ticks(1));
        // block_for(Duration::from_ticks(1));
        // block_for(Duration::from_ticks(1));
        // block_for(Duration::from_ticks(1));
        // block_for(Duration::from_nanos(1));
        // block_for(Duration::from_nanos(1));
        // block_for(Duration::from_nanos(1));
        // block_for(Duration::from_nanos(1));
        // Timer::after(Duration::from_micros_floor(1)).await;
        // Timer::after(Duration::from_micros_floor(1)).await;
        // Timer::after(Duration::from_micros_floor(1)).await;
        // Timer::after(Duration::from_micros_floor(1)).await;
        // Timer::after(Duration::from_micros_floor(4)).await;
        // Timer::after_micros(1).await;
      }
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after(Duration::from_micros_floor(1)).await;
      // Timer::after_micros(1).await;
  }

  pub async fn display_cycle(&mut self) {
    // self.banks.disable_all();
    let li = LedIntensity::MAX.into();
    for iteration in 0..li {
      for bank in (0..BANKS).rev() {
        // self.banks.enable(bank);
        let all_levels = self.banks.led_sequence[bank].to_levels(iteration);
        for mut levels in all_levels.into_iter().rev() {
          levels.reverse();
          self.reg.send_bits(levels).await;
        }
        self.reg.latch().await;
        // self.banks.disable(bank);
      }
      // Timer::after_micros(MAX_LED_PWM/BANKS as u64/li as u64).await;
      Timer::after_nanos(1).await;
    }

    // self.reg.send_bits([Level::High; 32]);
    // self.reg.latch();
    // Timer::after_micros(MAX_LED_PWM).await;
  }

  // pub async fn display_cycle(&mut self) {
  //   for pin in &mut self.banks {
  //     pin.set_high();
  //     for i in 0u8..200 {
  //       let iter = self.leds.into_iter().rev().array_chunks::<LEDS_PER_CHIP>();
  //       self.leds.into_iter().rev();
  //       while let ledCHIPS = iter.next_chunk() {

  //       }
  //       for ledCHIPS in self.leds.into_iter().rev().array_chunks() {

  //       }
  //     }
  //     pin.set_low();
  //   }
  //   // self.reg.send_bits(data);
  //   // self.reg.latch();
  // }
}