use embassy_stm32::{gpio::{Output, AnyPin, Pin, Level}, Peripheral};

use crate::{shift_register::{DMAShiftRegister, set_bit}, led::{Led, LedIntensity}};

const MAX_LEDS_PER_CHIP: usize = 16;

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

  pub fn to_levels(&self, iteration: u8) -> [[Level; MAX_LEDS_PER_CHIP]; CHIPS] {
    let mut result = [[Level::Low; MAX_LEDS_PER_CHIP]; CHIPS];
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
  bank_enable_pins: [AnyPin; BANKS],
  led_sequence: [LedChipSequence<CHIPS, LEDS_PER_CHIP>; BANKS],
}

// The buffer size for BANKS=1, CHIPS=1
// The final buffer size depends on number of chips and banks.
const BASIC_BUFSIZE: usize = DMAShiftRegister::CLOCKS_PER_BIT * // DMA values needed to send a bit
                             MAX_LEDS_PER_CHIP *                // BANKS * CHIPS * 16 LED channels to steer
                             LedIntensity::max_usize();         // brightness levels

pub struct LedDriver<
  const BANKS: usize = 1,
  const CHIPS: usize = 1,
  const LEDS_PER_CHIP: usize = 5
> where [Led; CHIPS * LEDS_PER_CHIP]: Sized {
  reg: DMAShiftRegister,
  banks: LedBankSet<BANKS, CHIPS, LEDS_PER_CHIP>,
}

impl<
  const BANKS: usize,
  const CHIPS: usize,
  const LEDS_PER_CHIP: usize
> LedDriver<BANKS, CHIPS, LEDS_PER_CHIP>
  where [Led; CHIPS * LEDS_PER_CHIP]: Sized,
{
  // This puts the compiler in a cyclic redundancy when used in function signatures.
  // So, for now, we type these manually.
  // const BUFSIZE: usize = BASIC_BUFSIZE*BANKS*CHIPS;

  // pub const fn banks(&self) -> usize {
  //   BANKS
  // }

  // pub const fn chips(&self) -> usize {
  //   CHIPS
  // }

  pub const fn buffer_size() -> usize {
    BASIC_BUFSIZE*BANKS*CHIPS
  }

  pub fn new(
    sck: impl Pin,
    sin: impl Pin,
    lat: impl Pin,
    mut bank_enable_pins: [AnyPin; BANKS],
  ) -> LedDriver<BANKS, CHIPS, LEDS_PER_CHIP> {
    let reg = DMAShiftRegister::new(sck, sin, lat);

    for pin in &mut bank_enable_pins {
      // We need to make sure the Output<> destructors are not called after new() finishes.
      core::mem::forget(Output::new(pin.into_ref(), Level::Low, embassy_stm32::gpio::Speed::VeryHigh));
    }

    let banks = LedBankSet{
      bank_enable_pins,
      led_sequence: [Default::default(); BANKS],
    };
    LedDriver{reg, banks}
  }

  /// Retrieve all LED instances for a given bank.
  /// ```
  /// let leds = driver.get_bank_leds(0);
  /// leds[0].r = LedIntensity::MAX;
  /// ```
  pub fn get_bank_leds(&mut self, bank: usize) -> &mut [Led; CHIPS * LEDS_PER_CHIP] {
    &mut self.banks.led_sequence[bank].leds
  }

  /// Sets bits in a given input buffer to steer PWMed LEDS in a DMA cycle.
  /// If the output buffer exceeds the needed space, the buffer values are
  /// repeated until the buffer is filled. If a section of the buffer is not
  /// big enough to fit a full cycle, it is untouched.
  /// 
  /// This function only, sets/unsets bits that it is responsible for.
  /// The buffer does not need to be re-initialized to be re-used with
  /// different LED settings.
  /// 
  /// Given the above, the buffer input can be shared with multiple LedDriver
  /// instances of different CHIPS & BANK sizes.
  pub fn calculate_output_buffer<const N: usize>(&mut self, buf: &mut [u16; N]) {
    // This whole cycle repeats for as long as there is enough space in the buffer
    for buf_chunk in buf.chunks_exact_mut(BASIC_BUFSIZE*BANKS*CHIPS) {

      let mut buf_i = 0;

      // Re-do buffer filling for all intensity levels to achieve "PWM".
      for intensity_index in LedIntensity::min_u8()..LedIntensity::max_u8() {

        // Go over each LED bank.
        for bank_index in 0..BANKS {

          // We're sending the data for the _next_ bank over the serial bus now.
          // The previous bank needs to be lighted up while we finish this.
          let prev_bank_index = if bank_index == 0 { BANKS-1 } else { bank_index - 1 };

          let bank_pin = self.banks.bank_enable_pins[prev_bank_index].pin();

          // Get the LED bits to send over serial.
          let logic_levels_per_chip = self.banks.led_sequence[bank_index].to_levels(intensity_index);

          // To prevent glitchy stuff when stopping DMA and sending a brand new buffer,
          // we prevent sending the first latch bit at the first DMA output.
          // Instead the first latch is set after a full serial output.
          // Correct latching in circular DMA is preserved by setting a compensation
          // latch bit at the very end of the whole buffer.
          if buf_i != 0 {
            self.reg.latch(&mut buf_chunk[buf_i]);
          }
          // The first bit must be sent last through our shift register.
          // So, in terms of logic levels to send, everything needs to be inverted.
          for mut chip_levels in logic_levels_per_chip.into_iter().rev() {
            chip_levels.reverse();

            let n = self.reg.encode_bits(chip_levels, &mut buf_chunk[buf_i..]);
            for i in 0..n {
              set_bit(&mut buf_chunk[buf_i+i], bank_pin);
              // No need to unset other bank bits, we don't dynamically
              // change BANKS/CHIPS size so other bank bits can never be already set.
              // unset_bit(&mut buf_chunk[i], _);
            }
            buf_i += n;
          }
        }

      }

      self.reg.latch(&mut buf_chunk[buf_i-1]);
    }
  }

}