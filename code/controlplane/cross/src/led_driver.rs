use embassy_stm32::gpio::{Level, Pin};

use crate::{
    led::{Led, LedIntensity},
    shift_register::{set_bit, DMAShiftRegister},
};

const MAX_LEDS_PER_CHIP: usize = 16;

/// A series of LEDs to control the RGB values for.
/// ```
/// use cross::led_driver::LedChipSequence;
/// use cross::led::LedIntensity;
///
/// let mut leds = LedChipSequence::<2,3>::default();
/// let leds = leds.borrow_mut();
/// assert_eq!(leds.len(), 6);
///
/// leds[0].r.set(LedIntensity::max_u8());
/// leds[0].g.set(1);
/// assert_eq!(leds[0].g.as_u8(), 1);
/// assert_eq!(leds[0].b.as_u8(), 0);
/// ```
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct LedChipSequence<const CHIPS: usize, const LEDS_PER_CHIP: usize>
where
    [Led; CHIPS * LEDS_PER_CHIP]: Sized,
{
    leds: [Led; CHIPS * LEDS_PER_CHIP],
}

impl<const CHIPS: usize, const LEDS_PER_CHIP: usize> Default
    for LedChipSequence<CHIPS, LEDS_PER_CHIP>
where
    [Led; CHIPS * LEDS_PER_CHIP]: Sized,
{
    fn default() -> Self {
        Self {
            leds: [Led::default(); CHIPS * LEDS_PER_CHIP],
        }
    }
}

impl<const CHIPS: usize, const LEDS_PER_CHIP: usize> LedChipSequence<CHIPS, LEDS_PER_CHIP>
where
    [Led; CHIPS * LEDS_PER_CHIP]: Sized,
{
    pub fn borrow_mut(&mut self) -> &mut [Led; CHIPS * LEDS_PER_CHIP] {
        &mut self.leds
    }

    /// Converts the LED settings to a series of logical bits to send to the LED driver chip(s)
    /// for a given iteration of the PWM cycle.
    /// Note that these are logical bits, they still need to be converted to physical bits
    /// to send over the serial connection.
    pub fn to_levels(&self, iteration: u8) -> [[Level; MAX_LEDS_PER_CHIP]; CHIPS] {
        let mut result = [[Level::Low; MAX_LEDS_PER_CHIP]; CHIPS];
        for i_chip in 0..CHIPS {
            for i_led in 0..LEDS_PER_CHIP {
                let chip_levels = &mut result[i_chip];
                let led = &self.leds[i_chip * LEDS_PER_CHIP + i_led];
                let levels_index = i_led * 3;
                chip_levels[levels_index..levels_index + 3]
                    .copy_from_slice(&led.to_levels(iteration))
            }
        }
        result
    }
}

/// A set of `LedChipSequence`s combined into one or more banks.
pub struct LedBankSet<const BANKS: usize, const CHIPS: usize, const LEDS_PER_CHIP: usize>
where
    [Led; CHIPS * LEDS_PER_CHIP]: Sized,
{
    pub bank_enable_pins: [u8; BANKS],
    led_sequence: [LedChipSequence<CHIPS, LEDS_PER_CHIP>; BANKS],
}

// The buffer size for BANKS=1, CHIPS=1
// The final buffer size depends on number of chips and banks.
const BASIC_BUFSIZE: usize = DMAShiftRegister::CLOCKS_PER_BIT * // DMA values needed to send a bit
                             MAX_LEDS_PER_CHIP *                // 16 LED channels to steer
                             (LedIntensity::max_usize() + 0); // brightness levels + 1 idle cycle for bank switching

/// A driver acting as an abstraction layer between logical LEDs and the serial connection to the driver IC.
/// ```ignore
/// // A driver for 2 banks, 1 chip, 5 RGB LEDs per chip.
/// let mut buffer = [0u16; LedDriver::<2, 1, 5>::buffer_size()];
/// let mut driver = LedDriver::<2, 1, 5>::new(sck, sin, lat, [bank1.degrade(), bank2.degrade()]);
/// driver.init(&mut buffer);
///
/// let leds = driver.get_bank_leds(1);
/// leds[0].r = LedIntensity::MAX;
///
/// driver.write_serial(&mut buffer);
/// ```
pub struct LedDriver<const BANKS: usize = 1, const CHIPS: usize = 1, const LEDS_PER_CHIP: usize = 5>
where
    [Led; CHIPS * LEDS_PER_CHIP]: Sized,
{
    reg: DMAShiftRegister,
    pub banks: LedBankSet<BANKS, CHIPS, LEDS_PER_CHIP>,
}

impl<const BANKS: usize, const CHIPS: usize, const LEDS_PER_CHIP: usize>
    LedDriver<BANKS, CHIPS, LEDS_PER_CHIP>
where
    [Led; CHIPS * LEDS_PER_CHIP]: Sized,
{
    const BUFSIZE: usize = BASIC_BUFSIZE * BANKS * CHIPS;
    /// The required size of a u16 (ODR) buffer to use with this driver.
    pub const fn buffer_size() -> usize {
        Self::BUFSIZE
    }

    pub const fn bank_count(&self) -> usize {
        BANKS
    }

    pub const fn chip_count(&self) -> usize {
        CHIPS
    }

    pub const fn leds_count(&self) -> usize {
        LEDS_PER_CHIP*CHIPS
    }

    #[cfg(test)]
    pub fn testnew(
        sck: u8,
        sin: u8,
        lat: u8,
        bank_enable_pins: [u8; BANKS],
    ) -> LedDriver<BANKS, CHIPS, LEDS_PER_CHIP> {
        let reg = DMAShiftRegister::testnew(sck, sin, lat);

        let banks = LedBankSet {
            bank_enable_pins,
            led_sequence: [Default::default(); BANKS],
        };
        LedDriver { reg, banks }
    }

    /// Creates a new driver with the given serial and bank pins
    /// ```ignore
    /// // clock bits can be shared with multiple drivers, and has to initialized by you.
    /// let sck = DMAShiftRegister::init_dma_pin(p.PB0);
    /// let sin = p.PB1;
    /// let lat = p.PB2;
    /// let bank1 = p.PB3;
    /// let bank2 = p.PB4;
    /// // A driver for 2 banks, 1 chip, 5 RGB LEDs per chip.
    /// let mut driver = LedDriver::<2, 1, 5>::new(sck, sin, lat, [bank1.degrade(), bank2.degrade()]);
    /// ```
    pub fn new(
        sck: u8,
        sin: impl Pin,
        lat: impl Pin,
        bank_enable_pins: [impl Pin; BANKS],
    ) -> LedDriver<BANKS, CHIPS, LEDS_PER_CHIP> {
        let reg = DMAShiftRegister::new(sck, sin, lat);

        let bank_enable_pins = bank_enable_pins.map(|pin| DMAShiftRegister::init_dma_pin(pin));

        let banks = LedBankSet {
            bank_enable_pins,
            led_sequence: [Default::default(); BANKS],
        };
        LedDriver { reg, banks }
    }

    /// Retrieve all LED instances for a given bank.
    /// ```ignore
    /// let leds = driver.get_bank_leds(0);
    /// leds[0].r = LedIntensity::MAX;
    /// ```
    pub fn get_bank_leds(&mut self, bank: usize) -> &mut [Led; CHIPS * LEDS_PER_CHIP] {
        self.banks.led_sequence[bank].borrow_mut()
    }

    /// Sets control bits in a given dma buffer to steer PWMed LEDS in a DMA cycle.
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
    ///
    /// This function sets all bits except serial data bits. For that,
    /// use `calculate_output_buffer()`.
    /// The other bits set here are always the same regardless of serial
    /// input, and thus can be set only once during initialization.
    pub fn init<const N: usize>(&self, buf: &mut [u16; N]) {
        // If the output buffer exceeds the needed space, the buffer values are
        // repeated until the buffer is filled. If a section of the buffer is not
        // big enough to fit a full cycle, it is untouched.
        for buf_chunk in buf.chunks_exact_mut(BASIC_BUFSIZE * BANKS * CHIPS) {
            let mut i_chunk = 0;

            // We handle each bank one by one
            for bank_index in 0..BANKS {
                let bank_pin = self.banks.bank_enable_pins[bank_index];

                // Re-do buffer filling for all intensity levels to achieve "PWM".
                for i_intensity in LedIntensity::min_u8()..LedIntensity::max_u8() {
                    // Chips are chained, the data to send over serial scales linearly with chip count
                    for _i_chip in 0..CHIPS {
                        // We sacrifice our top intensity cycle to allow the bank MOSFET to shut off in time for the next
                        // bank LEDs to light up.
                        // We do this on the first intensity cycle instead of the last because being on our first intensity
                        // cycle means we just switched to this bank, and thus the last bank values are still latched while
                        // we are sending these new values.
                        // This does not apply if we have only one BANK
                        // This assumes LedIntensity is a decent range, otherwise the intensity losses
                        // would be big. e.g. With 2 intensity levels we'd slash our max intensity in half.
                        if BANKS == 1 || i_intensity != LedIntensity::min_u8() {
                            // Enable this bank for the whole duration of data transfer
                            for i in 0..MAX_LEDS_PER_CHIP * DMAShiftRegister::CLOCKS_PER_BIT {
                                set_bit(&mut buf_chunk[i_chunk + i], bank_pin);
                            }
                        }

                        for _logic_bit_i in 0..MAX_LEDS_PER_CHIP {
                            // let slice = &mut buf_chunk[chunk_i..chunk_i+DMAShiftRegister::CLOCKS_PER_BIT];
                            // let mut slice = Self::as_sin_slice(&mut buf_chunk[chunk_i..chunk_i+DMAShiftRegister::CLOCKS_PER_BIT]);
                            let slice = array_mut_ref![
                                buf_chunk,
                                i_chunk,
                                DMAShiftRegister::CLOCKS_PER_BIT
                            ];
                            self.reg.init_sck(slice);
                            // set_bit(&mut slice[0], 8);
                            i_chunk += DMAShiftRegister::CLOCKS_PER_BIT;
                        }
                    }

                    self.reg.latch(&mut buf_chunk[i_chunk - 1]);
                }
            }
        }
    }

    /// Sets serial bits in a given dma buffer to steer PWMed LEDS in a DMA cycle.
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
    pub fn write_serial<const N: usize>(&self, buf: &mut [u16; N]) {
        // similar loops as init()
        for buf_chunk in buf.chunks_exact_mut(BASIC_BUFSIZE * BANKS * CHIPS) {
            let mut i_chunk = 0;
            for i_bank in 0..BANKS {
                for i_intensity in LedIntensity::min_u8()..LedIntensity::max_u8() {
                    // Get logical bits to send over serial
                    let logic_levels_per_chip =
                        self.banks.led_sequence[i_bank].to_levels(i_intensity);
                    for mut chip_logic_levels in logic_levels_per_chip
                        .into_iter()
                        // The first bit must be sent last through our shift register.
                        // So, in terms of logic levels to send, everything needs to be inverted.
                        .rev()
                    {
                        chip_logic_levels.reverse();

                        for l in chip_logic_levels {
                            let slice = array_mut_ref![
                                buf_chunk,
                                i_chunk,
                                DMAShiftRegister::CLOCKS_PER_BIT
                            ];
                            self.reg.set_sin(slice, l);
                            i_chunk += DMAShiftRegister::CLOCKS_PER_BIT;
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::File;
    use std::io::Write;

    fn assert_b(a: u16, b: u16, index: usize) {
        assert_eq!(a, b, "mismatch at {}: {:#016b} != {:#016b}", index, a, b);
    }

    fn compare_buffer(input: &[u16], offset: usize, expected: &[u16]) {
        for i in 0..expected.len() {
            assert_b(input[offset + i], expected[i], offset + i);
        }
    }

    fn bit_is_set(input: u16, n: usize) -> bool {
        input & (1 << n) != 0
    }

    #[test]
    fn test_smallest() {
        // smallest possible buffer, one bank, one chip
        let mut buffer = [0u16; LedDriver::<1, 1, 5>::buffer_size()];
        let mut driver = LedDriver::<1, 1, 5>::testnew(1, 2, 3, [0]);

        for led in driver.get_bank_leds(0) {
            led.r.set(100);
            led.g.set(100);
            led.b.set(100);
        }

        driver.init(&mut buffer);
        driver.write_serial(&mut buffer);

        let mut f = File::create("/tmp/buffer_smallest.txt").expect("Unable to create file");
        for b in buffer {
            f.write(format!("{:016b}\n", b).as_bytes());
        }

        for i in 0..buffer.len() {
            // Being one BANK, expect bank to be enabled at all times
            assert!(bit_is_set(buffer[i], 0), "bank disabled on index {}", i);

            // All LEDs should be on.
            // So we always send a HIGH on sin except on the 16th locical bit (5LEDs * 3RGB = 15), which is the first sent one
            if i % (MAX_LEDS_PER_CHIP * DMAShiftRegister::CLOCKS_PER_BIT)
                < DMAShiftRegister::CLOCKS_PER_BIT
            {
                assert!(!bit_is_set(buffer[i], 2), "sin bit is set on index {}", i);
            } else {
                assert!(
                    bit_is_set(buffer[i], 2),
                    "sin bit is not set on index {}",
                    i
                );
            }

            if i % DMAShiftRegister::CLOCKS_PER_BIT == 1 {
                assert!(
                    bit_is_set(buffer[i], 1),
                    "sck bit is not set on index {}",
                    i
                );
            } else {
                assert!(!bit_is_set(buffer[i], 1), "sck bit is set on index {}", i);
            }
        }
    }

    #[test]
    fn test_rgb() {
        // smallest possible buffer, one bank, one chip
        let mut buffer = [0u16; LedDriver::<1, 1, 5>::buffer_size()];
        let mut driver = LedDriver::<1, 1, 5>::testnew(1, 2, 3, [0]);

        let mut leds = driver.get_bank_leds(0);
        leds[0].r.set(100);
        leds[1].g.set(100);
        leds[2].b.set(100);

        driver.init(&mut buffer);
        driver.write_serial(&mut buffer);

        let mut f = File::create("/tmp/buffer_rgb.txt").expect("Unable to create file");
        for b in buffer {
            f.write(format!("{:016b}\n", b).as_bytes());
        }
    }

    #[test]
    fn test_rgb_all() {
        // smallest possible buffer, one bank, one chip
        let mut buffer = [0u16; LedDriver::<1, 1, 5>::buffer_size()];
        let mut driver = LedDriver::<1, 1, 5>::testnew(1, 2, 3, [0]);

        let mut leds = driver.get_bank_leds(0);
        for led in leds {
            led.r.set(100);
            led.g.set(100);
            led.b.set(100);
        }

        driver.init(&mut buffer);
        driver.write_serial(&mut buffer);

        let mut f = File::create("/tmp/buffer_rgb_all.txt").expect("Unable to create file");
        for b in buffer {
            f.write(format!("{:016b}\n", b).as_bytes());
        }
    }

    #[test]
    fn test_two_banks() {
        // two banks, one chip
        let mut buffer = [0u16; LedDriver::<2, 1, 5>::buffer_size()];
        let mut driver = LedDriver::<2, 1, 5>::testnew(2, 3, 4, [0, 1]);

        for led in driver.get_bank_leds(0) {
            led.r.set(100);
            led.g.set(100);
            led.b.set(100);
        }
        for led in driver.get_bank_leds(1) {
            led.r.set(100);
            led.g.set(100);
            led.b.set(100);
        }

        driver.init(&mut buffer);
        driver.write_serial(&mut buffer);

        let mut f = File::create("/tmp/buffer_two_banks.txt").expect("Unable to create file");
        for b in buffer {
            f.write(format!("{:016b}\n", b).as_bytes());
        }

        for i in 0..buffer.len() {
            // With two banks, we except only one bank to be enabled at a time,
            // except for when we just switched banks, where none are enabled for a short time.
            let cycle_size = MAX_LEDS_PER_CHIP * DMAShiftRegister::CLOCKS_PER_BIT;
            let bank_size = cycle_size * LedIntensity::max_usize();
            if i % bank_size < cycle_size {
                assert!(
                    !bit_is_set(buffer[i], 0),
                    "bank 0 is not disabled on index {}",
                    i
                );
                assert!(
                    !bit_is_set(buffer[i], 1),
                    "bank 1 is not disabled on index {}",
                    i
                );
            } else if i < bank_size {
                assert!(
                    bit_is_set(buffer[i], 0),
                    "bank 0 is not enabled on index {}",
                    i
                );
                assert!(
                    !bit_is_set(buffer[i], 1),
                    "bank 1 is not disabled on index {}",
                    i
                );
            } else {
                assert!(
                    !bit_is_set(buffer[i], 0),
                    "bank 0 is not disabled on index {}",
                    i
                );
                assert!(
                    bit_is_set(buffer[i], 1),
                    "bank 1 is not enabled on index {}",
                    i
                );
            }

            // All LEDs should be on.
            // So we always send a HIGH on sin except on the 16th locical bit (5LEDs * 3RGB = 15), which is the first sent one
            if i % cycle_size < DMAShiftRegister::CLOCKS_PER_BIT {
                assert!(!bit_is_set(buffer[i], 3), "sin bit is set on index {}", i);
            } else {
                assert!(
                    bit_is_set(buffer[i], 3),
                    "sin bit is not set on index {}",
                    i
                );
            }
        }
    }

    #[test]
    fn test_big() {
        // five banks, two chips
        let mut buffer = [0u16; LedDriver::<5, 2, 5>::buffer_size()];
        let mut driver = LedDriver::<5, 2, 5>::testnew(5, 6, 7, [0, 1, 2, 3, 4]);

        // for led in driver.get_bank_leds(0) {
        //   led.r.set(100);
        //   led.g.set(0);
        //   led.b.set(0);
        // }
        // for led in driver.get_bank_leds(1) {
        //   led.r.set(0);
        //   led.g.set(100);
        //   led.b.set(0);
        // }
        // for led in driver.get_bank_leds(2) {
        //   led.r.set(0);
        //   led.g.set(0);
        //   led.b.set(100);
        // }
        // for led in driver.get_bank_leds(3) {
        //   led.r.set(0);
        //   led.g.set(100);
        //   led.b.set(100);
        // }
        // for led in driver.get_bank_leds(4) {
        //   led.r.set(100);
        //   led.g.set(100);
        //   led.b.set(0);
        // }

        // let leds = driver.get_bank_leds(0);
        // for led in leds.iter_mut().step_by(3) {
        //     led.r.set(1);
        // }

        for bank in 0..5 {
            for led in driver.get_bank_leds(bank) {
                led.r.set(100);
                led.g.set(100);
                led.b.set(100);
            }
        }

        driver.init(&mut buffer);
        driver.write_serial(&mut buffer);

        let mut f = File::create("/tmp/buffer_big.txt").expect("Unable to create file");
        for b in buffer {
            f.write(format!("{:016b}\n", b).as_bytes());
        }

        for i in 0..buffer.len() {
            // With two banks, we expect only one bank to be enabled at a time,
            // except for when we just switched banks, where none are enabled for a short time.
            let cycle_size = MAX_LEDS_PER_CHIP * DMAShiftRegister::CLOCKS_PER_BIT;
            let bank_size = 2 * cycle_size * LedIntensity::max_usize();
            if i % bank_size < cycle_size {
                assert!(
                    !bit_is_set(buffer[i], 0),
                    "bank 0 is not disabled on index {}",
                    i
                );
                assert!(
                    !bit_is_set(buffer[i], 1),
                    "bank 1 is not disabled on index {}",
                    i
                );
                assert!(
                    !bit_is_set(buffer[i], 2),
                    "bank 2 is not disabled on index {}",
                    i
                );
                assert!(
                    !bit_is_set(buffer[i], 3),
                    "bank 3 is not disabled on index {}",
                    i
                );
                assert!(
                    !bit_is_set(buffer[i], 4),
                    "bank 4 is not disabled on index {}",
                    i
                );
            }

            // All LEDs should be on.
            // So we always send a HIGH on sin except on the 16th locical bit (5LEDs * 3RGB = 15), which is the first sent one
            if i % cycle_size < DMAShiftRegister::CLOCKS_PER_BIT {
                assert!(!bit_is_set(buffer[i], 6), "sin bit is set on index {}", i);
            } else {
                assert!(
                    bit_is_set(buffer[i], 6),
                    "sin bit is not set on index {} {:016b}",
                    i,
                    buffer[i]
                );
            }
        }
    }
}
