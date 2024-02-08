use embassy_stm32::gpio::Level;

pub enum LedColour {
    RED,
    GREEN,
    BLUE,
}

#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct LedIntensity(u8);

impl From<u8> for LedIntensity {
    fn from(v: u8) -> Self {
        let mut r = LedIntensity::default();
        r.set(v);
        r
    }
}

impl Into<u8> for LedIntensity {
    fn into(self) -> u8 {
        self.0
    }
}

impl Into<usize> for LedIntensity {
    fn into(self) -> usize {
        self.0.into()
    }
}

impl LedIntensity {
    pub const MIN: LedIntensity = LedIntensity(0);
    pub const MAX: LedIntensity = LedIntensity(20);

    pub const fn max_u8() -> u8 {
        Self::MAX.0
    }
    pub const fn max_usize() -> usize {
        Self::MAX.0 as usize
    }
    pub const fn min_u8() -> u8 {
        Self::MIN.0
    }
    pub const fn min_usize() -> usize {
        Self::MIN.0 as usize
    }

    /// Given a position in the PWM cycle, return ON|OFF level.
    /// e.g. If LedIntensity::MAX==20 and self.0==10 (50%),
    /// only return ON if iteration is 0-9.
    pub fn to_level(&self, iteration: u8) -> Level {
        (self.0 > iteration).into()
    }

    /// Set the intensity to the given value if it is between MIN and MAX.
    /// If outside the valid range, it will be corrected to MIN or MAX.
    pub fn set(&mut self, value: u8) {
        self.0 = value.clamp(Self::MIN.into(), Self::MAX.into());
    }

    pub fn as_u8(&self) -> u8 {
        self.0
    }
}

/// A RGB LED
/// ```
/// use cross::led::*;
///
/// let mut led = Led::default();
///
/// led.r = LedIntensity::MAX;
/// led.g.set(1);
/// assert_eq!(led.r, LedIntensity::MAX);
/// assert_eq!(led.g.as_u8(), 1);
/// assert_eq!(led.b.as_u8(), 0);
/// assert_eq!(led.b, LedIntensity::MIN);
/// ```
#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Led {
    pub r: LedIntensity,
    pub g: LedIntensity,
    pub b: LedIntensity,
}

impl Led {
    /// Given a position in the PWM cycle, return ON|OFF levels for R,G,B.
    /// e.g. If LedIntensity::MAX==20 and
    pub fn to_levels(&self, iteration: u8) -> [Level; 3] {
        [
            self.r.to_level(iteration),
            self.g.to_level(iteration),
            self.b.to_level(iteration),
        ]
    }

    pub fn set(&mut self, colour: &LedColour, value: u8) {
        let ptr = match colour {
            LedColour::RED => &mut self.r,
            LedColour::GREEN => &mut self.g,
            LedColour::BLUE => &mut self.b,
        };

        ptr.set(value);
    }
}
