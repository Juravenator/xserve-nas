use embassy_stm32::gpio::Level;

pub enum LedColour {
  RED,
  GREEN,
  BLUE,
}

#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct LedIntensity (u8);

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
    pub const MIN: LedIntensity = LedIntensity (0 );
    pub const MAX: LedIntensity = LedIntensity (20 );

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

    pub fn to_level(&self, iteration: u8) -> Level {
      (self.0 > iteration).into()
    }

    pub fn set(&mut self, value: u8) {
      self.0 = value.clamp(Self::MIN.into(), Self::MAX.into());
    }
}

#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Led {
  pub r: LedIntensity,
  pub g: LedIntensity,
  pub b: LedIntensity,
}

impl Led {

  pub fn to_levels(&self, iteration: u8) -> [Level; 3] {
    [self.r.to_level(iteration), self.g.to_level(iteration), self.b.to_level(iteration)]
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