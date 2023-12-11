use embassy_stm32::gpio::Level;

#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct LedIntensity {
  intensity: u8
}

impl From<u8> for LedIntensity {
    fn from(v: u8) -> Self {
        LedIntensity { intensity: v }
    }
}

impl Into<u8> for LedIntensity {
    fn into(self) -> u8 {
        self.intensity
    }
}

impl LedIntensity {
    pub const MIN: LedIntensity = LedIntensity { intensity: u8::MIN };
    pub const MAX: LedIntensity = LedIntensity { intensity: u8::MAX };

    pub fn to_level(&self, iteration: u8) -> Level {
      (self.intensity > iteration).into()
    }
}

#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Led {
  pub r: LedIntensity,
  pub g: LedIntensity,
  pub b: LedIntensity,
}

impl Led {
  pub fn new() -> Led {
    Led::default()
  }

  pub fn to_levels(&self, iteration: u8) -> [Level; 3] {
    [self.r.to_level(iteration), self.g.to_level(iteration), self.b.to_level(iteration)]
  }
}