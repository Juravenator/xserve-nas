#![no_std]
#![no_main]
#![feature(type_alias_impl_trait, generic_const_exprs, iter_array_chunks)]

use core::arch::asm;
use core::pin::pin;
use core::sync::atomic::AtomicBool;

use defmt::*;
use embassy_executor::{Executor, Spawner, InterruptExecutor};
use embassy_stm32::rcc::{Hse, Sysclk, PllSource, Pll, PllPreDiv, AHBPrescaler, APBPrescaler};
use embassy_stm32::time::Hertz;
use embassy_stm32::{interrupt, Config as STM32Config};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::{PB9, PB8};
use embassy_stm32::{gpio::{Level, Input, Output, Speed, Pull}, peripherals::*};
use embassy_time::{Timer, Ticker, Duration, Instant};
use futures::Future;
use futures::{poll};
use led::LedIntensity;
use led_driver::LedDriver;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

mod shift_register;
mod led;
mod led_driver;

static EXECUTOR_BLINK: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LS: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn USART2() {
    EXECUTOR_BLINK.on_interrupt()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = {
        let mut config: STM32Config = Default::default();

        config.enable_debug_during_sleep = false;

        config.rcc.hsi = false;
        config.rcc.hse = Some(Hse{
            freq: Hertz::mhz(25),
            mode: embassy_stm32::rcc::HseMode::Oscillator,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll{
            prediv: PllPreDiv::DIV25,
            mul: embassy_stm32::rcc::PllMul::MUL336,
            divp: Some(embassy_stm32::rcc::PllPDiv::DIV4),
            divq: Some(embassy_stm32::rcc::PllQDiv::DIV7),
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;

        embassy_stm32::init(config)
    };

    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    // Right-side daughterboard LED control
    let sck_r = Output::new(p.PA0, Level::Low, Speed::Low);
    let sin_r = Output::new(p.PA1, Level::Low, Speed::Low);
    let lat_r = Output::new(p.PA2, Level::Low, Speed::Low);
    let r_nen = Output::new(p.PA3, Level::Low, Speed::Low);
    // Right-side daughterboard sensors
    // let sens_lock = Input::new(p.PA10, Pull::Down);

    // Left-side daughterboard LED control
    let sck_l = Output::new(p.PA4, Level::Low, Speed::Low);
    let sin_l = Output::new(p.PA5, Level::Low, Speed::Low);
    let lat_l = Output::new(p.PA6, Level::Low, Speed::Low);
    let l_nen1 = Output::new(p.PA7, Level::Low, Speed::Low);
    let l_nen2 = Output::new(p.PB0, Level::Low, Speed::Low);
    // Left-side daughterboard sensors
    let btn_warn = Input::new(p.PB2, Pull::Down);
    let btn_volume = Input::new(p.PB10, Pull::Down);

    // Center daughterboard LED control
    let mut sck_ls = Output::new(p.PB15, Level::Low, Speed::Low);
    let mut sin_ls = Output::new(p.PA8, Level::Low, Speed::Low);
    let mut lat_ls = Output::new(p.PA9, Level::Low, Speed::Low);
    let ls1_nen = Output::new(p.PB9, Level::Low, Speed::Low);
    let ls2_nen = Output::new(p.PB8, Level::Low, Speed::Low);
    let ls3_nen = Output::new(p.PB7, Level::Low, Speed::Low);
    let ls4_nen = Output::new(p.PB6, Level::Low, Speed::Low);
    let ls5_nen = Output::new(p.PB1, Level::Low, Speed::Low);

    // SATA status LED control
    let sck_rg = Output::new(p.PB5, Level::Low, Speed::Low);
    let sin_rg = Output::new(p.PB4, Level::Low, Speed::Low);
    let lat_rg = Output::new(p.PB3, Level::Low, Speed::Low);

    // SATA sensors
    let sck_sens = Output::new(p.PA15, Level::Low, Speed::Low);
    let sin_sens = Output::new(p.PA12, Level::Low, Speed::Low);
    let nload_sens = Output::new(p.PA11, Level::Low, Speed::Low);

    // interrupt::USART2.set_priority(Priority::P6);
    // let spawner = EXECUTOR_BLINK.start(interrupt::USART2);
    // spawner.spawn(run_blink(led)).unwrap();

    // let executor = EXECUTOR_LS.init(Executor::new());
    // executor.run(|s| s.spawn(run_ls2(sck_ls, sin_ls, lat_ls, ls1_nen, ls2_nen)).unwrap());

    send_bits(&mut sck_ls, &mut sin_ls, [Level::Low; 32]).await;
    trigger_lat(&mut lat_ls).await;
    // Timer::after_secs(1).await;

    // let mut driver = LedDriver::<_, _, _, 2, 2, 5>::new(sck_ls, sin_ls, lat_ls, [ls1_nen.degrade(), ls2_nen.degrade()]);

    // let leds = driver.get_bank_leds(0);
    // for led in leds.into_iter() {
    //     led.r = 200.into();
    // }
    // leds[1].r = LedIntensity::MIN;

    // let leds = driver.get_bank_leds(1);
    // for led in leds.into_iter() {
    //     led.g = 200.into();
    // }
    // leds[0].g = LedIntensity::MIN;

    // loop {
    //     driver.display_cycle_test().await;
    // }
    let mut a: i32 = 0;
    let mut b: i32 = 0;
    led.set_high();
    Timer::after_millis(1000).await;
    led.set_low();
    Timer::after_millis(1000).await;
    loop {
        led.toggle();
        unsafe {
            // wait for 4000*1000=4_000_000 instructions
            // when no other code is running, the LED toggles every ~.5s?
            asm!(
                "MOV  {0}, #0", // Initialize the counter
                "2:",
                "ADD  {0}, {0}, #1", // Increment it

                "MOV  {1}, #0", // Initialize the counter
                "3:",
                "ADD  {1}, {1}, #1", // Increment it
                "CMP  {1}, #4000", // Check the limit
                "BLE  3b", // Continue looping if not finished
                ";",
                "CMP  {0}, #1000", // Check the limit
                "BLE  2b", // Continue looping if not finished
                ";",
                inout(reg) a, inout(reg) b,
                options(pure, nomem, nostack),
            );
        }
    }

    // let executor = EXECUTOR_LS.init(Executor::new());
    // executor.run(|s| s.spawn(run_ls(sck_ls, sin_ls, lat_ls)).unwrap());
    // let executor = EXECUTOR_BLINK.init(Executor::new());
    // executor.run(|s| s.spawn(run_blink(led)).unwrap());
}

#[embassy_executor::task]
async fn run_ls2(
    mut sck: Output<'static, PB15>,
    mut sin: Output<'static, PA8>,
    mut lat: Output<'static, PA9>,
    ls1_nen: Output<'static, PB9>,
    ls2_nen: Output<'static, PB8>,
) {
    send_bits(&mut sck, &mut sin, [Level::Low; 32]).await;
    trigger_lat(&mut lat).await;
    Timer::after_secs(1).await;

    let mut driver = LedDriver::<_, _, _, 2, 2, 5>::new(sck, sin, lat, [ls1_nen.degrade(), ls2_nen.degrade()]);

    let leds = driver.get_bank_leds(0);
    for led in leds.into_iter() {
        led.r = 200.into();
    }
    leds[1].r = LedIntensity::MIN;

    let leds = driver.get_bank_leds(1);
    for led in leds.into_iter() {
        led.g = 200.into();
    }
    leds[0].g = LedIntensity::MIN;

    loop {
        driver.display_cycle_test().await;
        // Timer::after_secs(1).await;
    }
}

// // nice LED pulses
// #[embassy_executor::task]
// async fn run_ls(
//     mut sck: Output<'static, PB15>,
//     mut sin: Output<'static, PA8>,
//     mut lat: Output<'static, PA9>
// ) {
//     let mut m = 0;
//     loop {
//         send_bits(&mut sck, &mut sin, [Level::High; 32]).await;
//         trigger_lat(&mut lat).await;
//         Timer::after_micros(1).await;
//         send_bits(&mut sck, &mut sin, [Level::Low; 32]).await;
//         trigger_lat(&mut lat).await;
//         Timer::after_micros(m).await;

//         m = match m {
//             n if n >= 20_000 => 1,
//             _ => m + 100,
//         }
//     }
// }

// // ticker example
// #[embassy_executor::task]
// async fn run_ls(
//     mut sck: Output<'static, PB15>,
//     mut sin: Output<'static, PA8>,
//     mut lat: Output<'static, PA9>
// ) {
//     let mut ticker = Ticker::every(Duration::from_millis(500));
//     loop {
//         for m in [0, 1, 10, 100, 1000, 10_000] {
//             // let t = pin!(ticker.next());
//             loop {
//                 send_bits(&mut sck, &mut sin, [Level::High; 32]).await;
//                 trigger_lat(&mut lat).await;
//                 Timer::after_micros(1).await;
//                 send_bits(&mut sck, &mut sin, [Level::Low; 32]).await;
//                 trigger_lat(&mut lat).await;
//                 Timer::after_micros(m).await;

//                 if let core::task::Poll::Ready(_) = poll!(ticker.next()) {
//                     break;
//                 }
//                 // if TICK.load(core::sync::atomic::Ordering::Relaxed) {
//                 //     TICK.store(false, core::sync::atomic::Ordering::Relaxed);
//                 //     break;
//                 // }
//             }
//         }
//     }
// }

#[embassy_executor::task]
async fn run_ls(
    mut sck: Output<'static, PB15>,
    mut sin: Output<'static, PA8>,
    mut lat: Output<'static, PA9>
) {
    // init with all LEDs off
    send_bits(&mut sck, &mut sin, [Level::Low; 32]).await;
    trigger_lat(&mut lat).await;

    loop {
        for ledlevels in [
            [Level::Low, Level::Low, Level::High], // red
            [Level::Low, Level::High, Level::Low], // green
            [Level::High, Level::Low, Level::Low], // blue
        ] {
            // for each (chained) chip
            for _ in 0..2 {
                // 16th output is not used, fill with 0
                send_bits(&mut sck, &mut sin, [Level::Low; 1]).await;
                // for each LED
                for _ in 0..5 {
                    send_bits(&mut sck, &mut sin, ledlevels).await;
                }
            }
            trigger_lat(&mut lat).await;
            Timer::after_millis(1000).await;
        }
    }
}


async fn send_bits<const N: usize>(
    sck: &mut Output<'static, PB15>,
    sin: &mut Output<'static, PA8>,
    levels: [Level; N]
) {
    for l in levels {
        sin.set_level(l);
        sck.set_high();
        // Timer::after_millis(10).await;
        sck.set_low();
        // Timer::after_millis(10).await;
    }
    sin.set_low();
}

async fn trigger_lat(lat: &mut Output<'static, PA9>) {
    lat.set_high();
    // Timer::after_millis(10).await;
    lat.set_low();
    // Timer::after_millis(10).await;
}

#[embassy_executor::task]
async fn run_blink(mut led: Output<'static, PC13>) {
    // let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        info!("high");
        led.set_high();
        // ticker.next().await;
        Timer::after_millis(1000).await;

        info!("low");
        led.set_low();
        // ticker.next().await;
        Timer::after_millis(1000).await;
    }
}