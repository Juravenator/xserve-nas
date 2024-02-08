use embassy_stm32::pac;
use embassy_stm32::pac::dma::regs::Ndtr;
use embassy_stm32::pac::timer::regs::Arr16;
use embassy_stm32::peripherals::TIM1;
use embassy_stm32::rcc::low_level::RccPeripheral;
use embassy_stm32::timer::low_level::{
    Basic16bitInstance, CaptureCompare16bitInstance, GeneralPurpose16bitInstance,
};
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::{CountingMode, OutputCompareMode};

// References:
// - https://embassy.dev/dev/layer_by_layer.html
// - https://www.st.com/resource/en/datasheet/stm32f401cc.pdf (DS9716, block diagram, AF mappings)
// - https://www.st.com/resource/en/reference_manual/rm0368-stm32f401xbc-and-stm32f401xde-advanced-armbased-32bit-mcus-stmicroelectronics.pdf (RM0368, registers)

/// Create our two DMA operations using TIM1 as a source.
pub fn init(tim1: TIM1) -> [DMAStream; 2] {
    init_timer(tim1);
    init_dma(&[4, 5]);

    [DMAStream(4), DMAStream(5)]
}

/// Wrapper for a memory-to-peripheral cyclic DMA operation.
/// ```ignore
/// let dmas = dma::init(p.TIM1);
/// dmas[0].set_source(&buffer);
/// dmas[0].set_destination(pac::GPIOA.odr().as_ptr() as u32);
/// dmas[0].enable();
///
/// // edit buffer and restart DMA from the beginning
/// dmas[0].disable();
/// &buffer[0] = 0xFF;
/// dmas[0].enable();
/// ```
#[derive(Default, Clone, Copy, Debug)]
pub struct DMAStream(usize);

impl DMAStream {
    /// Set source buffer where to DMA read from.
    pub fn set_source<T>(&self, buf: &[T]) {
        pac::DMA2
            .st(self.0)
            .ndtr()
            .write_value(Ndtr(buf.len() as u32)); // stream this many values
        pac::DMA2.st(self.0).m0ar().write_value(buf.as_ptr() as u32); // source address
    }

    /// Set destination pointer where to write source buffer values to.
    pub fn set_destination(&self, ptr: u32) {
        pac::DMA2.st(self.0).par().write_value(ptr); // destination address
    }

    /// Start the buffer from the beginning.
    /// No effect when already enabled, disable first to restart.
    pub fn enable(&self) {
        pac::DMA2.st(self.0).cr().modify(|w| w.set_en(true)); // enable
    }

    /// Stops the current DMA without waiting to finish the current DMA cycle.
    pub fn disable(&self) {
        pac::DMA2.st(self.0).cr().modify(|w| w.set_en(false)); // enable
    }
}

// fn init_af() {
//     // Alternate Function pins are configured in the Embassy HAL by declaring a PwmPin.
//     // It works, but there's more AFs than PWM. Embassy just exposes one AF so hence the naming I guess.
//     //
//     // Embassy HAL has an internal function to set any AF mode on a pin. It could basically consist of
//     // declaring an Input/Output::new() and running set_as_af(). Alas, the function is private.
//     //
//     // The commented HAL code and the uncommented registry writes are equivalent.

//     // let ch1 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
//     pac::RCC.ahb1enr().modify(|w| w.set_gpioaen(true));
//     pac::GPIOA.bsrr().write(|w| w.set_br(8, true));
//     pac::GPIOA.afr(8 / 8).modify(|w| {
//         w.set_afr(8 % 8, 0x01);
//     });
//     pac::GPIOA.otyper().modify(|w| {
//         w.set_ot(8, pac::gpio::vals::Ot::PUSHPULL);
//     });
//     pac::GPIOA.pupdr().modify(|w| {
//         w.set_pupdr(8, Pull::None.into());
//         // w.set_pupdr(8, pac::gpio::vals::Pupdr::FLOATING);
//     });
//     pac::GPIOA.moder().modify(|w| {
//         w.set_moder(8, pac::gpio::vals::Moder::ALTERNATE);
//     });
//     pac::GPIOA.ospeedr().modify(|w| {
//         w.set_ospeedr(8, pac::gpio::vals::Ospeedr::VERYHIGHSPEED);
//     });
// }

/// Setup timer 1.
fn init_timer(mut tim1: TIM1) {
    // The Embassy HAL can do almost all we need.
    //
    // The uncommented HAL code and the commented registry writes under it are equivalent.
    // Although the HAL code should be preferred where possible, the subtle errata for
    // enable_and_reset() should demonstrate why.

    TIM1::enable_and_reset();
    // pac::RCC.apb2enr().modify(|w| w.set_tim1en(true));
    // // // Errata: ES0005 - 2.1.11 Delay after an RCC peripheral clock enabling
    // // // only for stm32f2
    // // cortex_m::asm::dsb();
    // pac::RCC.apb2rstr().modify(|w| w.set_syscfgrst(true));
    // pac::RCC.apb2rstr().modify(|w| w.set_syscfgrst(false));

    tim1.set_counting_mode(CountingMode::EdgeAlignedUp);
    // pac::TIM1.cr1().modify(|w| {
    //     // w.set_cen(false);
    //     let (cms, dir) = CountingMode::EdgeAlignedUp.into();
    //     w.set_dir(dir);
    //     w.set_cms(cms);
    // });

    // tim1.set_frequency(Hertz(2));

    // 21Mhz (84Mhz / (2 clk signal) / 1 / 2)
    // LED driver chips are 25Mhz max, this is the closest we can get.
    pac::TIM1.psc().write(|w| w.set_psc(1 - 1)); // prescaler // max 65535
    pac::TIM1.arr().write_value(Arr16(2 - 1)); // counter period // max 65535

    // // 10.5Mhz (84Mhz / (2 clk signal) / 4 / 4)
    // pac::TIM1.psc().write(|w| w.set_psc(8-1)); // prescaler // max 65535
    // pac::TIM1.arr().write_value(Arr16(8-1)); // counter period // max 65535

    // // 2Hz (84Mhz / (2 clk signal) / 60_000 / 350)
    // pac::TIM1.psc().write(|w| w.set_psc(60_000 - 1)); // prescaler // max 65535
    // pac::TIM1.arr().write_value(Arr16(350-1)); // counter period // max 65535

    pac::TIM1.egr().write(|w| {
        // w.set_ccg(3, true);
        w.set_ug(true);
    });
    pac::TIM1.cr1().modify(|w| {
        w.set_urs(pac::timer::vals::Urs::ANYEVENT);
    });

    tim1.start();
    // pac::TIM1.cr1().modify(|r| r.set_cen(true));

    tim1.enable_outputs();
    // pac::TIM1.bdtr().modify(|w| w.set_moe(true));

    tim1.set_output_compare_mode(Channel::Ch1, OutputCompareMode::Toggle);
    // pac::TIM1.ccmr_output(0).write(|ccmr| {
    //     ccmr.set_ocm(0, OutputCompareMode::Toggle.into());
    //     // ccmr.set_ocm(0, pac::timer::vals::Ocm::TOGGLE);
    // });

    tim1.enable_channel(Channel::Ch1, true);
    // tim1.enable_channel(Channel::Ch4, true);
    // pac::TIM1.ccer().modify(|w| {
    //     w.set_cce(Channel::Ch1.raw(), true);
    // });

    // Extra stuff from Cube HAL code not found in Embassy HAL.
    //
    // The only required registry write here is connecting the timer and DMA controller.
    // It is missing in the Embassy HAL.
    pac::TIM1.dier().modify(|dier| {
        dier.set_ccde(3, true);
        dier.set_ude(true); // enable DMA on TIM1_UP
    });
    // The others are default values, or not used in timing calculations.
    pac::TIM1.cr1().modify(|cr1| {
        cr1.set_ckd(pac::timer::vals::Ckd::DIV1); // internal clock division
        cr1.set_arpe(true); // auto-reload preload
        cr1.set_opm(pac::timer::vals::Opm::DISABLED); // one pulse mode
        cr1.set_udis(false); // update enable
    });
    pac::TIM1.cr2().modify(|cr2| {
        cr2.set_ccpc(false); // TRGO master/slave mode
        cr2.set_mms(pac::timer::vals::Mms::UPDATE); // TRGO trigger event selection
                                                    // cr2.set_ccus(true); // capture/control update
        cr2.set_ccds(pac::timer::vals::Ccds::ONCOMPARE);
    });
    // pac::TIM1.egr().write(|w| {
    //     w.set_ug(true); // update generation
    // });
    pac::TIM1.ccmr_output(0).modify(|ccmr| {
        // ccmr.set_ccs(0, pac::timer::vals::CcmrOutputCcs::OUTPUT); // output compare is output
        ccmr.set_ocpe(0, pac::timer::vals::Ocpe::DISABLED); // output compare preload enable
    });
    // pac::TIM1.ccr(0).modify(|ccr| {
    //     ccr.set_ccr(0); // compare value
    // });
    // pac::TIM1.rcr().write_value(Rcr(0)); // repetition counter
}

fn init_dma(streams: &[usize]) {
    // Embassy HAL is not used here. This code is purely made by setting up the project in
    // CubeIDE and seeing what the generated code does.

    // MX_DMA_Init
    // __HAL_RCC_DMA2_CLK_ENABLE();
    pac::RCC.ahb1enr().modify(|w| {
        // w.set_dma1en(true);
        w.set_dma2en(true);
    });
    // HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
    // pac::Interrupt::DMA2_STREAM5.set_priority(Priority::P0);
    // pac::Interrupt::DMA2_STREAM4.set_priority(Priority::P0);
    for stream in streams {
        pac::DMA2.st(*stream).cr().modify(|w| {
            // w.set_en(false);
            w.set_chsel(6);
            w.set_circ(pac::dma::vals::Circ::ENABLED); // circular mode
                                                       // w.set_circ(pac::dma::vals::Circ::DISABLED); // circular mode
                                                       // w.set_ct(pac::dma::vals::Ct::MEMORY0); // double buffer bank select
            w.set_dbm(pac::dma::vals::Dbm::DISABLED); // double buffer mode
            w.set_dir(pac::dma::vals::Dir::MEMORYTOPERIPHERAL);
            w.set_minc(pac::dma::vals::Inc::INCREMENTED); // memory increment
            w.set_pinc(pac::dma::vals::Inc::FIXED); // peripheral not incrumented
            w.set_msize(pac::dma::vals::Size::BITS16); // memory size = word
            w.set_psize(pac::dma::vals::Size::BITS16);
            w.set_mburst(pac::dma::vals::Burst::SINGLE); // no bursting
            w.set_pburst(pac::dma::vals::Burst::SINGLE);
            w.set_pfctrl(pac::dma::vals::Pfctrl::DMA); // DMA sets flow
            w.set_pl(pac::dma::vals::Pl::HIGH); // priority
            w.set_tcie(true); // done interrupt enable
            w.set_teie(true); // error interrupt enable
        });

        // Done in CubeIDE, breaks everything here.
        // pac::DMA2.ifcr(*stream).write_value(Ixr(0)); // clear all interrupt flags
    }
}
