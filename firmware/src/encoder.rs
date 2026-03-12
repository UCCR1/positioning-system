macro_rules! declare_encoders {
    ($mod_name:ident, [$($unit:literal),+]) => {
        paste::paste! {
            mod $mod_name {
                use critical_section::Mutex;
                use core::{cell::RefCell, sync::atomic::{AtomicI32, Ordering}};
                use esp_hal::{
                    handler,
                    pcnt::{unit::Unit, Pcnt, channel::{CtrlMode, EdgeMode}},
                    peripherals::PCNT,
                    gpio::{AnyPin, interconnect::InputSignal, InputConfig, Pull, Input},
                    interrupt::Priority
                };

                use esp_hal::pcnt::unit::Counter;
                use uom::si::{angle::revolution, f32::Angle};


                // It is assumed that the encoders are configured to maximum precision, which is 2048PPR
                // Using dual channel quadrature encoding means we multiply this by 4 to get total ticks
                const QUADRATURE_RESOLUTION: u32 = 2048;
                const TICKS_PER_REVOLUTION: u32 = QUADRATURE_RESOLUTION * 4;

                pub struct QuadratureEncoder<const U: usize> {
                    counter: Counter<'static, U>,
                    offset: &'static AtomicI32,
                }

                impl<const NUM: usize> QuadratureEncoder<NUM> {
                    pub fn ticks(&self) -> i32 {
                        self.counter.get() as i32 + self.offset.load(Ordering::SeqCst)
                    }

                    pub fn position(&self) -> Angle {
                        Angle::new::<revolution>(self.ticks() as f32 / TICKS_PER_REVOLUTION as f32)
                    }
                }

                $(
                    static [<UNIT $unit>]: Mutex<RefCell<Option<Unit<'static, $unit>>>> = Mutex::new(RefCell::new(None));
                    static [<VALUE $unit>]: AtomicI32 = AtomicI32::new(0);
                )+

                const THRESHOLD: i16 = i16::MAX / 2;

                #[handler(priority = Priority::Priority2)]
                fn interrupt_handler() {
                    critical_section::with(|cs| {
                        $(
                            if let Some(unit) = [<UNIT $unit>].borrow_ref_mut(cs).as_mut() {
                                if unit.interrupt_is_set() {
                                    let events = unit.events();

                                    if events.high_limit {
                                        [<VALUE $unit>].fetch_add(THRESHOLD as i32, Ordering::SeqCst);
                                    } else if events.low_limit {
                                        [<VALUE $unit>].fetch_add(-THRESHOLD as i32, Ordering::SeqCst);
                                    }

                                    unit.reset_interrupt();
                                }
                            }
                        )+
                    });
                }

                fn configure_quadrature<const U: usize>(
                    unit: &Unit<'static, U>,
                    pin_a: InputSignal,
                    pin_b: InputSignal,
                ) {
                    unit.set_high_limit(Some(THRESHOLD)).unwrap();
                    unit.set_low_limit(Some(-THRESHOLD)).unwrap();
                    unit.set_filter(Some(1023)).unwrap();
                    unit.clear();

                    let ch0 = &unit.channel0;
                    ch0.set_ctrl_signal(pin_a.clone());
                    ch0.set_edge_signal(pin_b.clone());
                    ch0.set_ctrl_mode(CtrlMode::Reverse, CtrlMode::Keep);
                    ch0.set_input_mode(EdgeMode::Increment, EdgeMode::Decrement);

                    // Use dual channel counter to double number of pulses
                    let ch1 = &unit.channel1;
                    ch1.set_ctrl_signal(pin_b);
                    ch1.set_edge_signal(pin_a);
                    ch1.set_ctrl_mode(CtrlMode::Reverse, CtrlMode::Keep);
                    ch1.set_input_mode(EdgeMode::Decrement, EdgeMode::Increment);

                    unit.listen();
                    unit.resume();
                }

                pub fn init(
                    pcnt_peripheral: PCNT<'static>,
                    $([<unit_ $unit _pins>]: (AnyPin<'static>, AnyPin<'static>),)+
                ) -> ($(QuadratureEncoder<$unit>,)+) {
                    let mut pcnt = Pcnt::new(pcnt_peripheral);

                    pcnt.set_interrupt_handler(interrupt_handler);

                    let config = InputConfig::default().with_pull(Pull::Up);

                    $(
                        configure_quadrature(
                            &pcnt.[<unit $unit>],
                            Input::new([<unit_ $unit _pins>].0, config).into(),
                            Input::new([<unit_ $unit _pins>].1, config).into()
                        );

                        let [<enc $unit>] = QuadratureEncoder {
                            counter: pcnt.[<unit $unit>].counter.clone(),
                            offset: &[<VALUE $unit>],
                        };

                        critical_section::with(|cs| [<UNIT $unit>].borrow_ref_mut(cs).replace(pcnt.[<unit $unit>]));
                    )+

                    ($([<enc $unit>],)+)
                }
            }
        }
    }
}
