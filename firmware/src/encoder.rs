use core::cell::RefCell;

use critical_section::Mutex;
use esp_hal::{
    gpio::{AnyPin, interconnect::PeripheralInput},
    handler,
    interrupt::Priority,
    pcnt::{
        Pcnt,
        channel::{CtrlMode, EdgeMode},
        unit::{Counter, Unit},
    },
    peripherals::PCNT,
};
use static_cell::StaticCell;

const THRESHOLD: i16 = i16::MAX / 2;

static ENC0: StaticCell<QuadratureEncoder<0>> = StaticCell::new();
static ENC1: StaticCell<QuadratureEncoder<1>> = StaticCell::new();

pub static ENCODERS: Mutex<RefCell<[Option<&'static mut dyn Encoder>; 2]>> =
    Mutex::new(RefCell::new([None, None]));

pub trait Encoder: Send {
    fn position(&self) -> i32;
    fn handle_interrupt(&mut self);
}

pub struct QuadratureEncoder<'a, const U: usize> {
    unit: Unit<'a, U>,
    counter: Counter<'a, U>,
    offset: i32,
}

impl<'a, const NUM: usize> QuadratureEncoder<'a, NUM> {
    pub fn new(
        unit: Unit<'a, NUM>,
        pin_a: impl PeripheralInput<'a>,
        pin_b: impl PeripheralInput<'a>,
    ) -> Self {
        unit.set_high_limit(Some(THRESHOLD)).unwrap();
        unit.set_low_limit(Some(-THRESHOLD)).unwrap();
        unit.set_filter(Some(1023)).unwrap();
        unit.clear();

        let channel = &unit.channel0;

        channel.set_ctrl_signal(pin_a);
        channel.set_edge_signal(pin_b);
        channel.set_ctrl_mode(CtrlMode::Reverse, CtrlMode::Keep);
        channel.set_input_mode(EdgeMode::Increment, EdgeMode::Decrement);

        unit.listen();
        unit.resume();

        let counter = unit.counter.clone();

        QuadratureEncoder {
            unit,
            counter,
            offset: 0,
        }
    }
}

impl<'a, const U: usize> Encoder for QuadratureEncoder<'a, U> {
    fn position(&self) -> i32 {
        self.counter.get() as i32 + self.offset
    }

    fn handle_interrupt(&mut self) {
        if self.unit.interrupt_is_set() {
            let events = self.unit.events();

            if events.high_limit {
                self.offset += THRESHOLD as i32;
            } else if events.low_limit {
                self.offset -= THRESHOLD as i32;
            }

            self.unit.reset_interrupt();
        }
    }
}

pub fn init_encoders<'a>(
    pcnt_periphal: PCNT<'static>,
    pins: [(AnyPin<'static>, AnyPin<'static>); 2],
) {
    let mut pcnt = Pcnt::new(pcnt_periphal);

    pcnt.set_interrupt_handler(interrupt_handler);

    let [(pin_1a, pin_1b), (pin_2a, pin_2b)] = pins;

    let enc0 = ENC0.init(QuadratureEncoder::new(pcnt.unit0, pin_1a, pin_1b));
    let enc1 = ENC1.init(QuadratureEncoder::new(pcnt.unit1, pin_2a, pin_2b));

    critical_section::with(|cs| {
        let mut encoders = ENCODERS.borrow_ref_mut(cs);

        encoders[0] = Some(enc0);
        encoders[1] = Some(enc1);
    })
}

#[handler(priority = Priority::Priority2)]
fn interrupt_handler() {
    critical_section::with(|cs| {
        for enc in ENCODERS.borrow_ref_mut(cs).iter_mut().flatten() {
            enc.handle_interrupt();
        }
    });
}
