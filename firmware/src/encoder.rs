use core::{
    cell::RefCell,
    sync::atomic::{AtomicI32, Ordering},
};

use critical_section::Mutex;
use esp_hal::{
    gpio::{AnyPin, Input, InputConfig, Pull, interconnect::InputSignal},
    handler,
    interrupt::Priority,
    pcnt::{
        Pcnt,
        channel::{CtrlMode, EdgeMode},
        unit::{Counter, Unit},
    },
    peripherals::PCNT,
};
use uom::si::{angle::revolution, f32::Angle};

const THRESHOLD: i16 = i16::MAX / 2;

pub struct QuadratureEncoder<const U: usize> {
    counter: Counter<'static, U>,
    offset: &'static AtomicI32,
}

// It is assumed that the encoders are configured to maximum precision, which is 2048PPR
// Using dual channel quadrature encoding means we multiply this by 4 to get total ticks
const QUADRATURE_RESOLUTION: u32 = 2048;
const TICKS_PER_REVOLUTION: u32 = QUADRATURE_RESOLUTION * 4;

impl<const NUM: usize> QuadratureEncoder<NUM> {
    pub fn ticks(&self) -> i32 {
        self.counter.get() as i32 + self.offset.load(Ordering::SeqCst)
    }

    pub fn position(&self) -> Angle {
        Angle::new::<revolution>(self.ticks() as f32 / TICKS_PER_REVOLUTION as f32)
    }
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

static UNIT0: Mutex<RefCell<Option<Unit<'static, 0>>>> = Mutex::new(RefCell::new(None));
static UNIT1: Mutex<RefCell<Option<Unit<'static, 1>>>> = Mutex::new(RefCell::new(None));

static VALUES: [AtomicI32; 2] = [AtomicI32::new(0), AtomicI32::new(0)];

pub fn init_encoders(
    pcnt_periphal: PCNT<'static>,
    pins: [(AnyPin<'static>, AnyPin<'static>); 2],
) -> (QuadratureEncoder<0>, QuadratureEncoder<1>) {
    let mut pcnt = Pcnt::new(pcnt_periphal);

    pcnt.set_interrupt_handler(interrupt_handler);

    let config = InputConfig::default().with_pull(Pull::Up);

    let [(pin_1a, pin_1b), (pin_2a, pin_2b)] =
        pins.map(|(a, b)| (Input::new(a, config).into(), Input::new(b, config).into()));

    configure_quadrature(&pcnt.unit0, pin_1a, pin_1b);

    let enc0 = QuadratureEncoder {
        counter: pcnt.unit0.counter.clone(),
        offset: &VALUES[0],
    };

    critical_section::with(|cs| UNIT0.borrow_ref_mut(cs).replace(pcnt.unit0));

    configure_quadrature(&pcnt.unit1, pin_2a, pin_2b);

    let enc1 = QuadratureEncoder {
        counter: pcnt.unit1.counter.clone(),
        offset: &VALUES[1],
    };

    critical_section::with(|cs| UNIT1.borrow_ref_mut(cs).replace(pcnt.unit1));

    (enc0, enc1)
}

fn handle_interrupt<const U: usize>(unit: &Unit<'static, U>, offset: &AtomicI32) {
    if unit.interrupt_is_set() {
        let events = unit.events();

        if events.high_limit {
            offset.fetch_add(THRESHOLD as i32, Ordering::SeqCst);
        } else if events.low_limit {
            offset.fetch_add(-THRESHOLD as i32, Ordering::SeqCst);
        }

        unit.reset_interrupt();
    }
}

#[handler(priority = Priority::Priority2)]
fn interrupt_handler() {
    critical_section::with(|cs| {
        if let Some(u) = UNIT0.borrow_ref_mut(cs).as_mut() {
            handle_interrupt(u, &VALUES[0]);
        }

        if let Some(u) = UNIT1.borrow_ref_mut(cs).as_mut() {
            handle_interrupt(u, &VALUES[1]);
        }
    });
}
