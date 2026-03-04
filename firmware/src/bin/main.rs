#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::{
    cell::RefCell,
    sync::atomic::{AtomicI32, Ordering},
};

use critical_section::Mutex;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Pull},
    handler,
    interrupt::Priority,
    main,
    pcnt::{Pcnt, channel, unit},
    time::{Duration, Instant},
};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static UNIT0: Mutex<RefCell<Option<unit::Unit<0>>>> = Mutex::new(RefCell::new(None));
static VALUE: AtomicI32 = AtomicI32::new(0);

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.2.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let _peripherals = esp_hal::init(config);

    let mut pcnt = Pcnt::new(_peripherals.PCNT);

    pcnt.set_interrupt_handler(interrupt_handler);

    let u0 = pcnt.unit0;

    u0.set_high_limit(Some(100)).unwrap();
    u0.set_low_limit(Some(-100)).unwrap();
    u0.set_filter(Some(1023)).unwrap();
    u0.clear();

    let ch0 = &u0.channel0;
    let config = InputConfig::default().with_pull(Pull::Up);
    let pin_a = Input::new(_peripherals.GPIO13, config);
    let pin_b = Input::new(_peripherals.GPIO14, config);

    ch0.set_ctrl_signal(pin_a);
    ch0.set_edge_signal(pin_b);
    ch0.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
    ch0.set_input_mode(channel::EdgeMode::Increment, channel::EdgeMode::Decrement);

    u0.listen();
    u0.resume();
    let counter = u0.counter.clone();

    critical_section::with(|cs| UNIT0.borrow_ref_mut(cs).replace(u0));

    let mut last_value = 0;

    loop {
        let value = counter.get() as i32 + VALUE.load(Ordering::SeqCst);

        if value != last_value {
            last_value = value;
        }

        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(10) {}
    }
}

#[handler(priority = Priority::Priority2)]
fn interrupt_handler() {
    critical_section::with(|cs| {
        let mut u0 = UNIT0.borrow_ref_mut(cs);

        if let Some(u0) = u0.as_mut() {
            if u0.interrupt_is_set() {
                let events = u0.events();

                if events.high_limit {
                    VALUE.fetch_add(100, Ordering::SeqCst);
                } else if events.low_limit {
                    VALUE.fetch_add(-100, Ordering::SeqCst);
                }

                u0.reset_interrupt();
            }
        }
    })
}
