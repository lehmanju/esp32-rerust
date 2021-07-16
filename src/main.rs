#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(raw_vec_internals)]

use core::fmt::Write;
use esp32_hal::prelude::*;

extern crate alloc;
use alloc::collections::vec_deque::VecDeque;
use alloc::rc::Rc;
use core::cell::RefCell;
use core::panic::PanicInfo;

use esp32_hal::clock_control::{sleep, ClockControl};
use esp32_hal::dport::Split;
use esp32_hal::dprintln;
use esp32_hal::serial::config::Config;
use esp32_hal::serial::Serial;
use esp32_hal::target;

#[cfg(feature = "external_ram")]
use esp32_hal::alloc::EXTERNAL_ALLOCATOR;
use esp32_hal::alloc::{Allocator, DRAM_ALLOCATOR};

#[global_allocator]
pub static GLOBAL_ALLOCATOR: Allocator = DRAM_ALLOCATOR;

mod generated {
    use rerust::rerust;

    rerust! {
        let x = Var::<u32>(1u32);
        let y = x.map(|x: &u32| -> u32 {x * 2});
        let z = x.map(|x: &u32| -> u32 {x * 3});
        let pin t = (y,z).map(|y: &u32, z: &u32| -> u32 {y + z});
    }
}

static SERIAL: CriticalSectionSpinLockMutex<
    Option<
        esp32_hal::serial::Serial<
            esp32::UART0,
            esp32_hal::gpio::Gpio1<esp32_hal::gpio::Unknown>,
            esp32_hal::gpio::Gpio3<esp32_hal::gpio::Unknown>,
        >,
    >,
> = CriticalSectionSpinLockMutex::new(None);

#[entry]
fn main() -> ! {
    let dp = target::Peripherals::take().expect("Failed to obtain Peripherals");

    let mut timg0 = dp.TIMG0;
    let mut timg1 = dp.TIMG1;

    let (_, dport_clock_control) = dp.DPORT.split();

    // (https://github.com/espressif/openocd-esp32/blob/97ba3a6bb9eaa898d91df923bbedddfeaaaf28c9/src/target/esp32.c#L431)
    // openocd disables the watchdog timer on halt
    // we will do it manually on startup
    disable_timg_wdts(&mut timg0, &mut timg1);

    let clkcntrl = ClockControl::new(
        dp.RTCCNTL,
        dp.APB_CTRL,
        dport_clock_control,
        esp32_hal::clock_control::XTAL_FREQUENCY_AUTO,
    )
    .unwrap();

    let (clkcntrl_config, mut watchdog) = clkcntrl.freeze().unwrap();
    watchdog.disable();

    let gpios = dp.GPIO.split();
    // setup serial controller
    let mut serial: Serial<_, _, _> = Serial::new(
        dp.UART0,
        esp32_hal::serial::Pins {
            tx: gpios.gpio1,
            rx: gpios.gpio3,
            cts: None,
            rts: None,
        },
        Config::default().baudrate(115_200.Hz()),
        clkcntrl_config,
    )
    .unwrap();

    writeln!(serial, "\n\nESP32 started\n\n").unwrap();
    (&SERIAL).lock(|val| *val = Some(serial));

    let mut prog = generated::Program::new();
    let sink: Rc<RefCell<VecDeque<generated::Input>>> = prog.sink();

    let observer = Rc::new(RefCell::new(observer_cb)) as Rc<_>;
    prog.observe_t(Rc::downgrade(&observer));

    sink.borrow_mut()
        .push_back(generated::Input { var_0: Some(8u32) });
    prog.init();
    loop {
        sink.borrow_mut()
            .push_back(generated::Input { var_0: Some(2u32) });
        sink.borrow_mut()
            .push_back(generated::Input { var_0: Some(5u32) });
        prog.run();
        sleep(1.s());
        prog.run();
        sleep(1.s());
    }
}

fn observer_cb(t: &u32) {
    (&SERIAL).lock(|serial| {
        let serial = serial.as_mut().unwrap();
        writeln!(serial, "\nobserver {:?}\n", t).unwrap()
    })
}

#[alloc_error_handler]
fn alloc_error_handler(layout: core::alloc::Layout) -> ! {
    panic!(
        "Error allocating  {} bytes of memory with alignment {}",
        layout.size(),
        layout.align()
    );
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    dprintln!("\n\n*** {:?}", info);
    loop {}
}

const WDT_WKEY_VALUE: u32 = 0x50D83AA1;

fn disable_timg_wdts(timg0: &mut target::TIMG0, timg1: &mut target::TIMG1) {
    timg0
        .wdtwprotect
        .write(|w| unsafe { w.bits(WDT_WKEY_VALUE) });
    timg1
        .wdtwprotect
        .write(|w| unsafe { w.bits(WDT_WKEY_VALUE) });

    timg0.wdtconfig0.write(|w| unsafe { w.bits(0x0) });
    timg1.wdtconfig0.write(|w| unsafe { w.bits(0x0) });
}
