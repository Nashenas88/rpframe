//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{pio::PIOExt, rosc::RingOscillator, Timer},
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::PwmPin;
use panic_probe as _;
use rand_core::RngCore as _;
use smart_leds::{brightness, colors::FUCHSIA, SmartLedsWrite as _};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use adafruit_qt_py_rp2040 as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use ws2812_pio::Ws2812;

const NUM_LEDS: usize = 10;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm1 = &mut pwm_slices.pwm1;
    pwm1.set_ph_correct();
    pwm1.enable();
    let pwm2 = &mut pwm_slices.pwm2;
    pwm2.set_ph_correct();
    pwm2.enable();
    let pwm3 = &mut pwm_slices.pwm3;
    pwm3.set_ph_correct();
    pwm3.enable();
    let pwm5 = &mut pwm_slices.pwm5;
    pwm5.set_ph_correct();
    pwm5.enable();
    let pwm6 = &mut pwm_slices.pwm6;
    pwm6.set_ph_correct();
    pwm6.enable();

    let channel1b = &mut pwm1.channel_b;
    channel1b.output_to(pins.mosi);

    let channel2a = &mut pwm2.channel_a;
    channel2a.output_to(pins.miso);

    let channel2b = &mut pwm2.channel_b;
    channel2b.output_to(pins.rx);

    let channel3a = &mut pwm3.channel_a;
    channel3a.output_to(pins.sclk);

    let channel5a = &mut pwm5.channel_a;
    channel5a.output_to(pins.a3);

    let channel5b = &mut pwm5.channel_b;
    channel5b.output_to(pins.a2);

    let channel6a = &mut pwm6.channel_a;
    channel6a.output_to(pins.a1);

    let channel6b = &mut pwm6.channel_b;
    channel6b.output_to(pins.a0);

    // Ordering maps to pinout at top of carrier board.
    let mut stars = [
        channel3a as &mut dyn PwmPin<Duty = u16>,
        channel1b as &mut dyn PwmPin<Duty = u16>,
        channel2b as &mut dyn PwmPin<Duty = u16>,
        channel2a as &mut dyn PwmPin<Duty = u16>,
        channel5a as &mut dyn PwmPin<Duty = u16>,
        channel6a as &mut dyn PwmPin<Duty = u16>,
        channel5b as &mut dyn PwmPin<Duty = u16>,
        channel6b as &mut dyn PwmPin<Duty = u16>,
    ];

    let led = pins.tx.into_function();
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::<_, _, _, _, smart_leds::RGB8>::new(
        led,
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    ws.write(brightness((0..NUM_LEDS).map(|_| FUCHSIA), 16))
        .unwrap();

    let mut rosc = RingOscillator::new(pac.ROSC).initialize();
    let mut rand_bytes = [0, 0, 0];
    let neg_bytes = [32, -32];

    const START: i32 = u16::MAX as i32 / 4;
    // TODO custom twinkle for each star
    loop {
        let neg = rosc.get_random_bit();
        rosc.fill_bytes(&mut rand_bytes);
        for channel in &mut stars {
            channel.set_duty((START + (rand_bytes[0] as i32) * neg_bytes[neg as usize]) as u16);
        }
        delay.delay_ms(rand_bytes[1] as u32);
    }
}

// End of file
