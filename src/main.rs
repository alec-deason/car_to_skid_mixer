#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_semihosting as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [SIO_IRQ_PROC1])]
mod app {
    use embedded_hal::digital::v2::OutputPin;
    use embedded_hal::PwmPin;
    use fugit::MicrosDurationU32;
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, timer::Alarm, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    const STEERING_UPDATE_TIME: MicrosDurationU32 = MicrosDurationU32::millis(10);
    const PULSE_DURATION: MicrosDurationU32 = MicrosDurationU32::millis(20);

    type InputThrottlePin = hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionPwm>;
    type InputSteeringPin = hal::gpio::Pin<hal::gpio::bank0::Gpio3, hal::gpio::FunctionPwm>;

    #[shared]
    struct Shared {
        alarm0: hal::timer::Alarm0,
        steering: i32,
        throttle: i32,
    }

    #[local]
    struct Local {
        led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,

        pwm_left: hal::pwm::Slice<hal::pwm::Pwm2, hal::pwm::FreeRunning>,
        pwm_right: hal::pwm::Slice<hal::pwm::Pwm4, hal::pwm::FreeRunning>,
        pwm_throttle: hal::pwm::Slice<hal::pwm::Pwm0, hal::pwm::InputHighRunning>,
        pwm_steering: hal::pwm::Slice<hal::pwm::Pwm1, hal::pwm::InputHighRunning>,
        throttle_input: InputThrottlePin,
        steering_input: InputSteeringPin,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);

        let _clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut led = pins.led.into_push_pull_output();
        // These two pins provide ground to the two servo connectors
        let _ = pins.gpio10.into_push_pull_output().set_low();
        let _ = pins.gpio22.into_push_pull_output().set_low();

        let pwm_slices = hal::pwm::Slices::new(c.device.PWM, &mut resets);
        let mut pwm_right = pwm_slices.pwm4;
        let channel = &mut pwm_right.channel_b;
        channel.output_to(pins.gpio9);
        pwm_right.set_div_int(62);
        pwm_right.set_ph_correct();
        pwm_right.enable();
        let mut pwm_left = pwm_slices.pwm2;
        let channel = &mut pwm_left.channel_b;
        channel.output_to(pins.gpio21);
        pwm_left.set_div_int(62);
        pwm_left.set_ph_correct();
        pwm_left.enable();

        let mut pwm_throttle: hal::pwm::Slice<_, hal::pwm::InputHighRunning> =
            pwm_slices.pwm0.into_mode();
        pwm_throttle.set_div_int(125);
        pwm_throttle.enable();
        let channel = &mut pwm_throttle.channel_b;
        let throttle_input = channel.input_from(pins.gpio1);
        channel.enable();
        throttle_input.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);

        let mut pwm_steering: hal::pwm::Slice<_, hal::pwm::InputHighRunning> =
            pwm_slices.pwm1.into_mode();
        pwm_steering.set_div_int(125);
        pwm_steering.enable();
        let channel = &mut pwm_steering.channel_b;
        let steering_input = channel.input_from(pins.gpio3);
        channel.enable();
        steering_input.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        let mut alarm0 = timer.alarm_0().unwrap();
        let _ = alarm0.schedule(STEERING_UPDATE_TIME);
        alarm0.enable_interrupt();

        led.set_low().unwrap();
        (
            Shared {
                throttle: 0,
                steering: 0,
                alarm0,
            },
            Local {
                led,
                pwm_left,
                pwm_right,
                pwm_throttle,
                pwm_steering,
                throttle_input,
                steering_input,
            },
        )
    }

    #[task(priority=4, binds=TIMER_IRQ_0, local=[led, pwm_left, pwm_right], shared=[alarm0, throttle, steering])]
    fn start_pulse(cx: start_pulse::Context) {
        (cx.shared.alarm0, cx.shared.throttle, cx.shared.steering).lock(
            |alarm, throttle, steering| {
                let angle = (*steering).min(2000).max(1000);
                let throttle = (*throttle).min(2000).max(1000);
                let right_throttle;
                let left_throttle;

                let mut true_throttle = (1500 - throttle) as f32 / 500.0;

                // Scale down the throttle signal to avoid breaking the speed of light (and flipping over)
                true_throttle /= 5.0;

                let mut angle = (1500 - angle) as f32 / 500.0;
                if angle > -0.99 || angle < 0.99 {
                    let _ = cx.local.led.set_high();
                } else {
                    let _ = cx.local.led.set_low();
                }

                if true_throttle < 0.0 {
                    angle *= -1.0;
                }

                let steering_tightness = 1.0;
                if angle < 0.0 {
                    right_throttle = true_throttle;
                    left_throttle = (true_throttle * (1.0 + angle * steering_tightness));
                } else {
                    left_throttle = true_throttle;
                    right_throttle = (true_throttle * (1.0 - angle * steering_tightness));
                }
                let left_throttle = left_throttle * 500.0 + 1500.0;
                let right_throttle = right_throttle * 500.0 + 1500.0;

                cx.local.pwm_left.channel_b.set_duty(left_throttle as u16);
                cx.local.pwm_right.channel_b.set_duty(right_throttle as u16);

                alarm.clear_interrupt();
                let _ = alarm.schedule(PULSE_DURATION);
            },
        );
    }

    #[task(priority=3, binds=IO_IRQ_BANK0, local=[throttle_input, steering_input, pwm_throttle, pwm_steering], shared=[throttle, steering])]
    fn read_input(mut cx: read_input::Context) {
        if cx
            .local
            .throttle_input
            .interrupt_status(hal::gpio::Interrupt::EdgeLow)
        {
            let pulse_width_us = cx.local.pwm_throttle.get_counter();
            cx.local.pwm_throttle.set_counter(0);
            cx.local
                .throttle_input
                .clear_interrupt(hal::gpio::Interrupt::EdgeLow);
            cx.shared.throttle.lock(|t| *t = pulse_width_us as i32);
        } else if cx
            .local
            .steering_input
            .interrupt_status(hal::gpio::Interrupt::EdgeLow)
        {
            let pulse_width_us = cx.local.pwm_steering.get_counter();
            cx.local.pwm_steering.set_counter(0);
            cx.local
                .steering_input
                .clear_interrupt(hal::gpio::Interrupt::EdgeLow);
            cx.shared.steering.lock(|t| *t = pulse_width_us as i32);
        }
    }
}
