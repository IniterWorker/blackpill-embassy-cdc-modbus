#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::{
    bind_interrupts,
    gpio::{AnyPin, Level, Output, Pin, Speed},
    peripherals,
    rcc::{
        AHBPrescaler, APBPrescaler, Config as RccConfig, Hse, HseMode, LsConfig, LseConfig,
        LseDrive, LseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllSource, RtcClockSource,
        Sysclk,
    },
    time::Hertz,
    usb_otg::{self, Driver, Instance},
    Config,
};
use embassy_sync::blocking_mutex::{self, raw::CriticalSectionRawMutex};
use embassy_time::Timer;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    driver::EndpointError,
    Builder,
};
use heapless::Vec;
use rmodbus::{
    server::context::ModbusContext, server::storage::ModbusStorage, server::ModbusFrame,
    ModbusProto,
};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_FS => usb_otg::InterruptHandler<peripherals::USB_OTG_FS>;
});

const C_MODBUS_MAX: usize = 32;
const D_MODBUS_MAX: usize = 16;
const I_MODBUS_MAX: usize = 0;
const H_MODBUS_MAX: usize = 32;

static MUTEX_CTX_MODBUS: blocking_mutex::Mutex<
    CriticalSectionRawMutex,
    RefCell<ModbusStorage<C_MODBUS_MAX, D_MODBUS_MAX, I_MODBUS_MAX, H_MODBUS_MAX>>,
> = blocking_mutex::Mutex::new(RefCell::new(ModbusStorage::new()));

fn configuration(init: &mut Config) {
    let pll_conf = Pll {
        prediv: PllPreDiv::DIV25,
        mul: PllMul::MUL384,
        divq: Some(PllQDiv::DIV8),
        divp: Some(PllPDiv::DIV4),
        divr: None,
    };

    let hse_conf = Hse {
        freq: Hertz(25_000_000),   // External crystal frequency: 25 MHz
        mode: HseMode::Oscillator, // Using a crystal oscillator
    };

    let lse_conf = LseConfig {
        frequency: Hertz(32_768),                 // Frequency of the LSE crystal
        mode: LseMode::Oscillator(LseDrive::Low), // Using a crystal, not bypass mode
    };

    let ls_config = LsConfig {
        rtc: RtcClockSource::LSE, // Use LSE as the RTC clock source
        lsi: false,               // Disable the internal LSI oscillator
        lse: Some(lse_conf),      // Enable the external LSE crystal
    };

    let mut rcc_config = RccConfig::default();
    rcc_config.hsi = false; // Enable HSI as fallback
    rcc_config.hse = Some(hse_conf); // Enable the external 25 MHz HSE clock
    rcc_config.ahb_pre = AHBPrescaler::DIV1; // Set AHB prescaler (no division)
    rcc_config.apb1_pre = APBPrescaler::DIV2; // Set APB1 prescaler (max 42 MHz)
    rcc_config.apb2_pre = APBPrescaler::DIV1; // Set APB2 prescaler (max 84 MHz)
    rcc_config.ls = ls_config; // Not used in this context
    rcc_config.pll = Some(pll_conf); // Apply the PLL configuration
    rcc_config.sys = Sysclk::PLL1_P;
    rcc_config.pll_src = PllSource::HSE;
    rcc_config.plli2s = None;

    init.rcc = rcc_config;
}

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, Speed::Low);
    let mut u = 0u16;

    loop {
        // Timekeeping is globally available, no need to mess with hardware timers.
        led.set_high();
        Timer::after_millis(150).await;
        led.set_low();
        Timer::after_millis(150).await;

        // Mocking some data
        u = u.wrapping_add(1);

        MUTEX_CTX_MODBUS.lock(|x| {
            let mut ctx = x.borrow_mut();

            for i in 0..(H_MODBUS_MAX as u16) {
                let _ = ctx.set_holding(i, u);
            }
        });
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut init_cfg = Config::default();
    configuration(&mut init_cfg);

    let p = embassy_stm32::init(init_cfg);

    spawner.spawn(blink(p.PC13.degrade())).unwrap();

    let mut config = embassy_stm32::usb_otg::Config::default();
    config.vbus_detection = false;

    let mut ep_out_buffer = [0u8; 256];
    let driver = Driver::new_fs(
        p.USB_OTG_FS,
        Irqs,
        p.PA12,
        p.PA11,
        &mut ep_out_buffer,
        config,
    );

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = modbus_handler(&mut class).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer Overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn modbus_handler<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    const MAX_PACKET_SIZE: usize = 64; // Maximum USB CDC packet size for Full-Speed
    let mut buf = [0u8; 256];
    let mut response: Vec<u8, 256> = Vec::new();

    loop {
        let _n = class.read_packet(&mut buf).await?;

        let mut frame = ModbusFrame::new(1, &buf, ModbusProto::Rtu, &mut response);
        if frame.parse().is_err() {
            error!("Server error");
        }
        if frame.processing_required {
            let result = if frame.readonly {
                MUTEX_CTX_MODBUS.lock(|x| {
                    let val = x.borrow();
                    frame.process_read(&*val)
                })
            } else {
                MUTEX_CTX_MODBUS.lock(|x| {
                    let mut val = x.borrow_mut();
                    frame.process_write(&mut *val)
                })
            };
            if result.is_err() {
                error!("frame processing error");
                continue;
            }
            if frame.response_required {
                frame.finalize_response().unwrap();

                // Split response into multiple 64-byte packets if needed
                let total_len = response.len();
                let mut offset = 0;

                while offset < total_len {
                    let end = (offset + MAX_PACKET_SIZE).min(total_len);
                    let packet = &response[offset..end];
                    class.write_packet(packet).await?;
                    offset = end;
                }
            }
        }
    }
}
