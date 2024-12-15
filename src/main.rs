
#![allow(clippy::empty_loop)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]


// use cortex_m::peripheral::syst::SystClkSource;
use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{gpio::{Active, IOPinSpeed, OutputSpeed}, pac, prelude::*, serial::{Config, Serial}};
use crate::IOPinSpeed::Mhz50;

type GpiobParts = stm32f1xx_hal::gpio::gpiob::Parts ;
use stm32f1xx_hal::timer::SysDelay;

type Uart1 = Serial<
  pac::USART1, 
  ( stm32f1xx_hal::gpio::Pin<'A', 9, stm32f1xx_hal::gpio::Alternate>, 
    stm32f1xx_hal::gpio::Pin<'A', 10>)>;

use core::fmt::Write;

#[entry]
fn main() -> ! {

  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  let syst = cp.SYST;

  let mut flash = dp.FLASH.constrain();
  let rcc = dp.RCC.constrain();

  let clocks = rcc.cfgr.freeze(&mut flash.acr);
  let mut afio = dp.AFIO.constrain();

  let mut gpioa = dp.GPIOA.split();

  // let mut led = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);

  //let mut delay = hal::timer::Timer::syst(cp.SYST, &clocks).delay();
  // or
  let mut delay: SysDelay = syst.delay(&clocks);

  let mut gpiob: GpiobParts = dp.GPIOB.split();
  let mut dht11_pin = gpiob.pb12.into_pull_up_input(&mut gpiob.crh);
  let mut read_data = Dht11Data {
      humi_int    : 0,
      humi_deci   : 0,
      temp_int    : 0,
      temp_deci   : 0,
      check_sum   : 0
    };

  let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
  let rx = gpioa.pa3;
                                                                                                                                                        
  // USART1
  // let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
  // let rx = gpioa.pa10;
  let serial
   = Serial::new(
    dp.USART2, 
    (tx, rx),
    &mut afio.mapr,
     Config::default().baudrate(115200.bps()), &clocks);

  let (mut tx, _rx) = serial.split();
  let crh: &mut stm32f1xx_hal::gpio::Cr<'B', true> = &mut gpiob.crh;
  let mut read_succ = false;
  loop {
    (read_succ, dht11_pin) = read_data_dht11(&mut read_data, crh, dht11_pin, &mut delay);
    if read_succ {
      // hprintln!("humi {}.{}", read_data.humi_int, read_data.humi_deci);
      writeln!(tx, "humi {}.{}", read_data.humi_int, read_data.humi_deci).unwrap();
      // hprintln!("temp {}.{}", read_data.temp_int, read_data.temp_deci);
      writeln!(tx, "temp {}.{}", read_data.temp_int, read_data.temp_deci).unwrap();
    }
    else {
      writeln!(tx, "read error").unwrap();
    }
    // Use `embedded_hal_02::DelayMs` trait
    // led.set_low();
    // delay.delay_ms(1_000_u16);
    // or use `fugit` duration units
    delay.delay(1.secs());
    // led.set_high();
  }
}

struct Dht11Data 
{
  humi_int    : u8,
  humi_deci   : u8,
  temp_int    : u8,
  temp_deci   : u8,
  check_sum   : u8
}

type GpioCr = stm32f1xx_hal::gpio::Cr<'B', true>;

type Dht11Pin<Mode> = stm32f1xx_hal::gpio::Pin<'B', 12, Mode>;

type OutOpenDrain = stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>;

fn config_dht11_data_out<Mode1 : Active>(
  crh: &mut GpioCr, pin: Dht11Pin<Mode1> 
) -> Dht11Pin<OutOpenDrain>
{
    let mut dht11 = pin.into_open_drain_output(crh);
    dht11.set_speed(crh, Mhz50);
    dht11
}

type InPullUp =  stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>;

fn config_dht11_data_in<Mode1 : Active>(
  crh: &mut GpioCr, pin: Dht11Pin<Mode1> 
) -> Dht11Pin<InPullUp>
{
  pin.into_pull_up_input(crh)
}

fn dht11_read_data(pin: Dht11Pin<InPullUp>, delay : &mut SysDelay, read_data : &mut u8) -> Dht11Pin<InPullUp>
{
  let mut dht11_read_byt: u8 = 0;
  for i in 0..8 {
    while pin.is_low() {
    }
    delay.delay_us(40 as u8);

    if pin.is_high() {
      while pin.is_high() {
      }
      dht11_read_byt |= 0x1 << (7 - i) as u8;
    }
    else {
      dht11_read_byt &= !(0x1 << (7 - i));
    }
  }
  *read_data = dht11_read_byt;
  pin
}

fn read_data_dht11<Mode1 : Active>(
  read_data: &mut Dht11Data, crh: &mut GpioCr, pin: Dht11Pin<Mode1>,  delay : &mut SysDelay
) -> (bool, Dht11Pin<InPullUp>)
{
  let mut out_pin = config_dht11_data_out(crh, pin);
  out_pin.set_low();
  delay.delay_ms(20 as u8);
  out_pin.set_high();
  let mut in_pin = config_dht11_data_in(crh, out_pin);
  delay.delay_us(20 as u8);
  if in_pin.is_high() {
    return (false, in_pin);
  }

  let mut cnt_timer = 0;
  while in_pin.is_low() {
    if cnt_timer > 83 {
      return (false, in_pin);
    }
    cnt_timer += 1;
    delay.delay_us(1_u32);
  }

  cnt_timer = 0;

  while in_pin.is_high() {
    if cnt_timer > 87 {
      return (false, in_pin);
    }
    cnt_timer += 1;
    delay.delay_us(1_u32);
  }

  in_pin = dht11_read_data(in_pin, delay, &mut read_data.humi_int);
  in_pin = dht11_read_data(in_pin, delay, &mut read_data.humi_deci);
  in_pin = dht11_read_data(in_pin, delay, &mut read_data.temp_int);
  in_pin = dht11_read_data(in_pin, delay, &mut read_data.temp_deci);
  in_pin = dht11_read_data(in_pin, delay, &mut read_data.check_sum);

  delay.delay_us(54_u32);
  (true, in_pin)
}
