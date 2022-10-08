#![no_std]
// We always pull in `std` during tests, because it's just easier
// to write tests when you can assume you're on a capable platform
#[cfg(any(feature = "std", test))]
#[macro_use]
extern crate std;

// When we're building for a no-std target, we pull in `core`, but alias
// it as `std` so the `use` statements are the same between `std` and `core`.
#[cfg(all(not(feature = "std"), not(test)))]
#[macro_use]
extern crate core as std;

mod axp192_registers;
use core::convert::{Into, TryInto};
use core::panic;

use crate::axp192_registers::{IoCtl, Registers};
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use log::info;

#[allow(unused)]
enum GpioMode {
    Input,
    OpenDrainOutput,
    ChargeControl,
    Floating,
    ADCInput,
    LowOutput,
    PWMOutput,
}

#[derive(Debug)]
enum VoltageTarget {
    Esp,
    Lcd,
}

#[derive(Debug)]
// Periph is ldo2
// vibrate is ldo3
enum LDOTarget {
    Periph,
    Vibrate,
}

#[derive(Debug)]
enum PowerMode {
    Internal,
    External,
}

#[allow(unused)]
#[derive(Debug)]
enum BatteryVoltageCharge {
    Charge31,
    Charge30,
    Charge30low,
    Charge25,
}

#[derive(Debug)]
enum ChargeCurrent {}

#[allow(unused)]
#[derive(Debug)]
enum GpioPin {
    Gpio0,
    Gpio1,
    Gpio2,
    Gpio3,
    Gpio4,
}

pub enum BatteryState {
    Charging(u8),
    Discharging(u8),
}

pub struct AXP192<I2CMaster> {
    i2c: I2CMaster,
    addr: u8,
}

static BATTERY_LEVEL_DIS: [f32; 101] = [
    4.1767, 4.1415, 4.1074, 4.0953, 4.0821, 4.0689, 4.0601, 4.0491, 4.0436, 4.0370, 4.0315, 4.0238,
    4.0161, 4.0007, 3.9919, 3.9809, 3.9732, 3.9644, 3.9589, 3.9468, 3.9369, 3.9292, 3.9281, 3.9182,
    3.9160, 3.9072, 3.9050, 3.9028, 3.8951, 3.8863, 3.8830, 3.8775, 3.8665, 3.8599, 3.8489, 3.8467,
    3.8401, 3.8346, 3.8302, 3.8236, 3.8203, 3.8137, 3.8071, 3.8027, 3.7950, 3.7895, 3.7840, 3.7818,
    3.7785, 3.7730, 3.7686, 3.7675, 3.7653, 3.7631, 3.7576, 3.7576, 3.7565, 3.7532, 3.7455, 3.7466,
    3.7466, 3.7411, 3.7389, 3.7378, 3.7345, 3.7323, 3.7257, 3.7246, 3.7213, 3.7191, 3.7147, 3.7147,
    3.7136, 3.7114, 3.7081, 3.7059, 3.7026, 3.7026, 3.6971, 3.6971, 3.6949, 3.6927, 3.6894, 3.6839,
    3.6762, 3.6685, 3.6663, 3.6608, 3.6498, 3.6432, 3.6388, 3.6355, 3.6322, 3.6212, 3.6234, 3.6146,
    3.5959, 3.5629, 3.5266, 3.4584, 3.3506,
];

impl<I2CMaster, E> AXP192<I2CMaster>
where
    // I2CMaster: Write + WriteRead + embedded_hal::prelude::_embedded_hal_blocking_i2c_Write>::Error: std::fmt::Debug
    I2CMaster: Write<Error = E> + WriteRead<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new<DELAY>(delay: &mut DELAY, i2c: I2CMaster) -> Self
    where
        DELAY: DelayMs<u16>,
    {
        let mut axp192 = AXP192 { addr: 0x34, i2c };

        axp192.set_vbus_limit(false);
        axp192.init_gpio(GpioPin::Gpio1, GpioMode::OpenDrainOutput);
        axp192.init_gpio(GpioPin::Gpio2, GpioMode::OpenDrainOutput);
        axp192.set_rtc_chg(true);
        axp192.set_dcdc_voltage(VoltageTarget::Esp, 3350);
        axp192.set_dcdc_voltage(VoltageTarget::Lcd, 2800);
        axp192.set_ldo_voltage(LDOTarget::Periph, 3300);
        axp192.set_ldo_voltage(LDOTarget::Vibrate, 2000);
        axp192.enable_ldo(LDOTarget::Periph, true);
        axp192.enable_ldo(LDOTarget::Vibrate, false);
        axp192.enable_dcdc(VoltageTarget::Esp, true);
        axp192.enable_dcdc(VoltageTarget::Lcd, true);
        axp192.set_charging_current(100);
        // set gpio43 to (read & 0x72) | 0x84
        // set pek to 0x4c
        axp192
            .i2c
            .write(axp192.addr, &[Registers::Pek.into(), 0x4c])
            .unwrap();
        // set 0x82 to oxff
        axp192.set_lcd_rst(false);
        let _ = delay.delay_ms(100);
        axp192.set_lcd_rst(true);
        let _ = delay.delay_ms(100);

        if axp192.get_vbus() {
            let cur_state = axp192.get_state(Registers::VbusIpsoutChannel);
            axp192
                .i2c
                .write(
                    axp192.addr,
                    &[Registers::VbusIpsoutChannel.into(), cur_state | 0x80],
                )
                .unwrap();
            axp192.set_bus_power_mode(PowerMode::Internal)
        } else {
            axp192.set_bus_power_mode(PowerMode::External)
        }
        // Finish init process
        axp192
    }
    fn enable_dcdc(&mut self, t_dc: VoltageTarget, state: bool) {
        let cur_state = self.get_state(Registers::Dcdc13Ldo23Control);
        let val = 0x01;
        let mark = match t_dc {
            VoltageTarget::Esp => val << 1,
            VoltageTarget::Lcd => val,
        };
        match state {
            true => self
                .i2c
                .write(
                    self.addr,
                    &[Registers::Dcdc13Ldo23Control.into(), (cur_state | mark)],
                )
                .unwrap(),
            false => self
                .i2c
                .write(
                    self.addr,
                    &[Registers::Dcdc13Ldo23Control.into(), (cur_state & !mark)],
                )
                .unwrap(),
        }
    }
    fn enable_ldo(&mut self, t_ldo: LDOTarget, state: bool) {
        let cur_state = self.get_state(Registers::Dcdc13Ldo23Control);
        let val = match state {
            true => 0x01,
            false => 0x00,
        };
        let mark = match t_ldo {
            LDOTarget::Periph => val << 2,
            LDOTarget::Vibrate => val << 3,
        };
        match state {
            true => self
                .i2c
                .write(
                    self.addr,
                    &[Registers::Dcdc13Ldo23Control.into(), (cur_state | mark)],
                )
                .unwrap(),
            false => self
                .i2c
                .write(
                    self.addr,
                    &[Registers::Dcdc13Ldo23Control.into(), (cur_state & !mark)],
                )
                .unwrap(),
        }
    }
    #[allow(unused)]
    fn check_vbus(&mut self) -> bool {
        let mut buf = [0_u8];
        self.i2c
            .write_read(self.addr, &[Registers::PowerStatus.into()], &mut buf)
            .expect("Unable to read Power status");
        let ret: bool = (buf[0] & 0x08) != 0;
        ret
    }
    // Select source for BUS_5V
    // 0 : use internal power
    // 1 : powered externally
    fn set_bus_power_mode(&mut self, mode: PowerMode) {
        match mode {
            PowerMode::Internal => {
                let cur_state_gpio = self.get_state(Registers::Gpio0Ldoio0Voltage);
                self.i2c
                    .write(self.addr, &[(cur_state_gpio & 0x0f) | 0xf8])
                    .expect("Failed to write LDO Voltage");
                self.init_gpio(GpioPin::Gpio0, GpioMode::PWMOutput);
                let cur_state_boost = self.get_state(Registers::ExtenDcdc2Control);
                self.i2c
                    .write(
                        self.addr,
                        &[Registers::ExtenDcdc2Control.into(), cur_state_boost | 0x04],
                    )
                    .unwrap();
            }
            PowerMode::External => {
                let cur_state_boost = self.get_state(Registers::ExtenDcdc2Control);
                self.i2c
                    .write(
                        self.addr,
                        &[Registers::ExtenDcdc2Control.into(), cur_state_boost & !0x04],
                    )
                    .unwrap();
                self.init_gpio(GpioPin::Gpio0, GpioMode::Floating);
            }
        };
    }
    fn init_gpio_34(&mut self, gpio_num: GpioPin, mode: GpioMode) {
        let mut or_val = match mode {
            GpioMode::ChargeControl => 0x00,
            GpioMode::OpenDrainOutput => 0x01,
            GpioMode::Input => 0x02,
            _ => panic!("Incorrect mode for GPIO 3 or 4"),
        };
        let and_val = match gpio_num {
            GpioPin::Gpio3 => {
                or_val |= 0x80;
                0x7c
            }
            GpioPin::Gpio4 => {
                or_val = 0x80 | or_val << 2;
                0x73
            }
            _ => panic!("Incorrect pin passed to init for gpio 3 or 4"),
        };
        let cur_state = self.get_state(Registers::Gpio43FunctionControl);
        self.i2c
            .write(
                self.addr,
                &[
                    Registers::Gpio43FunctionControl.into(),
                    (cur_state & and_val) | or_val,
                ],
            )
            .unwrap();
    }
    fn init_gpio_20(&mut self, gpio_num: GpioPin, mode: GpioMode) {
        // TODO Fix so it can turn off gpios
        let or_val = match mode {
            GpioMode::OpenDrainOutput => 0x00,
            GpioMode::Input => 0x01,
            GpioMode::PWMOutput => 0x02,
            GpioMode::ADCInput => 0x04,
            GpioMode::LowOutput => 0x05,
            GpioMode::Floating => 0x07,
            _ => panic!("Got incorrect mode for gpio pin {:?}", gpio_num),
        };
        let reg = match gpio_num {
            GpioPin::Gpio0 => Registers::Gpio0Control,
            GpioPin::Gpio1 => Registers::Gpio1Control,
            GpioPin::Gpio2 => Registers::Gpio2Control,
            _ => panic!("Incorrect gpio_num passed in as {:?}", gpio_num),
        };
        let and_val = 0xf8; //Clears the last 3 bits but preserves all others
        let cur_state = self.get_state(reg);
        self.i2c
            .write(self.addr, &[reg.into(), (cur_state & and_val) | or_val])
            .expect("Failed to init gpio");
    }
    fn init_gpio(&mut self, gpio_num: GpioPin, mode: GpioMode) {
        match gpio_num {
            GpioPin::Gpio0 | GpioPin::Gpio1 | GpioPin::Gpio2 => self.init_gpio_20(gpio_num, mode),
            GpioPin::Gpio3 | GpioPin::Gpio4 => self.init_gpio_34(gpio_num, mode),
        };
    }
    fn get_state(&mut self, reg: Registers) -> u8 {
        let mut buf = [0];
        self.i2c
            .write_read(self.addr, &[reg.into()], &mut buf)
            .expect("Failed to read state of register");
        u8::from_be_bytes(buf)
    }
    fn set_gpio_state(&mut self, gpio: GpioPin, state: bool) {
        //TODO check if gpio is enabled
        let reg = match gpio {
            GpioPin::Gpio0 | GpioPin::Gpio1 | GpioPin::Gpio2 => Registers::Gpio20SignalStatus,
            GpioPin::Gpio3 | GpioPin::Gpio4 => Registers::Gpio43SignalStatus,
        };
        let cur_state = self.get_state(reg);
        info!(
            "Setting gpio {:?} to state {} with current being {:?}",
            gpio, state, reg
        );
        let or_val = match gpio {
            GpioPin::Gpio0 => 0x01,
            GpioPin::Gpio1 => 0x02,
            GpioPin::Gpio2 => 0x04,
            GpioPin::Gpio3 => 0x01,
            GpioPin::Gpio4 => 0x02,
        };
        let and_val = match gpio {
            GpioPin::Gpio0 => 0xfe,
            GpioPin::Gpio1 => 0xfd,
            GpioPin::Gpio2 => 0xfb,
            GpioPin::Gpio3 => 0xfe,
            GpioPin::Gpio4 => 0xfd,
        };

        match state {
            true => self
                .i2c
                .write(self.addr, &[reg.into(), (cur_state & and_val) | or_val])
                .expect("Unable to write gpio status"),
            false => self
                .i2c
                .write(self.addr, &[reg.into(), (cur_state & and_val) & !or_val])
                .expect("Unable to write gpio status"),
        }
    }

    fn set_charging_current(&mut self, current: u8) {
        let cur_state = self.get_state(Registers::ChargeControl1);
        let data = (cur_state & 0xf0) | (current | 0x0f);
        self.i2c
            .write(self.addr, &[Registers::ChargeControl1.into(), data])
            .unwrap();
    }
    fn set_rtc_chg(&mut self, state: bool) {
        let cur_state = self.get_state(Registers::BatteryChargeControl);
        let and_val = 0x1c;
        let or_val = 0xa2;
        match state {
            true => self
                .i2c
                .write(
                    self.addr,
                    &[
                        Registers::BatteryChargeControl.into(),
                        (cur_state & and_val) | or_val,
                    ],
                )
                .expect("Failed to set rtc battery charging"),
            false => self
                .i2c
                .write(
                    self.addr,
                    &[
                        Registers::BatteryChargeControl.into(),
                        (cur_state & and_val) & !or_val,
                    ],
                )
                .expect("Failed to set rtc battery charging"),
        }
    }
    fn set_vbus_limit(&mut self, state: bool) {
        let or_val = match state {
            true => 0x02,
            false => 0x00,
        };
        let cur_state = self.get_state(Registers::VbusIpsoutChannel);
        self.i2c
            .write(
                self.addr,
                &[
                    Registers::VbusIpsoutChannel.into(),
                    (cur_state & 0x04) | or_val,
                ],
            )
            .expect("Failed to set vbus limit");
    }
    fn set_dcdc_voltage(&mut self, target: VoltageTarget, voltage: u16) {
        // if 3000 <= voltage && voltage <= 3400{
        if !(2500..3400).contains(&voltage) {
            panic!("Wrong Voltage sent {:?}", voltage);
        }
        let v_target: u8 = if voltage < 700 {
            0
        } else {
            ((voltage - 700) / 25)
                .try_into()
                .expect("Too large to fit into u8")
        };
        // 0 esp
        // 2 lcd
        let t_reg = match target {
            VoltageTarget::Lcd => Registers::Dcdc3Voltage,
            VoltageTarget::Esp => Registers::Dcdc1Voltage,
        };
        let cur_state = self.get_state(t_reg);
        self.i2c
            .write(
                self.addr,
                &[t_reg.into(), (cur_state & 0x80) | (v_target & 0x7F)],
            )
            .expect("Failed to set_voltage");
    }
    fn set_ldo_voltage(&mut self, t_ldo: LDOTarget, voltage: u16) {
        let v_target: u8 = if voltage > 3300 {
            15
        } else {
            (voltage / 100) as u8 - 18
        };
        let reg = Registers::Ldo23Voltage;
        let and_val = match t_ldo {
            LDOTarget::Periph => 0x0f,
            LDOTarget::Vibrate => 0xf0,
        };
        let or_val = match t_ldo {
            LDOTarget::Periph => v_target << 4,
            LDOTarget::Vibrate => v_target,
        };
        let cur_state = self.get_state(reg);
        self.i2c
            .write(self.addr, &[reg.into(), (cur_state & and_val) | or_val])
            .unwrap();
    }
    fn get_vbus(&mut self) -> bool {
        let status = self.get_state(Registers::PowerStatus) & 0x08;
        status != 0
    }
    // LDOEnable = 0x12,
    // LDOVoltage = 0x28,
    // VbusLimit = 0x30,
    // RtcChg = 0x35,
    // Gpio1 = 0x92,
    // Gpio2 = 0x93,

    // ESPVoltage = 0x26,
    // LCDVoltage = 0x27,
    pub fn set_led(&mut self, state: bool) {
        info!("Setting led to {}", state);
        self.set_gpio_state(GpioPin::Gpio1, state);
    }
    pub fn set_lcd_rst(&mut self, state: bool) {
        self.set_gpio_state(GpioPin::Gpio4, state)
    }
    fn get_charging(&mut self) -> bool {
        let cur_state = self.get_state(Registers::PowerStatus);
        cur_state & 0x04 != 0
    }

    fn get_bat_voltage(&mut self) -> f32 {
        let adc_lsb: f32 = 1.1 / 1000.0;
        let mut buf = [0; 4];
        self.i2c
            .write_read(self.addr, &[Registers::BatteryVoltage.into()], &mut buf)
            .unwrap();
        f32::from_be_bytes(buf) * adc_lsb
    }

    pub fn get_battery_percentage(&mut self) -> u8 {
        let v = self.get_bat_voltage();
        // let per = look_up(v);
        // match self.get_charging() {
        //     true => BatteryState::Discharging(per.try_into().unwrap()),
        //     false => BatteryState::Charging(per.try_into().unwrap())
        // }
        look_up(v)
    }

    // TODO add turn on function
    // TODO add sleep functions
    // TODO add charging table functions
}

fn look_up(val: f32) -> u8 {
    info!("Converting {:?}v to battery soc", val);
    if val > BATTERY_LEVEL_DIS[0] {
        return 100;
    }
    if val < BATTERY_LEVEL_DIS[100] {
        return 0;
    }
    for i in 1..101 {
        if BATTERY_LEVEL_DIS[i - 1] > val && BATTERY_LEVEL_DIS[i] < val {
            return (100 - i).try_into().unwrap();
        }
    }
    panic!("Didn't find value");
}

#[cfg(test)]
mod tests {
    use crate::look_up;

    #[test]
    fn bat_look_up_100() {
        let bat_soc = look_up(4.5);
        assert_eq!(100, bat_soc);
    }
    #[test]
    fn bat_look_up_10() {
        let bat_soc = look_up(3.640);
        assert_eq!(10, bat_soc);
    }
    #[test]
    fn bat_look_up_99() {
        let bat_soc = look_up(4.15);
        assert_eq!(99, bat_soc);
    }
    #[test]
    fn bat_look_up_1() {
        let bat_soc = look_up(3.3750);
        assert_eq!(0, bat_soc);
    }
    #[test]
    fn bat_look_up_0() {
        let bat_soc = look_up(3.2);
        assert_eq!(0, bat_soc);
    }
}
