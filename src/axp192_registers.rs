use num_enum::IntoPrimitive;

#[allow(unused)]
#[derive(IntoPrimitive, Debug, Clone, Copy)]
#[repr(u8)]
pub enum Registers {
    /* Power control registers */
    PowerStatus = 0x00,
    ChargeStatus = 0x01,
    OtgVsusStatus = 0x04,
    DataBuffer0 = 0x06,
    DataBuffer1 = 0x07,
    DataBuffer2 = 0x08,
    DataBuffer3 = 0x09,
    DataBuffer4 = 0x0a,
    DataBuffer5 = 0x0b,
    /* Output control: 2 EXTEN, 0 DCDC2 */
    ExtenDcdc2Control = 0x10,
    /* Power output control: 6 EXTEN, 4 DCDC= , 3 LDO3, 2 LDO2, 1 DCDC3, 0 DCDC1 */
    Dcdc13Ldo23Control = 0x12,
    Dcdc2Voltage = 0x23,
    Dcdc2Slope = 0x25,
    Dcdc1Voltage = 0x26,
    Dcdc3Voltage = 0x27,
    /* Output voltage control: 7-4 LDO2, 3-0= LDO3 */
    Ldo23Voltage = 0x28,
    VbusIpsoutChannel = 0x30,
    ShutdownVoltage = 0x31,
    ShutdownBatteryChgledControl = 0x32,
    ChargeControl1 = 0x33,
    ChargeControl2 = 0x34,
    BatteryChargeControl = 0x35,
    Pek = 0x36,
    DcdcFrequency = 0x37,
    BatteryChargeLowTemp = 0x38,
    BatteryChargeHighTemp = 0x39,
    ApsLowPower1 = 0x3A,
    ApsLowPower2 = 0x3B,
    BatteryDischargeLowTemp = 0x3c,
    BatteryDischargeHighTemp = 0x3d,
    DcdcMode = 0x80,
    AdcEnable1 = 0x82,
    AdcEnable2 = 0x83,
    AdcRateTsPin = 0x84,
    Gpio30InputRange = 0x85,
    Gpio0AdcIrqRising = 0x86,
    Gpio0AdcIrqFalling = 0x87,
    TimerControl = 0x8a,
    VbusMonitor = 0x8b,
    TempShutdownControl = 0x8f,
    /* GPIO control registers */
    Gpio0Control = 0x90,
    Gpio0Ldoio0Voltage = 0x91,
    Gpio1Control = 0x92,
    Gpio2Control = 0x93,
    Gpio20SignalStatus = 0x94,
    Gpio43FunctionControl = 0x95,
    Gpio43SignalStatus = 0x96,
    Gpio20PulldownControl = 0x97,
    Pwm1Frequency = 0x98,
    Pwm1DutyCycle1 = 0x99,
    Pwm1DutyCycle2 = 0x9a,
    Pwm2Frequency = 0x9b,
    Pwm2DutyCycle1 = 0x9c,
    Pwm2DutyCycle2 = 0x9d,
    NRstoGpio5Control = 0x9e,

    /* Interrupt control registers */
    EnableControl1 = 0x40,
    EnableControl2 = 0x41,
    EnableControl3 = 0x42,
    EnableControl4 = 0x43,
    EnableControl5 = 0x4a,
    IrqStatus1 = 0x44,
    IrqStatus2 = 0x45,
    IrqStatus3 = 0x46,
    IrqStatus4 = 0x47,
    IrqStatus5 = 0x4d,

    /* ADC data registers */
    AcinVoltage = 0x56,
    AcinCurrent = 0x58,
    VbusVoltage = 0x5a,
    VbusCurrent = 0x5c,
    Temp = 0x5e,
    TsInput = 0x62,
    Gpio0Voltage = 0x64,
    Gpio1Voltage = 0x66,
    Gpio2Voltage = 0x68,
    Gpio3Voltage = 0x6a,
    BatteryPower = 0x70,
    BatteryVoltage = 0x78,
    ChargeCurrent = 0x7a,
    DischargeCurrent = 0x7c,
    ApsVoltage = 0x7e,
    ChargeCoulomb = 0xb0,
    DischargeCoulomb = 0xb4,
    CoulombCounterControl = 0xb8,

    /* Computed ADC */
    CoulombCounter = 0xff,
}

#[allow(unused, clippy::enum_variant_names)]
#[derive(IntoPrimitive, Debug)]
#[repr(u8)]
pub enum IoCtl {
    /* IOCTL commands */
    CoulombCounterEnable = 0x80,
    CoulombCounterDisable = 0x00,
    CoulombCounterSuspend = 0xc0,
    CoulombCounterClear = 0xa0,
}

#[cfg(test)]
mod tests {

    use crate::Registers;

    #[test]
    fn check_into() {
        let t1: u8 = Registers::ChargeStatus.into();
        assert_eq!(0x01, t1);
        let t2: &u8 = &Registers::BatteryVoltage.into();
        assert_eq!(&0x78, t2);
    }
}
