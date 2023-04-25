//! host library for the serial protocol to the Raspberry Pico

use serialport::SerialPort;

pub fn connect(port_name: &str) -> Result<Box<dyn SerialPort>, serialport::Error> {
    serialport::new(port_name, 115200)
        .data_bits(serialport::DataBits::Eight)
        .flow_control(serialport::FlowControl::None)
        .stop_bits(serialport::StopBits::One)
        .parity(serialport::Parity::None)
        .timeout(std::time::Duration::from_millis(10))
        .open()
}
