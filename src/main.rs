use anyhow::{Result, Context};
use crc16::MODBUS;
use paho_mqtt::{Client, Message};
use serialport::{DataBits, Parity, StopBits, SerialPortType, SerialPort};
use std::{thread::sleep, time::Duration};

const SUNRISE_ADDRESS: u8 = 0x68;

struct Sunrise {
    port: Box<dyn SerialPort>,
}

impl Sunrise {
    fn new(port_name: &str) -> Result<Sunrise> {
        let port = serialport::new(port_name, 9600)
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .timeout(Duration::from_secs(2))
            .open()
            .context("Failed to open serial port")?;

        Ok(Sunrise { port })
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        let crc = crc16::State::<MODBUS>::calculate(data);
        let crc_bytes = crc.to_le_bytes();

        let mut buf = Vec::with_capacity(data.len() + 2);
        buf.extend_from_slice(data);
        buf.extend_from_slice(&crc_bytes);

        self.port.write_all(&buf).context("Failed to write to serial port")
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<()> {
        self.port.read_exact(buf).context("Failed to read from serial port")
    }

    fn read_input_register(&mut self, address: u16, count: u16) -> Result<Vec<u16>> {
        let mut buf = [0u8; 6];
        buf[0] = SUNRISE_ADDRESS;
        buf[1] = 0x04;
        buf[2..4].copy_from_slice(&address.to_be_bytes());
        buf[4..6].copy_from_slice(&count.to_be_bytes());
        self.write(&buf)?;

        let mut buf = Vec::with_capacity(5 + 2 * count as usize);
        buf.resize(5 + 2 * count as usize, 0);
        self.read_exact(&mut buf)?;

        if buf[0] != SUNRISE_ADDRESS { anyhow::bail!("Invalid address"); }
        if buf[1] != 0x04 { anyhow::bail!("Invalid function code"); }
        if buf[2] != (count * 2) as u8 { anyhow::bail!("Invalid byte count"); }

        let crc = crc16::State::<MODBUS>::calculate(&buf[..buf.len() - 2]);
        let crc_bytes = crc.to_le_bytes();
        if buf[buf.len() - 2] != crc_bytes[0] || buf[buf.len() - 1] != crc_bytes[1] {
            anyhow::bail!("Invalid CRC");
        }

        Ok(buf[3..3 + (count * 2) as usize]
            .chunks_exact(2)
            .map(|chunk| u16::from_be_bytes([chunk[0], chunk[1]]))
            .collect())
    }

    fn write_single_register(&mut self, address: u16, value: u16) -> Result<()> {
        let mut buf = [0u8; 9];
        buf[0] = SUNRISE_ADDRESS;
        buf[1] = 0x10;
        buf[2..4].copy_from_slice(&address.to_be_bytes());
        buf[4..6].copy_from_slice(&0x01_u16.to_be_bytes());
        buf[6] = 0x02;
        buf[7..9].copy_from_slice(&value.to_be_bytes());
        self.write(&buf)?;

        let mut buf = [0u8; 8];
        self.read_exact(&mut buf)?;

        if buf[0] != SUNRISE_ADDRESS { anyhow::bail!("Invalid address"); }
        if buf[1] != 0x10 { anyhow::bail!("Invalid function code"); }
        if buf[2..4] != address.to_be_bytes() { anyhow::bail!("Invalid address"); }
        if buf[4..6] != 0x01_u16.to_be_bytes() { anyhow::bail!("Invalid length"); }

        let crc = crc16::State::<MODBUS>::calculate(&buf[..buf.len() - 2]);
        let crc_bytes = crc.to_le_bytes();
        if buf[buf.len() - 2] != crc_bytes[0] || buf[buf.len() - 1] != crc_bytes[1] {
            anyhow::bail!("Invalid CRC");
        }

        Ok(())
    }

    fn get_measurement(&mut self) -> Result<Measurement> {
        let res = self.read_input_register(0x0, 5)?;

        let errors = SunriseError::from_byte(res[0]);

        if !errors.is_empty() {
            anyhow::bail!("Sunrise reported errors: {:?}", errors);
        }

        // .[1] and .[2] are reserved
        let co2 = res[3];
        let temperature = res[4] as f32 / 100.0;

        Ok(Measurement { co2, temperature })
    }

    fn target_calibration(&mut self, target: u16) -> Result<()> {
        // write calibration target
        self.write_single_register(0x02, target)
            .context("Failed to write calibration target")?;

        // write calibration command
        self.write_single_register(0x01, 0x7c05)
            .context("Failed to write calibration command")?;

        sleep(Duration::from_secs(1));

        let res = self.read_input_register(0x00, 1)
            .context("Failed to read calibration status")?;

        println!("Calibration status: {:#04x}", res[0]);

        Ok(())
    }
}

struct Measurement {
    temperature: f32,
    co2: u16,
}

#[derive(Debug)]
enum SunriseError {
    Fatal,
    Communication,
    Algorithm,
    Calibration,
    SelfDiagnostics,
    OutOfRange,
    Memory,
    NoMeasurementCompleted,
    LowInternalVoltage,
    MeasurementTimeout,
    AbnormalSignalLevel,
}

impl SunriseError {
    fn from_byte(byte: u16) -> Vec<SunriseError> {
        let mut errors = vec![];
        if byte & (1 << 0) != 0 { errors.push(SunriseError::Fatal); }
        if byte & (1 << 1) != 0 { errors.push(SunriseError::Communication); }
        if byte & (1 << 2) != 0 { errors.push(SunriseError::Algorithm); }
        if byte & (1 << 3) != 0 { errors.push(SunriseError::Calibration); }
        if byte & (1 << 4) != 0 { errors.push(SunriseError::SelfDiagnostics); }
        if byte & (1 << 5) != 0 { errors.push(SunriseError::OutOfRange); }
        if byte & (1 << 6) != 0 { errors.push(SunriseError::Memory); }
        if byte & (1 << 7) != 0 { errors.push(SunriseError::NoMeasurementCompleted); }
        if byte & (1 << 8) != 0 { errors.push(SunriseError::LowInternalVoltage); }
        if byte & (1 << 9) != 0 { errors.push(SunriseError::MeasurementTimeout); }
        if byte & (1 << 10) != 0 { errors.push(SunriseError::AbnormalSignalLevel); }
        errors
    }
}

fn main() -> Result<()> {
    let port_name = serialport::available_ports()
        .expect("listing ports")
        .into_iter()
        .find(|p| match &p.port_type {
            SerialPortType::UsbPort(info) => {
                info.serial_number == Some("AH034FKQ".to_owned())
            }
            _ => false
        })
        .expect("finding port")
        .port_name;

    let mut sunrise = Sunrise::new(&port_name)
        .context("Failed to init Sunrise connection")?;

    // sunrise.target_calibration(560).context("Calibration failed")?;

    let client = Client::new(("tcp://localhost:1883", "sunrise")).expect("creating mqtt client");
    client.connect(None).expect("connecting");

    loop {
        match sunrise.get_measurement().context("Failed to get measurement") {
            Ok(Measurement { temperature, co2 }) => {
                println!("co2: {co2}, temp: {temperature}");

                let message = Message::new("sunrise/co2", co2.to_string(), 0);
                client.publish(message).context("Failed to publish CO2")?;

                let message = Message::new("sunrise/temp", temperature.to_string(), 0);
                client.publish(message).context("Failed to publish temperature")?;
            }
            Err(e) => {
                println!("Error: {e}");

                for cause in e.chain() {
                    println!("  caused by: {cause}");
                }
            }
        }

        sleep(Duration::from_secs(5));
    }
}
